#!/usr/bin/env python3
"""
EvilCrow RF v2 — GNU Radio Source Block

Provides a source that reads demodulated data from the EvilCrow RF v2
and outputs it as complex float samples for GNU Radio flowgraphs.

Standalone test:
    python gnuradio_source.py --port COM8 --freq 433.92e6
"""

VERSION = "2.0.0"

import logging
import threading
import queue
import time
import argparse
import sys

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("pyserial is required: pip install pyserial")

from connection import EvilCrowConnection

log = logging.getLogger(__name__)

try:
    from gnuradio import gr
    import pmt
    HAS_GNURADIO = True
except ImportError:
    HAS_GNURADIO = False


def find_serial_port() -> str:
    """Auto-detect EvilCrow serial port."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        if any(c in desc for c in ['cp210', 'ch340', 'ch9102', 'ftdi']):
            return p.device
        if (p.vid or 0) == 0x10C4:
            return p.device
    ports = serial.tools.list_ports.comports()
    if ports:
        return ports[0].device
    raise RuntimeError('No serial ports found')


class EvilCrowSource:
    """
    Core SDR source that reads from EvilCrow serial.

    Can be used standalone or wrapped in a GNU Radio block.
    """

    def __init__(self, port: str, frequency: float = 433.92e6,
                 modulation: int = 2, baudrate: int = 115200):
        self.port = port
        self.frequency = frequency
        self.modulation = modulation
        self.baudrate = baudrate

        self.conn: EvilCrowConnection = None
        self.connected = False
        self.streaming = False
        self.sample_queue: queue.Queue = queue.Queue(maxsize=20000)
        self._thread: threading.Thread = None

    def connect(self) -> bool:
        """Connect to device, auto-enable SDR mode, and configure."""
        try:
            self.conn = EvilCrowConnection(self.port, self.baudrate)
            self.conn.open()

            if not self.conn.enable_sdr():
                print('[WARN] sdr_enable did not return SUCCESS')

            if not self.conn.verify_device():
                print('[ERR] Not an EvilCrow SDR device')
                return False

            self.connected = True
            print(f'[OK] Connected to EvilCrow SDR on {self.port}')

            self.set_frequency(self.frequency)
            self.set_modulation(self.modulation)

            return True
        except Exception as e:
            print(f'[ERR] Connect failed: {e}')
            return False

    def set_frequency(self, freq_hz: float):
        if self.connected:
            self.conn.send_command(f'set_freq {int(freq_hz)}')
            self.frequency = freq_hz

    def set_modulation(self, mod: int):
        if self.connected:
            self.conn.send_command(f'set_modulation {mod}')
            self.modulation = mod

    def set_bandwidth(self, bw_khz: float):
        if self.connected:
            self.conn.send_command(f'set_bandwidth {bw_khz}')

    def start_streaming(self) -> bool:
        if not self.connected:
            return False
        ser = self.conn.ser
        ser.write(b'rx_start\n')
        time.sleep(0.2)
        ser.reset_input_buffer()
        self.streaming = True
        self._thread = threading.Thread(target=self._read_worker, daemon=True)
        self._thread.start()
        print('[OK] RX streaming started')
        return True

    def stop_streaming(self):
        self.streaming = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.connected and self.conn.is_open:
            self.conn.ser.write(b'rx_stop\n')
            time.sleep(0.1)
        print('[OK] RX streaming stopped')

    def _read_worker(self):
        ser = self.conn.ser
        while self.streaming and ser and ser.is_open:
            try:
                avail = ser.in_waiting
                if avail > 0:
                    data = ser.read(min(avail, 512))
                    for b in data:
                        sample = complex((b - 128) / 128.0, 0.0)
                        try:
                            self.sample_queue.put_nowait(sample)
                        except queue.Full:
                            try:
                                self.sample_queue.get_nowait()
                            except queue.Empty:
                                pass
                            self.sample_queue.put_nowait(sample)
                else:
                    time.sleep(0.002)
            except Exception:
                break

    def read_samples(self, count: int) -> np.ndarray:
        samples = []
        deadline = time.time() + 2.0
        while len(samples) < count and time.time() < deadline:
            try:
                s = self.sample_queue.get(timeout=0.05)
                samples.append(s)
            except queue.Empty:
                continue
        if not samples:
            return np.zeros(count, dtype=np.complex64)
        return np.array(samples[:count], dtype=np.complex64)

    def close(self):
        if self.streaming:
            self.stop_streaming()
        if self.conn and self.conn.is_open:
            try:
                self.conn.send_command('sdr_disable')
            except Exception:
                pass
            self.conn.close()
        print('[OK] Disconnected')


# ── GNU Radio Block ────────────────────────────────────────────

if HAS_GNURADIO:
    class EvilCrowGRSource(gr.sync_block):
        """GNU Radio source block for EvilCrow RF v2 SDR."""

        def __init__(self, port: str = 'COM8', frequency: float = 433.92e6,
                     modulation: int = 2):
            gr.sync_block.__init__(
                self, name='EvilCrow SDR Source',
                in_sig=None, out_sig=[np.complex64],
            )
            self.source = EvilCrowSource(port, frequency, modulation)
            self.message_port_register_in(pmt.intern('freq'))
            self.set_msg_handler(pmt.intern('freq'), self._handle_freq)

        def start(self):
            if self.source.connect():
                self.source.start_streaming()
                return True
            return False

        def stop(self):
            self.source.close()
            return True

        def work(self, input_items, output_items):
            out = output_items[0]
            n = len(out)
            samples = self.source.read_samples(n)
            out[:len(samples)] = samples
            if len(samples) < n:
                out[len(samples):] = 0
            return n

        def _handle_freq(self, msg):
            if pmt.is_number(msg):
                self.source.set_frequency(pmt.to_double(msg))


# ── GRC XML block definition ──────────────────────────────────

GRC_BLOCK_XML = """<?xml version="1.0"?>
<block>
  <n>EvilCrow SDR Source</n>
  <key>evilcrow_sdr_source</key>
  <category>[EvilCrow RF]</category>
  <import>from gnuradio_source import EvilCrowGRSource</import>
  <make>EvilCrowGRSource($port, $frequency, $modulation)</make>
  <param><n>Serial Port</n><key>port</key><type>string</type><value>COM8</value></param>
  <param><n>Frequency (Hz)</n><key>frequency</key><type>real</type><value>433.92e6</value></param>
  <param><n>Modulation</n><key>modulation</key><type>int</type><value>2</value></param>
  <source><n>out</n><type>complex</type></source>
  <sink><n>freq</n><type>message</type><optional>1</optional></sink>
  <doc>
EvilCrow RF v2 SDR Source Block.
Reads demodulated data from CC1101 via USB serial.
  </doc>
</block>
"""


# ── Standalone test ────────────────────────────────────────────

def standalone_test(port: str, freq: float, duration: float):
    print(f'\n=== EvilCrow SDR Standalone Test ===')
    print(f'Port: {port}  Freq: {freq/1e6:.2f} MHz  Duration: {duration}s\n')

    source = EvilCrowSource(port, frequency=freq, modulation=2)
    if not source.connect():
        return

    source.start_streaming()
    time.sleep(duration)

    samples = source.read_samples(2048)
    source.close()

    print(f'\nCaptured {len(samples)} samples')
    if len(samples) > 0:
        power = np.mean(np.abs(samples) ** 2)
        peak = np.max(np.abs(samples))
        print(f'Average power: {power:.6f}')
        print(f'Peak amplitude: {peak:.4f}')
        outfile = f'capture_{freq/1e6:.0f}MHz.raw'
        samples.tofile(outfile)
        print(f'Saved to {outfile} (complex64 format)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='EvilCrow RF v2 — GNU Radio SDR Source')
    parser.add_argument('--port', type=str, default=None)
    parser.add_argument('--freq', type=float, default=433.92e6)
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()

    port = args.port or find_serial_port()
    standalone_test(port, args.freq, args.duration)
