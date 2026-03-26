#!/usr/bin/env python3
"""
EvilCrow RF v2 — SDR Control Library

Python interface for the EvilCrow RF v2 SDR mode.
Communicates via USB serial using HackRF-compatible text commands.

IMPORTANT: The CC1101 is NOT a true SDR. This module provides:
  - Spectrum scanning (real RSSI measurements per frequency step)
  - Raw RX streaming (demodulated bytes from CC1101 FIFO, not raw IQ)
  - Frequency / modulation / bandwidth configuration

Usage:
    from evilcrow_sdr import EvilCrowSDR

    with EvilCrowSDR('/dev/ttyUSB0') as sdr:
        sdr.set_frequency(433.92e6)
        sdr.set_modulation('ASK')
        sdr.start_rx()
        data = sdr.read_raw(256)
        sdr.stop_rx()
"""

import time
import threading
import queue
from typing import Dict, List, Optional, Tuple

from connection import EvilCrowConnection

# ── CC1101 parameter limits ────────────────────────────────────────

CC1101_LIMITS = {
    'frequency_bands_mhz': [(300.0, 348.0), (387.0, 464.0), (779.0, 928.0)],
    'bandwidths_khz': [58, 68, 81, 102, 116, 135, 162, 203,
                       232, 270, 325, 406, 464, 541, 650, 812],
    'data_rate_baud': (600, 500_000),
    'modulations': {
        0: '2-FSK', 1: 'GFSK', 2: 'ASK/OOK', 3: '4-FSK', 4: 'MSK',
    },
    'fifo_bytes': 64,
}


def is_valid_frequency(freq_hz: float) -> bool:
    freq_mhz = freq_hz / 1e6
    return any(lo <= freq_mhz <= hi
               for lo, hi in CC1101_LIMITS['frequency_bands_mhz'])


def print_cc1101_limits():
    print('\n╔══════════════════════════════════════════════════════╗')
    print('║        CC1101 SDR Parameter Limits                  ║')
    print('╠══════════════════════════════════════════════════════╣')
    print('║ Frequency bands:                                   ║')
    for lo, hi in CC1101_LIMITS['frequency_bands_mhz']:
        line = f'{lo:>9.1f} – {hi:>7.1f} MHz'
        print(f'║     {line:<49}║')
    print('║                                                    ║')
    print('║ Bandwidth (kHz, discrete values):                  ║')
    bws = CC1101_LIMITS['bandwidths_khz']
    line = '  '.join(f'{b}' for b in bws[:8])
    print(f'║   {line:<50}║')
    line = '  '.join(f'{b}' for b in bws[8:])
    print(f'║   {line:<50}║')
    print('║                                                    ║')
    lo, hi = CC1101_LIMITS['data_rate_baud']
    print(f'║ Data rate: {lo:,} – {hi:,} Baud{" " * 21}║')
    print('║                                                    ║')
    print('║ Modulations:                                       ║')
    for k, v in CC1101_LIMITS['modulations'].items():
        print(f'║   {k} = {v:<46}║')
    print('║                                                    ║')
    print(f'║ RX FIFO: {CC1101_LIMITS["fifo_bytes"]} bytes{" " * 36}║')
    print('║ NOTE: NOT a true SDR — no raw IQ output.           ║')
    print('║       Data is demodulated bytes from CC1101 FIFO.  ║')
    print('╚══════════════════════════════════════════════════════╝\n')


class EvilCrowSDR:
    """
    EvilCrow RF v2 USB SDR control interface.

    Uses EvilCrowConnection for serial link management.
    """

    MODULATIONS = {
        '2FSK': 0, 'GFSK': 1, 'ASK': 2, 'OOK': 2,
        'ASK/OOK': 2, '4FSK': 3, 'MSK': 4,
    }

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout: float = 2.0, debug: bool = False):
        self.conn = EvilCrowConnection(port, baudrate, timeout, debug)
        self._streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        self._rx_queue: queue.Queue = queue.Queue(maxsize=50000)

        # Tracked state
        self.frequency_hz: float = 433.92e6
        self.modulation: int = 2
        self.bandwidth_khz: float = 650.0
        self.data_rate_baud: float = 3793.72

        self._connect()

    def _connect(self):
        self.conn.open()

        print('[...] Enabling SDR mode...')
        if self.conn.enable_sdr():
            print('[OK] SDR mode enabled via serial')
        else:
            print('[WARN] sdr_enable did not return SUCCESS')
            print('       Trying board_id_read anyway...')

        if not self.conn.verify_device():
            raise ConnectionError(
                f'Device on {self.conn.port} did not respond as EvilCrow SDR.\n'
                'Firmware may need updating (sdr_enable serial command).'
            )
        print(f'[OK] Connected to EvilCrow SDR on {self.conn.port}')

    def send_command(self, command: str) -> str:
        return self.conn.send_command(command)

    # ── Configuration commands ─────────────────────────────────

    def set_frequency(self, freq_hz: float) -> bool:
        if not is_valid_frequency(freq_hz):
            print(f'[WARN] {freq_hz/1e6:.2f} MHz is outside CC1101 bands')
            return False
        resp = self.send_command(f'set_freq {int(freq_hz)}')
        if 'SUCCESS' in resp:
            self.frequency_hz = freq_hz
            print(f'[OK] Frequency: {freq_hz/1e6:.3f} MHz')
            return True
        print(f'[ERR] set_freq failed: {resp}')
        return False

    def set_modulation(self, mod: str) -> bool:
        mod_upper = mod.upper()
        if mod_upper not in self.MODULATIONS:
            print(f'[ERR] Unknown modulation: {mod}')
            return False
        mod_id = self.MODULATIONS[mod_upper]
        resp = self.send_command(f'set_modulation {mod_id}')
        if 'SUCCESS' in resp:
            self.modulation = mod_id
            print(f'[OK] Modulation: {mod_upper} ({mod_id})')
            return True
        print(f'[ERR] set_modulation failed: {resp}')
        return False

    def set_bandwidth(self, bw_khz: float) -> bool:
        resp = self.send_command(f'set_bandwidth {bw_khz}')
        if 'SUCCESS' in resp:
            self.bandwidth_khz = bw_khz
            print(f'[OK] Bandwidth: {bw_khz:.1f} kHz')
            return True
        print(f'[ERR] set_bandwidth failed: {resp}')
        return False

    def set_data_rate(self, rate_hz: float) -> bool:
        resp = self.send_command(f'set_sample_rate {int(rate_hz)}')
        if 'SUCCESS' in resp:
            self.data_rate_baud = rate_hz
            print(f'[OK] Data rate: {rate_hz/1000:.2f} kBaud')
            return True
        print(f'[ERR] set_sample_rate failed: {resp}')
        return False

    def set_gain(self, gain_db: int) -> bool:
        resp = self.send_command(f'set_gain {gain_db}')
        if 'SUCCESS' in resp:
            print(f'[OK] Gain: {gain_db} dB (AGC mode)')
            return True
        return False

    def get_status(self) -> Dict[str, str]:
        resp = self.send_command('sdr_status')
        info = {}
        for line in resp.split('\n'):
            if ':' in line:
                key, _, val = line.partition(':')
                info[key.strip()] = val.strip()
        return info

    def enable_sdr(self) -> bool:
        return self.conn.enable_sdr()

    def disable_sdr(self) -> bool:
        resp = self.send_command('sdr_disable')
        ok = 'SUCCESS' in resp.upper()
        if ok:
            print('[OK] SDR mode disabled')
        else:
            print(f'[ERR] sdr_disable failed: {resp}')
        return ok

    def get_device_info(self) -> str:
        return self.send_command('board_id_read')

    def get_sdr_info(self) -> str:
        return self.send_command('sdr_info')

    # ── Spectrum scan ──────────────────────────────────────────

    def spectrum_scan(self, start_hz: float, end_hz: float,
                      step_khz: float = 100) -> List[Tuple[float, int]]:
        start_mhz = start_hz / 1e6
        end_mhz = end_hz / 1e6
        print(f'[SCAN] {start_mhz:.2f} - {end_mhz:.2f} MHz, step {step_khz:.0f} kHz')
        self.send_command(
            f'spectrum_scan {start_mhz:.2f} {end_mhz:.2f} {step_khz:.0f}')
        print('[OK] Spectrum scan requested. Results arrive via BLE.')
        return []

    # ── Raw RX streaming ───────────────────────────────────────

    def start_rx(self) -> bool:
        resp = self.send_command('rx_start')
        if 'SUCCESS' in resp:
            self._streaming = True
            self._stream_thread = threading.Thread(
                target=self._rx_worker, daemon=True)
            self._stream_thread.start()
            print('[OK] RX streaming started')
            return True
        print(f'[ERR] rx_start failed: {resp}')
        return False

    def stop_rx(self) -> bool:
        self._streaming = False
        if self._stream_thread:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
        resp = self.send_command('rx_stop')
        if 'SUCCESS' in resp:
            print('[OK] RX streaming stopped')
            return True
        return False

    def read_raw(self, count: int = 1024, timeout: float = 2.0) -> bytes:
        result = bytearray()
        deadline = time.time() + timeout
        while len(result) < count and time.time() < deadline:
            try:
                chunk = self._rx_queue.get(timeout=0.1)
                result.extend(chunk)
            except queue.Empty:
                continue
        return bytes(result[:count])

    def _rx_worker(self):
        ser = self.conn.ser
        while self._streaming and ser and ser.is_open:
            try:
                avail = ser.in_waiting
                if avail > 0:
                    data = ser.read(min(avail, 256))
                    if data:
                        try:
                            self._rx_queue.put_nowait(data)
                        except queue.Full:
                            try:
                                self._rx_queue.get_nowait()
                            except queue.Empty:
                                pass
                            self._rx_queue.put_nowait(data)
                else:
                    time.sleep(0.005)
            except Exception:
                break

    # ── Cleanup ────────────────────────────────────────────────

    def __repr__(self) -> str:
        state = 'streaming' if self._streaming else 'idle'
        return (f"<EvilCrowSDR port={self.conn.port} "
                f"freq={self.frequency_hz / 1e6:.3f}MHz "
                f"mod={self.modulation} state={state}>")

    def close(self):
        if self._streaming:
            self.stop_rx()
        if self.conn.is_open:
            try:
                self.send_command('sdr_disable')
            except Exception:
                pass
            self.conn.close()
            print('[OK] Disconnected (SDR mode disabled)')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ── Quick test ─────────────────────────────────────────────────

if __name__ == '__main__':
    import sys

    debug = '--debug' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--debug']
    port = args[0] if args else 'COM8'

    print_cc1101_limits()

    try:
        with EvilCrowSDR(port, debug=debug) as sdr:
            print('\n--- Device Info ---')
            print(sdr.get_device_info())

            print('\n--- SDR Info (CC1101 Limits) ---')
            print(sdr.get_sdr_info())

            print('\n--- Status ---')
            for k, v in sdr.get_status().items():
                print(f'  {k}: {v}')

            sdr.set_frequency(433.92e6)
            sdr.set_modulation('ASK')
            sdr.set_bandwidth(650)

            print('\n--- RX Test (3 seconds) ---')
            sdr.start_rx()
            time.sleep(3)
            data = sdr.read_raw(256, timeout=0.5)
            sdr.stop_rx()
            print(f'Received {len(data)} bytes')
            if data:
                print(f'Hex: {data[:32].hex()}')

    except Exception as e:
        print(f'Error: {e}')
