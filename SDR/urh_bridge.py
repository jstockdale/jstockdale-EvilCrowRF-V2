#!/usr/bin/env python3
"""
EvilCrow RF v2 — URH Bridge (RTL-TCP compatible)

Acts as an RTL-TCP server so Universal Radio Hacker (URH) can connect
to the EvilCrow RF v2 as if it were an RTL-SDR dongle.

Usage:
    python urh_bridge.py                  # Auto-detect port
    python urh_bridge.py --port COM8      # Specify serial port
    python urh_bridge.py --tcp-port 1235  # Custom TCP port

Then in URH: File > New Project > "RTL-TCP" source > localhost:1234
"""

VERSION = "2.0.0"

import logging
import socket
import struct
import threading
import time
import sys
import argparse
import select

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("pyserial is required: pip install pyserial")

from connection import EvilCrowConnection

log = logging.getLogger(__name__)


def find_evilcrow_port() -> str:
    """Auto-detect EvilCrow serial port (CP2102 / CH340 USB-UART)."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or '').lower()
        vid = p.vid or 0
        if any(chip in desc for chip in ['cp210', 'ch340', 'ch9102', 'ftdi']):
            return p.device
        if vid == 0x10C4:
            return p.device
    if ports:
        return ports[0].device
    raise RuntimeError('No serial ports found. Is the device connected?')


class URHBridge:
    """RTL-TCP compatible bridge for EvilCrow RF v2."""

    def __init__(self, serial_port: str, tcp_port: int = 1234):
        self.serial_port = serial_port
        self.tcp_port = tcp_port
        self.conn: EvilCrowConnection = None
        self.server: socket.socket = None
        self.client: socket.socket = None
        self.running = False

    def log(self, msg: str):
        print(f'[{time.strftime("%H:%M:%S")}] {msg}')

    def connect_device(self) -> bool:
        """Open serial connection to EvilCrow and enable SDR mode."""
        try:
            self.log(f'Connecting to {self.serial_port}...')
            self.conn = EvilCrowConnection(self.serial_port)
            self.conn.open()

            self.log('Enabling SDR mode via serial...')
            if self.conn.enable_sdr():
                self.log('SDR mode enabled.')
            else:
                self.log('sdr_enable did not return SUCCESS.')
                self.log('Trying board_id_read anyway...')

            if not self.conn.verify_device():
                self.log('Device did not respond as EvilCrow SDR.')
                self.log('Firmware may need updating (sdr_enable support).')
                return False

            self.log('Device connected and SDR mode verified.')

            # Set initial config
            self.conn.send_command('set_freq 433920000')
            self.conn.send_command('set_sample_rate 250000')
            self.conn.send_command('set_gain 15')

            return True
        except Exception as e:
            self.log(f'Connection failed: {e}')
            return False

    def start_server(self) -> bool:
        """Start TCP server for URH connections."""
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind(('127.0.0.1', self.tcp_port))
            self.server.listen(1)
            self.log(f'TCP server listening on 127.0.0.1:{self.tcp_port}')
            self.log(f'In URH: File > New > "RTL-TCP" source > localhost:{self.tcp_port}')
            return True
        except Exception as e:
            self.log(f'Server start failed: {e}')
            return False

    def handle_rtl_command(self, data: bytes):
        """Handle RTL-TCP 5-byte commands from URH."""
        if len(data) < 5:
            return
        cmd = data[0]
        param = struct.unpack('>I', data[1:5])[0]
        ser = self.conn.ser

        if cmd == 0x01:
            self.log(f'  Freq: {param} Hz ({param/1e6:.3f} MHz)')
            ser.write(f'set_freq {param}\n'.encode())
        elif cmd == 0x02:
            self.log(f'  Rate: {param} Hz')
            ser.write(f'set_sample_rate {param}\n'.encode())
        elif cmd == 0x04:
            gain = param // 10
            self.log(f'  Gain: {gain} dB')
            ser.write(f'set_gain {gain}\n'.encode())
        elif cmd == 0x05:
            pass  # CC1101 always uses AGC
        else:
            self.log(f'  Unknown RTL cmd: 0x{cmd:02X} param={param}')

    def stream_data(self):
        """Read CC1101 FIFO data and stream to URH as 8-bit unsigned IQ."""
        ser = self.conn.ser
        self.log('Starting RX and data stream...')
        ser.write(b'rx_start\n')
        time.sleep(0.2)
        ser.reset_input_buffer()

        sample_count = 0
        last_log = time.time()

        try:
            while self.running and self.client:
                avail = ser.in_waiting
                if avail > 0:
                    raw = ser.read(min(avail, 512))
                    if raw:
                        iq_buf = bytearray(len(raw) * 2)
                        for i, b in enumerate(raw):
                            iq_buf[i * 2] = b
                            iq_buf[i * 2 + 1] = 127
                        try:
                            self.client.sendall(iq_buf)
                            sample_count += len(raw)
                        except (BrokenPipeError, OSError):
                            self.log('Client disconnected during stream.')
                            break
                else:
                    silence = bytes([127, 127]) * 64
                    try:
                        self.client.sendall(silence)
                    except (BrokenPipeError, OSError):
                        break
                    time.sleep(0.005)

                now = time.time()
                if now - last_log >= 5.0:
                    self.log(f'  Streamed {sample_count} samples')
                    last_log = now

        finally:
            try:
                ser.write(b'rx_stop\n')
                time.sleep(0.1)
            except Exception:
                pass
            self.log(f'Stream stopped ({sample_count} total samples)')

    def handle_client(self, client: socket.socket, addr):
        """Handle a URH client connection."""
        self.client = client
        self.log(f'URH connected from {addr[0]}:{addr[1]}')

        try:
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # RTL-TCP DongleInfo header (12 bytes)
            header = b'RTL0' + struct.pack('>II', 1, 1)
            client.sendall(header)
            self.log('Sent RTL-TCP header')

            time.sleep(0.1)

            self.running = True
            stream_t = threading.Thread(target=self.stream_data, daemon=True)
            stream_t.start()

            while self.running:
                try:
                    ready, _, _ = select.select([client], [], [], 1.0)
                    if ready:
                        data = client.recv(1024)
                        if not data:
                            break
                        for i in range(0, len(data) - 4, 5):
                            self.handle_rtl_command(data[i:i + 5])
                    if not stream_t.is_alive():
                        break
                except (ConnectionResetError, OSError):
                    break

        finally:
            self.running = False
            if stream_t.is_alive():
                stream_t.join(timeout=2.0)
            try:
                client.close()
            except Exception:
                pass
            self.client = None
            self.log('URH disconnected — ready for next connection.')

    def run(self):
        """Main entry point."""
        if not self.connect_device():
            return False
        if not self.start_server():
            return False

        self.log('Bridge ready. Waiting for URH...')
        try:
            while True:
                client, addr = self.server.accept()
                self.handle_client(client, addr)
                self.log('Ready for next connection...')
        except KeyboardInterrupt:
            self.log('Shutting down.')
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        for s in [self.client, self.server]:
            if s:
                try: s.close()
                except Exception: pass
        if self.conn and self.conn.is_open:
            try:
                self.conn.ser.write(b'rx_stop\n')
                time.sleep(0.1)
                self.conn.send_command('sdr_disable')
            except Exception:
                pass
            self.conn.close()
        self.log('Cleanup done.')


def main():
    parser = argparse.ArgumentParser(
        description='EvilCrow RF v2 — URH Bridge (RTL-TCP compatible)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port (auto-detect if omitted)')
    parser.add_argument('--tcp-port', type=int, default=1234,
                        help='TCP server port (default: 1234)')
    args = parser.parse_args()

    port = args.port or find_evilcrow_port()
    bridge = URHBridge(serial_port=port, tcp_port=args.tcp_port)
    bridge.run()


if __name__ == '__main__':
    main()
