"""
connection.py — EvilCrow RF v2 serial connection management.

Single source of truth for opening and initializing the serial link.
All EvilCrow tools (evilcrow_sdr.py, urh_bridge.py, gnuradio_source.py)
use this module instead of implementing their own connection logic.

Handles:
    - Port open without triggering an ESP32 reset (DTR/RTS suppression).
    - Boot drain when a reset does occur (macOS CH340/CP2102 chips).
    - Firmware parser flush (stale 0xAA bytes from boot noise).
    - Command send/receive with proper framing and sentinel detection.
"""

from __future__ import annotations

import time
from typing import List, Optional

import serial


class EvilCrowConnection:
    """Managed serial connection to an EvilCrow RF v2 device."""

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout: float = 2.0, debug: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug = debug
        self.ser: Optional[serial.Serial] = None

    # ── Context manager ───────────────────────────────────────────

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ── Lifecycle ─────────────────────────────────────────────────

    def open(self):
        """Open the serial port and prepare the firmware for commands."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            dsrdtr=False,
            rtscts=False,
        )
        # Suppress DTR/RTS to avoid resetting the ESP32.  On some
        # macOS USB-serial chips (CH340, CP2102) the reset happens
        # regardless, so we also do a full boot drain.
        self.ser.dtr = False
        self.ser.rts = False

        self._wait_for_boot()
        self._flush_parser()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    # ── Command interface ─────────────────────────────────────────

    def send_command(self, command: str, timeout: Optional[float] = None) -> str:
        """Send a text command and return the response.

        Drains stale RX data first, sends ``command + '\\n'``, then reads
        lines until a HACKRF_SUCCESS / HACKRF_ERROR sentinel is seen or
        the timeout expires.

        Returns:
            Multi-line response string (newline-joined).
        """
        if not self.is_open:
            raise RuntimeError('Serial port not open')

        timeout = timeout or self.timeout

        # Drain stale data from a previous multi-line response
        if self.ser.in_waiting > 0:
            stale = self.ser.read(self.ser.in_waiting)
            if self.debug:
                print(f'[DBG] drained {len(stale)} stale bytes')
            time.sleep(0.02)

        cmd_bytes = (command + '\n').encode('ascii')
        if self.debug:
            print(f'[DBG] TX: {cmd_bytes!r}')
        self.ser.write(cmd_bytes)
        self.ser.flush()
        time.sleep(0.05)

        lines: List[str] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ser.in_waiting > 0:
                raw = self.ser.readline()
                if self.debug:
                    print(f'[DBG] RX: {raw!r}')
                line = raw.decode('ascii', errors='replace').strip()
                if line:
                    lines.append(line)
                if 'SUCCESS' in line or 'ERROR' in line:
                    break
            else:
                time.sleep(0.01)

        result = '\n'.join(lines)
        if self.debug and not lines:
            print(f'[DBG] send_command("{command}") timed out')
        return result

    def send_raw(self, data: bytes, read_timeout: float = 0.3) -> bytes:
        """Send raw bytes and return whatever comes back."""
        if not self.is_open:
            raise RuntimeError('Serial port not open')
        self.ser.write(data)
        self.ser.flush()
        time.sleep(read_timeout)
        resp = b''
        while self.ser.in_waiting:
            resp += self.ser.read(self.ser.in_waiting)
        return resp

    # ── SDR mode helpers ──────────────────────────────────────────

    def enable_sdr(self) -> bool:
        """Enable SDR mode on the device.  Returns True on success."""
        resp = self.send_command('sdr_enable')
        return 'SUCCESS' in resp.upper()

    def verify_device(self) -> bool:
        """Verify the device identifies as EvilCrow SDR."""
        resp = self.send_command('board_id_read')
        return 'HACKRF' in resp.upper()

    # ── Internal ──────────────────────────────────────────────────

    def _wait_for_boot(self, quiet_period: float = 0.5,
                       max_wait: float = 8.0):
        """Drain boot output until the serial line is quiet.

        Reads and discards all incoming data.  Returns once no new data
        has arrived for ``quiet_period`` seconds, or after ``max_wait``
        seconds (whichever comes first).
        """
        if self.debug:
            print(f'[DBG] waiting for boot output to settle...')
        deadline = time.time() + max_wait
        last_rx = time.time()

        while time.time() < deadline:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                if self.debug:
                    snippet = data[-60:] if len(data) > 60 else data
                    print(f'[DBG] boot drain: {len(data)} bytes …{snippet!r}')
                last_rx = time.time()
            else:
                if time.time() - last_rx >= quiet_period:
                    if self.debug:
                        elapsed = time.time() - (deadline - max_wait)
                        print(f'[DBG] boot settled after {elapsed:.1f}s')
                    return
                time.sleep(0.05)

    def _flush_parser(self):
        """Flush the firmware's serial parser state.

        During boot, UART noise can leave stale bytes (including 0xAA)
        in the firmware's command buffer, putting the parser into binary
        frame mode.  Sending 600 newlines forces a buffer overflow
        (512-byte buffer) that resets the index to 0, followed by
        harmless empty lines that produce no output.
        """
        if self.debug:
            print('[DBG] flushing firmware parser...')
        self.ser.write(b'\n' * 600)
        self.ser.flush()
        time.sleep(0.3)
        if self.ser.in_waiting > 0:
            stale = self.ser.read(self.ser.in_waiting)
            if self.debug:
                print(f'[DBG] parser flush drained {len(stale)} bytes')
        self.ser.reset_input_buffer()
