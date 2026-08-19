#!/usr/bin/env python3
"""
SLAMTEC RPLIDAR S2 serial protocol driver.

Deliberately ROS-free. This module knows about bytes on a wire and nothing
else, which means it can be unit-tested without a ROS environment, reused in
a plain Python tool, and reasoned about independently of the node wrapping it.

WHY THIS EXISTS RATHER THAN USING rplidar_ros:
    The Debian-packaged rplidar_ros (SDK 1.12.0) segfaults on this device
    inside RPlidarDriverImplCommon::startMotor(), dereferencing an invalid
    mutex pointer (gdb reported mutex=0x1). That is the legacy
    rp::standalone::rplidar API, which predates S2 support -- the S2 is meant
    to be driven through SLAMTEC's newer sl:: API. Rather than patch and
    rebuild a vendor SDK on-device, this implements the wire protocol directly.

PROTOCOL SUMMARY:
    Host -> device: two-byte commands 0xA5 <cmd>, or
                    0xA5 <cmd> <len> <payload> <checksum> when carrying data.

    Device -> host: a 7-byte response descriptor
        [0xA5][0x5A][len:30 bits | mode:2 bits, LE u32][data_type]
    followed by the payload.

    Scan data arrives as an unbroken stream of 5-byte nodes with no
    per-node descriptor -- see _parse_nodes() for the bit layout and the
    resync rule.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Iterator, Optional

import serial

# --- Framing constants -----------------------------------------------------
SYNC_BYTE = 0xA5
SYNC_BYTE2 = 0x5A

# --- Commands --------------------------------------------------------------
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

# --- Response data types ---------------------------------------------------
DTYPE_INFO = 0x04
DTYPE_HEALTH = 0x06
DTYPE_SCAN = 0x81

log = logging.getLogger("s2_protocol")


@dataclass
class ScanPoint:
    """One measurement sample.

    angle_deg    0-360, increasing clockwise viewed from above
    distance_mm  0 means "no return" (out of range, or absorbed)
    quality      signal strength, 0-63
    start_flag   True on the first sample of a new revolution
    """
    angle_deg: float
    distance_mm: float
    quality: int
    start_flag: bool


class RPLidarError(RuntimeError):
    """Protocol-level failure: bad sync, timeout, or short read."""


class RPLidarS2:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 1.0):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._scanning = False

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    def open(self) -> None:
        log.info("opening %s @ %d baud", self.port_name, self.baudrate)
        self._ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            # All flow control off. The S2 uses none, and leaving DTR under
            # our own control matters -- see set_motor() below.
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        # CRITICAL: the device may still be mid-scan from a previous session
        # that exited without stopping it. In that state it streams binary
        # continuously and ignores GET_INFO, which looks identical to a dead
        # or misconfigured device. Stop first, then flush, then talk.
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        try:
            self._send_cmd(CMD_STOP)
            time.sleep(0.05)
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            if self._scanning:
                self.stop()
            self.set_motor(False)
        except Exception:
            pass
        finally:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def __enter__(self) -> "RPLidarS2":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Framing
    # ------------------------------------------------------------------

    def _send_cmd(self, cmd: int, payload: Optional[bytes] = None) -> None:
        if self._ser is None:
            raise RPLidarError("port not open")
        if payload:
            frame = bytes([SYNC_BYTE, cmd, len(payload) & 0xFF]) + payload
            # Checksum is XOR across every byte of the frame including sync.
            checksum = 0
            for b in frame:
                checksum ^= b
            frame += bytes([checksum & 0xFF])
        else:
            frame = bytes([SYNC_BYTE, cmd])
        self._ser.write(frame)
        self._ser.flush()

    def _read_descriptor(self) -> tuple:
        """Read a 7-byte response descriptor -> (payload_len, mode, data_type)."""
        if self._ser is None:
            raise RPLidarError("port not open")
        header = self._ser.read(7)
        if len(header) != 7:
            raise RPLidarError("descriptor timeout: got %d/7 bytes" % len(header))
        if header[0] != SYNC_BYTE or header[1] != SYNC_BYTE2:
            raise RPLidarError(
                "bad descriptor sync: %#04x %#04x" % (header[0], header[1]))
        raw = struct.unpack("<I", header[2:6])[0]
        return (raw & 0x3FFFFFFF), ((raw >> 30) & 0x03), header[6]

    def _read_exact(self, n: int) -> bytes:
        if self._ser is None:
            raise RPLidarError("port not open")
        buf = bytearray()
        deadline = time.monotonic() + max(self.timeout, 1.0)
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            elif time.monotonic() > deadline:
                raise RPLidarError("short read: %d/%d" % (len(buf), n))
        return bytes(buf)

    # ------------------------------------------------------------------
    # Device interrogation
    # ------------------------------------------------------------------

    def get_info(self) -> dict:
        self._send_cmd(CMD_GET_INFO)
        length, _mode, dtype = self._read_descriptor()
        if dtype != DTYPE_INFO or length != 20:
            raise RPLidarError(
                "unexpected info descriptor: type=%#x len=%d" % (dtype, length))
        d = self._read_exact(20)
        return {
            "model": d[0],
            "firmware_minor": d[1],
            "firmware_major": d[2],
            "hardware": d[3],
            "serial": d[4:20].hex().upper(),
        }

    def get_health(self) -> dict:
        self._send_cmd(CMD_GET_HEALTH)
        length, _mode, dtype = self._read_descriptor()
        if dtype != DTYPE_HEALTH or length != 3:
            raise RPLidarError(
                "unexpected health descriptor: type=%#x len=%d" % (dtype, length))
        d = self._read_exact(3)
        code = d[0]
        return {
            "status": {0: "OK", 1: "WARNING", 2: "ERROR"}.get(code, "UNKNOWN(%d)" % code),
            "code": code,
            "error_code": struct.unpack("<H", d[1:3])[0],
        }

    # ------------------------------------------------------------------
    # Motor control
    # ------------------------------------------------------------------

    def set_motor(self, on: bool) -> None:
        """Enable or disable the spin motor.

        On the CP2102-based S2 adapter the motor brake is gated by DTR:
        DTR LOW releases the brake and lets the rotor spin. This is inverted
        relative to intuition, and it is what the vendor SDK does internally.

        The packaged rplidar_ros driver crashes inside startMotor() on this
        device; driving DTR directly sidesteps that code path entirely.
        """
        if self._ser is None:
            raise RPLidarError("port not open")
        self._ser.dtr = not on

    # ------------------------------------------------------------------
    # Scanning
    # ------------------------------------------------------------------

    def stop(self) -> None:
        if self._ser is None:
            return
        self._send_cmd(CMD_STOP)
        time.sleep(0.05)
        self._scanning = False
        self._ser.reset_input_buffer()

    def reset(self) -> None:
        if self._ser is None:
            return
        self._send_cmd(CMD_RESET)
        time.sleep(0.8)  # boot banner + firmware reinit
        self._ser.reset_input_buffer()
        self._scanning = False

    def start_scan(self) -> None:
        """Issue SCAN and consume the response descriptor.

        After this returns, the port carries an unbroken stream of 5-byte
        measurement nodes until stop() is called.
        """
        self._send_cmd(CMD_SCAN)
        length, mode, dtype = self._read_descriptor()
        if dtype != DTYPE_SCAN or length != 5 or mode != 1:
            raise RPLidarError(
                "unexpected scan descriptor: type=%#x len=%d mode=%d"
                % (dtype, length, mode))
        self._scanning = True

    def _parse_nodes(self, buf: bytearray):
        """Consume whole 5-byte nodes from buf, yielding ScanPoints.

        Node layout across 5 bytes:
            byte0: [quality:6][!S:1][S:1]      S = start-of-revolution flag
            byte1: [angle_q6 low 7 bits][C:1]  C = check bit, always 1
            byte2: [angle_q6 high 8 bits]
            byte3: distance_q2 low byte
            byte4: distance_q2 high byte

        Two redundant integrity bits are built into the format: S and !S must
        differ, and C must be 1. Both failing means we are misaligned.

        RESYNC RULE -- the detail most implementations get wrong: on a framing
        error we discard exactly ONE byte, not five. Nodes are 5 bytes wide,
        but corruption can begin at any byte offset. Dropping 5 preserves the
        misalignment forever; dropping 1 lets the parser walk back into phase
        within at most 4 iterations.
        """
        while len(buf) >= 5:
            b0, b1 = buf[0], buf[1]
            s = b0 & 0x01
            ns = (b0 >> 1) & 0x01
            check = b1 & 0x01

            if s == ns or check != 1:
                del buf[0]
                continue

            quality = b0 >> 2
            angle_q6 = (buf[2] << 7) | (b1 >> 1)
            dist_q2 = (buf[4] << 8) | buf[3]

            del buf[0:5]
            yield ScanPoint(
                angle_deg=angle_q6 / 64.0,
                distance_mm=dist_q2 / 4.0,
                quality=quality,
                start_flag=bool(s),
            )

    def iter_scans(self):
        """Yield one complete revolution (list of ScanPoint) at a time.

        Revolution boundaries come from the device's own start_flag rather
        than from angle wraparound, which is more robust when samples are
        dropped near 0 degrees.
        """
        if self._ser is None:
            raise RPLidarError("port not open")

        buf = bytearray()
        current = []

        while self._scanning:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting if waiting else 1)
            if not chunk:
                continue
            buf.extend(chunk)

            for pt in self._parse_nodes(buf):
                if pt.start_flag and current:
                    yield current
                    current = []
                current.append(pt)
