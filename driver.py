from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import serial  # type: ignore[import-not-found]


def _checksum8(data: Iterable[int]) -> int:
    """Return 8‑bit checksum used by SERVO42C (sum of bytes & 0xFF)."""

    s = 0
    for b in data:
        s = (s + (b & 0xFF)) & 0xFF
    return s


@dataclass
class MksServoBus:
    """Physical UART bus shared by one or more SERVO42C motors.

    Parameters
    ----------
    port:
        Serial port name, e.g. ``"/dev/ttyUSB0"``.
    baudrate:
        Baudrate configured on the motors (default 38400).
    timeout:
        Read timeout in seconds.
    """

    port: str
    baudrate: int = 38400
    timeout: float = 0.2
    debug: bool = False

    def __post_init__(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self) -> None:
        self._ser.close()

    # --- low level helpers -------------------------------------------------

    def _write(self, data: bytes) -> None:
        if self.debug:
            print("TX:", " ".join(f"{b:02X}" for b in data))
        self._ser.write(data)

    def _read_exact(self, n: int) -> bytes:
        buf = self._ser.read(n)
        if self.debug:
            print(f"RX({n}):", " ".join(f"{b:02X}" for b in buf), f"len={len(buf)}")
        if len(buf) != n:
            raise TimeoutError(f"Expected {n} bytes, got {len(buf)}")
        return buf

    # --- protocol helpers --------------------------------------------------

    def _build_downlink(self, addr: int, func: int, payload: bytes = b"") -> bytes:
        if not (0xE0 <= addr <= 0xE9):
            raise ValueError("addr out of range (0xE0‑0xE9)")
        frame_wo_crc = bytes([addr, func]) + payload
        crc = _checksum8(frame_wo_crc)
        return frame_wo_crc + bytes([crc])

    def _tx_rx(
        self,
        addr: int,
        func: int,
        payload: bytes = b"",
        reply_len: Optional[int] = None,
    ) -> bytes:
        frame = self._build_downlink(addr, func, payload)
        self._write(frame)
        # All replies start with addr; caller specifies total reply length when known.
        if reply_len is None:
            # For variable‑length replies (not really used in this lib yet)
            header = self._read_exact(1)
            return header + self._ser.read(self._ser.in_waiting)
        data = self._read_exact(reply_len)
        # basic checksum check if at least 2 bytes
        if reply_len >= 2:
            body, crc = data[:-1], data[-1]
            if _checksum8(body) != crc:
                raise ValueError("CRC mismatch in reply")
        return data

    # --- high level helpers ------------------------------------------------

    def motor(self, addr: int) -> "MksServoMotor":
        """Return helper object bound to a specific slave address."""

        return MksServoMotor(self, addr)


@dataclass
class MksServoMotor:
    """High‑level operations for a single SERVO42C motor.

    ``addr`` is the full slave address byte (0xE0‑0xE9).
    """

    bus: MksServoBus
    addr: int = 0xE0

    # --- configuration -----------------------------------------------------

    def set_mode_uart(self) -> None:
        """Ensure work mode is CR_UART (Mode = 2, command 0x82)."""

        # Downlink: addr 0x82 [mode] CRC, Uplink: addr [status] CRC
        self._expect_status(
            self.bus._tx_rx(self.addr, 0x82, bytes([0x02]), reply_len=3)
        )

    def set_enable(self, enabled: bool = True) -> None:
        """Enable/disable driver in CR_UART mode (command 0xF3)."""

        val = 0x01 if enabled else 0x00
        self._expect_status(self.bus._tx_rx(self.addr, 0xF3, bytes([val]), reply_len=3))

    def set_protect(self, enabled: bool) -> None:
        """Enable/disable locked-rotor protection ("Protect", command 0x88).

        When enabled, the driver will release the motor if a locked-rotor
        condition is detected.
        """

        val = 0x01 if enabled else 0x00
        self._expect_status(self.bus._tx_rx(self.addr, 0x88, bytes([val]), reply_len=3))

    def read_protect_status(self) -> int:
        """Return shaft protection state (command 0x3E).

        1 = protected (locked-rotor detected), 2 = not protected.
        """

        data = self.bus._tx_rx(self.addr, 0x3E, reply_len=3)
        if data[0] != self.addr:
            raise ValueError("Unexpected address in reply")
        return data[1]

    def clear_protect(self) -> None:
        """Release locked-rotor protection state (command 0x3D)."""

        self._expect_status(self.bus._tx_rx(self.addr, 0x3D, reply_len=3))

    def set_motor_type(self, is_1p8_degree: bool = True) -> None:
        """Set motor type ("MotType", command 0x81).

        ``True`` selects 1.8° (type=1), ``False`` selects 0.9° (type=0).
        """

        val = 0x01 if is_1p8_degree else 0x00
        self._expect_status(self.bus._tx_rx(self.addr, 0x81, bytes([val]), reply_len=3))

    def set_mstep(self, mstep: int) -> None:
        """Set subdivision ("MStep", command 0x84).

        ``mstep`` is written directly as 0-255; see manual for supported
        values. It must match your physical microstep configuration.
        """

        if not (0 <= mstep <= 0xFF):
            raise ValueError("mstep must be 0-255")
        self._expect_status(
            self.bus._tx_rx(self.addr, 0x84, bytes([mstep & 0xFF]), reply_len=3)
        )

    def set_en_polarity(self, mode: str) -> None:
        """Set EN pin behaviour ("En", command 0x85).

        ``mode`` is one of: "L" (active low), "H" (active high),
        "Hold" (always enabled).
        """

        mapping = {"L": 0x00, "H": 0x01, "Hold": 0x02}
        try:
            val = mapping[mode]
        except KeyError as exc:
            raise ValueError("mode must be 'L', 'H' or 'Hold'") from exc
        self._expect_status(self.bus._tx_rx(self.addr, 0x85, bytes([val]), reply_len=3))

    def set_auto_sleep(self, enabled: bool) -> None:
        """Enable/disable OLED auto-sleep ("AutoSDD", command 0x87)."""

        val = 0x01 if enabled else 0x00
        self._expect_status(self.bus._tx_rx(self.addr, 0x87, bytes([val]), reply_len=3))

    def set_mplyer(self, enabled: bool) -> None:
        """Enable/disable internal 256 subdivision ("MPlyer", command 0x89)."""

        val = 0x01 if enabled else 0x00
        self._expect_status(self.bus._tx_rx(self.addr, 0x89, bytes([val]), reply_len=3))

    def set_uart_baud(self, baud_index: int) -> None:
        """Set UART baud rate ("UartBaud", command 0x8A).

        ``baud_index`` is 1-6 corresponding to 9600, 19200, 25000, 38400,
        57600, 115200. This must match your host serial settings after
        power-cycle.
        """

        if not (1 <= baud_index <= 6):
            raise ValueError("baud_index must be 1-6 as per manual")
        self._expect_status(
            self.bus._tx_rx(self.addr, 0x8A, bytes([baud_index]), reply_len=3)
        )

    def set_uart_addr_offset(self, offset: int) -> None:
        """Set UART slave address offset ("UartAddr", command 0x8B).

        The real address becomes ``0xE0 + offset``. 0-9 are allowed.
        Note that subsequent communication must use the new address.
        """

        if not (0 <= offset <= 9):
            raise ValueError("offset must be 0-9")
        self._expect_status(
            self.bus._tx_rx(self.addr, 0x8B, bytes([offset]), reply_len=3)
        )

    def restore_defaults(self) -> None:
        """Restore default parameters ("Restore", command 0x3F).

        Requires power-cycle afterwards, and you must reconfigure UART.
        """

        self._expect_status(self.bus._tx_rx(self.addr, 0x3F, reply_len=3))

    # --- zero-mode helpers -------------------------------------------------

    def set_zero_mode(self, mode: str) -> None:
        """Configure zero-mode behaviour ("0_Mode", command 0x90).

        ``mode``: "Disable", "DirMode" or "NearMode".
        """

        mapping = {"Disable": 0x00, "DirMode": 0x01, "NearMode": 0x02}
        try:
            val = mapping[mode]
        except KeyError as exc:
            raise ValueError("mode must be 'Disable', 'DirMode' or 'NearMode'") from exc
        self._expect_status(self.bus._tx_rx(self.addr, 0x90, bytes([val]), reply_len=3))

    def zero_set_current_position(self) -> None:
        """Set current position as zero ("Set 0", command 0x91)."""

        self._expect_status(
            self.bus._tx_rx(self.addr, 0x91, bytes([0x00]), reply_len=3)
        )

    def zero_set_speed(self, speed_index: int) -> None:
        """Set zero-return speed ("0_Speed", command 0x92).

        0 is fastest, 4 is slowest.
        """

        if not (0 <= speed_index <= 4):
            raise ValueError("speed_index must be 0-4")
        self._expect_status(
            self.bus._tx_rx(self.addr, 0x92, bytes([speed_index]), reply_len=3)
        )

    def zero_set_dir(self, cw: bool = True) -> None:
        """Set zero-return direction ("0_Dir", command 0x93)."""

        val = 0x00 if cw else 0x01
        self._expect_status(self.bus._tx_rx(self.addr, 0x93, bytes([val]), reply_len=3))

    def zero_goto(self) -> None:
        """Trigger return to zero ("Goto 0", command 0x94)."""

        self._expect_status(
            self.bus._tx_rx(self.addr, 0x94, bytes([0x00]), reply_len=3)
        )

    # --- PID / ACC / torque -----------------------------------------------

    def set_pid(self, kp: int, ki: int, kd: int) -> None:
        """Set position PID gains (commands 0xA1/0xA2/0xA3).

        Values are 16-bit integers as in the manual.
        """

        for name, func, value in (("kp", 0xA1, kp), ("ki", 0xA2, ki), ("kd", 0xA3, kd)):
            if not (0 <= value <= 0xFFFF):
                raise ValueError(f"{name} must be 0-0xFFFF")
            payload = value.to_bytes(2, byteorder="big", signed=False)
            self._expect_status(self.bus._tx_rx(self.addr, func, payload, reply_len=3))

    def set_acc(self, acc: int) -> None:
        """Set acceleration parameter ("ACC", command 0xA4)."""

        if not (0 <= acc <= 0xFFFF):
            raise ValueError("acc must be 0-0xFFFF")
        payload = acc.to_bytes(2, byteorder="big", signed=False)
        self._expect_status(self.bus._tx_rx(self.addr, 0xA4, payload, reply_len=3))

    def set_max_torque(self, max_t: int) -> None:
        """Set maximum torque ("MaxT", command 0xA5).

        Manual limits this to 0-0x4B0.
        """

        if not (0 <= max_t <= 0x4B0):
            raise ValueError("max_t must be 0-0x4B0")
        payload = max_t.to_bytes(2, byteorder="big", signed=False)
        self._expect_status(self.bus._tx_rx(self.addr, 0xA5, payload, reply_len=3))

    # --- status / telemetry -----------------------------------------------

    def read_encoder(self) -> Tuple[int, int]:
        """Return ``(carry, value)`` from command 0x30.

        ``value`` is 0‑0xFFFF, carry is signed 32‑bit.

        Observed firmware reply (8 bytes total):
        addr, carry(4 bytes, signed), value(2 bytes, unsigned), crc
        """
        data = self.bus._tx_rx(self.addr, 0x30, reply_len=8)
        # _tx_rx has already verified CRC
        if data[0] != self.addr:
            raise ValueError(f"Unexpected address in reply: 0x{data[0]:02X}")
        carry = int.from_bytes(data[1:5], byteorder="big", signed=True)
        value = int.from_bytes(data[5:7], byteorder="big", signed=False)
        return carry, value

    def read_enable_state(self) -> int:
        """Return enable state from command 0x3A (1=enabled, 2=disabled)."""

        data = self.bus._tx_rx(self.addr, 0x3A, reply_len=3)
        if data[0] != self.addr:
            raise ValueError("Unexpected address in reply")
        return data[1]

    # --- motion commands ---------------------------------------------------

    def run_constant_speed(
        self, rpm: float, mstep: int = 16, direction_cw: bool = True
    ) -> None:
        """Run motor at (approximately) ``rpm`` in CR_UART mode.

        Uses command 0xF6 (VAL). ``mstep`` must match driver subdivision.
        Positive ``rpm`` means CW when ``direction_cw=True``.
        """

        if rpm < 0:
            rpm = -rpm
            direction_cw = not direction_cw
        if rpm == 0:
            self.stop()
            return

        # Vrpm = (Speed * 30000) / (Mstep * steps_per_rev)
        # Solve for Speed (7‑bit, 1‑127)
        steps_per_rev = 200  # assume 1.8° motor
        speed = int(round(rpm * mstep * steps_per_rev / 30000.0))
        if speed < 1:
            speed = 1
        if speed > 0x7F:
            speed = 0x7F

        val = speed & 0x7F
        if not direction_cw:
            val |= 0x80

        self._expect_status(self.bus._tx_rx(self.addr, 0xF6, bytes([val]), reply_len=3))

    def run_pulses(
        self, rpm: float, pulses: int, mstep: int = 16, direction_cw: bool = True
    ) -> None:
        """Run a fixed number of ``pulses`` at a given ``rpm`` (command 0xFD).

        This maps to a specific motion distance depending on motor step angle
        and micro‑step setting configured on the driver.
        """

        if rpm < 0:
            rpm = -rpm
            direction_cw = not direction_cw
        if rpm <= 0:
            raise ValueError("rpm must be non‑zero")
        if pulses <= 0:
            raise ValueError("pulses must be positive")

        steps_per_rev = 200
        speed = int(round(rpm * mstep * steps_per_rev / 30000.0))
        if speed < 1 or speed > 0x7F:
            raise ValueError("rpm outside supported range for given mstep")

        val = speed & 0x7F
        if not direction_cw:
            val |= 0x80

        payload = bytes([val]) + pulses.to_bytes(4, byteorder="big", signed=False)
        # First reply: status 1 (starting)
        self._expect_status(self.bus._tx_rx(self.addr, 0xFD, payload, reply_len=3))
        # Second reply: status 2 (complete) arrives asynchronously when done
        completion_packet = self.bus._read_exact(3)
        self._expect_status(completion_packet, expected=(2,))

    def stop(self) -> None:
        """Stop motor (command 0xF7)."""

        self._expect_status(self.bus._tx_rx(self.addr, 0xF7, reply_len=3))

    # --- internal helpers --------------------------------------------------

    @staticmethod
    def _expect_status(reply: bytes, expected: Tuple[int, ...] = (1,)) -> None:
        if len(reply) < 2:
            raise ValueError("Short reply from motor")
        status = reply[1]
        if status not in expected:
            raise RuntimeError(f"Servo reported status {status}, expected {expected}")
