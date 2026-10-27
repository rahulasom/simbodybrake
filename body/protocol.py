"""
Protocol definitions, Enums, DBC integration, and CAN message bit-level encoding/decoding.
Fully compliant with body.md, icd.md, and DBC definitions using cantools.
"""

from __future__ import annotations
import struct
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Optional, Dict, Any

import cantools
from .config import DBC_DIR


class VehicleState(IntEnum):
    OFF = 0
    STARTUP = 1
    READY = 2
    SHUTDOWN = 3

    @classmethod
    def from_raw(cls, value: int) -> VehicleState:
        try:
            return cls(value)
        except ValueError:
            return cls.OFF


class BrakeMachineState(IntEnum):
    WAKEUP = 0
    CHECKS = 1
    READY = 2
    BRAKING = 3
    FAULT = 4

    @classmethod
    def from_raw(cls, value: int) -> BrakeMachineState:
        try:
            return cls(value)
        except ValueError:
            return cls.WAKEUP


class FaultCode(IntEnum):
    NO_FAULT = 0x00
    SENSOR_FAULT = 0x01
    ACTUATOR_FAULT = 0x02
    COMMS_TIMEOUT = 0x03
    SELFTEST_FAIL = 0x04
    UNDERPRESSURE_FAULT = 0x05
    OVERPRESSURE_FAULT = 0x06

    @classmethod
    def from_raw(cls, value: int) -> FaultCode:
        try:
            return cls(value)
        except ValueError:
            return cls.NO_FAULT


class DegradedMode(IntEnum):
    NONE = 0x0
    REDUCED_CAPACITY = 0x1
    OVERHEATED_DERATING = 0x2
    SLOW_RESPONSE = 0x3
    BACKUP_CIRCUIT = 0x4

    @classmethod
    def from_raw(cls, value: int) -> DegradedMode:
        try:
            return cls(value)
        except ValueError:
            return cls.NONE


class DbcManager:
    """Loads and holds cantools DBC databases for state and brake frames."""

    def __init__(self, dbc_dir: Optional[Path] = None):
        target_dir = Path(dbc_dir) if dbc_dir else DBC_DIR
        self.state_dbc_path = target_dir / "vehicle_state.dbc"
        self.brake_dbc_path = target_dir / "vehicle_brake.dbc"

        self.db_state = cantools.database.load_file(str(self.state_dbc_path))
        self.db_brake = cantools.database.load_file(str(self.brake_dbc_path))

    def decode_message(self, arbitration_id: int, data: bytes) -> Dict[str, Any]:
        """Decodes raw payload using cantools DBC."""
        if arbitration_id in (0x100, 0x101):
            return self.db_state.decode_message(arbitration_id, data)
        elif arbitration_id in (0x200, 0x201, 0x202):
            return self.db_brake.decode_message(arbitration_id, data)
        raise KeyError(f"Unknown CAN ID 0x{arbitration_id:X}")


# Global DBC manager instance
_dbc_manager: Optional[DbcManager] = None


def get_dbc_manager() -> DbcManager:
    global _dbc_manager
    if _dbc_manager is None:
        _dbc_manager = DbcManager()
    return _dbc_manager


@dataclass
class BCVehicleStateMsg:
    """
    Message ID 0x100 (256): BC_VehicleState
    DLC: 3
    Byte 0: VehicleState (bits 0-3), FaultState (bit 4), Reserved (bits 5-7)
    Byte 1: AliveCounter (bits 0-7)
    Byte 2: AmbientTemperature (bits 0-7, signed int8)
    """
    vehicle_state: VehicleState = VehicleState.OFF
    fault_state: bool = False
    alive_counter: int = 0
    ambient_temperature: int = 25

    def encode(self) -> bytes:
        b0 = (int(self.vehicle_state) & 0x0F) | ((1 if self.fault_state else 0) << 4)
        b1 = self.alive_counter & 0xFF
        b2 = struct.pack("b", max(-40, min(85, int(self.ambient_temperature))))[0]
        return bytes([b0, b1, b2])

    @classmethod
    def decode(cls, data: bytes) -> BCVehicleStateMsg:
        if len(data) < 3:
            raise ValueError(f"BCVehicleState requires at least 3 bytes, got {len(data)}")
        state = VehicleState.from_raw(data[0] & 0x0F)
        fault = bool((data[0] >> 4) & 0x01)
        alive = data[1]
        temp = struct.unpack("b", bytes([data[2]]))[0]
        return cls(vehicle_state=state, fault_state=fault, alive_counter=alive, ambient_temperature=temp)


@dataclass
class BCBrakeCommandMsg:
    """
    Message ID 0x101 (257): BC_BrakeCommand
    DLC: 3
    Byte 0: BrakeCommandPercent (0..100)
    Byte 1: BrakeEnable (bit 0), CmdReserved (bits 1-7)
    Byte 2: RollingCounter (0..255)
    """
    brake_command_percent: int = 0
    brake_enable: bool = False
    rolling_counter: int = 0

    def encode(self) -> bytes:
        b0 = max(0, min(100, int(self.brake_command_percent))) & 0xFF
        b1 = 1 if self.brake_enable else 0
        b2 = self.rolling_counter & 0xFF
        return bytes([b0, b1, b2])

    @classmethod
    def decode(cls, data: bytes) -> BCBrakeCommandMsg:
        if len(data) < 3:
            raise ValueError(f"BCBrakeCommand requires at least 3 bytes, got {len(data)}")
        cmd = min(100, data[0])
        enable = bool(data[1] & 0x01)
        counter = data[2]
        return cls(brake_command_percent=cmd, brake_enable=enable, rolling_counter=counter)


@dataclass
class BRKStatusFeedbackMsg:
    """
    Message ID 0x200 (512): BRK_StatusFeedback
    DLC: 4
    Byte 0: BrakeMachineState (bits 0-3), Reserved (bits 4-7)
    Byte 1-2: ActualBrakePressure (16-bit uint, Intel Little-Endian, 0.1 psi/bit)
    Byte 3: BrakeActiveFlag (bit 0), Reserved (bits 1-7)
    """
    brake_machine_state: BrakeMachineState = BrakeMachineState.WAKEUP
    actual_brake_pressure: float = 0.0
    brake_active_flag: bool = False

    def encode(self) -> bytes:
        b0 = int(self.brake_machine_state) & 0x0F
        raw_p = max(0, min(1200, int(round(self.actual_brake_pressure * 10.0))))
        p_bytes = struct.pack("<H", raw_p)
        b3 = 1 if self.brake_active_flag else 0
        return bytes([b0, p_bytes[0], p_bytes[1], b3])

    @classmethod
    def decode(cls, data: bytes) -> BRKStatusFeedbackMsg:
        if len(data) < 4:
            raise ValueError(f"BRKStatusFeedback requires at least 4 bytes, got {len(data)}")
        state = BrakeMachineState.from_raw(data[0] & 0x0F)
        raw_p = struct.unpack("<H", data[1:3])[0]
        pressure = round(raw_p * 0.1, 1)
        active = bool(data[3] & 0x01)
        return cls(brake_machine_state=state, actual_brake_pressure=pressure, brake_active_flag=active)


@dataclass
class BRKFaultStatusMsg:
    """
    Message ID 0x201 (513): BRK_FaultStatus
    DLC: 3
    Byte 0: FaultActive (bit 0), Reserved (bits 1-7)
    Byte 1: FaultCode (0..255)
    Byte 2: SubErrorCode (0..255)
    """
    fault_active: bool = False
    fault_code: FaultCode = FaultCode.NO_FAULT
    sub_error_code: int = 0

    def encode(self) -> bytes:
        b0 = 1 if self.fault_active else 0
        b1 = int(self.fault_code) & 0xFF
        b2 = self.sub_error_code & 0xFF
        return bytes([b0, b1, b2])

    @classmethod
    def decode(cls, data: bytes) -> BRKFaultStatusMsg:
        if len(data) < 3:
            raise ValueError(f"BRKFaultStatus requires at least 3 bytes, got {len(data)}")
        active = bool(data[0] & 0x01)
        code = FaultCode.from_raw(data[1])
        sub = data[2]
        return cls(fault_active=active, fault_code=code, sub_error_code=sub)


@dataclass
class BRKWarningStatusMsg:
    """
    Message ID 0x202 (514): BRK_WarningStatus
    DLC: 3
    Byte 0: WarningActive (bit 0), DegradedMode (bits 1-4), Reserved (bits 5-7)
    Byte 1-2: MaxAvailablePressure (16-bit uint, Intel Little-Endian, 0.1 psi/bit)
    """
    warning_active: bool = False
    degraded_mode: DegradedMode = DegradedMode.NONE
    max_available_pressure: float = 100.0

    def encode(self) -> bytes:
        b0 = (1 if self.warning_active else 0) | ((int(self.degraded_mode) & 0x0F) << 1)
        raw_p = max(0, min(1200, int(round(self.max_available_pressure * 10.0))))
        p_bytes = struct.pack("<H", raw_p)
        return bytes([b0, p_bytes[0], p_bytes[1]])

    @classmethod
    def decode(cls, data: bytes) -> BRKWarningStatusMsg:
        if len(data) < 3:
            raise ValueError(f"BRKWarningStatus requires at least 3 bytes, got {len(data)}")
        active = bool(data[0] & 0x01)
        mode = DegradedMode.from_raw((data[0] >> 1) & 0x0F)
        raw_p = struct.unpack("<H", data[1:3])[0]
        max_p = round(raw_p * 0.1, 1)
        return cls(warning_active=active, degraded_mode=mode, max_available_pressure=max_p)
