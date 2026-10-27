"""
Tests for protocol serialization, deserialization, and DBC compatibility.
"""

import math
import cantools

from body.protocol import (
    VehicleState,
    BrakeMachineState,
    FaultCode,
    DegradedMode,
    BCVehicleStateMsg,
    BCBrakeCommandMsg,
    BRKStatusFeedbackMsg,
    BRKFaultStatusMsg,
    BRKWarningStatusMsg,
    get_dbc_manager,
)


def test_vehicle_state_msg_roundtrip():
    msg = BCVehicleStateMsg(
        vehicle_state=VehicleState.READY,
        fault_state=True,
        alive_counter=128,
        ambient_temperature=-15,
    )
    raw = msg.encode()
    assert len(raw) == 3

    decoded = BCVehicleStateMsg.decode(raw)
    assert decoded.vehicle_state == VehicleState.READY
    assert decoded.fault_state is True
    assert decoded.alive_counter == 128
    assert decoded.ambient_temperature == -15


def test_vehicle_state_dbc_parity():
    dbc_mgr = get_dbc_manager()
    msg = BCVehicleStateMsg(
        vehicle_state=VehicleState.STARTUP,
        fault_state=True,
        alive_counter=55,
        ambient_temperature=32,
    )
    raw = msg.encode()
    dbc_decoded = dbc_mgr.decode_message(0x100, raw)
    assert dbc_decoded["VehicleState"] == "STARTUP"
    assert dbc_decoded["FaultState"] == "FaultActive"
    assert dbc_decoded["BC_AliveCounter"] == 55
    assert dbc_decoded["AmbientTemperature"] == 32


def test_brake_command_msg_roundtrip():
    msg = BCBrakeCommandMsg(
        brake_command_percent=75,
        brake_enable=True,
        rolling_counter=200,
    )
    raw = msg.encode()
    assert len(raw) == 3

    decoded = BCBrakeCommandMsg.decode(raw)
    assert decoded.brake_command_percent == 75
    assert decoded.brake_enable is True
    assert decoded.rolling_counter == 200


def test_brake_command_clamping():
    msg = BCBrakeCommandMsg(brake_command_percent=150)
    raw = msg.encode()
    decoded = BCBrakeCommandMsg.decode(raw)
    assert decoded.brake_command_percent == 100


def test_brake_command_dbc_parity():
    dbc_mgr = get_dbc_manager()
    msg = BCBrakeCommandMsg(
        brake_command_percent=45,
        brake_enable=True,
        rolling_counter=77,
    )
    raw = msg.encode()
    dbc_decoded = dbc_mgr.decode_message(0x101, raw)
    assert dbc_decoded["BrakeCommandPercent"] == 45
    assert dbc_decoded["BrakeEnable"] == "Enabled"
    assert dbc_decoded["BC_CmdRollingCounter"] == 77


def test_status_feedback_msg_roundtrip():
    msg = BRKStatusFeedbackMsg(
        brake_machine_state=BrakeMachineState.BRAKING,
        actual_brake_pressure=45.2,
        brake_active_flag=True,
    )
    raw = msg.encode()
    assert len(raw) == 4

    decoded = BRKStatusFeedbackMsg.decode(raw)
    assert decoded.brake_machine_state == BrakeMachineState.BRAKING
    assert abs(decoded.actual_brake_pressure - 45.2) < 0.1
    assert decoded.brake_active_flag is True


def test_status_feedback_dbc_parity():
    dbc_mgr = get_dbc_manager()
    msg = BRKStatusFeedbackMsg(
        brake_machine_state=BrakeMachineState.READY,
        actual_brake_pressure=0.0,
        brake_active_flag=False,
    )
    raw = msg.encode()
    dbc_decoded = dbc_mgr.decode_message(0x200, raw)
    assert dbc_decoded["BrakeMachineState"] == "ready"
    assert dbc_decoded["ActualBrakePressure"] == 0.0
    assert dbc_decoded["BrakeActiveFlag"] == "Inactive"


def test_fault_status_msg_roundtrip():
    msg = BRKFaultStatusMsg(
        fault_active=True,
        fault_code=FaultCode.SENSOR_FAULT,
        sub_error_code=0x2A,
    )
    raw = msg.encode()
    assert len(raw) == 3

    decoded = BRKFaultStatusMsg.decode(raw)
    assert decoded.fault_active is True
    assert decoded.fault_code == FaultCode.SENSOR_FAULT
    assert decoded.sub_error_code == 0x2A


def test_fault_status_dbc_parity():
    dbc_mgr = get_dbc_manager()
    msg = BRKFaultStatusMsg(
        fault_active=True,
        fault_code=FaultCode.UNDERPRESSURE_FAULT,
        sub_error_code=0x05,
    )
    raw = msg.encode()
    dbc_decoded = dbc_mgr.decode_message(0x201, raw)
    assert dbc_decoded["FaultActive"] == "FaultPresent"
    assert dbc_decoded["FaultCode"] == "UNDERPRESSURE"
    assert dbc_decoded["SubErrorCode"] == 0x05


def test_warning_status_msg_roundtrip():
    msg = BRKWarningStatusMsg(
        warning_active=True,
        degraded_mode=DegradedMode.REDUCED_CAPACITY,
        max_available_pressure=80.0,
    )
    raw = msg.encode()
    assert len(raw) == 3

    decoded = BRKWarningStatusMsg.decode(raw)
    assert decoded.warning_active is True
    assert decoded.degraded_mode == DegradedMode.REDUCED_CAPACITY
    assert abs(decoded.max_available_pressure - 80.0) < 0.1


def test_warning_status_dbc_parity():
    dbc_mgr = get_dbc_manager()
    msg = BRKWarningStatusMsg(
        warning_active=True,
        degraded_mode=DegradedMode.OVERHEATED_DERATING,
        max_available_pressure=65.0,
    )
    raw = msg.encode()
    dbc_decoded = dbc_mgr.decode_message(0x202, raw)
    assert dbc_decoded["WarningActive"] == "WarningActive"
    assert dbc_decoded["DegradedMode"] == "OVERHEATED_DERATING"
    assert abs(dbc_decoded["MaxAvailablePressure"] - 65.0) < 0.1
