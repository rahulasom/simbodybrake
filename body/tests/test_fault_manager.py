"""
Tests for Fault Manager: pressure discrepancy debouncing, MCU faults,
degraded warning ingestion, and fault latching/clearing.
"""

from body.fault_manager import FaultManager
from body.protocol import (
    VehicleState,
    FaultCode,
    DegradedMode,
    BRKFaultStatusMsg,
    BRKWarningStatusMsg,
)


def test_pressure_discrepancy_debouncing():
    fm = FaultManager()

    # Command 50 psi (50%), feedback 15.0 psi -> Delta = 35.0 psi (> 10 psi threshold)
    t0 = 100.0
    res = fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0)
    assert res is None
    assert fm.is_fault_latched is False

    # After 200 ms: still within 300 ms debounce window
    res = fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0 + 0.200)
    assert res is None
    assert fm.is_fault_latched is False

    # After 350 ms: persistence threshold exceeded -> Critical fault triggered
    res = fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0 + 0.350)
    assert res is not None
    assert fm.is_fault_latched is True
    assert fm.fault_status == "PRESSURE_LOW"
    assert "Commanded: 50.0 psi, Actual: 15.0 psi" in fm.fault_description


def test_pressure_discrepancy_recovery_before_timeout():
    fm = FaultManager()
    t0 = 100.0

    # Delta > 10 psi starts at t0
    fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0)

    # At 250 ms, actuator catches up: pressure rises to 46.0 psi (delta = 4.0 <= 10.0)
    fm.evaluate_pressure_tracking(VehicleState.READY, 50, 46.0, current_time=t0 + 0.250)

    # At 400 ms: no fault triggered because debounce filter was reset
    res = fm.evaluate_pressure_tracking(VehicleState.READY, 50, 46.0, current_time=t0 + 0.400)
    assert res is None
    assert fm.is_fault_latched is False


def test_mcu_fault_ingestion_and_clear():
    fm = FaultManager()

    # Normal initially
    assert fm.is_fault_latched is False

    # Ingest SENSOR_FAULT
    fault_msg = BRKFaultStatusMsg(
        fault_active=True,
        fault_code=FaultCode.SENSOR_FAULT,
        sub_error_code=0x11,
    )
    fm.ingest_mcu_fault_message(fault_msg)
    assert fm.is_fault_latched is True
    assert fm.fault_status == "MCU_FAULT"

    # Cannot clear while MCU still asserts fault
    ok, reason = fm.acknowledge_and_clear()
    assert ok is False
    assert "still asserting active fault" in reason

    # MCU fault clears on bus
    fm.ingest_mcu_fault_message(BRKFaultStatusMsg(fault_active=False))
    assert fm.mcu_fault_active is False
    # But UI fault remains latched until user acknowledges!
    assert fm.is_fault_latched is True

    # Operator acknowledges
    ok, reason = fm.acknowledge_and_clear()
    assert ok is True
    assert fm.is_fault_latched is False
    assert fm.fault_status == "NORMAL"


def test_warning_status_ingestion():
    fm = FaultManager()

    warn_msg = BRKWarningStatusMsg(
        warning_active=True,
        degraded_mode=DegradedMode.REDUCED_CAPACITY,
        max_available_pressure=80.0,
    )
    w_active, mode, max_p = fm.ingest_warning_message(warn_msg)
    assert w_active is True
    assert mode == DegradedMode.REDUCED_CAPACITY
    assert max_p == 80.0

    # Warning clears
    clear_msg = BRKWarningStatusMsg(
        warning_active=False,
        degraded_mode=DegradedMode.NONE,
        max_available_pressure=100.0,
    )
    w_active, mode, max_p = fm.ingest_warning_message(clear_msg)
    assert w_active is False
    assert mode == DegradedMode.NONE
    assert max_p == 100.0


def test_fault_reset_and_active_property():
    fm = FaultManager()
    t0 = 100.0
    fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0)
    fm.evaluate_pressure_tracking(VehicleState.READY, 50, 15.0, current_time=t0 + 0.35)
    assert fm.is_brake_fault_active is True
    assert fm.is_fault_latched is True

    # Call reset() (REQ-BC-FLT-005)
    fm.reset()
    assert fm.is_brake_fault_active is False
    assert fm.is_fault_latched is False
    assert fm.fault_status == "NORMAL"
