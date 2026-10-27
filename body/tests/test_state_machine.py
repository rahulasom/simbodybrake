"""
Tests for Vehicle State Machine transitions, timeouts, and interlocks.
"""

from body.fault_manager import FaultManager
from body.protocol import VehicleState, BrakeMachineState, BRKStatusFeedbackMsg
from body.state_machine import VehicleStateMachine


def test_initial_state_and_interlocks():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    assert sm.state == VehicleState.OFF
    # Brake command prohibited in OFF
    assert sm.set_brake_command(50) is False
    assert sm.commanded_percent == 0

    cmd_msg = sm.build_brake_command_msg()
    assert cmd_msg.brake_command_percent == 0
    assert cmd_msg.brake_enable is False


def test_startup_handshake_success():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    # Power On
    assert sm.request_power_on(current_time=100.0) is True
    assert sm.state == VehicleState.STARTUP

    # Still in startup while brake MCU is in wakeup or checks
    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.WAKEUP),
        current_time=100.05,
    )
    assert sm.state == VehicleState.STARTUP

    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.CHECKS),
        current_time=100.25,
    )
    assert sm.state == VehicleState.STARTUP

    # MCU reports ready -> Handshake complete -> Transition to READY
    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.READY),
        current_time=100.30,
    )
    assert sm.state == VehicleState.READY


def test_startup_timeout_fault():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    # Power On at t=0
    sm.request_power_on(current_time=0.0)
    assert sm.state == VehicleState.STARTUP

    # Tick at 1.9s -> Still STARTUP
    sm.on_cyclic_tick(current_time=1.9)
    assert sm.state == VehicleState.STARTUP
    assert fault_mgr.is_fault_latched is False

    # Tick at 2.1s (> 2.0s timeout) -> Reverts to OFF, triggers fault
    sm.on_cyclic_tick(current_time=2.1)
    assert sm.state == VehicleState.OFF
    assert fault_mgr.is_fault_latched is True
    assert fault_mgr.fault_status == "STARTUP_TIMEOUT"


def test_ready_brake_control_and_shutdown():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    # Start and reach READY
    sm.request_power_on(current_time=1.0)
    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.READY),
        current_time=1.2,
    )
    assert sm.state == VehicleState.READY

    # Set brake command to 60%
    assert sm.set_brake_command(60) is True
    assert sm.commanded_percent == 60
    assert sm.brake_enable is True

    tx_cmd = sm.build_brake_command_msg()
    assert tx_cmd.brake_command_percent == 60
    assert tx_cmd.brake_enable is True

    # Request Shutdown at t=10.0
    assert sm.request_shutdown(current_time=10.0) is True
    assert sm.state == VehicleState.SHUTDOWN
    # Commands must be automatically clamped to 0%
    assert sm.commanded_percent == 0
    assert sm.brake_enable is False

    # Simulate 50ms cycles during shutdown
    for i in range(1, 10):
        sm.on_cyclic_tick(current_time=10.0 + (i * 0.05))
        assert sm.state == VehicleState.SHUTDOWN

    # 10th cycle (500 ms elapsed)
    sm.on_cyclic_tick(current_time=10.51)
    assert sm.state == VehicleState.OFF


def test_estop_immediate_reset():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    sm.request_power_on(current_time=1.0)
    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.READY),
        current_time=1.1,
    )
    sm.set_brake_command(80)
    assert sm.commanded_percent == 80

    # ESTOP
    sm.request_estop()
    assert sm.state == VehicleState.OFF
    assert sm.commanded_percent == 0
    assert sm.brake_enable is False


def test_fault_state_broadcast_and_reset_on_shutdown_and_off():
    fault_mgr = FaultManager()
    sm = VehicleStateMachine(fault_manager=fault_mgr)

    sm.request_power_on(current_time=1.0)
    sm.handle_brake_feedback(
        BRKStatusFeedbackMsg(brake_machine_state=BrakeMachineState.READY),
        current_time=1.1,
    )
    assert sm.state == VehicleState.READY
    assert sm.build_vehicle_state_msg().fault_state is False

    # Trigger pressure discrepancy fault
    fault_mgr.evaluate_pressure_tracking(VehicleState.READY, 50, 0.0, current_time=2.0)
    fault_mgr.evaluate_pressure_tracking(VehicleState.READY, 50, 0.0, current_time=2.35)
    assert fault_mgr.is_brake_fault_active is True
    assert sm.build_vehicle_state_msg().fault_state is True

    # Transition to SHUTDOWN resets fault detection and CAN FaultState
    sm.request_shutdown(current_time=3.0)
    assert sm.state == VehicleState.SHUTDOWN
    assert fault_mgr.is_brake_fault_active is False
    assert sm.build_vehicle_state_msg().fault_state is False

    # Power off
    sm.request_estop()
    assert sm.state == VehicleState.OFF
    assert sm.build_vehicle_state_msg().fault_state is False
