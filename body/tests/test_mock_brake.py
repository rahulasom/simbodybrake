"""
Tests for embedded TM4C123 Mock Brake Controller emulation.
"""

import time
import pytest
import can

from body.config import (
    CAN_ID_BC_VEHICLE_STATE,
    CAN_ID_BC_BRAKE_COMMAND,
    CAN_ID_BRK_STATUS_FEEDBACK,
    CAN_ID_BRK_WARNING_STATUS,
)
from body.mock_brake import MockBrakeController
from body.protocol import (
    VehicleState,
    BrakeMachineState,
    DegradedMode,
    BCVehicleStateMsg,
    BCBrakeCommandMsg,
    BRKStatusFeedbackMsg,
    BRKWarningStatusMsg,
)


def test_mock_brake_lifecycle_and_derating():
    # Use virtual CAN bus for inter-thread testing
    test_channel = "body_test_mock_vcan"
    bus_ecu = can.Bus(interface="virtual", channel=test_channel)
    bus_tester = can.Bus(interface="virtual", channel=test_channel)

    mock = MockBrakeController(bus=bus_ecu)
    mock.start()

    try:
        # Give mock brief moment to initialize
        time.sleep(0.08)

        # 1. Send BC_VehicleState: STARTUP
        v_msg = BCVehicleStateMsg(
            vehicle_state=VehicleState.STARTUP,
            alive_counter=1,
            ambient_temperature=25,
        )
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_VEHICLE_STATE,
            data=v_msg.encode(),
        ))

        # Wait ~260 ms for mock to finish wakeup (50ms) + checks (200ms) -> ready
        time.sleep(0.30)
        assert mock.state == BrakeMachineState.READY

        # 2. Transition vehicle to READY and send 50% brake command
        v_msg.vehicle_state = VehicleState.READY
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_VEHICLE_STATE,
            data=v_msg.encode(),
        ))

        cmd_msg = BCBrakeCommandMsg(
            brake_command_percent=50,
            brake_enable=True,
            rolling_counter=1,
        )
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_BRAKE_COMMAND,
            data=cmd_msg.encode(),
        ))

        # Allow pressure emulation ramp
        time.sleep(0.35)
        assert mock.state == BrakeMachineState.BRAKING
        assert mock.actual_pressure_psi >= 45.0  # Ramped up near 50 psi

        # 3. Test Thermal Derating: set Ambient Temp = 55 deg C (> 50 deg C)
        v_msg.ambient_temperature = 55
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_VEHICLE_STATE,
            data=v_msg.encode(),
        ))

        # Send full 100% command
        cmd_msg.brake_command_percent = 100
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_BRAKE_COMMAND,
            data=cmd_msg.encode(),
        ))

        time.sleep(0.20)
        # Must activate warning and clamp target to 80.0 psi
        assert mock.warning_active is True
        assert mock.degraded_mode == DegradedMode.REDUCED_CAPACITY
        assert mock.max_available_pressure == 80.0
        assert mock.target_pressure_psi == 80.0
        assert mock.actual_pressure_psi <= 80.05

        # 4. Temperature cools down to 25 deg C
        v_msg.ambient_temperature = 25
        bus_tester.send(can.Message(
            arbitration_id=CAN_ID_BC_VEHICLE_STATE,
            data=v_msg.encode(),
        ))
        time.sleep(0.15)
        assert mock.warning_active is False
        assert mock.max_available_pressure == 100.0

    finally:
        mock.stop()
        bus_ecu.shutdown()
        bus_tester.shutdown()
