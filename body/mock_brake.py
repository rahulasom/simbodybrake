"""
Mock Brake Controller (Emulated TM4C123G Microcontroller).
Strictly follows brake.md and icd.md.
Used for offline testing, verification, and standalone demo execution.
"""

from __future__ import annotations
import threading
import time
from typing import Optional

import can

from .config import (
    CAN_CYCLE_TIME_SEC,
    CAN_ID_BC_VEHICLE_STATE,
    CAN_ID_BC_BRAKE_COMMAND,
    CAN_ID_BRK_STATUS_FEEDBACK,
    CAN_ID_BRK_FAULT_STATUS,
    CAN_ID_BRK_WARNING_STATUS,
    DEGRADED_TEMP_THRESHOLD,
)
from .protocol import (
    VehicleState,
    BrakeMachineState,
    FaultCode,
    DegradedMode,
    BCVehicleStateMsg,
    BCBrakeCommandMsg,
    BRKStatusFeedbackMsg,
    BRKFaultStatusMsg,
    BRKWarningStatusMsg,
)


class MockBrakeController(threading.Thread):
    """
    Simulates the TM4C123G Brake Controller firmware over CAN:
    - 5-state FSM: wakeup -> checks -> ready <-> braking / fault
    - Dynamic hydraulic pressure emulation (ramp up / ramp down)
    - Ambient temperature derating (>50 deg C -> REDUCED_CAPACITY, max 80 psi)
    - Fault injection support (underpressure, sensor fault, etc.)
    - 50 ms cyclic Tx tasks
    """

    def __init__(
        self,
        bus: Optional[can.BusABC] = None,
        interface: str = "virtual",
        channel: any = "body_vcan_0",
        bitrate: int = 250000,
        auto_respond_to_power_on: bool = True,
    ):
        super().__init__(name="MockBrakeController", daemon=True)
        self._owns_bus = False
        if bus is not None:
            self.bus = bus
        else:
            self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
            self._owns_bus = True

        self.auto_respond = auto_respond_to_power_on
        self.running = False

        # State Machine
        self.state: BrakeMachineState = BrakeMachineState.WAKEUP
        self._state_timer_start: float = time.monotonic()
        self._prev_vehicle_state: VehicleState = VehicleState.OFF

        # Input registers (from BC)
        self.rx_vehicle_state: VehicleState = VehicleState.OFF
        self.rx_ambient_temp: int = 25
        self.rx_cmd_percent: int = 0
        self.rx_brake_enable: bool = False

        # Emulated pressure model
        self.actual_pressure_psi: float = 0.0
        self.target_pressure_psi: float = 0.0
        self.ramp_rate_psi_per_sec: float = 200.0  # Hydraulic ramp speed

        # Fault & Warning state
        self.fault_active: bool = False
        self.fault_code: FaultCode = FaultCode.NO_FAULT
        self.sub_error_code: int = 0

        self.warning_active: bool = False
        self.degraded_mode: DegradedMode = DegradedMode.NONE
        self.max_available_pressure: float = 100.0

        # Fault injection flag: if True, deliberately simulate stuck low pressure
        self.inject_underpressure_fault: bool = False

    def inject_fault(self, code: FaultCode = FaultCode.SENSOR_FAULT, sub_code: int = 0x01) -> None:
        """Injects a critical MCU fault flag."""
        self.fault_active = True
        self.fault_code = code
        self.sub_error_code = sub_code
        self.state = BrakeMachineState.FAULT

    def clear_fault(self) -> None:
        """Clears MCU fault flag and returns to READY."""
        self.fault_active = False
        self.fault_code = FaultCode.NO_FAULT
        self.sub_error_code = 0
        self.inject_underpressure_fault = False
        if self.rx_vehicle_state == VehicleState.READY:
            self.state = BrakeMachineState.READY
        else:
            self.state = BrakeMachineState.WAKEUP
            self._state_timer_start = time.monotonic()

    def run(self) -> None:
        self.running = True
        next_cycle = time.perf_counter()

        while self.running:
            now = time.monotonic()
            self._process_rx_frames()
            self._update_fsm(now)
            self._update_pressure_model()
            self._send_tx_frames()

            next_cycle += CAN_CYCLE_TIME_SEC
            sleep_time = next_cycle - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_cycle = time.perf_counter()

    def stop(self) -> None:
        self.running = False
        if self._owns_bus and self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass

    def _process_rx_frames(self) -> None:
        """Non-blocking drain of relevant incoming frames."""
        try:
            while True:
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break
                if msg.arbitration_id == CAN_ID_BC_VEHICLE_STATE:
                    v_msg = BCVehicleStateMsg.decode(bytes(msg.data))
                    self.rx_vehicle_state = v_msg.vehicle_state
                    self.rx_ambient_temp = v_msg.ambient_temperature
                elif msg.arbitration_id == CAN_ID_BC_BRAKE_COMMAND:
                    c_msg = BCBrakeCommandMsg.decode(bytes(msg.data))
                    self.rx_cmd_percent = c_msg.brake_command_percent
                    self.rx_brake_enable = c_msg.brake_enable
        except Exception:
            pass

    def _update_fsm(self, now: float) -> None:
        """Evaluates TM4C123 5-state transitions."""
        if self.fault_active:
            self.state = BrakeMachineState.FAULT
            self.target_pressure_psi = 0.0
            return

        # Temperature derating evaluation
        if self.rx_ambient_temp > DEGRADED_TEMP_THRESHOLD:
            self.warning_active = True
            self.degraded_mode = DegradedMode.REDUCED_CAPACITY
            self.max_available_pressure = 80.0
        else:
            self.warning_active = False
            self.degraded_mode = DegradedMode.NONE
            self.max_available_pressure = 100.0

        # State progression
        if self.state == BrakeMachineState.WAKEUP:
            if now - self._state_timer_start >= 0.050:  # 50 ms in wakeup
                self.state = BrakeMachineState.CHECKS
                self._state_timer_start = now

        elif self.state == BrakeMachineState.CHECKS:
            if now - self._state_timer_start >= 0.200:  # 200 ms self checks
                self.state = BrakeMachineState.READY

        elif self.state == BrakeMachineState.READY:
            if self.rx_vehicle_state == VehicleState.READY and self.rx_brake_enable and self.rx_cmd_percent > 0:
                self.state = BrakeMachineState.BRAKING

        elif self.state == BrakeMachineState.BRAKING:
            if not (self.rx_vehicle_state == VehicleState.READY and self.rx_brake_enable and self.rx_cmd_percent > 0):
                self.state = BrakeMachineState.READY

        # If BC returns to OFF from active state, reset to WAKEUP if auto_respond
        if self.rx_vehicle_state == VehicleState.OFF and self.auto_respond and not self.fault_active:
            if self._prev_vehicle_state != VehicleState.OFF:
                self.state = BrakeMachineState.WAKEUP
                self._state_timer_start = now
        self._prev_vehicle_state = self.rx_vehicle_state

    def _update_pressure_model(self) -> None:
        """Ramps hydraulic pressure toward target."""
        if self.state == BrakeMachineState.BRAKING and not self.fault_active:
            # Clamping to max available pressure if degraded
            requested = float(self.rx_cmd_percent)
            if requested > self.max_available_pressure:
                requested = self.max_available_pressure
            self.target_pressure_psi = requested
        else:
            self.target_pressure_psi = 0.0

        if self.inject_underpressure_fault and self.target_pressure_psi > 20.0:
            # Deliberately limit pressure to 10 psi to trigger discrepancy fault
            target = 10.0
        else:
            target = self.target_pressure_psi

        step = self.ramp_rate_psi_per_sec * CAN_CYCLE_TIME_SEC
        if self.actual_pressure_psi < target:
            self.actual_pressure_psi = min(target, self.actual_pressure_psi + step)
        elif self.actual_pressure_psi > target:
            self.actual_pressure_psi = max(target, self.actual_pressure_psi - step)

    def _send_tx_frames(self) -> None:
        """Broadcasts 0x200, 0x201, 0x202 frames."""
        try:
            # 0x200 BRK_StatusFeedback
            active_flag = (self.actual_pressure_psi >= 2.0)
            status_msg = BRKStatusFeedbackMsg(
                brake_machine_state=self.state,
                actual_brake_pressure=self.actual_pressure_psi,
                brake_active_flag=active_flag,
            )
            msg_200 = can.Message(
                arbitration_id=CAN_ID_BRK_STATUS_FEEDBACK,
                is_extended_id=False,
                data=status_msg.encode(),
            )
            self.bus.send(msg_200)

            # 0x201 BRK_FaultStatus
            fault_msg = BRKFaultStatusMsg(
                fault_active=self.fault_active,
                fault_code=self.fault_code,
                sub_error_code=self.sub_error_code,
            )
            msg_201 = can.Message(
                arbitration_id=CAN_ID_BRK_FAULT_STATUS,
                is_extended_id=False,
                data=fault_msg.encode(),
            )
            self.bus.send(msg_201)

            # 0x202 BRK_WarningStatus
            warn_msg = BRKWarningStatusMsg(
                warning_active=self.warning_active,
                degraded_mode=self.degraded_mode,
                max_available_pressure=self.max_available_pressure,
            )
            msg_202 = can.Message(
                arbitration_id=CAN_ID_BRK_WARNING_STATUS,
                is_extended_id=False,
                data=warn_msg.encode(),
            )
            self.bus.send(msg_202)
        except Exception:
            pass
