"""
Vehicle Body Controller State Machine.
Implements REQ-BC-STATE-001 through REQ-BC-STATE-004.
"""

from __future__ import annotations
import time
from typing import Callable, Optional

from .config import (
    STARTUP_TIMEOUT_SEC,
    SHUTDOWN_HOLD_SEC,
    SHUTDOWN_CYCLES,
    AMBIENT_TEMP_DEFAULT,
    AMBIENT_TEMP_MIN,
    AMBIENT_TEMP_MAX,
)
from .fault_manager import FaultManager
from .protocol import (
    VehicleState,
    BrakeMachineState,
    BCVehicleStateMsg,
    BCBrakeCommandMsg,
    BRKStatusFeedbackMsg,
)


class VehicleStateMachine:
    """
    Supervisory state machine coordinating OFF, STARTUP, READY, and SHUTDOWN states,
    handshake with Brake Controller, and cyclic telemetry generation.
    """

    def __init__(
        self,
        fault_manager: Optional[FaultManager] = None,
        on_state_change: Optional[Callable[[VehicleState, VehicleState, str], None]] = None,
    ):
        self.state: VehicleState = VehicleState.OFF
        self.fault_mgr: FaultManager = fault_manager if fault_manager else FaultManager()
        self.on_state_change: Optional[Callable[[VehicleState, VehicleState, str], None]] = on_state_change

        # Commands and Controls
        self.commanded_percent: int = 0
        self.brake_enable: bool = False
        self.ambient_temperature: int = AMBIENT_TEMP_DEFAULT

        # Rolling counters
        self.alive_counter: int = 0
        self.cmd_rolling_counter: int = 0

        # Timing references
        self._startup_start_time: Optional[float] = None
        self._shutdown_start_time: Optional[float] = None
        self._shutdown_cycles_count: int = 0

        # Telemetry cache
        self.last_brake_state: BrakeMachineState = BrakeMachineState.WAKEUP
        self.last_actual_pressure: float = 0.0
        self.last_brake_active: bool = False

    def _transition_to(self, new_state: VehicleState, reason: str) -> None:
        old_state = self.state
        if old_state == new_state:
            return
        self.state = new_state
        # Reset fault detection state when transitioning to OFF or SHUTDOWN (REQ-BC-FLT-005)
        if new_state in (VehicleState.OFF, VehicleState.SHUTDOWN):
            self.fault_mgr.reset()
        if self.on_state_change:
            self.on_state_change(old_state, new_state, reason)

    # -------------------------------------------------------------------------
    # Operator Commands
    # -------------------------------------------------------------------------
    def request_power_on(self, current_time: Optional[float] = None) -> bool:
        """
        Operator clicks 'Power On' or 'Retry' (OFF -> STARTUP or FAULT -> STARTUP).
        Recovery from FAULT only allowed if the fault condition is resolved.
        """
        now = current_time if current_time is not None else time.monotonic()

        if self.state == VehicleState.OFF:
            self._startup_start_time = now
            self.commanded_percent = 0
            self.brake_enable = False
            self._transition_to(VehicleState.STARTUP, "Operator initiated Power On")
            return True

        elif self.state == VehicleState.FAULT:
            # Recovery from FAULT requires fault condition to be cleared/resolved (REQ-BC-STATE-005)
            cleared, _ = self.fault_mgr.acknowledge_and_clear()
            if not cleared:
                return False
            self._startup_start_time = now
            self.commanded_percent = 0
            self.brake_enable = False
            self._transition_to(VehicleState.STARTUP, "Operator initiated retry from FAULT")
            return True

        return False

    def request_retry(self, current_time: Optional[float] = None) -> bool:
        """
        Alias for request_power_on when recovering from FAULT state.
        """
        return self.request_power_on(current_time=current_time)

    def request_shutdown(self, current_time: Optional[float] = None) -> bool:
        """
        Operator clicks 'Power Off / Shutdown' (READY -> SHUTDOWN or STARTUP abort -> FAULT).
        """
        now = current_time if current_time is not None else time.monotonic()

        if self.state == VehicleState.READY:
            # Ramp down commands to 0%
            self.commanded_percent = 0
            self.brake_enable = False
            self._shutdown_start_time = now
            self._shutdown_cycles_count = 0
            self._transition_to(VehicleState.SHUTDOWN, "Operator initiated Shutdown")
            return True
        elif self.state == VehicleState.STARTUP:
            # Startup aborted by operator transitions to FAULT
            self.commanded_percent = 0
            self.brake_enable = False
            self._startup_start_time = None
            self.fault_mgr.trigger_startup_aborted_fault()
            self._transition_to(VehicleState.FAULT, "Startup sequence aborted by operator")
            return True

        return False

    def request_estop(self) -> None:
        """
        Emergency Stop immediately clamps brake commands to 0% and transitions to OFF.
        Permitted from any state (READY -> OFF, FAULT -> OFF, etc.).
        """
        self.commanded_percent = 0
        self.brake_enable = False
        self._startup_start_time = None
        self._shutdown_start_time = None
        self._transition_to(VehicleState.OFF, "EMERGENCY STOP TRIGGERED")

    def trigger_fault(self, reason: str) -> None:
        """
        Transitions to FAULT state when a fault condition is detected.
        Clamps brake command to 0% and disables braking.
        """
        self.commanded_percent = 0
        self.brake_enable = False
        self._startup_start_time = None
        self._shutdown_start_time = None
        self._transition_to(VehicleState.FAULT, f"Fault triggered: {reason}")

    def set_brake_command(self, percent: int) -> bool:
        """
        Adjusts brake command percentage (0..100%). Permitted only when in READY state.
        """
        if self.state != VehicleState.READY:
            return False
        clamped = max(0, min(100, int(percent)))
        self.commanded_percent = clamped
        self.brake_enable = True
        return True

    def set_ambient_temperature(self, temp_c: int) -> None:
        """
        Updates ambient temperature (-40 to +85 deg C).
        """
        self.ambient_temperature = max(AMBIENT_TEMP_MIN, min(AMBIENT_TEMP_MAX, int(temp_c)))

    # -------------------------------------------------------------------------
    # Periodic Cycle & CAN Updates (Called every 50 ms)
    # -------------------------------------------------------------------------
    def on_cyclic_tick(self, current_time: Optional[float] = None) -> None:
        """
        Periodic 50 ms task. Updates rolling counters and manages startup/shutdown timeout timers.
        """
        now = current_time if current_time is not None else time.monotonic()
        self.alive_counter = (self.alive_counter + 1) & 0xFF
        self.cmd_rolling_counter = (self.cmd_rolling_counter + 1) & 0xFF

        # STARTUP Handshake Timeout Evaluation (> 2.0s)
        if self.state == VehicleState.STARTUP and self._startup_start_time is not None:
            elapsed = now - self._startup_start_time
            if elapsed > STARTUP_TIMEOUT_SEC:
                self.fault_mgr.trigger_startup_timeout_fault(elapsed)
                self._startup_start_time = None
                self.commanded_percent = 0
                self.brake_enable = False
                self._transition_to(VehicleState.FAULT, f"Startup handshake timeout ({elapsed:.2f}s > {STARTUP_TIMEOUT_SEC}s)")

        # SHUTDOWN Hold Evaluation (min 500 ms / 10 cycles)
        elif self.state == VehicleState.SHUTDOWN and self._shutdown_start_time is not None:
            self._shutdown_cycles_count += 1
            elapsed = now - self._shutdown_start_time
            if elapsed >= SHUTDOWN_HOLD_SEC and self._shutdown_cycles_count >= SHUTDOWN_CYCLES:
                self._shutdown_start_time = None
                self._transition_to(VehicleState.OFF, "Shutdown hold sequence complete (brakes 0%, 500ms elapsed)")

    def handle_brake_feedback(self, msg: BRKStatusFeedbackMsg, current_time: Optional[float] = None) -> None:
        """
        Ingests 0x200 BRK_StatusFeedback from the Brake Controller.
        """
        self.last_brake_state = msg.brake_machine_state
        self.last_actual_pressure = msg.actual_brake_pressure
        self.last_brake_active = msg.brake_active_flag

        # In STARTUP: Advance to READY when Brake Controller reaches 'ready' state
        if self.state == VehicleState.STARTUP:
            if msg.brake_machine_state == BrakeMachineState.READY:
                self._startup_start_time = None
                self._transition_to(VehicleState.READY, "Brake MCU reports 'ready' - Handshake successful")
            elif msg.brake_machine_state == BrakeMachineState.FAULT:
                self._startup_start_time = None
                self.fault_mgr.trigger_fault("MCU_FAULT", "Brake MCU reported FAULT state during STARTUP")
                self.trigger_fault("Brake MCU reported FAULT state during STARTUP")

        # In READY: Evaluate external MCU state and pressure discrepancy
        elif self.state == VehicleState.READY:
            if msg.brake_machine_state == BrakeMachineState.FAULT:
                self.fault_mgr.trigger_fault("MCU_FAULT", "Brake MCU reported FAULT state in READY")
                self.trigger_fault("Brake MCU reported FAULT state in READY")
            else:
                fault_ret = self.fault_mgr.evaluate_pressure_tracking(
                    vehicle_state=self.state,
                    commanded_percent=self.commanded_percent,
                    actual_pressure=msg.actual_brake_pressure,
                    current_time=current_time,
                )
                if fault_ret:
                    self.trigger_fault(fault_ret[1])

    # -------------------------------------------------------------------------
    # Frame Builders for Transmit Loop
    # -------------------------------------------------------------------------
    def build_vehicle_state_msg(self) -> BCVehicleStateMsg:
        fault_active = (self.state == VehicleState.FAULT) or self.fault_mgr.is_fault_active
        return BCVehicleStateMsg(
            vehicle_state=self.state,
            fault_state=fault_active,
            alive_counter=self.alive_counter,
            ambient_temperature=self.ambient_temperature,
        )

    def build_brake_command_msg(self) -> BCBrakeCommandMsg:
        # Safety interlock: force 0% and disabled if not READY
        if self.state == VehicleState.READY:
            cmd = self.commanded_percent
            enable = self.brake_enable
        else:
            cmd = 0
            enable = False

        return BCBrakeCommandMsg(
            brake_command_percent=cmd,
            brake_enable=enable,
            rolling_counter=self.cmd_rolling_counter,
        )
