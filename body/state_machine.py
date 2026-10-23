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
        if self.on_state_change:
            self.on_state_change(old_state, new_state, reason)

    # -------------------------------------------------------------------------
    # Operator Commands
    # -------------------------------------------------------------------------
    def request_power_on(self, current_time: Optional[float] = None) -> bool:
        """
        Operator clicks 'Power On' (OFF -> STARTUP).
        """
        if self.state != VehicleState.OFF:
            return False

        now = current_time if current_time is not None else time.monotonic()
        self._startup_start_time = now
        self.commanded_percent = 0
        self.brake_enable = False
        self._transition_to(VehicleState.STARTUP, "Operator initiated Power On")
        return True

    def request_shutdown(self, current_time: Optional[float] = None) -> bool:
        """
        Operator clicks 'Power Off / Shutdown' (READY -> SHUTDOWN or STARTUP abort).
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
            # Startup aborted
            self.commanded_percent = 0
            self.brake_enable = False
            self._startup_start_time = None
            self._transition_to(VehicleState.OFF, "Startup sequence aborted by operator")
            return True

        return False

    def request_estop(self) -> None:
        """
        Emergency Stop immediately clamps brake commands to 0% and transitions to OFF.
        """
        self.commanded_percent = 0
        self.brake_enable = False
        self._startup_start_time = None
        self._shutdown_start_time = None
        self._transition_to(VehicleState.OFF, "EMERGENCY STOP TRIGGERED")

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
                self._transition_to(VehicleState.OFF, f"Startup handshake timeout ({elapsed:.2f}s > {STARTUP_TIMEOUT_SEC}s)")

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

        # In READY: Evaluate pressure discrepancy
        self.fault_mgr.evaluate_pressure_tracking(
            vehicle_state=self.state,
            commanded_percent=self.commanded_percent,
            actual_pressure=msg.actual_brake_pressure,
            current_time=current_time,
        )

    # -------------------------------------------------------------------------
    # Frame Builders for Transmit Loop
    # -------------------------------------------------------------------------
    def build_vehicle_state_msg(self) -> BCVehicleStateMsg:
        return BCVehicleStateMsg(
            vehicle_state=self.state,
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
