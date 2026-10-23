"""
Fault detection and alerting logic manager.
Implements REQ-BC-FLT-001 through REQ-BC-FLT-004 and REQ-BC-ALARM-001/002.
"""

from __future__ import annotations
import time
from typing import Optional, Tuple

from .config import (
    PRESSURE_DELTA_THRESHOLD_PSI,
    PRESSURE_PERSISTENCE_SEC,
)
from .protocol import (
    FaultCode,
    DegradedMode,
    BRKFaultStatusMsg,
    BRKWarningStatusMsg,
    VehicleState,
)


class FaultManager:
    """
    Evaluates hydraulic pressure tracking margins, ingests Brake Controller fault
    and warning frames, and manages fault latching and user acknowledgment.
    """

    def __init__(self):
        # Latch state
        self.is_fault_latched: bool = False
        self.fault_status: str = "NORMAL"  # NORMAL, PRESSURE_LOW, MCU_FAULT, STARTUP_TIMEOUT
        self.fault_description: str = "System Nominal"

        # Persistence tracking for pressure discrepancy
        self._pressure_discrepancy_start: Optional[float] = None
        self._current_delta: float = 0.0

        # External MCU fault tracking
        self.mcu_fault_active: bool = False
        self.mcu_fault_code: FaultCode = FaultCode.NO_FAULT
        self.mcu_sub_error_code: int = 0

        # Degraded warning tracking
        self.warning_active: bool = False
        self.degraded_mode: DegradedMode = DegradedMode.NONE
        self.max_available_pressure: float = 100.0

    def evaluate_pressure_tracking(
        self,
        vehicle_state: VehicleState,
        commanded_percent: int,
        actual_pressure: float,
        current_time: Optional[float] = None,
    ) -> Optional[Tuple[str, str]]:
        """
        Monitors P_cmd vs P_act correlation. Triggers fault if (P_cmd - P_act) > 10.0 psi
        persists for longer than 300 ms while vehicle is in READY state.
        Returns (status, description) if a new fault triggered, or None.
        """
        now = current_time if current_time is not None else time.monotonic()

        # Pressure discrepancy is only evaluated when active braking commands are issued in READY
        if vehicle_state == VehicleState.READY and commanded_percent > 0:
            target_psi = float(commanded_percent)  # 1% = 1.0 psi
            delta = target_psi - actual_pressure
            self._current_delta = delta

            if delta > PRESSURE_DELTA_THRESHOLD_PSI:
                if self._pressure_discrepancy_start is None:
                    self._pressure_discrepancy_start = now
                elif (now - self._pressure_discrepancy_start) >= PRESSURE_PERSISTENCE_SEC:
                    if not self.is_fault_latched or self.fault_status != "PRESSURE_LOW":
                        self.is_fault_latched = True
                        self.fault_status = "PRESSURE_LOW"
                        self.fault_description = (
                            f"CRITICAL FAULT: Brake pressure below commanded threshold! "
                            f"Commanded: {target_psi:.1f} psi, Actual: {actual_pressure:.1f} psi "
                            f"(Delta: {delta:.1f} psi > {PRESSURE_DELTA_THRESHOLD_PSI:.1f} psi for > {int(PRESSURE_PERSISTENCE_SEC * 1000)}ms)"
                        )
                        return self.fault_status, self.fault_description
            else:
                self._pressure_discrepancy_start = None
        else:
            self._pressure_discrepancy_start = None
            self._current_delta = 0.0

        return None

    def ingest_mcu_fault_message(self, msg: BRKFaultStatusMsg) -> Optional[Tuple[str, str]]:
        """
        Ingests BRK_FaultStatus (0x201).
        Immediately latches critical fault if FaultActive == 1.
        """
        self.mcu_fault_active = msg.fault_active
        self.mcu_fault_code = msg.fault_code
        self.mcu_sub_error_code = msg.sub_error_code

        if msg.fault_active:
            self.is_fault_latched = True
            self.fault_status = "MCU_FAULT"
            self.fault_description = (
                f"MCU FAULT: External Brake Controller reported {msg.fault_code.name} "
                f"(Code 0x{int(msg.fault_code):02X}, SubCode 0x{msg.sub_error_code:02X})"
            )
            return self.fault_status, self.fault_description
        return None

    def ingest_warning_message(self, msg: BRKWarningStatusMsg) -> Tuple[bool, DegradedMode, float]:
        """
        Ingests BRK_WarningStatus (0x202).
        Non-critical degraded state warning.
        """
        self.warning_active = msg.warning_active
        self.degraded_mode = msg.degraded_mode
        self.max_available_pressure = msg.max_available_pressure
        return self.warning_active, self.degraded_mode, self.max_available_pressure

    def trigger_startup_timeout_fault(self, elapsed_sec: float) -> Tuple[str, str]:
        """
        Triggers when the Brake Controller fails to report 'ready' within 2000 ms during STARTUP.
        """
        self.is_fault_latched = True
        self.fault_status = "STARTUP_TIMEOUT"
        self.fault_description = (
            f"STARTUP FAULT: Brake MCU did not reach 'ready' within {elapsed_sec:.1f}s timeout."
        )
        return self.fault_status, self.fault_description

    def acknowledge_and_clear(self) -> Tuple[bool, str]:
        """
        Operator attempts to clear latched fault.
        Clears if the underlying physical conditions have returned to nominal.
        """
        if self.mcu_fault_active:
            return False, "Cannot clear fault: Brake MCU is still asserting active fault."

        if self._current_delta > PRESSURE_DELTA_THRESHOLD_PSI and self._pressure_discrepancy_start is not None:
            return False, "Cannot clear fault: Commanded vs actual pressure delta remains out of threshold."

        self.is_fault_latched = False
        self.fault_status = "NORMAL"
        self.fault_description = "System Nominal / Fault Cleared by Operator"
        self._pressure_discrepancy_start = None
        return True, "Fault successfully cleared."
