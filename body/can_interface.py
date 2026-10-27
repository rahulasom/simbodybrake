"""
CAN communication interface layer.
Handles Vector XL API initialization with automated virtual fallback (REQ-BC-HW-001..003),
50 ms high-resolution cyclic transmission, and non-blocking frame reception.
"""

from __future__ import annotations
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import can

from .config import (
    DEFAULT_CAN_INTERFACE,
    FALLBACK_CAN_INTERFACE,
    DEFAULT_CAN_CHANNEL,
    DEFAULT_CAN_BITRATE,
    CAN_CYCLE_TIME_SEC,
    CAN_ID_BC_VEHICLE_STATE,
    CAN_ID_BC_BRAKE_COMMAND,
    CAN_ID_BRK_STATUS_FEEDBACK,
    CAN_ID_BRK_FAULT_STATUS,
    CAN_ID_BRK_WARNING_STATUS,
)
from .fault_manager import FaultManager
from .logger import TelemetryLogger
from .protocol import (
    VehicleState,
    BrakeMachineState,
    DegradedMode,
    FaultCode,
    BRKStatusFeedbackMsg,
    BRKFaultStatusMsg,
    BRKWarningStatusMsg,
)
from .state_machine import VehicleStateMachine


@dataclass
class TelemetrySnapshot:
    """Thread-safe telemetry data snapshot dispatched to GUI."""
    timestamp: float
    vehicle_state: VehicleState
    brake_state: BrakeMachineState
    commanded_percent: int
    target_pressure: float
    actual_pressure: float
    brake_active_flag: bool
    ambient_temperature: int
    alive_counter: int
    warning_active: bool
    degraded_mode: DegradedMode
    max_available_pressure: float
    fault_latched: bool
    fault_status: str
    fault_description: str
    is_virtual_bus: bool
    bus_channel_info: str
    can_fault_state: bool = False
    telemetry_logging_active: bool = False
    telemetry_log_file: str = ""


class CanManager:
    """
    Manages the CAN bus connection, 50ms periodic TX thread, and RX listener thread.
    """

    def __init__(
        self,
        state_machine: VehicleStateMachine,
        fault_manager: FaultManager,
        logger: TelemetryLogger,
        preferred_interface: str = DEFAULT_CAN_INTERFACE,
        channel: int = DEFAULT_CAN_CHANNEL,
        bitrate: int = DEFAULT_CAN_BITRATE,
        telemetry_queue: Optional[queue.Queue] = None,
        event_queue: Optional[queue.Queue] = None,
    ):
        self.sm = state_machine
        self.fault_mgr = fault_manager
        self.logger = logger
        self.preferred_interface = preferred_interface
        self.channel = channel
        self.bitrate = bitrate
        self.telemetry_queue = telemetry_queue if telemetry_queue else queue.Queue()
        self.event_queue = event_queue if event_queue else queue.Queue()

        self.bus: Optional[can.BusABC] = None
        self.active_interface_name: str = ""
        self.active_channel: Any = self.channel
        self.is_virtual_fallback: bool = False
        self.bus_channel_info: str = ""

        self._running = False
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def initialize_bus(self) -> Tuple[bool, str]:
        """
        Attempts to connect to preferred interface (Vector). If unavailable,
        falls back to virtual bus channel according to REQ-BC-HW-003.
        """
        # Step 1: Attempt preferred interface (e.g. Vector)
        if self.preferred_interface.lower() == "vector":
            try:
                self.logger.log_event("CAN_INIT", f"Attempting connection to Vector CAN channel {self.channel} @ {self.bitrate} bps...")
                self.bus = can.Bus(
                    interface="vector",
                    channel=self.channel,
                    bitrate=self.bitrate,
                )
                self.active_interface_name = "vector"
                self.active_channel = self.channel
                self.is_virtual_fallback = False
                self.bus_channel_info = f"Vector XL (Ch {self.channel} @ {self.bitrate // 1000}k)"
                msg = f"Connected to Vector CAN channel {self.channel} @ {self.bitrate} bps"
                self.logger.log_event("CAN_ONLINE", msg)
                return True, msg
            except Exception as ex:
                self.logger.log_event(
                    "CAN_WARN",
                    f"Vector CAN interface unavailable ({ex}). Activating automated virtual channel fallback.",
                )

        # Step 2: Fallback to virtual channel
        try:
            channel_name = f"body_vcan_{self.channel}"
            self.bus = can.Bus(
                interface=FALLBACK_CAN_INTERFACE,
                channel=channel_name,
                bitrate=self.bitrate,
            )
            self.active_interface_name = "virtual"
            self.active_channel = channel_name
            self.is_virtual_fallback = True
            self.bus_channel_info = f"Virtual Bus ({channel_name} @ {self.bitrate // 1000}k)"
            msg = f"Active on Virtual CAN channel '{channel_name}' (Vector hardware not present)"
            self.logger.log_event("CAN_FALLBACK", msg)
            return True, msg
        except Exception as ex:
            err = f"Failed to initialize CAN bus (both Vector and Virtual failed): {ex}"
            self.logger.log_event("CAN_ERROR", err)
            return False, err

    def start(self) -> None:
        """Starts 50ms periodic TX and RX threads."""
        if not self.bus:
            ok, msg = self.initialize_bus()
            if not ok:
                raise RuntimeError(msg)

        self._running = True

        # RX Thread
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name="CAN_RX_Thread",
            daemon=True,
        )
        self._rx_thread.start()

        # TX Thread (50 ms periodic)
        self._tx_thread = threading.Thread(
            target=self._tx_loop,
            name="CAN_TX_Thread",
            daemon=True,
        )
        self._tx_thread.start()

        self.logger.log_event("CAN_THREADS", "CAN cyclic TX and RX threads started.")

    def stop(self) -> None:
        """Stops background threads and closes CAN bus."""
        self._running = False
        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=0.2)
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.2)

        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

        self.logger.log_event("CAN_SHUTDOWN", "CAN bus closed cleanly.")

    # -------------------------------------------------------------------------
    # TX Loop (50 ms periodic)
    # -------------------------------------------------------------------------
    def _tx_loop(self) -> None:
        next_deadline = time.perf_counter()

        while self._running:
            start_tick = time.monotonic()

            with self._lock:
                self.sm.on_cyclic_tick(current_time=start_tick)
                vstate_msg = self.sm.build_vehicle_state_msg()
                bcmd_msg = self.sm.build_brake_command_msg()

            # Pack and send 0x100 BC_VehicleState
            try:
                frame_100 = can.Message(
                    arbitration_id=CAN_ID_BC_VEHICLE_STATE,
                    is_extended_id=False,
                    data=vstate_msg.encode(),
                )
                self.bus.send(frame_100)
            except Exception as e:
                self.logger.log_event("CAN_TX_ERR", f"Error sending 0x100: {e}")

            # Pack and send 0x101 BC_BrakeCommand
            try:
                frame_101 = can.Message(
                    arbitration_id=CAN_ID_BC_BRAKE_COMMAND,
                    is_extended_id=False,
                    data=bcmd_msg.encode(),
                )
                self.bus.send(frame_101)
            except Exception as e:
                self.logger.log_event("CAN_TX_ERR", f"Error sending 0x101: {e}")

            # Log telemetry record to CSV
            with self._lock:
                self.logger.log_telemetry(
                    vehicle_state=self.sm.state,
                    can_fault_state=vstate_msg.fault_state,
                    ambient_temp=self.sm.ambient_temperature,
                    brake_state=self.sm.last_brake_state,
                    commanded_percent=self.sm.commanded_percent,
                    actual_pressure=self.sm.last_actual_pressure,
                    warning_active=self.fault_mgr.warning_active,
                    degraded_mode=self.fault_mgr.degraded_mode,
                    max_pressure=self.fault_mgr.max_available_pressure,
                    fault_status=self.fault_mgr.fault_status,
                    fault_description=self.fault_mgr.fault_description,
                )

                # Push UI telemetry snapshot
                snapshot = TelemetrySnapshot(
                    timestamp=start_tick,
                    vehicle_state=self.sm.state,
                    brake_state=self.sm.last_brake_state,
                    commanded_percent=self.sm.commanded_percent,
                    target_pressure=float(self.sm.commanded_percent),
                    actual_pressure=self.sm.last_actual_pressure,
                    brake_active_flag=self.sm.last_brake_active,
                    ambient_temperature=self.sm.ambient_temperature,
                    alive_counter=self.sm.alive_counter,
                    warning_active=self.fault_mgr.warning_active,
                    degraded_mode=self.fault_mgr.degraded_mode,
                    max_available_pressure=self.fault_mgr.max_available_pressure,
                    fault_latched=self.fault_mgr.is_fault_latched,
                    fault_status=self.fault_mgr.fault_status,
                    fault_description=self.fault_mgr.fault_description,
                    is_virtual_bus=self.is_virtual_fallback,
                    bus_channel_info=self.bus_channel_info,
                    can_fault_state=vstate_msg.fault_state,
                    telemetry_logging_active=self.logger.is_telemetry_logging_active,
                    telemetry_log_file=self.logger.csv_filename.name if (self.logger.is_telemetry_logging_active and self.logger.csv_filename) else "",
                )

            # Avoid unbound queue growth
            if self.telemetry_queue.qsize() > 50:
                try:
                    self.telemetry_queue.get_nowait()
                except queue.Empty:
                    pass
            self.telemetry_queue.put(snapshot)

            # High precision 50 ms loop sleep
            next_deadline += CAN_CYCLE_TIME_SEC
            sleep_sec = next_deadline - time.perf_counter()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                next_deadline = time.perf_counter()

    # -------------------------------------------------------------------------
    # RX Loop (Non-blocking listener)
    # -------------------------------------------------------------------------
    def _rx_loop(self) -> None:
        while self._running:
            try:
                msg = self.bus.recv(timeout=0.050)
                if msg is None:
                    continue

                rx_time = time.monotonic()

                # Filter standard 11-bit IDs
                if msg.is_extended_id:
                    continue

                if msg.arbitration_id == CAN_ID_BRK_STATUS_FEEDBACK:
                    status_data = BRKStatusFeedbackMsg.decode(bytes(msg.data))
                    with self._lock:
                        self.sm.handle_brake_feedback(status_data, current_time=rx_time)

                elif msg.arbitration_id == CAN_ID_BRK_FAULT_STATUS:
                    fault_data = BRKFaultStatusMsg.decode(bytes(msg.data))
                    with self._lock:
                        fault_ret = self.fault_mgr.ingest_mcu_fault_message(fault_data)
                    if fault_ret:
                        self.logger.log_event("FAULT_MCU", fault_ret[1])
                        self.event_queue.put(("FAULT", fault_ret[1]))

                elif msg.arbitration_id == CAN_ID_BRK_WARNING_STATUS:
                    warn_data = BRKWarningStatusMsg.decode(bytes(msg.data))
                    with self._lock:
                        w_active, mode, max_p = self.fault_mgr.ingest_warning_message(warn_data)
                    if w_active:
                        self.logger.log_event(
                            "WARN_DEGRADED",
                            f"Degraded warning: {mode.name}, Max P={max_p:.1f} psi",
                        )

            except Exception as ex:
                if self._running:
                    self.logger.log_event("CAN_RX_ERR", f"RX exception: {ex}")
                    time.sleep(0.01)
