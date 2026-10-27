"""
Thread-safe Telemetry CSV Logger and Application Event Logger.
Compliant with REQ-BC-LOG-001, REQ-BC-LOG-002, and REQ-BC-LOG-003.
"""

from __future__ import annotations
import csv
import logging
import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

from .config import LOGS_DIR
from .protocol import (
    VehicleState,
    BrakeMachineState,
    DegradedMode,
)


class TelemetryLogger:
    """
    Manages structured CSV telemetry logs and textual application event logs.
    Telemetry logging is disabled by default and can be started/stopped on demand.
    Guarantees synchronous file flush on disk for critical safety events and telemetry.
    """

    CSV_HEADER = [
        "Timestamp",
        "VehicleState",
        "CAN_FaultState",
        "AmbientTemperature_degC",
        "BrakeControllerState",
        "CommandedPercent",
        "ActualPressure_psi",
        "BrakeWarningActive",
        "BrakeDegradedMode",
        "MaxAvailablePressure_psi",
        "FaultStatus",
        "FaultDescription",
    ]

    def __init__(self, logs_dir: Optional[Path] = None, session_name: Optional[str] = None):
        self.logs_dir = Path(logs_dir) if logs_dir else LOGS_DIR
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.default_session_name = session_name

        self.log_filename = self.logs_dir / "body_controller.log"
        self.csv_filename: Optional[Path] = None

        self._lock = threading.Lock()
        self._csv_file = None
        self._csv_writer = None
        self.is_telemetry_logging_active: bool = False

        # Application logger setup (always active for system and diagnostic events)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._app_logger = logging.getLogger(f"BodyController_{timestamp_str}")
        self._app_logger.setLevel(logging.DEBUG)

        # File Handler (append to body_controller.log)
        file_handler = logging.FileHandler(self.log_filename, mode="a", encoding="utf-8")
        file_formatter = logging.Formatter(
            "%(asctime)s.%(msecs)03d [%(levelname)s] [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        file_handler.setFormatter(file_formatter)
        self._app_logger.addHandler(file_handler)

        self._closed = False
        self.log_event("LOGGER_INIT", f"Application event log initialized at {self.log_filename}")

    def start_telemetry_logging(self, session_name: Optional[str] = None) -> Path:
        """
        Starts telemetry CSV logging, creating a new timestamped CSV file.
        Returns the path to the newly active CSV file.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("Cannot start telemetry logging on closed TelemetryLogger.")

            if self.is_telemetry_logging_active and self._csv_file is not None:
                return self.csv_filename

            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            s_name = session_name if session_name else self.default_session_name
            suffix = f"_{s_name}" if s_name else ""
            self.csv_filename = self.logs_dir / f"body_controller_{timestamp_str}{suffix}.csv"

            self._csv_file = open(self.csv_filename, mode="w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(self.CSV_HEADER)
            self._csv_file.flush()
            self.is_telemetry_logging_active = True

        self.log_event("TELEMETRY_LOG_START", f"Telemetry logging started -> {self.csv_filename.name}")
        return self.csv_filename

    def stop_telemetry_logging(self) -> Optional[Path]:
        """
        Stops telemetry CSV logging and flushes/closes the active CSV file.
        Returns the path to the closed CSV file if one was active, or None.
        """
        closed_path: Optional[Path] = None
        with self._lock:
            if not self.is_telemetry_logging_active or self._csv_file is None:
                self.is_telemetry_logging_active = False
                return None

            self.is_telemetry_logging_active = False
            closed_path = self.csv_filename
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception:
                pass
            finally:
                self._csv_file = None
                self._csv_writer = None

        if closed_path:
            self.log_event("TELEMETRY_LOG_STOP", f"Telemetry logging stopped -> {closed_path.name}")
        return closed_path

    def toggle_telemetry_logging(self, session_name: Optional[str] = None) -> bool:
        """
        Toggles telemetry logging on or off. Returns True if now active, False if stopped.
        """
        if self.is_telemetry_logging_active:
            self.stop_telemetry_logging()
            return False
        else:
            self.start_telemetry_logging(session_name)
            return True

    def log_telemetry(
        self,
        vehicle_state: VehicleState,
        can_fault_state: bool,
        ambient_temp: int,
        brake_state: BrakeMachineState,
        commanded_percent: int,
        actual_pressure: float,
        warning_active: bool,
        degraded_mode: DegradedMode,
        max_pressure: float,
        fault_status: str,
        fault_description: str,
    ) -> None:
        if self._closed or not self.is_telemetry_logging_active:
            return

        iso_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]

        row = [
            iso_timestamp,
            vehicle_state.name,
            1 if can_fault_state else 0,
            ambient_temp,
            brake_state.name.lower(),
            commanded_percent,
            f"{actual_pressure:.1f}",
            1 if warning_active else 0,
            degraded_mode.name,
            f"{max_pressure:.1f}",
            fault_status,
            fault_description,
        ]

        with self._lock:
            if not self._closed and self.is_telemetry_logging_active and self._csv_writer is not None:
                self._csv_writer.writerow(row)
                self._csv_file.flush()

    def log_event(self, event_type: str, message: str, level: int = logging.INFO) -> None:
        """
        Logs operator interactions, state changes, or fault events with immediate synchronous flush.
        """
        if self._closed:
            return
        msg = f"[{event_type}] {message}"
        self._app_logger.log(level, msg)
        for handler in self._app_logger.handlers:
            handler.flush()

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self.log_event("LOGGER_CLOSE", "Closing telemetry logs")
                self._closed = True
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except Exception:
                    pass
                for handler in list(self._app_logger.handlers):
                    try:
                        handler.close()
                    except Exception:
                        pass
                    self._app_logger.removeHandler(handler)
