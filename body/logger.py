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
    Guarantees synchronous file flush on disk for critical safety events and telemetry.
    """

    CSV_HEADER = [
        "Timestamp",
        "VehicleState",
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

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = f"_{session_name}" if session_name else ""
        self.csv_filename = self.logs_dir / f"body_controller_{timestamp_str}{suffix}.csv"
        self.log_filename = self.logs_dir / "body_controller.log"

        self._lock = threading.Lock()
        self._csv_file = open(self.csv_filename, mode="w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_HEADER)
        self._csv_file.flush()

        # Application logger setup
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
        self.log_event("LOGGER_INIT", f"Telemetry logging to {self.csv_filename}")

    def log_telemetry(
        self,
        vehicle_state: VehicleState,
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
        if self._closed:
            return

        iso_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]

        row = [
            iso_timestamp,
            vehicle_state.name,
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
            if not self._closed:
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
