"""
Tests for CAN interface manager, Vector fallback, cyclic loop operation, and TelemetryLogger.
"""

import csv
import time
import tempfile
from pathlib import Path
import can

from body.can_interface import CanManager
from body.fault_manager import FaultManager
from body.logger import TelemetryLogger
from body.protocol import VehicleState, BrakeMachineState, DegradedMode
from body.state_machine import VehicleStateMachine


def test_telemetry_logger_disabled_by_default():
    """Verify TelemetryLogger starts in disabled state and does not create CSV by default."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        logger = TelemetryLogger(logs_dir=tmp_path)

        assert logger.is_telemetry_logging_active is False
        assert logger.csv_filename is None

        # log_event should write to body_controller.log
        logger.log_event("TEST_EVENT", "System event logged")
        assert (tmp_path / "body_controller.log").exists()

        # log_telemetry should NOT create any CSV file while disabled
        logger.log_telemetry(
            vehicle_state=VehicleState.OFF,
            can_fault_state=False,
            ambient_temp=25,
            brake_state=BrakeMachineState.WAKEUP,
            commanded_percent=0,
            actual_pressure=0.0,
            warning_active=False,
            degraded_mode=DegradedMode.NONE,
            max_pressure=100.0,
            fault_status="OK",
            fault_description="None",
        )

        csv_files = list(tmp_path.glob("*.csv"))
        assert len(csv_files) == 0

        logger.close()


def test_telemetry_logger_start_stop_toggle():
    """Verify start, stop, and toggle functionality of TelemetryLogger."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        logger = TelemetryLogger(logs_dir=tmp_path)

        # 1. Start logging
        csv_path = logger.start_telemetry_logging(session_name="unit_test")
        assert logger.is_telemetry_logging_active is True
        assert csv_path.exists()
        assert "unit_test" in csv_path.name

        # 2. Log telemetry rows
        logger.log_telemetry(
            vehicle_state=VehicleState.READY,
            can_fault_state=False,
            ambient_temp=28,
            brake_state=BrakeMachineState.BRAKING,
            commanded_percent=50,
            actual_pressure=49.5,
            warning_active=False,
            degraded_mode=DegradedMode.NONE,
            max_pressure=100.0,
            fault_status="OK",
            fault_description="None",
        )

        # 3. Stop logging
        stopped_path = logger.stop_telemetry_logging()
        assert stopped_path == csv_path
        assert logger.is_telemetry_logging_active is False

        # Read back CSV content
        with open(csv_path, mode="r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
            assert len(reader) == 2  # Header + 1 data row
            header, row1 = reader[0], reader[1]
            assert header == TelemetryLogger.CSV_HEADER
            assert row1[1] == "READY"
            assert row1[4] == "braking"
            assert row1[5] == "50"
            assert row1[6] == "49.5"

        # 4. Subsequent log_telemetry calls while stopped should not append
        logger.log_telemetry(
            vehicle_state=VehicleState.READY,
            can_fault_state=False,
            ambient_temp=30,
            brake_state=BrakeMachineState.BRAKING,
            commanded_percent=75,
            actual_pressure=74.0,
            warning_active=False,
            degraded_mode=DegradedMode.NONE,
            max_pressure=100.0,
            fault_status="OK",
            fault_description="None",
        )

        with open(csv_path, mode="r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
            assert len(reader) == 2  # Still just 2 rows

        # 5. Toggle logging
        active = logger.toggle_telemetry_logging()
        assert active is True
        assert logger.is_telemetry_logging_active is True

        active = logger.toggle_telemetry_logging()
        assert active is False
        assert logger.is_telemetry_logging_active is False

        logger.close()


def test_can_manager_virtual_fallback_and_cyclic_tx():
    with tempfile.TemporaryDirectory() as tmp_dir:
        logger = TelemetryLogger(logs_dir=Path(tmp_dir))
        fm = FaultManager()
        sm = VehicleStateMachine(fault_manager=fm)

        # Initializing with 'vector' on a machine without Vector HW must gracefully fallback to virtual
        cm = CanManager(
            state_machine=sm,
            fault_manager=fm,
            logger=logger,
            preferred_interface="vector",
            channel=0,
        )

        ok, msg = cm.initialize_bus()
        assert ok is True
        assert cm.is_virtual_fallback is True
        assert "Virtual" in cm.bus_channel_info

        # Start TX/RX threads
        cm.start()
        time.sleep(0.15)  # Allow 3 cyclic ticks (50ms each)

        # Telemetry queue must contain snapshots
        assert not cm.telemetry_queue.empty()
        snap = cm.telemetry_queue.get_nowait()
        assert snap.vehicle_state == VehicleState.OFF
        assert snap.alive_counter >= 1
        assert snap.is_virtual_bus is True
        assert snap.telemetry_logging_active is False

        # Stop CAN manager
        cm.stop()
        logger.close()
