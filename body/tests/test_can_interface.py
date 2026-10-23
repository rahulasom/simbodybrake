"""
Tests for CAN interface manager, Vector fallback, and cyclic loop operation.
"""

import time
import pytest
import can

from body.can_interface import CanManager
from body.fault_manager import FaultManager
from body.logger import TelemetryLogger
from body.protocol import VehicleState
from body.state_machine import VehicleStateMachine


def test_can_manager_virtual_fallback_and_cyclic_tx(tmp_path):
    logger = TelemetryLogger(logs_dir=tmp_path)
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

    # Stop CAN manager
    cm.stop()
    logger.close()
