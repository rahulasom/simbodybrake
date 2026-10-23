"""
Main entry point for Vehicle Body Controller Application.
Integrates CAN communication (Vector XL / Virtual), State Machine,
Fault Manager, Telemetry Logger, and Tkinter Dashboard.
"""

from __future__ import annotations
import argparse
import sys
import tkinter as tk
from typing import Optional

from .can_interface import CanManager
from .config import (
    DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_CHANNEL,
    DEFAULT_CAN_BITRATE,
)
from .fault_manager import FaultManager
from .logger import TelemetryLogger
from .mock_brake import MockBrakeController
from .state_machine import VehicleStateMachine
from .ui import BodyControllerUI


def parse_args():
    parser = argparse.ArgumentParser(
        description="Vehicle Body Controller (PC Supervisory Application)",
    )
    parser.add_argument(
        "--interface",
        type=str,
        default=DEFAULT_CAN_INTERFACE,
        help="CAN interface type: 'vector' (default) or 'virtual'.",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=DEFAULT_CAN_CHANNEL,
        help="CAN channel index/number (default: 0).",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=DEFAULT_CAN_BITRATE,
        help="CAN bus bitrate in bps (default: 250000).",
    )
    parser.add_argument(
        "--mock-brake",
        action="store_true",
        help="Start embedded TM4C123 Brake Controller mock simulator for standalone testing.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # 1. Initialize Logger
    logger = TelemetryLogger()
    logger.log_event("STARTUP", "Vehicle Body Controller initializing...")

    # 2. Initialize Fault Manager & State Machine
    fault_mgr = FaultManager()

    def on_state_transition(old_s, new_s, reason):
        logger.log_event(
            "STATE_TRANSITION",
            f"{old_s.name} -> {new_s.name} (Reason: {reason})",
        )

    state_machine = VehicleStateMachine(
        fault_manager=fault_mgr,
        on_state_change=on_state_transition,
    )

    # 3. Initialize CAN Interface (Vector XL with Virtual fallback)
    can_mgr = CanManager(
        state_machine=state_machine,
        fault_manager=fault_mgr,
        logger=logger,
        preferred_interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
    )

    ok, bus_msg = can_mgr.initialize_bus()
    if not ok:
        print(f"[FATAL] Could not initialize CAN bus: {bus_msg}", file=sys.stderr)
        sys.exit(1)

    # 4. Optional Mock Brake Controller for offline simulation
    mock_brake: Optional[MockBrakeController] = None
    if args.mock_brake or can_mgr.is_virtual_fallback:
        # If virtual fallback was triggered or user asked for mock-brake, start mock ECU
        logger.log_event("MOCK_ECU", "Starting embedded TM4C123 LaunchPad Brake Controller simulator...")
        mock_brake = MockBrakeController(
            interface=can_mgr.active_interface_name,
            channel=can_mgr.active_channel,
            bitrate=can_mgr.bitrate,
        )
        mock_brake.start()

    # 5. Start CAN Transmit/Receive threads
    can_mgr.start()

    # 6. Build and launch Tkinter GUI
    root = tk.Tk()
    ui = BodyControllerUI(
        root=root,
        state_machine=state_machine,
        fault_manager=fault_mgr,
        can_manager=can_mgr,
        logger=logger,
    )

    try:
        root.mainloop()
    finally:
        if mock_brake:
            mock_brake.stop()
        can_mgr.stop()
        logger.close()


if __name__ == "__main__":
    main()
