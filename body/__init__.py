"""
Vehicle Body Controller Package.
Compliant with body.md, icd.md, and constitution.md.
"""

from .config import *
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
    DbcManager,
    get_dbc_manager,
)
from .fault_manager import FaultManager
from .state_machine import VehicleStateMachine
from .can_interface import CanManager, TelemetrySnapshot
from .logger import TelemetryLogger
from .mock_brake import MockBrakeController

__version__ = "1.0.0"
