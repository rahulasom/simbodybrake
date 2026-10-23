"""
Configuration constants and parameters for the Vehicle Body Controller.
Conforming to body.md and icd.md.
"""

from pathlib import Path

# CAN Network Parameters
DEFAULT_CAN_INTERFACE = "vector"
FALLBACK_CAN_INTERFACE = "virtual"
DEFAULT_CAN_CHANNEL = 0
DEFAULT_CAN_BITRATE = 250000  # 250 kbps
CAN_CYCLE_TIME_SEC = 0.050    # 50 ms (20 Hz)

# CAN Standard Identifiers (11-bit)
CAN_ID_BC_VEHICLE_STATE = 0x100     # 256
CAN_ID_BC_BRAKE_COMMAND = 0x101     # 257
CAN_ID_BRK_STATUS_FEEDBACK = 0x200  # 512
CAN_ID_BRK_FAULT_STATUS = 0x201     # 513
CAN_ID_BRK_WARNING_STATUS = 0x202   # 514

# State Machine Timers & Thresholds
STARTUP_TIMEOUT_SEC = 2.0           # 2000 ms max wait for brake MCU to reach 'ready'
SHUTDOWN_HOLD_SEC = 0.5             # 500 ms (10 frames) SHUTDOWN broadcast before OFF
SHUTDOWN_CYCLES = 10

# Fault Detection Parameters
PRESSURE_DELTA_THRESHOLD_PSI = 10.0 # Delta = P_cmd - P_act > 10.0 psi
PRESSURE_PERSISTENCE_SEC = 0.300    # 300 ms persistence debounce filter (6 cycles)

# Ambient Temperature Constraints
AMBIENT_TEMP_MIN = -40
AMBIENT_TEMP_MAX = 85
AMBIENT_TEMP_DEFAULT = 25

# Degraded Mode Threshold
DEGRADED_TEMP_THRESHOLD = 50.0      # Degraded reduced capacity (> 50 deg C)

# UI Refresh Rate
UI_POLL_INTERVAL_MS = 20            # 20 ms poller for Tkinter thread-safe queues

# Paths
BASE_DIR = Path(__file__).resolve().parent
DBC_DIR = BASE_DIR / "dbc"
LOGS_DIR = BASE_DIR / "logs"
