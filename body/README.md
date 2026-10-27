# Vehicle Body Controller (PC Supervisory Application)

Implementation of the **Vehicle Body Controller (BC)** in Python conforming to [body.md](file:///d:/projects/Agents/simbodybrake/body.md), [icd.md](file:///d:/projects/Agents/simbodybrake/icd.md), and [constitution.md](file:///d:/projects/Agents/simbodybrake/constitution.md).

---

## 1. Architecture Overview

```
body/
├── __init__.py              # Package exports and versioning
├── main.py                  # CLI entry point and application bootstrap
├── config.py                # CAN IDs, timings, debounce thresholds, paths
├── protocol.py              # Bit-level packing/unpacking and cantools DBC integration
├── can_interface.py         # Vector XL API driver with virtual fallback & 50ms cyclic loops
├── state_machine.py         # Vehicle FSM (OFF, STARTUP, READY, SHUTDOWN)
├── fault_manager.py         # Pressure discrepancy detector (300ms filter), MCU fault & warning manager
├── logger.py                # Synchronously-flushed CSV telemetry and application event logger
├── mock_brake.py            # Embedded TM4C123 LaunchPad Brake Controller simulator
├── ui.py                    # Tkinter / ttk automotive dashboard HMI
├── dbc/
│   ├── vehicle_state.dbc    # DBC for 0x100 BC_VehicleState & 0x101 BC_BrakeCommand
│   └── vehicle_brake.dbc    # DBC for 0x200 BRK_StatusFeedback, 0x201 BRK_FaultStatus, 0x202 BRK_WarningStatus
├── logs/                    # Generated CSV telemetry records and application log
└── tests/                   # Pytest test suite (protocol, FSM, faults, CAN, mock ECU)
```

---

## 2. Requirements Compliance Matrix

| Requirement | Implementation Module | Description |
| :--- | :--- | :--- |
| **REQ-BC-HW-001** | `can_interface.py` | Vector XL CAN Driver integration (`interface='vector'`, `vxlapi.dll`). |
| **REQ-BC-HW-002** | `config.py`, `can_interface.py` | 250 kbps nominal baud rate, 11-bit standard CAN identifiers. |
| **REQ-BC-HW-003** | `can_interface.py` | Automated fallback to `interface='virtual'` if Vector hardware is not present. |
| **REQ-BC-STATE-001** | `state_machine.py` | `OFF` state: 0% command, safe initial state upon startup. |
| **REQ-BC-STATE-002** | `state_machine.py` | `STARTUP` state: Handshake with Brake MCU, 2.0s timeout fault. |
| **REQ-BC-STATE-003** | `state_machine.py` | `READY` state: dynamic 0-100% braking permitted. |
| **REQ-BC-STATE-004** | `state_machine.py` | `SHUTDOWN` state: ramps command to 0%, holds 500 ms (10 frames) before `OFF`. |
| **REQ-BC-UI-001..008** | `ui.py` | Tkinter HMI, thread-safe queues, state controls, 0-100% slider, amber degraded warning, red critical fault banner. |
| **REQ-BC-CAN-001..005** | `protocol.py`, `dbc/` | 50 ms cyclic refresh, ICD bit-layout packing/unpacking and `cantools` DBC parity. |
| **REQ-BC-FLT-001..004** | `fault_manager.py` | $(P_{cmd} - P_{act}) > 10.0$ psi for $>300$ ms persistence detection; MCU fault ingestion; latched warning. |
| **REQ-BC-LOG-001..003** | `logger.py`, `ui.py` | On-demand CSV telemetry logging via UI Log button (disabled by default), synchronous disk flush, ISO 8601 timestamps, and application event logging (`body_controller.log`). |

---

## 3. Usage & Execution

### 3.1 Normal Mode (Vector XL Hardware)
Connects to Vector hardware channel 0 at 250 kbps:
```bash
python -m body.main
```
Or specify explicit interface and channel:
```bash
python -m body.main --interface vector --channel 0 --bitrate 250000
```

### 3.2 Offline / Standalone Simulation Mode
If no physical Vector hardware is present, the application automatically activates the **virtual CAN channel fallback** and launches the embedded **TM4C123 Brake Controller simulator** (`mock_brake.py`).

You can also explicitly run with the mock brake controller enabled:
```bash
python -m body.main --interface virtual --mock-brake
```

---

## 4. Running the Automated Test Suite

Execute the full suite of unit and integration tests using `pytest`:
```bash
pytest -v body/tests
```
All tests verify:
- Exact bit packing against `cantools` DBC decoding
- FSM state transitions, 2.0s startup handshake timeout, and 500ms shutdown hold
- 300 ms persistence debounce filter for pressure discrepancy faults
- External MCU fault ingestion and operator acknowledgment clearing
- Ambient temperature thermal derating (>50°C) triggering `REDUCED_CAPACITY` and 80 psi clamping
- Vector fallback to virtual bus and cyclic 50 ms message dispatch
