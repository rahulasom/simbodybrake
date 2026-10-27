"""
Test runner executing all unit tests in the body controller test suite.
Can be executed with: python -m body.tests.run_all
"""

import sys
import body.tests.test_protocol as tp
import body.tests.test_fault_manager as tfm
import body.tests.test_state_machine as tsm
import body.tests.test_can_interface as tci
import body.tests.test_mock_brake as tmb


def main():
    tests = [
        # Protocol & DBC parity tests
        tp.test_vehicle_state_msg_roundtrip,
        tp.test_vehicle_state_dbc_parity,
        tp.test_brake_command_msg_roundtrip,
        tp.test_brake_command_clamping,
        tp.test_brake_command_dbc_parity,
        tp.test_status_feedback_msg_roundtrip,
        tp.test_status_feedback_dbc_parity,
        tp.test_fault_status_msg_roundtrip,
        tp.test_fault_status_dbc_parity,
        tp.test_warning_status_msg_roundtrip,
        tp.test_warning_status_dbc_parity,

        # Fault Manager tests
        tfm.test_pressure_discrepancy_debouncing,
        tfm.test_pressure_discrepancy_recovery_before_timeout,
        tfm.test_mcu_fault_ingestion_and_clear,
        tfm.test_warning_status_ingestion,
        tfm.test_fault_reset_and_active_property,

        # State Machine tests
        tsm.test_initial_state_and_interlocks,
        tsm.test_startup_handshake_success,
        tsm.test_startup_timeout_fault,
        tsm.test_ready_brake_control_and_shutdown,
        tsm.test_estop_immediate_reset,
        tsm.test_fault_state_broadcast_and_reset_on_shutdown_and_off,

        # CAN interface and Logger tests
        tci.test_telemetry_logger_disabled_by_default,
        tci.test_telemetry_logger_start_stop_toggle,
        tci.test_can_manager_virtual_fallback_and_cyclic_tx,

        # Mock brake tests
        tmb.test_mock_brake_lifecycle_and_derating,
    ]

    failed = 0
    passed = 0
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1

    print(f"\n==========================================")
    print(f"Results: {passed} passed, {failed} failed (Total: {len(tests)})")
    print(f"==========================================")
    if failed > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
