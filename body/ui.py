"""
Vehicle Body Controller User Interface (Tkinter & ttk).
Fully compliant with REQ-BC-UI-001 through REQ-BC-UI-008.
"""

from __future__ import annotations
import queue
import tkinter as tk
from tkinter import ttk, messagebox
import time
from typing import Optional, Callable

from .config import (
    AMBIENT_TEMP_MIN,
    AMBIENT_TEMP_MAX,
    AMBIENT_TEMP_DEFAULT,
    UI_POLL_INTERVAL_MS,
)
from .can_interface import CanManager, TelemetrySnapshot
from .fault_manager import FaultManager
from .logger import TelemetryLogger
from .protocol import VehicleState, BrakeMachineState, DegradedMode
from .state_machine import VehicleStateMachine


class BodyControllerUI:
    """
    Tkinter Operator Dashboard for the Vehicle Body Controller.
    Thread-isolated from CAN cyclic loops via thread-safe queues.
    """

    def __init__(
        self,
        root: tk.Tk,
        state_machine: VehicleStateMachine,
        fault_manager: FaultManager,
        can_manager: CanManager,
        logger: TelemetryLogger,
    ):
        self.root = root
        self.sm = state_machine
        self.fault_mgr = fault_manager
        self.can_mgr = can_manager
        self.logger = logger

        # State tracking for UI
        self._last_state: VehicleState = VehicleState.OFF
        self._last_brake_state: BrakeMachineState = BrakeMachineState.WAKEUP
        self._flash_toggle: bool = False
        self._is_updating_slider: bool = False

        self.root.title("Vehicle Body Controller - Supervisory HMI")
        self.root.geometry("960x780")
        self.root.minsize(880, 700)

        # Apply dark/automotive styling theme
        self._configure_styles()

        # Build UI layout
        self._build_header_banner()
        self._build_top_status_panel()
        self._build_control_panels()
        self._build_alarms_panel()
        self._build_log_console()
        self._sync_log_button_state()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close_window)

        # Schedule 20 ms UI Queue Poller
        self.root.after(UI_POLL_INTERVAL_MS, self._poll_queues)

    def _configure_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")

        # Global backgrounds
        self.root.configure(bg="#F0F2F5")

        # LabelFrame styling
        style.configure(
            "Card.TLabelframe",
            background="#FFFFFF",
            relief="groove",
            borderwidth=1,
        )
        style.configure(
            "Card.TLabelframe.Label",
            font=("Segoe UI", 10, "bold"),
            background="#FFFFFF",
            foreground="#1A202C",
        )

        style.configure(
            "Primary.TButton",
            font=("Segoe UI", 10, "bold"),
            padding=6,
        )
        style.configure(
            "Estop.TButton",
            font=("Segoe UI", 10, "bold"),
            foreground="#B71C1C",
            padding=6,
        )

    # -------------------------------------------------------------------------
    # Layout Builders
    # -------------------------------------------------------------------------
    def _build_header_banner(self) -> None:
        header_frame = tk.Frame(self.root, bg="#1E293B", padx=16, pady=10)
        header_frame.pack(fill=tk.X, side=tk.TOP)

        title_label = tk.Label(
            header_frame,
            text="VEHICLE BODY CONTROLLER (HMI SUPERVISOR)",
            font=("Segoe UI", 14, "bold"),
            fg="#F8FAFC",
            bg="#1E293B",
        )
        title_label.pack(side=tk.LEFT)

        self.lbl_can_status = tk.Label(
            header_frame,
            text="CAN Link: Initializing...",
            font=("Consolas", 10, "bold"),
            fg="#38BDF8",
            bg="#1E293B",
        )
        self.lbl_can_status.pack(side=tk.RIGHT)

    def _build_top_status_panel(self) -> None:
        top_frame = tk.Frame(self.root, bg="#F0F2F5", padx=16, pady=8)
        top_frame.pack(fill=tk.X)

        # Vehicle State Group
        veh_card = ttk.LabelFrame(top_frame, text=" VEHICLE SUPERVISOR STATE ", style="Card.TLabelframe", padding=12)
        veh_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        state_row = tk.Frame(veh_card, bg="#FFFFFF")
        state_row.pack(fill=tk.X, pady=4)

        tk.Label(state_row, text="Current State:", font=("Segoe UI", 10), bg="#FFFFFF").pack(side=tk.LEFT)
        self.lbl_veh_state = tk.Label(
            state_row,
            text="OFF",
            font=("Segoe UI", 12, "bold"),
            bg="#E2E8F0",
            fg="#475569",
            padx=12,
            pady=4,
            relief="solid",
            bd=1,
        )
        self.lbl_veh_state.pack(side=tk.RIGHT)

        can_fault_row = tk.Frame(veh_card, bg="#FFFFFF")
        can_fault_row.pack(fill=tk.X, pady=2)
        tk.Label(can_fault_row, text="CAN FaultState (0x100):", font=("Segoe UI", 10), bg="#FFFFFF").pack(side=tk.LEFT)
        self.lbl_can_fault_state = tk.Label(
            can_fault_row,
            text="FALSE / NORMAL",
            font=("Segoe UI", 10, "bold"),
            bg="#DCFCE7",
            fg="#166534",
            padx=8,
            pady=2,
            relief="solid",
            bd=1,
        )
        self.lbl_can_fault_state.pack(side=tk.RIGHT)

        btn_row = tk.Frame(veh_card, bg="#FFFFFF")
        btn_row.pack(fill=tk.X, pady=(10, 0))

        self.btn_power_on = ttk.Button(btn_row, text="Power On", style="Primary.TButton", command=self.on_click_power_on)
        self.btn_power_on.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)

        self.btn_shutdown = ttk.Button(btn_row, text="Shutdown", command=self.on_click_shutdown)
        self.btn_shutdown.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)

        self.btn_estop = ttk.Button(btn_row, text="EMERGENCY STOP", style="Estop.TButton", command=self.on_click_estop)
        self.btn_estop.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)

        # Brake Controller MCU Telemetry Group
        brk_card = ttk.LabelFrame(top_frame, text=" BRAKE CONTROLLER (TI MCU) TELEMETRY ", style="Card.TLabelframe", padding=12)
        brk_card.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(8, 0))

        mcu_row = tk.Frame(brk_card, bg="#FFFFFF")
        mcu_row.pack(fill=tk.X, pady=2)
        tk.Label(mcu_row, text="Brake ECU State:", font=("Segoe UI", 10), bg="#FFFFFF").pack(side=tk.LEFT)
        self.lbl_mcu_state = tk.Label(
            mcu_row,
            text="WAKEUP",
            font=("Segoe UI", 11, "bold"),
            bg="#E2E8F0",
            fg="#334155",
            padx=8,
            pady=2,
        )
        self.lbl_mcu_state.pack(side=tk.RIGHT)

        press_row = tk.Frame(brk_card, bg="#FFFFFF")
        press_row.pack(fill=tk.X, pady=2)
        tk.Label(press_row, text="Measured Pressure:", font=("Segoe UI", 10), bg="#FFFFFF").pack(side=tk.LEFT)
        self.lbl_actual_pressure = tk.Label(
            press_row,
            text="0.0 psi",
            font=("Consolas", 12, "bold"),
            bg="#FFFFFF",
            fg="#0F172A",
        )
        self.lbl_actual_pressure.pack(side=tk.RIGHT)

        tgt_row = tk.Frame(brk_card, bg="#FFFFFF")
        tgt_row.pack(fill=tk.X, pady=2)
        tk.Label(tgt_row, text="Target / Commanded P:", font=("Segoe UI", 10), bg="#FFFFFF").pack(side=tk.LEFT)
        self.lbl_target_pressure = tk.Label(
            tgt_row,
            text="0.0 psi (0%)",
            font=("Consolas", 11),
            bg="#FFFFFF",
            fg="#475569",
        )
        self.lbl_target_pressure.pack(side=tk.RIGHT)

    def _build_control_panels(self) -> None:
        mid_frame = tk.Frame(self.root, bg="#F0F2F5", padx=16, pady=4)
        mid_frame.pack(fill=tk.X)

        # Ambient Temperature Card
        temp_card = ttk.LabelFrame(mid_frame, text=" ENVIRONMENT SETTINGS (ICD 0x100) ", style="Card.TLabelframe", padding=12)
        temp_card.pack(fill=tk.X, pady=(0, 8))

        temp_row = tk.Frame(temp_card, bg="#FFFFFF")
        temp_row.pack(fill=tk.X)

        tk.Label(
            temp_row,
            text="Ambient Temperature (°C):",
            font=("Segoe UI", 10, "bold"),
            bg="#FFFFFF",
        ).pack(side=tk.LEFT, padx=(0, 10))

        self.temp_var = tk.IntVar(value=AMBIENT_TEMP_DEFAULT)
        self.spin_temp = tk.Spinbox(
            temp_row,
            from_=AMBIENT_TEMP_MIN,
            to=AMBIENT_TEMP_MAX,
            increment=1,
            textvariable=self.temp_var,
            width=6,
            font=("Consolas", 11, "bold"),
            command=self.on_temp_changed,
        )
        self.spin_temp.pack(side=tk.LEFT, padx=6)
        self.spin_temp.bind("<Return>", lambda e: self.on_temp_changed())
        self.spin_temp.bind("<FocusOut>", lambda e: self.on_temp_changed())

        tk.Label(
            temp_row,
            text="[-40°C to +85°C]  (Note: >50°C triggers TI MCU Thermal Derating Warning)",
            font=("Segoe UI", 9, "italic"),
            fg="#64748B",
            bg="#FFFFFF",
        ).pack(side=tk.LEFT, padx=12)

        # Brake Command Interface Card
        brake_card = ttk.LabelFrame(mid_frame, text=" DYNAMIC BRAKE COMMAND INTERFACE (ICD 0x101) ", style="Card.TLabelframe", padding=12)
        brake_card.pack(fill=tk.X)

        # Slider and live value
        slider_frame = tk.Frame(brake_card, bg="#FFFFFF")
        slider_frame.pack(fill=tk.X, pady=(0, 6))

        self.lbl_slider_val = tk.Label(
            slider_frame,
            text="Brake Command: 0 %  (Target: 0.0 psi)",
            font=("Segoe UI", 11, "bold"),
            bg="#FFFFFF",
            fg="#1E293B",
        )
        self.lbl_slider_val.pack(anchor="w")

        self.brake_slider = ttk.Scale(
            slider_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            command=self.on_slider_move,
        )
        self.brake_slider.set(0)
        self.brake_slider.pack(fill=tk.X, pady=4)

        # Quick action buttons
        btn_frame = tk.Frame(brake_card, bg="#FFFFFF")
        btn_frame.pack(fill=tk.X, pady=(4, 0))

        self.btn_apply_100 = ttk.Button(
            btn_frame,
            text="Apply Brakes (100%)",
            command=lambda: self.set_brake_percent(100),
        )
        self.btn_apply_100.pack(side=tk.LEFT, padx=4)

        self.btn_release_0 = ttk.Button(
            btn_frame,
            text="Release Brakes (0%)",
            command=lambda: self.set_brake_percent(0),
        )
        self.btn_release_0.pack(side=tk.LEFT, padx=4)

        self.lbl_cap_warning = tk.Label(
            btn_frame,
            text="",
            font=("Segoe UI", 9, "bold"),
            fg="#D97706",
            bg="#FFFFFF",
        )
        self.lbl_cap_warning.pack(side=tk.RIGHT, padx=6)

    def _build_alarms_panel(self) -> None:
        alarm_frame = tk.Frame(self.root, bg="#F0F2F5", padx=16, pady=4)
        alarm_frame.pack(fill=tk.X)

        alarms_card = ttk.LabelFrame(alarm_frame, text=" SYSTEM ALERTS & DIAGNOSTICS ", style="Card.TLabelframe", padding=10)
        alarms_card.pack(fill=tk.X)

        # Degraded Warning Banner (Amber)
        self.frame_warn_banner = tk.Frame(alarms_card, bg="#FEF3C7", bd=1, relief="solid", padx=10, pady=6)
        self.frame_warn_banner.pack(fill=tk.X, pady=(0, 6))

        self.lbl_warn_text = tk.Label(
            self.frame_warn_banner,
            text="STATUS: Nominal Operation (No Degraded Warnings)",
            font=("Segoe UI", 10, "bold"),
            fg="#92400E",
            bg="#FEF3C7",
        )
        self.lbl_warn_text.pack(side=tk.LEFT)

        # Critical Fault Banner (Red)
        self.frame_fault_banner = tk.Frame(alarms_card, bg="#E2E8F0", bd=1, relief="solid", padx=10, pady=6)
        self.frame_fault_banner.pack(fill=tk.X)

        self.lbl_fault_text = tk.Label(
            self.frame_fault_banner,
            text="FAULT STATUS: Normal / No Critical Fault",
            font=("Segoe UI", 10, "bold"),
            fg="#334155",
            bg="#E2E8F0",
        )
        self.lbl_fault_text.pack(side=tk.LEFT)

        self.btn_ack_fault = ttk.Button(
            self.frame_fault_banner,
            text="Acknowledge / Clear Fault",
            command=self.on_click_ack_fault,
        )
        self.btn_ack_fault.pack(side=tk.RIGHT)
        self.btn_ack_fault.state(["disabled"])

    def _build_log_console(self) -> None:
        console_frame = tk.Frame(self.root, bg="#F0F2F5", padx=16, pady=4)
        console_frame.pack(fill=tk.BOTH, expand=True)

        log_card = ttk.LabelFrame(console_frame, text=" EVENT & TELEMETRY STREAM PREVIEW ", style="Card.TLabelframe", padding=8)
        log_card.pack(fill=tk.BOTH, expand=True)

        # Telemetry logging control bar (REQ-BC-LOG-001: Log button & disabled-by-default indicator)
        log_ctrl_bar = tk.Frame(log_card, bg="#FFFFFF", padx=4, pady=4)
        log_ctrl_bar.pack(fill=tk.X, side=tk.TOP, pady=(0, 6))

        self.btn_toggle_log = tk.Button(
            log_ctrl_bar,
            text="● Start Telemetry Logging",
            font=("Segoe UI", 9, "bold"),
            bg="#0284C7",
            fg="#FFFFFF",
            activebackground="#0369A1",
            activeforeground="#FFFFFF",
            relief="flat",
            padx=10,
            pady=3,
            cursor="hand2",
            command=self.on_click_toggle_log,
        )
        self.btn_toggle_log.pack(side=tk.LEFT, padx=(0, 8))

        self.lbl_log_status = tk.Label(
            log_ctrl_bar,
            text="Telemetry Log: DISABLED (Not recording)",
            font=("Consolas", 9, "bold"),
            fg="#64748B",
            bg="#FFFFFF",
        )
        self.lbl_log_status.pack(side=tk.LEFT, fill=tk.X, expand=True, anchor="w")

        # Console text area
        text_container = tk.Frame(log_card, bg="#0F172A")
        text_container.pack(fill=tk.BOTH, expand=True)

        self.txt_log = tk.Text(
            text_container,
            height=6,
            font=("Consolas", 9),
            bg="#0F172A",
            fg="#E2E8F0",
            relief="flat",
            wrap=tk.WORD,
        )
        scrollbar = ttk.Scrollbar(text_container, orient=tk.VERTICAL, command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------------------
    # UI Event Handlers
    # -------------------------------------------------------------------------
    def on_click_toggle_log(self) -> None:
        """Toggles CSV telemetry logging on/off upon user button click."""
        is_active = self.logger.toggle_telemetry_logging()
        if is_active:
            fname = self.logger.csv_filename.name if self.logger.csv_filename else "CSV"
            self._append_console_log(f"Telemetry recording STARTED -> {fname}")
        else:
            self._append_console_log("Telemetry recording STOPPED.")
        self._sync_log_button_state()

    def _sync_log_button_state(self) -> None:
        """Updates the Log button appearance and status label to reflect active recording state."""
        if self.logger.is_telemetry_logging_active:
            fname = self.logger.csv_filename.name if self.logger.csv_filename else ""
            self.btn_toggle_log.config(
                text="■ Stop Telemetry Logging",
                bg="#DC2626",
                activebackground="#B91C1C",
            )
            self.lbl_log_status.config(
                text=f"Telemetry Log: RECORDING -> {fname}",
                fg="#DC2626",
            )
        else:
            self.btn_toggle_log.config(
                text="● Start Telemetry Logging",
                bg="#0284C7",
                activebackground="#0369A1",
            )
            self.lbl_log_status.config(
                text="Telemetry Log: DISABLED (Not recording)",
                fg="#64748B",
            )
    def on_click_power_on(self) -> None:
        self.logger.log_event("UI_EVENT", "User clicked Power On")
        ok = self.sm.request_power_on()
        if not ok:
            messagebox.showwarning("State Warning", "Power On is only permitted when vehicle is in OFF state.")
        self._append_console_log(f"Requested Power On (OFF -> STARTUP). Handshaking with Brake MCU...")

    def on_click_shutdown(self) -> None:
        self.logger.log_event("UI_EVENT", "User clicked Shutdown")
        ok = self.sm.request_shutdown()
        if not ok:
            messagebox.showwarning("State Warning", "Shutdown is only permitted when vehicle is in READY or STARTUP.")
        self._append_console_log(f"Requested Shutdown sequence (Brakes ramping to 0%, holding 500ms)...")

    def on_click_estop(self) -> None:
        self.logger.log_event("UI_EVENT", "User triggered EMERGENCY STOP", level=30)
        self.sm.request_estop()
        self.brake_slider.set(0)
        self._append_console_log("EMERGENCY STOP ACTIVATED! Commanded 0 psi, vehicle reset to OFF.")

    def on_slider_move(self, val_str: str) -> None:
        if self._is_updating_slider:
            return
        val = int(round(float(val_str)))
        self.set_brake_percent(val)

    def set_brake_percent(self, percent: int) -> None:
        if self._is_updating_slider:
            return
        self._is_updating_slider = True
        try:
            if self.sm.state != VehicleState.READY:
                self.brake_slider.set(0)
                self.lbl_slider_val.config(text="Brake Command: 0 %  (Target: 0.0 psi)")
                return

            percent = max(0, min(100, int(percent)))
            self.brake_slider.set(percent)
            self.sm.set_brake_command(percent)
            self.lbl_slider_val.config(text=f"Brake Command: {percent} %  (Target: {percent:.1f} psi)")
            self.logger.log_event("UI_CMD", f"Brake command set to {percent}% ({percent} psi)")
        finally:
            self._is_updating_slider = False

    def on_temp_changed(self) -> None:
        try:
            val = int(self.temp_var.get())
            val = max(AMBIENT_TEMP_MIN, min(AMBIENT_TEMP_MAX, val))
            self.temp_var.set(val)
            self.sm.set_ambient_temperature(val)
            self.logger.log_event("UI_ENV", f"Ambient temperature set to {val} deg C")
            self._append_console_log(f"Ambient Temperature set to {val}°C (Broadcast on CAN ID 0x100)")
        except Exception:
            self.temp_var.set(AMBIENT_TEMP_DEFAULT)

    def on_click_ack_fault(self) -> None:
        self.logger.log_event("UI_EVENT", "User clicked Acknowledge / Clear Fault")
        cleared, msg = self.fault_mgr.acknowledge_and_clear()
        if cleared:
            self._append_console_log(f"[INFO] {msg}")
            self.btn_ack_fault.state(["disabled"])
        else:
            self._append_console_log(f"[WARN] {msg}")
            messagebox.showwarning("Cannot Clear Fault", msg)

    def on_close_window(self) -> None:
        """Clean shutdown on window close."""
        self.logger.log_event("UI_CLOSE", "Window close requested. Initiating clean termination.")
        # Ramp down brake & send shutdown
        self.sm.request_shutdown()
        # Brief sleep for shutdown frame transmission
        time.sleep(0.1)
        self.can_mgr.stop()
        self.logger.close()
        self.root.destroy()

    def _append_console_log(self, text: str) -> None:
        t_str = time.strftime("%H:%M:%S")
        self.txt_log.insert(tk.END, f"[{t_str}] {text}\n")
        self.txt_log.see(tk.END)

    # -------------------------------------------------------------------------
    # Periodic Queue Poller & Redraw
    # -------------------------------------------------------------------------
    def _poll_queues(self) -> None:
        # Drain events
        try:
            while True:
                evt_type, evt_msg = self.can_mgr.event_queue.get_nowait()
                self._append_console_log(f"[{evt_type}] {evt_msg}")
        except queue.Empty:
            pass

        # Drain telemetry snapshots (keep latest)
        latest_snapshot: Optional[TelemetrySnapshot] = None
        try:
            while True:
                latest_snapshot = self.can_mgr.telemetry_queue.get_nowait()
        except queue.Empty:
            pass

        if latest_snapshot:
            self._update_telemetry_views(latest_snapshot)

        # Re-schedule poller
        self.root.after(UI_POLL_INTERVAL_MS, self._poll_queues)

    def _update_telemetry_views(self, snap: TelemetrySnapshot) -> None:
        # CAN status banner
        if snap.is_virtual_bus:
            self.lbl_can_status.config(
                text=f"CAN Link: FALLBACK VIRTUAL ({snap.bus_channel_info})",
                fg="#F59E0B",
            )
        else:
            self.lbl_can_status.config(
                text=f"CAN Link: ACTIVE ({snap.bus_channel_info})",
                fg="#10B981",
            )

        # Vehicle State Badge & Controls enablement
        v_state = snap.vehicle_state
        if v_state != self._last_state:
            self._last_state = v_state
            self._append_console_log(f"Vehicle State changed -> {v_state.name}")

        state_colors = {
            VehicleState.OFF: ("#E2E8F0", "#475569"),
            VehicleState.STARTUP: ("#FEF08A", "#854D0E"),
            VehicleState.READY: ("#BBF7D0", "#166534"),
            VehicleState.SHUTDOWN: ("#FED7AA", "#9A3412"),
        }
        bg_col, fg_col = state_colors.get(v_state, ("#E2E8F0", "#475569"))
        self.lbl_veh_state.config(text=v_state.name, bg=bg_col, fg=fg_col)

        # Vehicle CAN Fault State Badge (REQ-BC-UI-009)
        if snap.can_fault_state:
            self.lbl_can_fault_state.config(
                text="TRUE / FAULT ACTIVE",
                bg="#FEE2E2",
                fg="#B91C1C",
            )
        else:
            self.lbl_can_fault_state.config(
                text="FALSE / NORMAL",
                bg="#DCFCE7",
                fg="#166534",
            )

        # Enable/Disable controls based on state
        if v_state == VehicleState.OFF:
            self.btn_power_on.state(["!disabled"])
            self.btn_shutdown.state(["disabled"])
            self.brake_slider.state(["disabled"])
            self.btn_apply_100.state(["disabled"])
            self.btn_release_0.state(["disabled"])
        elif v_state == VehicleState.STARTUP:
            self.btn_power_on.state(["disabled"])
            self.btn_shutdown.state(["!disabled"])
            self.brake_slider.state(["disabled"])
            self.btn_apply_100.state(["disabled"])
            self.btn_release_0.state(["disabled"])
        elif v_state == VehicleState.READY:
            self.btn_power_on.state(["disabled"])
            self.btn_shutdown.state(["!disabled"])
            self.brake_slider.state(["!disabled"])
            self.btn_apply_100.state(["!disabled"])
            self.btn_release_0.state(["!disabled"])
        elif v_state == VehicleState.SHUTDOWN:
            self.btn_power_on.state(["disabled"])
            self.btn_shutdown.state(["disabled"])
            self.brake_slider.state(["disabled"])
            self.btn_apply_100.state(["disabled"])
            self.btn_release_0.state(["disabled"])

        # Brake ECU State Badge
        m_state = snap.brake_state
        m_colors = {
            BrakeMachineState.WAKEUP: ("#E0F2FE", "#0369A1"),
            BrakeMachineState.CHECKS: ("#FEF9C3", "#854D0E"),
            BrakeMachineState.READY: ("#DBEAFE", "#1D4ED8"),
            BrakeMachineState.BRAKING: ("#DCFCE7", "#15803D"),
            BrakeMachineState.FAULT: ("#FEE2E2", "#B91C1C"),
        }
        mbg, mfg = m_colors.get(m_state, ("#E2E8F0", "#334155"))
        self.lbl_mcu_state.config(text=m_state.name.upper(), bg=mbg, fg=mfg)

        # Pressure Display
        self.lbl_actual_pressure.config(text=f"{snap.actual_pressure:.1f} psi")
        self.lbl_target_pressure.config(
            text=f"{snap.target_pressure:.1f} psi ({snap.commanded_percent}%)"
        )

        # Degraded Warning Banner (REQ-BC-UI-008)
        if snap.warning_active:
            self.frame_warn_banner.config(bg="#FEF3C7")
            self.lbl_warn_text.config(
                text=f"WARNING: Brake Degraded Mode - {snap.degraded_mode.name} (Max Pressure: {snap.max_available_pressure:.1f} psi)",
                bg="#FEF3C7",
                fg="#B45309",
            )
            if snap.commanded_percent > snap.max_available_pressure:
                self.lbl_cap_warning.config(
                    text=f"Command ({snap.commanded_percent}%) capped at {snap.max_available_pressure:.1f} psi!",
                    fg="#B45309",
                )
            else:
                self.lbl_cap_warning.config(text="")
        else:
            self.frame_warn_banner.config(bg="#F1F5F9")
            self.lbl_warn_text.config(
                text="STATUS: Nominal Operation (No Degraded Warnings)",
                bg="#F1F5F9",
                fg="#475569",
            )
            self.lbl_cap_warning.config(text="")

        # Critical Fault Banner (REQ-BC-UI-006 & REQ-BC-ALARM-001)
        if snap.fault_latched:
            self.frame_fault_banner.config(bg="#FEE2E2")
            self.lbl_fault_text.config(
                text=f"CRITICAL FAULT: {snap.fault_description}",
                bg="#FEE2E2",
                fg="#B91C1C",
            )
            self.btn_ack_fault.state(["!disabled"])

            # Flash / highlight pressure reading
            self._flash_toggle = not self._flash_toggle
            if self._flash_toggle:
                self.lbl_actual_pressure.config(bg="#FEE2E2", fg="#B91C1C")
            else:
                self.lbl_actual_pressure.config(bg="#FFFFFF", fg="#B91C1C")
        else:
            self.frame_fault_banner.config(bg="#F1F5F9")
            self.lbl_fault_text.config(
                text="FAULT STATUS: Normal / No Critical Fault",
                bg="#F1F5F9",
                fg="#334155",
            )
            self.btn_ack_fault.state(["disabled"])
            self.lbl_actual_pressure.config(bg="#FFFFFF", fg="#0F172A")

        # Telemetry logging state indicator
        self._sync_log_button_state()
