#!/usr/bin/env python3
"""Catch-the-leader control module with predictive intercept guidance."""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module, mp_settings, mp_util, multiproc
from MAVProxy.modules.mavproxy_map import mp_slipmap

try:
    from MAVProxy.modules.lib.wx_loader import wx  # type: ignore
except Exception:  # pragma: no cover - GUI optional
    wx = None


@dataclass
class VehicleState:
    """Container for vehicle kinematic state and availability."""

    sysid: int
    compid: int
    lat: Optional[float] = None
    lon: Optional[float] = None
    rel_alt: Optional[float] = None
    amsl_alt: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    vz: Optional[float] = None
    heading: Optional[float] = None
    mode: str = "UNKNOWN"
    armed: bool = False
    last_update: float = 0.0
    last_heartbeat: float = 0.0

    def identifier(self) -> str:
        return f"{self.sysid}:{self.compid}"

    def update_from_position(self, msg, timestamp: float) -> None:
        self.lat = msg.lat * 1.0e-7
        self.lon = msg.lon * 1.0e-7
        self.rel_alt = msg.relative_alt * 0.001
        self.amsl_alt = msg.alt * 0.001
        self.vx = msg.vx * 0.01
        self.vy = msg.vy * 0.01
        self.vz = msg.vz * 0.01
        if getattr(msg, "hdg", None) not in (None, 65535):
            self.heading = (msg.hdg * 0.01) % 360.0
        self.last_update = timestamp

    def update_from_heartbeat(self, msg, timestamp: float) -> None:
        self.mode = mavutil.mode_string_v10(msg)
        self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.last_heartbeat = timestamp

    def is_position_fresh(self, now: float, timeout: float) -> bool:
        return self.lat is not None and (now - self.last_update) <= timeout

    def is_heartbeat_fresh(self, now: float, timeout: float) -> bool:
        return self.last_heartbeat > 0.0 and (now - self.last_heartbeat) <= timeout

    def speed(self) -> Optional[float]:
        if self.vx is None or self.vy is None:
            return None
        return math.hypot(self.vx, self.vy)


@dataclass
class SpeedSelection:
    """Description of the currently requested follower speed profile."""

    profile: str
    value: float
    source: str
    forced_velocity: bool
    fallback: bool = False
    warning: Optional[str] = None


class NullCatchLeaderUI:
    """Fallback UI if wx is not available."""

    def post_update(self, _name: str, _payload: Optional[str] = None) -> None:
        pass

    def poll_commands(self) -> List[Tuple[str, Optional[str]]]:
        return []

    def close(self) -> None:
        pass


class CatchLeaderUI(NullCatchLeaderUI):
    """wxPython UI wrapper running in a helper process."""

    def __init__(self, title: str):
        if wx is None:
            raise RuntimeError("wxPython is not available")
        self.title = title
        self.parent_pipe, self.child_pipe = multiproc.Pipe()
        self.close_event = multiproc.Event()
        self.close_event.clear()
        self.child = multiproc.Process(target=self._child_task)
        self.child.daemon = True
        self.child.start()

    def _child_task(self) -> None:
        mp_util.child_close_fds()
        app = wx.App(False)
        frame = CatchLeaderFrame(self, self.title)
        frame.Show()
        app.MainLoop()

    def post_update(self, name: str, payload: Optional[str] = None) -> None:
        if not self.child.is_alive():
            return
        try:
            self.parent_pipe.send((name, payload))
        except (EOFError, BrokenPipeError):
            pass

    def poll_commands(self) -> List[Tuple[str, Optional[str]]]:
        if not self.child.is_alive():
            return []
        commands: List[Tuple[str, Optional[str]]] = []
        try:
            while self.parent_pipe.poll():
                commands.append(self.parent_pipe.recv())
        except (EOFError, BrokenPipeError):
            return []
        return commands

    def close(self) -> None:
        if not self.child.is_alive():
            return
        try:
            self.parent_pipe.send(("shutdown", None))
        except (EOFError, BrokenPipeError):
            pass
        self.close_event.set()
        self.child.join(timeout=1.5)


if wx is not None:

    class CatchLeaderFrame(wx.Frame):
        """Top-level window for the Catch-the-leader module."""

        def __init__(self, ui_state: CatchLeaderUI, title: str):
            super().__init__(None, title=title, size=(520, 420))
            self.ui_state = ui_state
            panel = wx.Panel(self)

            choices_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.leader_choice = wx.Choice(panel)
            self.follower_choice = wx.Choice(panel)
            self.leader_choice.Enable(False)
            self.follower_choice.Enable(False)
            choices_sizer.Add(wx.StaticText(panel, label="Leader:"),
                              flag=wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, border=4)
            choices_sizer.Add(self.leader_choice, proportion=1, flag=wx.RIGHT, border=8)
            choices_sizer.Add(wx.StaticText(panel, label="Follower:"),
                              flag=wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, border=4)
            choices_sizer.Add(self.follower_choice, proportion=1)

            self.leader_text = wx.StaticText(panel, label="Leader: ---")
            self.follower_text = wx.StaticText(panel, label="Follower: ---")
            self.range_text = wx.StaticText(panel, label="Range: ---  Δt: ---")
            self.target_text = wx.StaticText(panel, label="Target: ---")
            self.status_text = wx.StaticText(panel, label="Guidance: HOLD")
            self.system_text = wx.StaticText(panel, label="System status: Initialising")
            self.warning_text = wx.StaticText(panel, label="Warnings: none")
            self._warning_base_colour = self.warning_text.GetForegroundColour()

            self.alt_mode_toggle = wx.ToggleButton(panel, label="Altitude mode: Absolute")
            self.alt_mode_toggle.SetValue(False)
            self.alt_mode_toggle.Bind(
                wx.EVT_TOGGLEBUTTON,
                self._on_alt_mode_toggle,
            )

            target_filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self._suppress_target_filter_event = False
            self.target_filter_slider = wx.Slider(
                panel,
                minValue=0,
                maxValue=100,
                style=wx.SL_HORIZONTAL,
            )
            self.target_filter_slider.Bind(wx.EVT_SLIDER, self._on_target_filter_slider)
            self.target_filter_value = wx.StaticText(panel, label="Target smoothing α: 0.50")
            target_filter_sizer.Add(
                wx.StaticText(panel, label="Target smoothing:"),
                flag=wx.ALIGN_CENTER_VERTICAL | wx.RIGHT,
                border=4,
            )
            target_filter_sizer.Add(self.target_filter_slider, proportion=1, flag=wx.RIGHT, border=8)
            target_filter_sizer.Add(self.target_filter_value, flag=wx.ALIGN_CENTER_VERTICAL)

            btn_catch = wx.Button(panel, label="Catch now")
            btn_hold = wx.Button(panel, label="Hold guidance")
            btn_resume = wx.Button(panel, label="Resume auto")
            btn_clear = wx.Button(panel, label="Clear manual target")
            btn_fbwa = wx.Button(panel, label="Follower to FBWA")

            speed_profile_choices = ["Cruise", "Max", "Custom"]
            speed_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self._suppress_speed_profile_event = False
            self.speed_profile_choice = wx.Choice(panel, choices=speed_profile_choices)
            self.speed_profile_choice.Bind(wx.EVT_CHOICE, self._on_speed_profile_choice)
            self.custom_speed_input = wx.TextCtrl(panel, style=wx.TE_PROCESS_ENTER)
            self.custom_speed_input.Bind(wx.EVT_TEXT_ENTER, self._on_custom_speed_enter)
            self.custom_speed_button = wx.Button(panel, label="Set custom speed")
            self.custom_speed_button.Bind(wx.EVT_BUTTON, self._on_custom_speed_button)
            speed_sizer.Add(wx.StaticText(panel, label="Speed profile:"),
                            flag=wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, border=4)
            speed_sizer.Add(self.speed_profile_choice, flag=wx.RIGHT, border=8)
            speed_sizer.Add(wx.StaticText(panel, label="Custom speed (m/s):"),
                            flag=wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, border=4)
            speed_sizer.Add(self.custom_speed_input, proportion=1, flag=wx.RIGHT, border=4)
            speed_sizer.Add(self.custom_speed_button)
            self._update_speed_controls_enabled("unknown")

            self.speed_status_text = wx.StaticText(panel, label="Speed profile: ---")

            btn_catch.Bind(wx.EVT_BUTTON,
                           lambda evt: self.ui_state.child_pipe.send(("command", "catch")))
            btn_hold.Bind(wx.EVT_BUTTON,
                          lambda evt: self.ui_state.child_pipe.send(("command", "hold")))
            btn_resume.Bind(wx.EVT_BUTTON,
                            lambda evt: self.ui_state.child_pipe.send(("command", "resume")))
            btn_clear.Bind(wx.EVT_BUTTON,
                           lambda evt: self.ui_state.child_pipe.send(("command", "clear")))
            btn_fbwa.Bind(wx.EVT_BUTTON,
                          lambda evt: self.ui_state.child_pipe.send(("command", "fbwa")))

            self.leader_choice.Bind(
                wx.EVT_CHOICE,
                lambda evt: self._emit_selection("leader", self.leader_choice))
            self.follower_choice.Bind(
                wx.EVT_CHOICE,
                lambda evt: self._emit_selection("follower", self.follower_choice))

            self.log = wx.TextCtrl(panel, style=wx.TE_MULTILINE | wx.TE_READONLY)

            main_sizer = wx.BoxSizer(wx.VERTICAL)
            main_sizer.Add(choices_sizer, flag=wx.ALL | wx.EXPAND, border=4)
            for widget in (self.leader_text, self.follower_text,
                           self.range_text, self.target_text,
                           self.status_text, self.system_text,
                           self.warning_text):
                main_sizer.Add(widget, flag=wx.ALL | wx.EXPAND, border=4)

            main_sizer.Add(self.alt_mode_toggle, flag=wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, border=4)
            main_sizer.Add(speed_sizer, flag=wx.ALL | wx.EXPAND, border=4)
            main_sizer.Add(target_filter_sizer, flag=wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, border=4)
            main_sizer.Add(self.speed_status_text, flag=wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, border=4)

            button_sizer = wx.BoxSizer(wx.HORIZONTAL)
            for button in (btn_catch, btn_hold, btn_resume, btn_clear, btn_fbwa):
                button_sizer.Add(button, proportion=1, flag=wx.ALL, border=4)
            main_sizer.Add(button_sizer, flag=wx.EXPAND)
            main_sizer.Add(wx.StaticLine(panel), flag=wx.EXPAND | wx.ALL, border=4)
            main_sizer.Add(self.log, proportion=1, flag=wx.EXPAND | wx.ALL, border=4)

            panel.SetSizer(main_sizer)

            self.timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
            self.timer.Start(200)
            self.Bind(wx.EVT_CLOSE, self.on_close)

            self._set_target_filter_alpha(0.5)

        def _on_alt_mode_toggle(self, _event) -> None:
            mode = "relative" if self.alt_mode_toggle.GetValue() else "absolute"
            self.alt_mode_toggle.SetLabel(f"Altitude mode: {'Relative' if mode == 'relative' else 'Absolute'}")
            try:
                self.ui_state.child_pipe.send(("command", f"alt_mode:{mode}"))
            except (EOFError, BrokenPipeError):
                pass

        def _on_speed_profile_choice(self, _event) -> None:
            if self._suppress_speed_profile_event:
                return
            selection = self.speed_profile_choice.GetStringSelection().lower()
            if not selection:
                return
            try:
                self.ui_state.child_pipe.send(("command", f"speed_profile:{selection}"))
            except (EOFError, BrokenPipeError):
                pass

        def _on_custom_speed_enter(self, _event) -> None:
            self._send_custom_speed_command()

        def _on_custom_speed_button(self, _event) -> None:
            self._send_custom_speed_command()

        def _send_custom_speed_command(self) -> None:
            value = self.custom_speed_input.GetValue().strip()
            if not value:
                return
            try:
                self.ui_state.child_pipe.send(("command", f"custom_speed:{value}"))
            except (EOFError, BrokenPipeError):
                pass

        def _on_target_filter_slider(self, _event) -> None:
            if self._suppress_target_filter_event:
                return
            alpha = self.target_filter_slider.GetValue() / 100.0
            self._update_target_filter_label(alpha)
            try:
                self.ui_state.child_pipe.send(("command", f"target_filter_alpha:{alpha:.3f}"))
            except (EOFError, BrokenPipeError):
                pass

        def _emit_selection(self, kind: str, choice: "wx.Choice") -> None:
            selection = choice.GetStringSelection()
            if not selection:
                return
            try:
                self.ui_state.child_pipe.send(("command", f"select_{kind}:{selection}"))
            except (EOFError, BrokenPipeError):
                pass

        def append_log(self, text: str) -> None:
            timestamp = time.strftime("%H:%M:%S")
            self.log.AppendText(f"[{timestamp}] {text}\n")

        def update_warning(self, text: str, log: Optional[str] = None) -> None:
            has_warning = bool(text) and text.lower() != "warnings: none"
            colour = self._warning_base_colour
            if has_warning:
                lowered = text.lower()
                if any(word in lowered for word in ("lost", "disarmed")):
                    colour = wx.Colour(220, 53, 69)  # red for critical warnings
                else:
                    colour = wx.Colour(255, 140, 0)  # orange for general warnings
            self.warning_text.SetForegroundColour(colour)
            self.warning_text.SetLabel(text)
            if log:
                self.append_log(log)

        def on_timer(self, _event) -> None:
            try:
                while self.ui_state.child_pipe.poll():
                    name, payload = self.ui_state.child_pipe.recv()
                    if name == "leader" and payload is not None:
                        self.leader_text.SetLabel(payload)
                    elif name == "follower" and payload is not None:
                        self.follower_text.SetLabel(payload)
                    elif name == "range" and payload is not None:
                        self.range_text.SetLabel(payload)
                    elif name == "target" and payload is not None:
                        self.target_text.SetLabel(payload)
                    elif name == "status" and payload is not None:
                        self.status_text.SetLabel(payload)
                    elif name == "system" and payload is not None:
                        self.system_text.SetLabel(payload)
                    elif name == "warning" and payload is not None:
                        self.update_warning(payload)
                    elif name == "log" and payload is not None:
                        self.append_log(payload)
                    elif name == "vehicles" and isinstance(payload, list):
                        self._update_vehicle_choices(payload)
                    elif name == "leader_selection" and payload is not None:
                        self._set_choice_selection(self.leader_choice, payload)
                    elif name == "follower_selection" and payload is not None:
                        self._set_choice_selection(self.follower_choice, payload)
                    elif name == "alt_mode" and payload in ("relative", "absolute"):
                        self._update_alt_mode_toggle(payload)
                    elif name == "speed_profile" and payload is not None:
                        self._set_speed_profile(payload)
                    elif name == "custom_speed" and payload is not None:
                        self._set_custom_speed(payload)
                    elif name == "speed_status" and payload is not None:
                        self.speed_status_text.SetLabel(payload)
                    elif name == "target_filter_alpha" and payload is not None:
                        try:
                            self._set_target_filter_alpha(float(payload))
                        except ValueError:
                            pass
                    elif name == "shutdown":
                        self.Destroy()
                        return
            except (EOFError, BrokenPipeError):
                self.Destroy()

        def _update_vehicle_choices(self, options: List[str]) -> None:
            self.Freeze()
            try:
                self.leader_choice.Clear()
                self.follower_choice.Clear()
                for option in options:
                    self.leader_choice.Append(option)
                    self.follower_choice.Append(option)
                enable = bool(options)
                self.leader_choice.Enable(enable)
                self.follower_choice.Enable(enable)
            finally:
                self.Thaw()

        def _set_choice_selection(self, choice: "wx.Choice", value: str) -> None:
            idx = choice.FindString(value)
            if idx != wx.NOT_FOUND:
                choice.SetSelection(idx)

        def _set_speed_profile(self, profile: str) -> None:
            label_map = {"cruise": "Cruise", "max": "Max", "custom": "Custom"}
            display = label_map.get(profile.lower())
            self._suppress_speed_profile_event = True
            try:
                if display is None:
                    self.speed_profile_choice.SetSelection(wx.NOT_FOUND)
                else:
                    idx = self.speed_profile_choice.FindString(display)
                    if idx != wx.NOT_FOUND:
                        self.speed_profile_choice.SetSelection(idx)
                self._update_speed_controls_enabled(profile.lower())
            finally:
                self._suppress_speed_profile_event = False

        def _set_custom_speed(self, value: str) -> None:
            if self.custom_speed_input.GetValue() != value:
                self.custom_speed_input.SetValue(value)

        def _update_speed_controls_enabled(self, profile: str) -> None:
            enable_custom = profile == "custom"
            self.custom_speed_input.Enable(enable_custom)
            self.custom_speed_button.Enable(enable_custom)

        def _update_alt_mode_toggle(self, mode: str) -> None:
            is_relative = mode == "relative"
            self.alt_mode_toggle.SetValue(is_relative)
            label = "Altitude mode: Relative" if is_relative else "Altitude mode: Absolute"
            self.alt_mode_toggle.SetLabel(label)

        def _update_target_filter_label(self, alpha: float) -> None:
            self.target_filter_value.SetLabel(f"Target smoothing α: {alpha:.2f}")

        def _set_target_filter_alpha(self, alpha: float) -> None:
            clamped = max(0.0, min(1.0, alpha))
            slider_value = int(round(clamped * 100))
            self._suppress_target_filter_event = True
            try:
                if self.target_filter_slider.GetValue() != slider_value:
                    self.target_filter_slider.SetValue(slider_value)
                self._update_target_filter_label(clamped)
            finally:
                self._suppress_target_filter_event = False

        def on_close(self, event) -> None:
            try:
                self.ui_state.child_pipe.send(("command", "ui_closed"))
            except (EOFError, BrokenPipeError):
                pass
            event.Skip()


class CatchLeader(mp_module.MPModule):
    """Predictive intercept guidance for a follower airframe."""

    def __init__(self, mpstate):
        super().__init__(mpstate, "catchleader", "Catch the leader", multi_vehicle=True)
        self.catch_settings = mp_settings.MPSettings([
            ("leader_sysid", int, 1),
            ("leader_compid", int, 1),
            ("follower_sysid", int, 2),
            ("follower_compid", int, 1),
            ("follower_speed", float, 20.0),
            mp_settings.MPSetting(
                "speed_profile",
                str,
                "custom",
                choice=["cruise", "max", "custom"],
            ),
            ("max_lookahead", float, 25.0),
            ("min_closing", float, 1.0),
            ("update_period", float, 0.5),
            ("target_alt_offset", float, 0.0),
            ("min_distance", float, 5.0),
            ("position_timeout", float, 3.0),
            ("heartbeat_timeout", float, 4.5),
            ("use_relative_alt", bool, False),
            ("target_filter_alpha", float, 0.5),
        ])
        self.add_command(
            "catchleader",
            self.cmd_catchleader,
            "Catch-the-leader control",
            [
                "status",
                "set (CATCHSETTING)",
                "speed <cruise|max|custom> [value]",
                "alt_mode:<relative|absolute>",
                "catch",
                "hold",
                "resume",
                "fbwa",
                "goto lat lon [alt]",
                "clear",
            ],
        )
        self.add_completion_function("(CATCHSETTING)", self.catch_settings.completion)

        self.vehicle_states: dict[Tuple[int, int], VehicleState] = {}
        self.leader = self._ensure_state(
            self.catch_settings.leader_sysid,
            self.catch_settings.leader_compid,
        )
        self.follower = self._ensure_state(
            self.catch_settings.follower_sysid,
            self.catch_settings.follower_compid,
        )
        self.guidance_state = "hold"
        self.manual_target: Optional[Tuple[float, float, float]] = None
        self.last_sent = 0.0
        self.last_warning: Optional[str] = None
        self.last_range_status = 0.0
        self.current_status: Optional[str] = None
        self._map_target_key = "CatchLeaderTarget"
        self._map_target_added = False
        self._map_icon_image = None
        self._last_vehicle_refresh = 0.0
        self._last_leader_label: Optional[str] = None
        self._last_follower_label: Optional[str] = None
        self._last_target_info: Optional[Tuple[Tuple[float, float, float], bool, Optional[float]]] = None
        self._last_speed_selection: Optional[SpeedSelection] = None
        self._last_speed_command_value: Optional[float] = None
        self._last_speed_profile_command: Optional[str] = None
        self._last_speed_source_command: Optional[str] = None
        self._velocity_override_warning_sent = False
        self._follower_speed_smoothed: Optional[float] = None
        self._speed_telemetry_warning_sent = False
        self.filtered_target: Optional[Tuple[float, float, float]] = None

        self.ui = self._create_ui()
        self._emit_vehicle_options()
        self._emit_status()
        self._emit_altitude_mode(initial=True)
        self._update_speed_selection(log_change=True)
        self.ui.post_update(
            "log",
            (
                "Target smoothing active - set target_filter_alpha (0.0-1.0) "
                "to adjust EMA strength"
            ),
        )
        alpha = max(0.0, min(1.0, self.catch_settings.target_filter_alpha))
        self.ui.post_update("target_filter_alpha", f"{alpha:.4f}")

    def _create_ui(self):
        if wx is None:
            return NullCatchLeaderUI()
        try:
            return CatchLeaderUI("MAVProxy: Catch The Leader")
        except Exception:
            return NullCatchLeaderUI()

    def _altitude_frame_suffix(self) -> str:
        return "AGL" if self.catch_settings.use_relative_alt else "AMSL"

    def _format_altitude_text(self, alt: float) -> str:
        return f"alt {alt:.1f}m {self._altitude_frame_suffix()}"

    def _format_target_label(self, target: Tuple[float, float, float], manual: bool) -> str:
        lat, lon, alt = target
        mode = "manual" if manual else "predicted"
        return f"Target: {mode} {lat:.6f} {lon:.6f} {self._format_altitude_text(alt)}"

    def _refresh_target_display(self) -> None:
        if not self._last_target_info:
            return
        target, manual, closing_time = self._last_target_info
        self.ui.post_update("target", self._format_target_label(target, manual))
        self._update_map_target(target, closing_time, manual)

    def _emit_altitude_mode(self, initial: bool = False) -> None:
        mode = "relative" if self.catch_settings.use_relative_alt else "absolute"
        self.ui.post_update("alt_mode", mode)
        now = self.get_time()
        self._last_leader_label = None
        self._last_follower_label = None
        self._refresh_vehicle_labels(now, update_selection=False)
        self._last_vehicle_refresh = now
        if not initial:
            self._refresh_target_display()

    def _get_vehicle_param(self, sysid: Optional[int], compid: Optional[int],
                           name: str, default=None):
        """Lookup a parameter for a specific vehicle without changing selection."""

        if sysid is None:
            return default

        name = name.upper()
        candidates: List[Tuple[int, int]] = []
        if compid is not None:
            candidates.append((int(sysid), int(compid)))
        # Common autopilot components to try as fallbacks
        candidates.extend([
            (int(sysid), 1),
            (int(sysid), 0),
        ])

        seen = set()
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            params = self.mpstate.mav_param_by_sysid.get(candidate)
            if not params:
                continue
            value = params.get(name, None)
            if value is not None:
                return value
        return default

    def _get_follower_param(self, name: str, default=None):
        return self._get_vehicle_param(
            getattr(self.catch_settings, "follower_sysid", None),
            getattr(self.catch_settings, "follower_compid", None),
            name,
            default,
        )

    def _resolve_speed_selection(self) -> SpeedSelection:
        profile = getattr(self.catch_settings, "speed_profile", "custom") or "custom"
        profile = profile.lower()
        if profile not in {"custom", "cruise", "max"}:
            profile = "custom"
        forced_velocity = profile == "max"
        follower_speed = float(self.catch_settings.follower_speed)
        value = max(follower_speed, 0.0)
        source = "follower_speed"
        fallback = False
        warning: Optional[str] = None

        profile_candidates = {
            "cruise": [
                ("AIRSPEED_CRUISE", 1.0),
                ("AIRSPEED_TRIM", 1.0),
                ("TRIM_ARSPD_CM", 0.01),
            ],
            "max": [
                ("AIRSPEED_MAX", 1.0),
                ("ARSPD_FBW_MAX", 0.01),
            ],
        }

        candidates = profile_candidates.get(profile, [])
        if candidates:
            candidate_value = None
            candidate_source = None
            for name, scale in candidates:
                param_value = self._get_follower_param(name, None)
                if param_value is None:
                    continue
                try:
                    scaled = float(param_value) * scale
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(scaled) or scaled <= 0.0:
                    continue
                candidate_value = scaled
                candidate_source = name
                break
            if candidate_value is not None:
                value = candidate_value
                source = candidate_source or source
            else:
                fallback = True
                wanted = ", ".join(name for name, _scale in candidates)
                warning = (f"{profile} profile parameters unavailable; "
                           f"using follower_speed ({value:.1f} m/s) instead. "
                           f"Checked {wanted}.")

        if value <= 0.0:
            zero_warning = ("Follower speed is non-positive; intercept guidance will "
                            "use a minimum of 0.1 m/s until updated.")
            warning = f"{warning} {zero_warning}" if warning else zero_warning

        return SpeedSelection(
            profile=profile,
            value=value,
            source=source,
            forced_velocity=forced_velocity,
            fallback=fallback,
            warning=warning,
        )

    @staticmethod
    def _speed_selection_changed(previous: Optional[SpeedSelection], current: SpeedSelection) -> bool:
        if previous is None:
            return True
        if previous.profile != current.profile:
            return True
        if previous.source != current.source:
            return True
        if previous.forced_velocity != current.forced_velocity:
            return True
        if previous.fallback != current.fallback:
            return True
        if abs(previous.value - current.value) > 0.1:
            return True
        return False

    def _format_speed_selection(self, selection: SpeedSelection) -> str:
        desc = f"{selection.profile} {selection.value:.1f} m/s"
        if selection.source == "follower_speed":
            desc += " (follower_speed)"
        else:
            desc += f" via {selection.source}"
        if selection.fallback:
            desc += " [fallback]"
        if selection.forced_velocity:
            desc += " [velocity override]"
        return desc

    def _emit_speed_selection(self, selection: SpeedSelection) -> None:
        status = f"Speed profile: {self._format_speed_selection(selection)}"
        if selection.warning:
            status += f" — {selection.warning}"
        follower_speed = self.catch_settings.follower_speed
        try:
            follower_display = float(follower_speed)
        except (TypeError, ValueError):
            follower_display = 0.0
        self.ui.post_update("speed_status", status)
        self.ui.post_update("speed_profile", selection.profile)
        self.ui.post_update("custom_speed", f"{max(follower_display, 0.0):.1f}")

    def _update_speed_selection(self, log_change: bool = False) -> SpeedSelection:
        selection = self._resolve_speed_selection()
        previous = self._last_speed_selection
        changed = self._speed_selection_changed(previous, selection)
        self._last_speed_selection = selection
        if log_change and changed:
            message = f"Speed profile set to {self._format_speed_selection(selection)}"
            if selection.warning:
                message += f". {selection.warning}"
            self.ui.post_update("log", message)
        if selection.warning:
            if not previous or previous.warning != selection.warning:
                self.ui.post_update("log", f"Speed profile warning: {selection.warning}")
        elif previous and previous.warning:
            self.ui.post_update("log", "Speed profile warning cleared")
        if changed:
            self._last_speed_command_value = None
            self._last_speed_profile_command = None
            self._last_speed_source_command = None
        self._emit_speed_selection(selection)
        return selection

    def _maybe_send_speed_command(self, selection: SpeedSelection) -> None:
        if self.guidance_state != "auto":
            return
        if selection.value <= 0.0:
            return
        if (self._last_speed_command_value is not None
                and abs(self._last_speed_command_value - selection.value) < 0.25
                and self._last_speed_profile_command == selection.profile
                and self._last_speed_source_command == selection.source):
            return

        def sender(mav):
            mav.command_long_send(
                self.catch_settings.follower_sysid,
                self.catch_settings.follower_compid,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                0,
                float(selection.value),
                -1,
                0,
                0,
                0,
                0,
            )

        self.mpstate.foreach_mav(
            self.catch_settings.follower_sysid,
            self.catch_settings.follower_compid,
            sender,
        )
        log_message = f"Requested {self._format_speed_selection(selection)}"
        if selection.warning:
            log_message += f" — {selection.warning}"
        self.ui.post_update("log", log_message)
        self._last_speed_command_value = selection.value
        self._last_speed_profile_command = selection.profile
        self._last_speed_source_command = selection.source

    def cmd_catchleader(self, args: List[str]) -> None:
        if not args:
            self._print_usage()
            return
        cmd = args[0].lower()
        if cmd == "status":
            print(self.status_report())
        elif cmd == "set":
            previous_mode = self.catch_settings.use_relative_alt
            previous_alpha = self.catch_settings.target_filter_alpha
            self.catch_settings.command(args[1:])
            self._update_speed_selection(log_change=True)
            self._refresh_sysids()
            changed = previous_mode != self.catch_settings.use_relative_alt
            self._on_altitude_mode_updated(changed, source="CLI set")
            if self.catch_settings.target_filter_alpha != previous_alpha:
                alpha = max(0.0, min(1.0, self.catch_settings.target_filter_alpha))
                self._reset_target_filter()
                self.ui.post_update(
                    "log",
                    (
                        f"Target filter alpha set to {alpha:.2f} - "
                        "higher values follow the leader more aggressively"
                    ),
                )
                self.ui.post_update("target_filter_alpha", f"{alpha:.4f}")
            if len(args) == 1 or (len(args) >= 2 and args[1].lower() == "speed_profile"):
                self._print_set_documentation()
        elif cmd == "speed":
            self._handle_speed_command(args[1:])
        elif cmd in ("catch", "resume"):
            self.set_guidance_state("auto")
        elif cmd == "hold":
            self.set_guidance_state("hold")
        elif cmd == "fbwa":
            self.set_follower_fbwa()
        elif cmd == "goto":
            self._handle_manual_target(args[1:])
        elif cmd == "clear":
            self.manual_target = None
            self.ui.post_update("target", "Target: ---")
            self.ui.post_update("log", "Manual target cleared")
            self._clear_map_target()
            self._last_target_info = None
            self._set_system_status("Awaiting intercept solution")
            self._reset_target_filter()
        elif cmd.startswith("alt_mode:"):
            self._handle_altitude_mode(cmd[len("alt_mode:"):], source="CLI command")
        else:
            self._print_usage()

    def _print_usage(self) -> None:
        print("Usage: catchleader <status|set|speed|alt_mode|catch|hold|resume|fbwa|goto|clear>")

    def _print_set_documentation(self) -> None:
        print("catchleader set usage notes:")
        print("  follower_speed <m/s>  - manual airspeed used with the custom profile (default)")
        print("  speed_profile <cruise|max|custom>  - choose how follower speed is selected")
        print("    cruise: use AIRSPEED_CRUISE/AIRSPEED_TRIM/TRIM_ARSPD_CM when available")
        print("    max:    use AIRSPEED_MAX/ARSPD_FBW_MAX and engage velocity override")
        print("    custom: rely on follower_speed without automatic parameter lookup")
        print("  min_closing <m/s>     - minimum closing rate enforced during intercept")
        print("  target_filter_alpha    - EMA gain (1.0 = raw target, lower = more smoothing)")
        print("The guidance logic prefers measured follower groundspeed; if telemetry is")
        print("missing it falls back to follower_speed and records a warning in the log.")

    def _handle_speed_command(self, params: List[str]) -> None:
        usage = "Usage: catchleader speed <cruise|max|custom> [value]"
        if not params:
            print(usage)
            return
        profile = params[0].lower()
        if profile not in ("cruise", "max", "custom"):
            print("Unknown speed profile. Expected cruise, max, or custom.")
            print(usage)
            return
        if profile == "custom" and len(params) >= 2:
            try:
                follower_speed = float(params[1])
            except ValueError:
                print("Invalid custom speed value")
                return
            if follower_speed <= 0:
                print("Custom speed must be positive")
                return
            self.catch_settings.set("follower_speed", follower_speed)
            print(f"Custom follower speed set to {follower_speed:.1f} m/s")
        elif profile != "custom" and len(params) >= 2:
            print("Warning: numeric value ignored for non-custom profiles")
        self.catch_settings.set("speed_profile", profile)
        selection = self._update_speed_selection(log_change=True)
        print(f"Speed profile set to {self._format_speed_selection(selection)}")

    def _refresh_sysids(self) -> None:
        self.leader = self._ensure_state(
            self.catch_settings.leader_sysid,
            self.catch_settings.leader_compid,
        )
        self.follower = self._ensure_state(
            self.catch_settings.follower_sysid,
            self.catch_settings.follower_compid,
        )
        self._emit_vehicle_options()
        self.ui.post_update("leader_selection", self.leader.identifier())
        self.ui.post_update("follower_selection", self.follower.identifier())

    def set_guidance_state(self, state: str) -> None:
        if state not in ("auto", "hold"):
            return
        previous_state = self.guidance_state
        self.guidance_state = state
        if state != previous_state:
            self._reset_target_filter()
        text = "Guidance: AUTO" if state == "auto" else "Guidance: HOLD"
        self.ui.post_update("status", text)
        self.ui.post_update("log", f"Guidance state changed to {state.upper()}")
        if state == "hold":
            self._set_system_status("Guidance paused by operator")
            self._clear_map_target()
            self._last_speed_command_value = None
            self._last_speed_profile_command = None
            self._last_speed_source_command = None
        else:
            self._set_system_status("Awaiting intercept solution")
            self._update_speed_selection(log_change=True)

    def _handle_manual_target(self, params: List[str]) -> None:
        if len(params) < 2:
            print("Usage: catchleader goto <lat> <lon> [alt]")
            return
        try:
            lat = float(params[0])
            lon = float(params[1])
            if len(params) >= 3:
                alt = float(params[2])
            else:
                default_alt = (self.leader.rel_alt if self.catch_settings.use_relative_alt
                               else self.leader.amsl_alt)
                alt = default_alt or 0.0
        except ValueError:
            print("Invalid coordinates")
            return
        target = (lat, lon, alt)
        self.manual_target = target
        self.set_guidance_state("auto")
        self._reset_target_filter()
        self.ui.post_update("target", self._format_target_label(target, True))
        self.ui.post_update("log", "Manual intercept target set")
        self._set_system_status("Guiding to manual target")
        self._last_target_info = (target, True, None)
        self._update_map_target(target, None, manual=True)

    def _ensure_state(self, sysid: int, compid: int) -> VehicleState:
        key = (sysid, compid)
        state = self.vehicle_states.get(key)
        if state is None:
            state = VehicleState(sysid=sysid, compid=compid)
            self.vehicle_states[key] = state
        return state

    def mavlink_packet(self, msg) -> None:
        now = self.get_time()
        mtype = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()
        key = (sysid, compid)
        existed = key in self.vehicle_states
        state = self._ensure_state(sysid, compid)
        if not existed:
            self._emit_vehicle_options()

        if mtype == "GLOBAL_POSITION_INT":
            state.update_from_position(msg, now)
            self._emit_vehicle_update(state)
        elif mtype == "HEARTBEAT":
            state.update_from_heartbeat(msg, now)
            self._emit_vehicle_update(state)

    def idle_task(self) -> None:
        self._process_ui_commands()
        now = self.get_time()
        self._update_warnings(now)
        if now - self._last_vehicle_refresh >= 0.75:
            self._refresh_vehicle_labels(now)
            self._last_vehicle_refresh = now

        if self.guidance_state != "auto":
            if self.manual_target is None:
                self._set_system_status("Guidance paused by operator")
            return
        if now - self.last_sent < self.catch_settings.update_period:
            return

        speed_selection = self._update_speed_selection(log_change=False)
        if self.manual_target is not None:
            target = self.manual_target
            closing_time: Optional[float] = None
            self._set_system_status("Guiding to manual target")
        else:
            intercept = self.compute_intercept(now, speed_selection)
            if intercept is None:
                self._clear_map_target()
                self._reset_target_filter()
                return
            target, closing_time = intercept

        target = self._filter_target(target)
        self._maybe_send_speed_command(speed_selection)
        self._send_target(target, speed_selection)
        self.last_sent = now
        self.ui.post_update(
            "status",
            f"Guidance: AUTO → {target[0]:.6f} {target[1]:.6f} {self._format_altitude_text(target[2])}",
        )
        if self.manual_target is not None:
            self.ui.post_update("target", self._format_target_label(target, True))
            if (self.follower.lat is not None and self.follower.lon is not None):
                rng = mp_util.gps_distance(self.follower.lat, self.follower.lon, target[0], target[1])
                self.ui.post_update("range", f"Range to manual: {rng:6.1f} m")
            self._last_target_info = (target, True, closing_time)
            self._update_map_target(target, closing_time, manual=True)
        else:
            rng = mp_util.gps_distance(self.follower.lat, self.follower.lon,
                                       self.leader.lat, self.leader.lon)
            self.ui.post_update("range", f"Range: {rng:6.1f} m  Δt: {closing_time:5.1f} s")
            self.ui.post_update("target", self._format_target_label(target, False))
            self._last_target_info = (target, False, closing_time)
            self._update_map_target(target, closing_time, manual=False)

    def _process_ui_commands(self) -> None:
        for name, payload in self.ui.poll_commands():
            if name != "command":
                continue
            if payload == "catch" or payload == "resume":
                self.manual_target = None
                self.set_guidance_state("auto")
                self._last_target_info = None
                self._clear_map_target()
                self._reset_target_filter()
            elif payload == "hold":
                self.set_guidance_state("hold")
            elif payload == "clear":
                self.manual_target = None
                self.ui.post_update("target", "Target: ---")
                self._set_system_status("Awaiting intercept solution")
                self._clear_map_target()
                self._last_target_info = None
                self._reset_target_filter()
            elif payload == "fbwa":
                self.set_follower_fbwa()
            elif payload and payload.startswith("select_leader:"):
                self._handle_ui_selection(payload[len("select_leader:"):], leader=True)
            elif payload and payload.startswith("select_follower:"):
                self._handle_ui_selection(payload[len("select_follower:"):], leader=False)
            elif payload and payload.startswith("alt_mode:"):
                self._handle_altitude_mode(payload[len("alt_mode:"):], source="UI")
            elif payload and payload.startswith("speed_profile:"):
                self._handle_ui_speed_profile(payload[len("speed_profile:"):])
            elif payload and payload.startswith("custom_speed:"):
                self._handle_ui_custom_speed(payload[len("custom_speed:"):])
            elif payload and payload.startswith("target_filter_alpha:"):
                self._handle_ui_target_filter_alpha(payload[len("target_filter_alpha:"):])
            elif payload == "ui_closed":
                self.ui.close()
                self.ui = NullCatchLeaderUI()

    def _handle_ui_selection(self, selection: str, leader: bool) -> None:
        try:
            sysid_str, compid_str = selection.split(":", 1)
            sysid = int(sysid_str)
            compid = int(compid_str)
        except ValueError:
            return
        if leader:
            self.catch_settings.set("leader_sysid", sysid)
            self.catch_settings.set("leader_compid", compid)
        else:
            self.catch_settings.set("follower_sysid", sysid)
            self.catch_settings.set("follower_compid", compid)
        self._refresh_sysids()
        who = "Leader" if leader else "Follower"
        self.ui.post_update("log", f"{who} set to {sysid}:{compid}")

    def _handle_ui_speed_profile(self, profile: str) -> None:
        desired = profile.strip().lower()
        if desired not in ("custom", "cruise", "max"):
            self.ui.post_update("log", f"Unknown speed profile '{profile}' from UI")
            return
        self.catch_settings.set("speed_profile", desired)
        self._update_speed_selection(log_change=True)

    def _handle_ui_custom_speed(self, value: str) -> None:
        text = value.strip()
        if not text:
            return
        try:
            follower_speed = float(text)
        except ValueError:
            self.ui.post_update("log", f"Invalid custom speed '{value}'")
            return
        if follower_speed <= 0:
            self.ui.post_update("log", "Custom speed must be positive")
            return
        self.catch_settings.set("follower_speed", follower_speed)
        self.ui.post_update("log", f"Custom follower speed set to {follower_speed:.1f} m/s")
        self._update_speed_selection(log_change=True)

    def _handle_ui_target_filter_alpha(self, value: str) -> None:
        text = value.strip()
        if not text:
            return
        try:
            alpha = float(text)
        except ValueError:
            self.ui.post_update("log", f"Invalid target filter alpha '{value}'")
            return
        clamped = max(0.0, min(1.0, alpha))
        previous = getattr(self.catch_settings, "target_filter_alpha", clamped)
        if math.isclose(previous, clamped, abs_tol=5.0e-4):
            self.ui.post_update("target_filter_alpha", f"{clamped:.4f}")
            return
        self.catch_settings.set("target_filter_alpha", clamped)
        self._reset_target_filter()
        self.ui.post_update(
            "log",
            (
                f"Target filter alpha set to {clamped:.2f} via UI — "
                "higher values follow the leader more aggressively"
            ),
        )
        self.ui.post_update("target_filter_alpha", f"{clamped:.4f}")

    def _handle_altitude_mode(self, mode: str, source: str = "command") -> None:
        desired = mode.strip().lower()
        if desired not in ("relative", "absolute"):
            return
        use_relative = desired == "relative"
        current = self.catch_settings.use_relative_alt
        if use_relative == current:
            self._on_altitude_mode_updated(False, source=source)
            return
        self.catch_settings.set("use_relative_alt", use_relative)
        self._on_altitude_mode_updated(True, source=source)

    def _on_altitude_mode_updated(self, changed: bool, source: str) -> None:
        if changed and self.manual_target is not None:
            self.manual_target = None
            self._last_target_info = None
            self.ui.post_update("target", "Target: ---")
            self.ui.post_update("log", "Manual target cleared due to altitude mode change")
            self._set_system_status("Awaiting intercept solution")
            self._clear_map_target()
        if changed:
            mode = "RELATIVE" if self.catch_settings.use_relative_alt else "ABSOLUTE"
            self.ui.post_update("log", f"Altitude mode set to {mode} ({source})")
        self._emit_altitude_mode()

    def _update_warnings(self, now: float) -> None:
        warnings: List[str] = []
        if not self.leader.is_heartbeat_fresh(now, self.catch_settings.heartbeat_timeout):
            warnings.append("Leader heartbeat lost")
        if not self.leader.is_position_fresh(now, self.catch_settings.position_timeout):
            warnings.append("Leader position stale")
        if not self.follower.is_position_fresh(now, self.catch_settings.position_timeout):
            warnings.append("Follower position stale")
        if not self.follower.is_heartbeat_fresh(now, self.catch_settings.heartbeat_timeout):
            warnings.append("Follower heartbeat lost")
        if not self.follower.armed:
            warnings.append("Follower disarmed")
        if self.follower.mode.upper() not in ("GUIDED", "GUIDED_NOGPS", "POSHOLD", "LOITER"):
            warnings.append(f"Follower mode {self.follower.mode}")
        text = "; ".join(warnings) if warnings else "Warnings: none"
        if text != self.last_warning:
            self.ui.post_update("warning", text)
            if warnings:
                log_text = f"Warnings updated: {text}"
            else:
                log_text = "Warnings cleared"
            self.ui.post_update("log", log_text)
            self.last_warning = text

    def compute_intercept(
        self,
        now: float,
        speed_selection: SpeedSelection,
    ) -> Optional[Tuple[Tuple[float, float, float], float]]:
        if not self.leader.is_heartbeat_fresh(now, self.catch_settings.heartbeat_timeout):
            self._set_system_status("Waiting for leader heartbeat")
            return None
        if not self.follower.is_heartbeat_fresh(now, self.catch_settings.heartbeat_timeout):
            self._set_system_status("Waiting for follower heartbeat")
            return None
        if not self.leader.is_position_fresh(now, self.catch_settings.position_timeout):
            self._set_system_status("Waiting for leader position updates")
            return None
        if not self.follower.is_position_fresh(now, self.catch_settings.position_timeout):
            self._set_system_status("Waiting for follower position updates")
            return None
        if self.leader.lat is None or self.leader.lon is None or self.follower.lat is None or self.follower.lon is None:
            self._set_system_status("Incomplete telemetry for intercept")
            return None
        if not self.follower.armed:
            self._set_system_status("Follower disarmed — waiting for ARM")
            return None
        if self.follower.mode.upper() not in ("GUIDED", "GUIDED_NOGPS", "POSHOLD", "LOITER"):
            self._set_system_status(f"Follower mode {self.follower.mode} — waiting for GUIDED/LOITER")
            return None

        actual_speed = self.follower.speed()
        follower_speed_source = "telemetry"
        if actual_speed is not None and math.isfinite(actual_speed):
            alpha = 0.35
            if self._follower_speed_smoothed is None:
                self._follower_speed_smoothed = actual_speed
            else:
                self._follower_speed_smoothed = (
                    alpha * actual_speed
                    + (1.0 - alpha) * self._follower_speed_smoothed
                )
            follower_speed = max(self._follower_speed_smoothed, 0.1)
            if self._speed_telemetry_warning_sent:
                self.ui.post_update(
                    "log",
                    "Follower speed telemetry restored; using measured groundspeed for intercept calculations.",
                )
                self._speed_telemetry_warning_sent = False
        else:
            follower_speed = max(speed_selection.value, 0.1)
            follower_speed_source = "fallback"
            if not self._speed_telemetry_warning_sent:
                self.ui.post_update(
                    "log",
                    (
                        "Follower speed telemetry unavailable — using configured "
                        f"follower_speed ({follower_speed:.1f} m/s) as fallback."
                    ),
                )
                self._speed_telemetry_warning_sent = True
        rng = mp_util.gps_distance(self.follower.lat, self.follower.lon,
                                   self.leader.lat, self.leader.lon)
        if rng < self.catch_settings.min_distance:
            self._set_system_status("Follower within minimum distance — holding position")
            return None

        bearing_to_leader = mp_util.gps_bearing(self.follower.lat, self.follower.lon,
                                                self.leader.lat, self.leader.lon)
        leader_speed = self.leader.speed() or 0.0
        if leader_speed > 0.01 and self.leader.vx is not None and self.leader.vy is not None:
            leader_course = math.degrees(math.atan2(self.leader.vy, self.leader.vx)) % 360.0
        else:
            leader_course = bearing_to_leader
        closing_projection = leader_speed * math.cos(math.radians(bearing_to_leader - leader_course))
        closing_rate_raw = follower_speed - closing_projection
        min_closing = max(self.catch_settings.min_closing, 0.1)
        if closing_rate_raw < min_closing:
            closing_rate = min_closing
            closing_limited = True
        else:
            closing_rate = closing_rate_raw
            closing_limited = False
        time_to_go = min(rng / closing_rate, self.catch_settings.max_lookahead)
        offset_distance = leader_speed * time_to_go
        predicted_lat, predicted_lon = mp_util.gps_newpos(
            self.leader.lat,
            self.leader.lon,
            leader_course,
            offset_distance,
        )
        if self.catch_settings.use_relative_alt:
            leader_alt = self.leader.rel_alt if self.leader.rel_alt is not None else 0.0
        else:
            leader_alt = self.leader.amsl_alt if self.leader.amsl_alt is not None else 0.0
        target_alt = leader_alt + self.catch_settings.target_alt_offset
        if follower_speed_source == "telemetry":
            speed_note = "measured"
        else:
            speed_note = "fallback"
        closing_text = f"{closing_rate:.1f} m/s"
        if closing_limited:
            closing_text += f" (min_closing {min_closing:.1f} m/s)"
        status = (
            f"Intercepting leader — ETA {time_to_go:4.1f}s "
            f"(spd {follower_speed:.1f} m/s {speed_note}, closing {closing_text})"
        )
        self._set_system_status(status)
        return (predicted_lat, predicted_lon, target_alt), time_to_go

    def _send_target(self, target: Tuple[float, float, float], speed_selection: SpeedSelection) -> None:
        lat, lon, alt = target
        frame = (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                 if self.catch_settings.use_relative_alt
                 else mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        velocity_valid = False
        vx = vy = vz = 0.0
        if speed_selection.forced_velocity and speed_selection.value > 0.0:
            follower_lat = self.follower.lat
            follower_lon = self.follower.lon
            if self.catch_settings.use_relative_alt:
                follower_alt = self.follower.rel_alt
            else:
                follower_alt = self.follower.amsl_alt
            if (follower_lat is not None and follower_lon is not None
                    and follower_alt is not None):
                bearing = mp_util.gps_bearing(follower_lat, follower_lon, lat, lon)
                horizontal_distance = mp_util.gps_distance(follower_lat, follower_lon, lat, lon)
                alt_error = alt - follower_alt
                distance_3d = math.hypot(horizontal_distance, alt_error)
                if distance_3d > 1e-3:
                    horizontal_ratio = horizontal_distance / distance_3d if distance_3d > 0.0 else 0.0
                    vx = speed_selection.value * horizontal_ratio * math.cos(math.radians(bearing))
                    vy = speed_selection.value * horizontal_ratio * math.sin(math.radians(bearing))
                    vz = speed_selection.value * (-alt_error / distance_3d)
                    velocity_valid = True

        if not velocity_valid:
            type_mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            )

        if speed_selection.forced_velocity and not velocity_valid:
            if not self._velocity_override_warning_sent:
                self.ui.post_update(
                    "log",
                    "Velocity override skipped — follower telemetry incomplete",
                )
                self._velocity_override_warning_sent = True
        else:
            if self._velocity_override_warning_sent and velocity_valid:
                self.ui.post_update("log", "Velocity override restored")
            self._velocity_override_warning_sent = False

        def sender(mav):
            timestamp = int((time.time() * 1000.0) % 0xFFFFFFFF)
            mav.set_position_target_global_int_send(
                timestamp,
                self.catch_settings.follower_sysid,
                self.catch_settings.follower_compid,
                frame,
                type_mask,
                int(lat * 1.0e7),
                int(lon * 1.0e7),
                alt,
                vx,
                vy,
                vz,
                0.0,
                0.0,
                0.0,
                float("nan"),
                float("nan"),
            )

        self.mpstate.foreach_mav(
            self.catch_settings.follower_sysid,
            self.catch_settings.follower_compid,
            sender,
        )
        self.ui.post_update(
            "log",
            (
                f"Sent intercept target {lat:.6f} {lon:.6f} {self._format_altitude_text(alt)}"
                f" | speed {self._format_speed_selection(speed_selection)}"
                + (" (velocity override active)" if velocity_valid else "")
            ),
        )

    def _refresh_vehicle_labels(self, now: float, update_selection: bool = False) -> None:
        leader_label = self._format_vehicle(self.leader, now)
        if leader_label != self._last_leader_label:
            self.ui.post_update("leader", leader_label)
            self._last_leader_label = leader_label

        follower_label = self._format_vehicle(self.follower, now)
        if follower_label != self._last_follower_label:
            self.ui.post_update("follower", follower_label)
            self._last_follower_label = follower_label

        if update_selection:
            self.ui.post_update("leader_selection", self.leader.identifier())
            self.ui.post_update("follower_selection", self.follower.identifier())
            if (self.manual_target is None
                    and self.leader.lat is not None and self.leader.lon is not None
                    and self.follower.lat is not None and self.follower.lon is not None
                    and (now - self.last_range_status > 1.0 or self.last_range_status == 0.0)):
                rng = mp_util.gps_distance(
                    self.follower.lat,
                    self.follower.lon,
                    self.leader.lat,
                    self.leader.lon,
                )
                self.ui.post_update("range", f"Range: {rng:6.1f} m  Δt: ---")
                self.last_range_status = now

    def _emit_vehicle_update(self, state: VehicleState) -> None:
        now = self.get_time()
        self._refresh_vehicle_labels(now, update_selection=True)
        self._last_vehicle_refresh = now

    def _emit_status(self) -> None:
        now = self.get_time()
        self._last_vehicle_refresh = 0.0
        self._last_leader_label = None
        self._last_follower_label = None
        self._refresh_vehicle_labels(now, update_selection=True)
        self._last_vehicle_refresh = now
        self.ui.post_update("status", "Guidance: HOLD")
        self.ui.post_update("warning", "Warnings: none")
        self._set_system_status("Awaiting intercept solution")

    def _format_vehicle(self, state: VehicleState, now: Optional[float] = None) -> str:
        latlon = "---"
        if state.lat is not None and state.lon is not None:
            latlon = f"{state.lat:.6f} {state.lon:.6f}"
        if self.catch_settings.use_relative_alt:
            alt_value = state.rel_alt
        else:
            alt_value = state.amsl_alt
        suffix = self._altitude_frame_suffix()
        if alt_value is not None:
            alt = f"{alt_value:.1f}m {suffix}"
        else:
            alt = f"--- {suffix}"
        speed = state.speed()
        speed_str = f"{speed:.1f}m/s" if speed is not None else "---"
        mode = state.mode
        armed = "ARMED" if state.armed else "DISARMED"
        pos_age = None
        hb_age = None
        if now is not None:
            if state.last_update > 0.0:
                pos_age = max(0.0, now - state.last_update)
            if state.last_heartbeat > 0.0:
                hb_age = max(0.0, now - state.last_heartbeat)
        pos_str = f"{pos_age:.1f}s" if pos_age is not None else "---"
        hb_str = f"{hb_age:.1f}s" if hb_age is not None else "---"
        return (f"Sys {state.sysid}:{state.compid} | {latlon} | alt {alt} | "
                f"spd {speed_str} | {mode} | {armed} | pos {pos_str} / hb {hb_str}")

    def status_report(self) -> str:
        now = self.get_time()
        parts = [
            f"Leader: {self._format_vehicle(self.leader, now)}",
            f"Follower: {self._format_vehicle(self.follower, now)}",
            f"Guidance state: {self.guidance_state.upper()}",
        ]
        selection = self._last_speed_selection or self._update_speed_selection(log_change=False)
        parts.append(f"Speed target: {self._format_speed_selection(selection)}")
        alpha = max(0.0, min(1.0, self.catch_settings.target_filter_alpha))
        parts.append(
            "Target filter alpha: "
            f"{alpha:.2f} (1.0 = raw target, lower = more smoothing)"
        )
        if self.manual_target is not None:
            lat, lon, alt = self.manual_target
            parts.append(
                f"Manual target: lat {lat:.6f} lon {lon:.6f} {self._format_altitude_text(alt)}"
            )
        if self.current_status:
            parts.append(f"System status: {self.current_status}")
        return "\n".join(parts)

    def _reset_target_filter(self) -> None:
        self.filtered_target = None

    def _filter_target(self, target: Tuple[float, float, float]) -> Tuple[float, float, float]:
        alpha = max(0.0, min(1.0, self.catch_settings.target_filter_alpha))
        if self.filtered_target is None or alpha >= 1.0:
            self.filtered_target = target
            return target
        prev_lat, prev_lon, prev_alt = self.filtered_target
        lat = prev_lat + alpha * (target[0] - prev_lat)
        lon = prev_lon + alpha * (target[1] - prev_lon)
        alt = prev_alt + alpha * (target[2] - prev_alt)
        self.filtered_target = (lat, lon, alt)
        return self.filtered_target

    def _emit_vehicle_options(self) -> None:
        if not self.vehicle_states:
            return
        options = [state.identifier() for state in sorted(
            self.vehicle_states.values(), key=lambda s: (s.sysid, s.compid))]
        self.ui.post_update("vehicles", options)

    def _set_system_status(self, text: str) -> None:
        if text == self.current_status:
            return
        self.current_status = text
        self.ui.post_update("system", f"System status: {text}")

    def set_follower_fbwa(self) -> None:
        def sender(mav):
            mav.set_mode_send(
                self.catch_settings.follower_sysid,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                5,
            )

        self.mpstate.foreach_mav(
            self.catch_settings.follower_sysid,
            self.catch_settings.follower_compid,
            sender,
        )
        self.ui.post_update("log", "Requested follower mode change to FBWA")
        self._set_system_status("FBWA mode requested for follower")

    def unload(self) -> None:
        super().unload()
        self._clear_map_target()
        self.ui.close()

    def _map_instance(self):
        return getattr(self.mpstate, "map", None)

    def _ensure_map_icon(self):
        if self._map_icon_image is False:
            return None
        if self._map_icon_image is not None:
            return self._map_icon_image
        mpmap = self._map_instance()
        if not mpmap:
            return None
        try:
            self._map_icon_image = mpmap.icon("flag.png")
        except Exception:
            self._map_icon_image = False
            return None
        return self._map_icon_image

    def _update_map_target(self, target: Tuple[float, float, float], closing_time: Optional[float], manual: bool) -> None:
        self._last_target_info = (target, manual, closing_time)
        mpmap = self._map_instance()
        if not mpmap:
            return
        icon = self._ensure_map_icon()
        if icon is None:
            return
        lat, lon, alt = target
        label_parts = ["target"]
        if manual:
            label_parts.append("manual")
        else:
            label_parts.append("predictive")
        if closing_time is not None:
            label_parts.append(f"ETA {closing_time:0.1f}s")
        label_parts.append(self._format_altitude_text(alt))
        label = " | ".join(label_parts)
        colour = (255, 255, 0) if manual else (255, 128, 0)
        latlon = (lat, lon)
        if not self._map_target_added:
            mpmap.add_object(
                mp_slipmap.SlipIcon(
                    self._map_target_key,
                    latlon,
                    icon,
                    layer="CatchLeader",
                    follow=False,
                    label=label,
                    colour=colour,
                )
            )
            self._map_target_added = True
        else:
            mpmap.set_position(
                self._map_target_key,
                latlon,
                # layer must be iterable; wrap string so UI doesn't iterate characters
                layer=["CatchLeader"],
                label=label,
                colour=colour,
            )

    def _clear_map_target(self) -> None:
        if not self._map_target_added:
            self._last_target_info = None
            return
        mpmap = self._map_instance()
        if not mpmap:
            self._map_target_added = False
            self._last_target_info = None
            return
        mpmap.add_object(mp_slipmap.SlipRemoveObject(self._map_target_key))
        self._map_target_added = False
        self._last_target_info = None


def init(mpstate):
    """Initialise module."""
    return CatchLeader(mpstate)
