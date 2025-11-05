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

            btn_catch = wx.Button(panel, label="Catch now")
            btn_hold = wx.Button(panel, label="Hold guidance")
            btn_resume = wx.Button(panel, label="Resume auto")
            btn_clear = wx.Button(panel, label="Clear manual target")
            btn_fbwa = wx.Button(panel, label="Follower to FBWA")

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
                        self.warning_text.SetLabel(payload)
                    elif name == "log" and payload is not None:
                        self.append_log(payload)
                    elif name == "vehicles" and isinstance(payload, list):
                        self._update_vehicle_choices(payload)
                    elif name == "leader_selection" and payload is not None:
                        self._set_choice_selection(self.leader_choice, payload)
                    elif name == "follower_selection" and payload is not None:
                        self._set_choice_selection(self.follower_choice, payload)
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
            ("max_lookahead", float, 25.0),
            ("min_closing", float, 1.0),
            ("update_period", float, 0.5),
            ("target_alt_offset", float, 0.0),
            ("min_distance", float, 5.0),
            ("position_timeout", float, 3.0),
            ("heartbeat_timeout", float, 4.5),
            ("use_relative_alt", bool, True),
        ])
        self.add_command(
            "catchleader",
            self.cmd_catchleader,
            "Catch-the-leader control",
            [
                "status",
                "set (CATCHSETTING)",
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

        self.ui = self._create_ui()
        self._emit_vehicle_options()
        self._emit_status()

    def _create_ui(self):
        if wx is None:
            return NullCatchLeaderUI()
        try:
            return CatchLeaderUI("MAVProxy: Catch The Leader")
        except Exception:
            return NullCatchLeaderUI()

    def cmd_catchleader(self, args: List[str]) -> None:
        if not args:
            self._print_usage()
            return
        cmd = args[0].lower()
        if cmd == "status":
            print(self.status_report())
        elif cmd == "set":
            self.catch_settings.command(args[1:])
            self._refresh_sysids()
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
        else:
            self._print_usage()

    def _print_usage(self) -> None:
        print("Usage: catchleader <status|set|catch|hold|resume|fbwa|goto|clear>")

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
        self.guidance_state = state
        text = "Guidance: AUTO" if state == "auto" else "Guidance: HOLD"
        self.ui.post_update("status", text)
        self.ui.post_update("log", f"Guidance state changed to {state.upper()}")
        if state == "hold":
            self._set_system_status("Guidance paused by operator")
            self._clear_map_target()
        else:
            self._set_system_status("Awaiting intercept solution")

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
        self.manual_target = (lat, lon, alt)
        self.set_guidance_state("auto")
        self.ui.post_update("target", f"Target: manual {lat:.6f} {lon:.6f} alt {alt:.1f}m")
        self.ui.post_update("log", "Manual intercept target set")
        self._set_system_status("Guiding to manual target")

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

        if self.guidance_state != "auto":
            if self.manual_target is None:
                self._set_system_status("Guidance paused by operator")
            return
        if now - self.last_sent < self.catch_settings.update_period:
            return

        if self.manual_target is not None:
            target = self.manual_target
            closing_time = 0.0
            self._set_system_status("Guiding to manual target")
        else:
            intercept = self.compute_intercept(now)
            if intercept is None:
                self._clear_map_target()
                return
            target, closing_time = intercept

        self._send_target(target)
        self.last_sent = now
        self.ui.post_update(
            "status",
            f"Guidance: AUTO → {target[0]:.6f} {target[1]:.6f} {target[2]:.1f}m",
        )
        if self.manual_target is not None:
            self.ui.post_update("target", f"Target: manual {target[0]:.6f} {target[1]:.6f} {target[2]:.1f}m")
            if (self.follower.lat is not None and self.follower.lon is not None):
                rng = mp_util.gps_distance(self.follower.lat, self.follower.lon, target[0], target[1])
                self.ui.post_update("range", f"Range to manual: {rng:6.1f} m")
            self._update_map_target(target, None, manual=True)
        else:
            rng = mp_util.gps_distance(self.follower.lat, self.follower.lon,
                                       self.leader.lat, self.leader.lon)
            self.ui.post_update("range", f"Range: {rng:6.1f} m  Δt: {closing_time:5.1f} s")
            self.ui.post_update(
                "target",
                f"Target: predicted {target[0]:.6f} {target[1]:.6f} alt {target[2]:.1f}m",
            )
            self._update_map_target(target, closing_time, manual=False)

    def _process_ui_commands(self) -> None:
        for name, payload in self.ui.poll_commands():
            if name != "command":
                continue
            if payload == "catch" or payload == "resume":
                self.manual_target = None
                self.set_guidance_state("auto")
            elif payload == "hold":
                self.set_guidance_state("hold")
            elif payload == "clear":
                self.manual_target = None
                self.ui.post_update("target", "Target: ---")
                self._set_system_status("Awaiting intercept solution")
                self._clear_map_target()
            elif payload == "fbwa":
                self.set_follower_fbwa()
            elif payload and payload.startswith("select_leader:"):
                self._handle_ui_selection(payload[len("select_leader:"):], leader=True)
            elif payload and payload.startswith("select_follower:"):
                self._handle_ui_selection(payload[len("select_follower:"):], leader=False)
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
            self.last_warning = text

    def compute_intercept(self, now: float) -> Optional[Tuple[Tuple[float, float, float], float]]:
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

        follower_speed = max(self.catch_settings.follower_speed, 0.1)
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
        closing_rate = follower_speed - closing_projection
        if closing_rate <= 0.01:
            closing_rate = max(self.catch_settings.min_closing, 0.1)
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
        self._set_system_status(f"Intercepting leader — ETA {time_to_go:4.1f}s")
        return (predicted_lat, predicted_lon, target_alt), time_to_go

    def _send_target(self, target: Tuple[float, float, float]) -> None:
        lat, lon, alt = target
        frame = (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                 if self.catch_settings.use_relative_alt
                 else mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

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
                0.0,
                0.0,
                0.0,
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
            f"Sent intercept target {lat:.6f} {lon:.6f} alt {alt:.1f}m",
        )

    def _emit_vehicle_update(self, state: VehicleState) -> None:
        label = self._format_vehicle(state)
        if state is self.leader:
            self.ui.post_update("leader", label)
            self.ui.post_update("leader_selection", state.identifier())
        elif state is self.follower:
            self.ui.post_update("follower", label)
            self.ui.post_update("follower_selection", state.identifier())
        if state is self.follower and self.manual_target is None:
            now = self.get_time()
            if (now - self.last_range_status > 1.0
                    and self.leader.lat is not None and self.leader.lon is not None
                    and self.follower.lat is not None and self.follower.lon is not None):
                rng = mp_util.gps_distance(self.follower.lat, self.follower.lon,
                                           self.leader.lat, self.leader.lon)
                self.ui.post_update("range", f"Range: {rng:6.1f} m  Δt: ---")
                self.last_range_status = now

    def _emit_status(self) -> None:
        self.ui.post_update("leader", self._format_vehicle(self.leader))
        self.ui.post_update("leader_selection", self.leader.identifier())
        self.ui.post_update("follower", self._format_vehicle(self.follower))
        self.ui.post_update("follower_selection", self.follower.identifier())
        self.ui.post_update("status", "Guidance: HOLD")
        self.ui.post_update("warning", "Warnings: none")
        self._set_system_status("Awaiting intercept solution")

    def _format_vehicle(self, state: VehicleState) -> str:
        latlon = "---"
        if state.lat is not None and state.lon is not None:
            latlon = f"{state.lat:.6f} {state.lon:.6f}"
        alt = "---"
        if state.rel_alt is not None:
            alt = f"{state.rel_alt:.1f}m"
        speed = state.speed()
        speed_str = f"{speed:.1f}m/s" if speed is not None else "---"
        mode = state.mode
        armed = "ARMED" if state.armed else "DISARMED"
        return (f"Sys {state.sysid}:{state.compid} | {latlon} | alt {alt} | "
                f"spd {speed_str} | {mode} | {armed}")

    def status_report(self) -> str:
        now = self.get_time()
        leader_age = now - self.leader.last_update if self.leader.last_update else float("inf")
        follower_age = now - self.follower.last_update if self.follower.last_update else float("inf")
        parts = [
            f"Leader: {self._format_vehicle(self.leader)} (age {leader_age:.1f}s)",
            f"Follower: {self._format_vehicle(self.follower)} (age {follower_age:.1f}s)",
            f"Guidance state: {self.guidance_state.upper()}",
        ]
        if self.manual_target is not None:
            parts.append(
                "Manual target: lat {0:.6f} lon {1:.6f} alt {2:.1f}m".format(*self.manual_target)
            )
        if self.current_status:
            parts.append(f"System status: {self.current_status}")
        return "\n".join(parts)

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
        mpmap = self._map_instance()
        if not mpmap:
            return
        icon = self._ensure_map_icon()
        if icon is None:
            return
        lat, lon, alt = target
        label_parts = ["CatchLeader target"]
        if manual:
            label_parts.append("manual")
        else:
            label_parts.append("predictive")
        if closing_time is not None:
            label_parts.append(f"ETA {closing_time:0.1f}s")
        label_parts.append(f"alt {alt:.1f}m")
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
            return
        mpmap = self._map_instance()
        if not mpmap:
            self._map_target_added = False
            return
        mpmap.add_object(mp_slipmap.SlipRemoveObject(self._map_target_key))
        self._map_target_added = False


def init(mpstate):
    """Initialise module."""
    return CatchLeader(mpstate)
