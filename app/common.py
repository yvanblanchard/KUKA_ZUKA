"""Shared Streamlit building blocks of app/jog_app.py and app/pendant.py.

Connection (real / mock, network test), control & speed, guarded actions, non-blocking moves,
status polling with controller messages, message log, recording results.
Session-state keys used here: client, driver, info, control, status, prev, link_ok, msgs, speed,
kind, host, port, runner (KRL program, jog app), toasts (pendant: also show action results as toasts).
"""
from __future__ import annotations

import ipaddress
import socket
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import pandas as pd  # noqa: E402
import streamlit as st  # noqa: E402

from kuka_zuka import (Config, DriverError, KRLDriver, KVPClient, KVPConnectionError,  # noqa: E402
                       KVPError, SafetyLimitError)
from kuka_zuka import krltypes as kt  # noqa: E402
from kuka_zuka import monitor  # noqa: E402
from kuka_zuka.mock import MockKRC, MockKVPServer  # noqa: E402

CFG = Config.load()
S = CFG.safety
MAX_OV = int(S.max_override)
REAL, MOCK = "Real KR C4", "Mock (simulator)"
STEPS_DEG = [v for v in (0.1, 0.5, 1.0, 2.0, 5.0, 10.0) if v <= S.max_rel_deg] or [S.max_rel_deg]
STEPS_MM = [v for v in (0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0) if v <= S.max_rel_mm] or [S.max_rel_mm]
LEVEL_ICON = {"INFO": "ℹ️", "WARN": "⚠️", "ERROR": "❌"}

ss = st.session_state


class RobotBusy(DriverError):
    pass


def init_state(**extra) -> None:
    defaults = {"client": None, "driver": None, "info": {}, "control": False, "status": None, "prev": {},
                "link_ok": True, "msgs": deque(maxlen=500), "speed": min(5, MAX_OV)}
    for k, v in {**defaults, **extra}.items():
        ss.setdefault(k, v)


# ====================================================================== messages / actions
def log(level: str, text: str) -> None:
    ss.msgs.append((datetime.now().strftime("%H:%M:%S"), level, text))


def _report(level: str, text: str) -> None:
    log(level, text)
    if ss.get("toasts"):
        st.toast(f"{LEVEL_ICON[level]} {text}")


def guarded(label: str, fn, *args, **kwargs) -> tuple[bool, object]:
    """Run a robot action and report the outcome. Never lets an exception kill the app.
    Returns ``(ok, result)``."""
    try:
        res = fn(*args, **kwargs)
    except (SafetyLimitError, RobotBusy) as exc:
        _report("WARN", f"{label} refused: {exc}")
    except KVPConnectionError as exc:
        _report("ERROR", f"{label}: connection error: {exc}")
    except (DriverError, KVPError, ValueError) as exc:
        _report("ERROR", f"{label}: {exc}")
    else:
        _report("INFO", label)
        return True, res
    return False, None


def program_running() -> bool:
    r = ss.get("runner")
    return r is not None and r.running


def start_move(label: str, fn, *args) -> bool:
    """Send a motion without blocking the UI; refuse (instead of queueing) while the robot moves."""
    def run():
        if program_running():
            raise RobotBusy("a KRL program is running")
        if ss.driver.is_busy():
            raise RobotBusy("the robot is still executing the previous move")
        fn(*args, wait=False)
    return guarded(label, run)[0]


# ====================================================================== state queries
def connected() -> bool:
    return ss.client is not None


def driver_running() -> bool:
    return bool(ss.status and ss.status["running"])


def motion_enabled() -> bool:
    return connected() and ss.control and driver_running() and not program_running()


def why_no_motion() -> str:
    if not ss.control:
        return "Motion disabled: this PC is not in control (sidebar → Take control)."
    if not ss.status:
        return "Motion disabled: no status from the controller."
    if program_running():
        return "Motion disabled: a KRL program is running."
    return ("Motion disabled: KRLDRIVER is not running. On the smartPAD: select KRLDRIVER, AUT mode, "
            "drives ON, start. It needs the arm connected.")


# ====================================================================== mock / network
@st.cache_resource
def get_mock() -> MockKVPServer:
    return MockKVPServer(port=0, krc=MockKRC(drives_available=True)).start()


def local_ipv4s() -> list[str]:
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ips)


def test_network() -> None:
    host, port = ss.host.strip(), int(ss.port)
    ips = local_ipv4s()
    _report("INFO", f"Network test → {host}:{port}. PC IPv4 addresses: {', '.join(ips) or 'none found'}")
    try:
        net = ipaddress.ip_network(f"{host}/16", strict=False)   # KLI default mask 255.255.0.0
        if not any(ipaddress.ip_address(ip) in net for ip in ips):
            last = 10 if not host.endswith(".10") else 11
            example = ".".join(host.split(".")[:3] + [str(last)])
            _report("WARN", f"No PC address in {net}: give the Ethernet port wired to the controller's "
                            f"KLI (X66) a static IP, e.g. {example} / 255.255.0.0")
    except ValueError:
        _report("WARN", f"'{host}' is not an IPv4 address")
    t0 = time.perf_counter()
    try:
        with socket.create_connection((host, port), timeout=2.0):
            pass
        _report("INFO", f"TCP {host}:{port} reachable in {(time.perf_counter() - t0) * 1000:.0f} ms: "
                        "KUKAVARPROXY is listening")
    except OSError as exc:
        _report("ERROR", f"TCP {host}:{port} unreachable ({exc}). Check: cable on X66 (KLI), PC IP in the "
                         "same subnet, KUKAVARPROXY.exe running on the controller, port 7000 open (KRC network "
                         "config → NAT).")


# ====================================================================== connection / control
def connection_form() -> None:
    """Sidebar inputs: controller kind, IP/port + network test (real) or arm simulation (mock)."""
    conn = connected()
    kind = st.radio("Controller", [REAL, MOCK], key="kind", disabled=conn)
    if kind == REAL:
        st.text_input("Robot IP (KLI port X66)", CFG.connection.host, key="host", disabled=conn)
        st.number_input("Port (KUKAVARPROXY)", 1, 65535, CFG.connection.port, key="port", disabled=conn)
        st.button("🩺 Test network", on_click=test_network, disabled=conn, width="stretch")
    else:
        srv = get_mock()
        ss.setdefault("mock_arm", srv.krc.drives_available)     # keyed: identity must not follow the state
        arm = st.checkbox("Simulate arm connected (KRLDRIVER running)", key="mock_arm",
                          help="Untick to mimic the bare KR C4 without the manipulator.")
        if arm != srv.krc.drives_available:
            srv.krc.set_drives(arm)


def connect() -> bool:
    if ss.kind == REAL:
        host, port, label = ss.host.strip(), int(ss.port), "real KR C4"
    else:
        srv = get_mock()
        host, port, label = srv.host, srv.port, "mock controller"
    try:
        client = KVPClient(host, port, timeout=CFG.connection.timeout_s).connect()
    except KVPError as exc:
        _report("ERROR", str(exc))
        return False
    ss.client, ss.driver = client, KRLDriver(client, CFG)
    ss.prev, ss.status, ss.link_ok = {}, None, True
    ss.info = monitor.read_info(client)
    details = ", ".join(f"{monitor.INFO_VARS[k]}: {v}" for k, v in ss.info.items() if v)
    log("INFO", f"Connected to {label} at {host}:{port}" + (f" ({details})" if details else ""))
    return True


def disconnect() -> None:
    if program_running():
        ss.runner.stop()
    if ss.control:
        try:
            ss.driver.release_control()
        except KVPError:
            pass
    ss.client.close()
    ss.client, ss.driver, ss.control, ss.status, ss.info = None, None, False, None, {}
    log("INFO", "Disconnected")


def apply_speed_cap() -> None:
    """Sync the speed slider with $OV_PRO and enforce the PC cap when in control."""
    ov = kt.try_int(ss.client.try_read("$OV_PRO"))
    if ov is None:
        return
    if ov > MAX_OV and ss.control:
        if guarded(f"Override lowered from {ov} % to the PC cap {MAX_OV} %", ss.driver.set_override, MAX_OV)[0]:
            ov = MAX_OV
    ss.speed = min(ov, MAX_OV)


def take_control() -> None:
    try:
        ss.driver.take_control()
        ss.control = True
        log("INFO", f"Control taken (PRO_ID = {ss.driver.pro_id}): jog and motion enabled")
    except (DriverError, KVPError) as exc:
        ss.control = False
        _report("WARN", f"Monitor only, motion disabled: {exc}")
    apply_speed_cap()


def release_control() -> None:
    if program_running():
        ss.runner.stop()
    guarded("Control released", ss.driver.release_control)
    ss.control = False


def set_speed() -> None:
    guarded(f"Speed (override) set to {ss.speed} %", ss.driver.set_override, ss.speed)


def soft_stop() -> None:
    if guarded("SOFT STOP: override set to 0 % (motion paused, not a safety function)", ss.driver.soft_stop)[0]:
        ss.speed = 0


def _connect_then(after_connect) -> None:
    if connect() and after_connect:
        after_connect()


def control_sidebar(after_connect=None) -> None:
    """Connect / disconnect, take / release control, speed slider and soft stop.
    ``after_connect`` is called once a connection is established."""
    conn = connected()
    c1, c2 = st.columns(2)
    c1.button("Connect", on_click=_connect_then, args=(after_connect,), disabled=conn, type="primary",
              width="stretch")
    c2.button("Disconnect", on_click=disconnect, disabled=not conn, width="stretch")
    if not conn:
        return
    st.caption(f"latency {ss.client.last_latency_ms:.1f} ms")
    if ss.control:
        st.button("🔒 Release control", on_click=release_control, width="stretch")
    else:
        st.button("🔓 Take control", on_click=take_control, width="stretch")
    st.header("🐢 Speed")
    st.slider("Override $OV_PRO [%]", 0, MAX_OV, key="speed", on_change=set_speed,
              disabled=not ss.control, help=f"Scales every motion. PC cap: {MAX_OV} % (config.yaml).")
    st.button("⏸ SOFT STOP (override 0 %)", on_click=soft_stop, type="primary",
              disabled=not ss.control, width="stretch")
    st.caption("Soft stop only pauses motion. It is **not** a safety function. Use the E-STOP.")


# ====================================================================== status polling
def poll_status() -> dict | None:
    """Read the controller status, log controller events, detect loss of control. Sets ``ss.status``."""
    c = ss.client
    try:
        stt = monitor.read_status(c)
        raw = stt["raw"]
        raw["KRLD_STATE"] = c.try_read("KRLD_STATE")
        raw["PRO_ID"] = c.try_read("PRO_ID")
    except KVPConnectionError as exc:
        if ss.link_ok:
            log("ERROR", f"Connection lost: {exc}")
            ss.link_ok = False
        ss.status = None
        return None
    if not ss.link_ok:
        log("INFO", "Connection restored")
        ss.link_ok = True
    for lvl, msg in monitor.controller_events(ss.prev, raw, MAX_OV):
        log(lvl, msg)
    ss.prev = dict(raw)
    if ss.control and ss.driver.pro_id is not None:
        pid = kt.try_int(raw.get("PRO_ID"))
        if pid is not None and pid != ss.driver.pro_id:
            log("WARN", f"Another client took control of the robot (PRO_ID = {pid}). Motion disabled.")
            ss.control, ss.driver.pro_id = False, None
    stt["running"] = monitor.krldriver_running(raw)
    stt["krld_state"] = kt.try_int(raw.get("KRLD_STATE"))
    ss.status = stt
    return stt


def status_metrics(stt: dict) -> None:
    raw = stt["raw"]
    alarm, saf = kt.try_bool(raw.get("$ALARM_STOP")), kt.try_bool(raw.get("$USER_SAF"))
    safety = "E-STOP" if alarm is False else "guard open" if saf is False else "OK"
    drives = {True: "ON", False: "OFF", None: "–"}[kt.try_bool(raw.get("$PERI_RDY"))]
    if not stt["running"]:
        krld = "not running"
    else:
        krld = {0: "idle", 1: "moving", 2: "error"}.get(stt["krld_state"], "?")
    c = st.columns(7)
    c[0].metric("Mode", kt.parse_enum(raw.get("$MODE_OP") or "–"))
    c[1].metric("Program", kt.parse_str(raw.get("$PRO_NAME1[]") or "") or "–")
    c[2].metric("KRLDRIVER", krld)
    c[3].metric("Drives", drives)
    c[4].metric("Safety", safety)
    c[5].metric("Override", f"{raw.get('$OV_PRO') or '–'} %")
    c[6].metric("Control", "this PC" if ss.control else "monitor only")


def messages_panel(height: int = 300) -> None:
    cols = st.columns([1, 1, 1, 1, 3])
    show = {lvl for lvl, col in zip(LEVEL_ICON, cols) if col.checkbox(lvl, True, key=f"show_{lvl}")}
    cols[3].button("Clear", on_click=ss.msgs.clear)
    n_err = sum(1 for _, lvl, _ in ss.msgs if lvl == "ERROR")
    n_warn = sum(1 for _, lvl, _ in ss.msgs if lvl == "WARN")
    cols[4].caption(f"{n_err} error(s), {n_warn} warning(s). Newest first.")
    lines = [f"{t}  {LEVEL_ICON[lvl]} {lvl:<5}  {msg}" for t, lvl, msg in reversed(ss.msgs) if lvl in show]
    st.code("\n".join(lines) or "(no messages)", language=None, height=height, wrap_lines=True)


# ====================================================================== recording results
def recording_results(rec, smooth: float, key: str = "rec") -> None:
    """Summary table, charts and CSV downloads of a finished :class:`~kuka_zuka.recorder.Recorder`."""
    try:
        joints, tcp, summary = rec.process(smooth_s=smooth)
    except ValueError as exc:
        st.error(str(exc))
        return
    ax_rows = [{"Axis": k, "min [°]": v["min_deg"], "max [°]": v["max_deg"],
                "max |vel| [°/s]": v["max_abs_vel_deg_s"], "max |acc| [°/s²]": v["max_abs_acc_deg_s2"]}
               for k, v in summary.get("axes", {}).items()]
    left, right = st.columns([3, 2])
    left.dataframe(pd.DataFrame(ax_rows).round(2), hide_index=True, width="stretch")
    info = {"duration [s]": summary.get("duration_s"), **{f"TCP {k}": v for k, v in summary.get("tcp", {}).items()},
            **{f"sampling {k}": v for k, v in summary.get("sampling", {}).items()}}
    right.dataframe(pd.DataFrame({"": list(info), "value": [round(v, 3) if isinstance(v, float) else v
                                                            for v in info.values()]}),
                    hide_index=True, width="stretch")

    axes = list(kt.AXES)
    groups = {"Joint angles [°]": (joints, axes),
              "Joint velocities [°/s]": (joints, [f"{k}_vel" for k in axes]),
              "Joint accelerations [°/s²]": (joints, [f"{k}_acc" for k in axes])}
    if tcp is not None:
        groups.update({"TCP position [mm]": (tcp, ["X", "Y", "Z"]),
                       "TCP speed [mm/s]": (tcp, ["tcp_speed", "vx", "vy", "vz"]),
                       "TCP acceleration [mm/s²]": (tcp, ["tcp_acc", "tcp_acc_tangential"]),
                       "TCP angular speed / acc [°/s, °/s²]": (tcp, ["tcp_ang_speed", "tcp_ang_acc"])})
    sel = st.multiselect("Charts", list(groups), ["Joint angles [°]", "Joint velocities [°/s]"]
                         + (["TCP speed [mm/s]"] if tcp is not None else []), key=f"{key}_charts")
    for g in sel:
        df_g, cols = groups[g]
        st.markdown(f"**{g}**, x: time [s]")
        st.line_chart(df_g.set_index("t")[cols], height=260)

    d = st.columns(4)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    d[0].download_button("Raw CSV", rec.raw_frame().to_csv(index=False), f"rec_{stamp}_raw.csv", key=f"{key}_raw")
    d[1].download_button("Joints CSV", joints.to_csv(index=False), f"rec_{stamp}_joints.csv", key=f"{key}_j")
    if tcp is not None:
        d[2].download_button("TCP CSV", tcp.to_csv(index=False), f"rec_{stamp}_tcp.csv", key=f"{key}_tcp")
    if d[3].button("💾 Save to recordings/", key=f"{key}_save"):
        paths = rec.save(ROOT / "recordings" / f"rec_{stamp}", smooth_s=smooth)
        st.success("Saved: " + ", ".join(p.name for p in paths))
