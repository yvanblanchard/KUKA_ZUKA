"""Simple KUKA KR C4 jog app (Streamlit).

    .venv\\Scripts\\streamlit run app/jog_app.py

* connect to the robot (with a network check)
* jog each joint A1..A6 with - / + buttons, step size + speed (override) control
* define a Cartesian target and move to it (PTP or LIN)
* paste a KRL program: run it through KRLDRIVER while recording the joints, or export it as .src
  and record its native run
* message log of controller infos / warnings / errors

Motion goes through KRLDRIVER (must be running on the controller, AUT mode, drives on).
NOT a safety device: keep the physical E-STOP within reach.
"""
from __future__ import annotations

import math

import streamlit as st

import common as cm
from common import CFG, MAX_OV, S, ss
from kuka_zuka import krlprog
from kuka_zuka import krltypes as kt
from kuka_zuka.recorder import Recorder

st.set_page_config(page_title="KUKA jog", page_icon="🦾", layout="wide")
REFRESH_S = 0.5
KRL_EXAMPLE = """\
; One instruction per line. Supported: PTP / LIN / CIRC / PTP_REL / LIN_REL, PTP HOME,
; DECL of points, $VEL.CP, BAS(#VEL_PTP,..), $OV_PRO, $APO, $BASE, $TOOL, $OUT[n], WAIT SEC
DECL E6AXIS P_START={A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}
BAS(#VEL_PTP, 10)
PTP P_START
PTP {A1 20, A2 -80, A3 100}
PTP_REL {A6 10}
WAIT SEC 0.5
PTP {A1 -20}
PTP P_START
"""

cm.init_state(step=1.0 if 1.0 in cm.STEPS_DEG else cm.STEPS_DEG[0], mtype="PTP", tgt_ok=False,
              **{f"tgt_{k}": 0.0 for k in kt.POSE},
              krl_code=KRL_EXAMPLE, krl_repeat=1, krl_record=True, krl_ok=False, krl_name="PCPROG",
              runner=None, rec=None, krl_watch=False, runner_log_idx=0)


# ====================================================================== motion
def jog(axis: int, sign: int) -> None:
    d = [0.0] * 6
    d[axis] = sign * float(ss.step)
    cm.start_move(f"Jog A{axis + 1} {d[axis]:+g}°", ss.driver.ptp_rel_axis, d)


def fill_target_from_current() -> None:
    try:
        pose = kt.pose_of(ss.client.read("$POS_ACT"))
    except (cm.KVPError, ValueError) as exc:
        cm.log("WARN", f"Could not read the current pose: {exc}")
        return
    for k, v in zip(kt.POSE, pose):
        ss[f"tgt_{k}"] = round(v, 2)


def go_target() -> None:
    pose = [float(ss[f"tgt_{k}"]) for k in kt.POSE]
    fn = ss.driver.lin if ss.mtype == "LIN" else ss.driver.ptp_pos
    txt = ", ".join(f"{k} {v:g}" for k, v in zip(kt.POSE, pose))
    cm.start_move(f"{ss.mtype} to target ({txt})", fn, pose)
    ss.tgt_ok = False


def after_connect() -> None:
    cm.take_control()
    fill_target_from_current()


# ====================================================================== KRL program
def recording() -> bool:
    return ss.rec is not None and ss.rec.running


def new_recorder(**kwargs) -> Recorder:
    return Recorder(ss.client.host, ss.client.port, period_s=0.01, timeout_s=CFG.connection.timeout_s, **kwargs)


def run_program() -> None:
    def start():
        prog = krlprog.parse(ss.krl_code, CFG)
        if not prog.ok or not prog.steps:
            raise ValueError("the program has errors or no instruction")
        if ss.driver.is_busy():
            raise cm.RobotBusy("the robot is still executing the previous move")
        rec = new_recorder() if ss.krl_record else None
        if rec is not None:
            ss.rec = rec
        ss.runner = krlprog.ProgramRunner(ss.driver, prog, ss.krl_repeat, rec).start()
        ss.runner_log_idx, ss.krl_watch = 0, True
    n = ss.krl_repeat
    cm.guarded("KRL program started" + (f" ({n} cycles)" if n > 1 else "")
               + (", recording joints" if ss.krl_record else ""), start)
    ss.krl_ok = False


def stop_program() -> None:
    ss.runner.stop()
    cm.log("WARN", "KRL program: stop requested, it ends after the current instruction "
                   "(use SOFT STOP to pause the current motion)")


def record_native() -> None:
    name = krlprog.program_name(ss.krl_name)
    ss.rec = new_recorder(wait_program_start=True, stop_on_program_end=True, program_name=name).start()
    ss.krl_watch = True
    cm.log("INFO", f"Recorder armed: waiting for program {name} to start on the smartPAD")


def stop_recording() -> None:
    ss.rec.stop()
    cm.log("INFO", f"Recording stopped: {ss.rec.n_samples} samples")


# ====================================================================== live panels
@st.fragment(run_every=REFRESH_S)
def status_panel() -> None:
    stt = cm.poll_status()
    if stt is None:
        st.error("No answer from the controller. Retrying…")
        return
    cm.status_metrics(stt)


@st.fragment(run_every=REFRESH_S)
def jog_panel() -> None:
    axes = ss.status["axes"] if ss.status else None
    ok = cm.motion_enabled()
    m = S.axis_margin_deg
    for i in range(6):
        lo, hi = S.axis_limits[i]
        c0, c1, c2, c3 = st.columns([1, 3, 1.2, 1.2], vertical_alignment="center")
        c0.markdown(f"**A{i + 1}**")
        val = f"{axes[i]:8.2f}°" if axes else "—"
        c1.markdown(f"`{val}` <small>limits [{lo + m:g}, {hi - m:g}]</small>", unsafe_allow_html=True)
        c2.button("➖", key=f"jog_m{i}", on_click=jog, args=(i, -1), disabled=not ok, width="stretch")
        c3.button("➕", key=f"jog_p{i}", on_click=jog, args=(i, +1), disabled=not ok, width="stretch")
    if not ok:
        st.caption(cm.why_no_motion())


@st.fragment(run_every=REFRESH_S)
def current_pose() -> None:
    pose = ss.status["pose"] if ss.status else None
    if pose:
        st.code("  ".join(f"{k} {v:9.2f}" for k, v in zip(kt.POSE, pose)), language=None)


@st.fragment(run_every=REFRESH_S)
def program_status() -> None:
    r, rec = ss.runner, ss.rec
    if r is not None:
        for lvl, msg in r.events[ss.runner_log_idx:]:
            cm.log(lvl, msg)
        ss.runner_log_idx = len(r.events)
        st.progress(r.n_done / r.total if r.total else 0.0,
                    text=f"Program: {r.status} · {r.n_done}/{r.total} instruction(s)"
                         + (f" · line {r.current.line}: `{r.current.text}`" if r.current else ""))
        if r.error:
            st.error(r.error)
    if rec is not None:
        st.caption(f"Recorder: {rec.status} · {rec.n_samples} samples · {rec.elapsed_s:.1f} s · "
                   f"{rec.rate_hz:.0f} Hz" + (f" · {rec.error}" if rec.error else ""))
    if ss.krl_watch and not cm.program_running() and not recording():
        ss.krl_watch = False
        st.rerun()                       # run / recording ended -> refresh buttons and results


def krl_panel() -> None:
    st.caption("KUKAVARPROXY can't copy a .src to the controller nor start a program. **Run on robot** "
               "interprets the code here and sends it instruction by instruction to KRLDRIVER (PC caps "
               "apply), while recording the joints over a second connection.")
    st.text_area("KRL instructions", key="krl_code", height=260)
    prog = krlprog.parse(ss.krl_code, CFG)
    for line, msg in prog.errors:
        st.error(f"Line {line}: {msg}")
    for line, msg in prog.warnings:
        st.warning(f"Line {line}: {msg}")
    with st.expander(f"Parsed: {len(prog.steps)} instruction(s), {prog.n_motions} motion(s)"):
        st.dataframe([{"line": s.line, "instruction": s.text, "action": krlprog.OP_LABELS[s.op]}
                      for s in prog.steps], hide_index=True, width="stretch")
    busy = cm.program_running() or recording()
    c = st.columns([1, 1, 2])
    c[0].number_input("Repeat", 1, 100, key="krl_repeat", disabled=busy)
    c[1].checkbox("Record joints", key="krl_record", disabled=busy)
    c[2].checkbox("I checked every target is reachable and the path is clear", key="krl_ok", disabled=busy)
    b1, b2 = st.columns(2)
    b1.button("▶ Run on robot", key="krl_run", on_click=run_program, type="primary", width="stretch",
              disabled=not (cm.motion_enabled() and prog.ok and prog.steps and ss.krl_ok and not recording()))
    b2.button("⏹ Stop after current instruction", key="krl_stop", on_click=stop_program, width="stretch",
              disabled=not cm.program_running())
    if not cm.motion_enabled() and not cm.program_running():
        st.caption(cm.why_no_motion())
    program_status()

    with st.expander("Run it natively on the controller instead (.src export)"):
        st.markdown("1. Download the files, copy them to `KRC:\\R1\\Program` (USB stick in the navigator, or "
                    "WorkVisual).  \n2. Arm the recorder below.  \n3. On the smartPAD: deselect KRLDRIVER, "
                    "select the program, BCO run in T1, then start it. The recording stops when the program "
                    "ends.  \n⚠️ PC-side caps don't apply to a native run, and the full KRL language is "
                    "available (loops, calls...).")
        st.text_input("Program name", key="krl_name", max_chars=24, disabled=busy)
        name = krlprog.program_name(ss.krl_name)
        src, dat = krlprog.to_src(ss.krl_code, name)
        d = st.columns(4)
        d[0].download_button(f"{name}.src", src, f"{name.lower()}.src", width="stretch")
        if dat:
            d[1].download_button(f"{name}.dat", dat, f"{name.lower()}.dat", width="stretch")
        d[2].button("⏺ Record next run", key="krl_rec_native", on_click=record_native, width="stretch",
                    disabled=not cm.connected() or busy)
        d[3].button("⏹ Stop recording", key="krl_rec_stop", on_click=stop_recording, width="stretch",
                    disabled=not recording() or cm.program_running())
        st.code(src, language=None)

    rec = ss.rec
    if rec is not None and not rec.running and rec.n_samples >= 3:
        st.subheader("Recorded joints")
        smooth = st.number_input("Smoothing window [s]", 0.02, 2.0, 0.1, 0.02, key="krl_smooth")
        cm.recording_results(rec, smooth, key="krl")


# ====================================================================== sidebar
with st.sidebar:
    st.header("🔌 Connection")
    cm.connection_form()
    cm.control_sidebar(after_connect=after_connect)
    if cm.connected():
        st.select_slider("Joint jog step [°]", options=cm.STEPS_DEG, key="step")
    st.divider()
    st.caption(f"{CFG.robot.model} · override ≤ {MAX_OV} % · step ≤ {S.max_rel_deg:g}°")

# ====================================================================== main
st.title("🦾 KUKA KR C4 jog")
if not cm.connected():
    st.info("Connect to the robot in the sidebar. Use **Test network** first if the connection fails, "
            "or **Mock** to try the app without hardware.")
    cm.messages_panel()
    st.stop()

status_panel()
tab_jog, tab_krl = st.tabs(["🕹️ Jog & target", "📜 KRL program"])
with tab_jog:
    left, right = st.columns(2, gap="large")
    with left:
        st.subheader("Joint jog")
        jog_panel()

    with right:
        st.subheader("Cartesian target")
        st.caption("TCP pose in the current $BASE / $TOOL [mm, °]. Turn (S, T) is kept from the current pose. "
                   "Current pose:")
        current_pose()
        cols = st.columns(6)
        for k, col in zip(kt.POSE, cols):
            col.number_input(f"{k} [{'mm' if k in 'XYZ' else '°'}]", key=f"tgt_{k}", step=1.0, format="%.2f")
        c1, c2 = st.columns(2)
        c1.button("📍 Use current pose", on_click=fill_target_from_current, width="stretch")
        c2.radio("Motion", ["PTP", "LIN"], key="mtype", horizontal=True,
                 help="PTP: fastest joint path (path not a straight line). LIN: straight TCP line.")
        pose = ss.status["pose"] if ss.status else None
        if pose:
            dist = math.dist(pose[:3], [ss[f"tgt_{k}"] for k in "XYZ"])
            rot = max(abs(ss[f"tgt_{k}"] - v) for k, v in zip("ABC", pose[3:]))
            msg = f"Distance to target: **{dist:.1f} mm**, max rotation change **{rot:.1f}°**"
            (st.warning if dist > 300 or rot > 45 else st.caption)(msg)
        st.checkbox("I checked the target is reachable and the path is clear", key="tgt_ok")
        st.button("▶ Move to target", key="go_target", on_click=go_target, type="primary", width="stretch",
                  disabled=not (cm.motion_enabled() and ss.tgt_ok))

with tab_krl:
    krl_panel()

st.subheader("Messages")
st.caption("Built from controller status variables (E-STOP, guard, drives, stop message, mode, program, "
           "KRLDRIVER) and from this app's commands. The smartPAD message texts can't be read through "
           "KUKAVARPROXY.")
cm.messages_panel()
