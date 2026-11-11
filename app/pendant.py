"""KUKA KR C4 virtual pendant (Streamlit).

    .venv\\Scripts\\streamlit run app/pendant.py

NOT a safety device: the physical E-STOP and the controller safety remain authoritative.
"""
from __future__ import annotations

import time
from datetime import datetime

import pandas as pd
import streamlit as st

import common as cm
from common import CFG, S, ss
from kuka_zuka import KRLDriver, KVPError
from kuka_zuka import krltypes as kt
from kuka_zuka import monitor
from kuka_zuka.recorder import Recorder

st.set_page_config(page_title="KUKA virtual pendant", page_icon="🦾", layout="wide")
cm.init_state(toasts=True, watch="$OV_PRO\n$AXIS_ACT\nKRLD_STATE\nKRLD_DONE")


# ============================================================== helpers
def do(label: str, fn, *args, **kwargs):
    """Guarded action (logged + toast); returns the result, None on failure."""
    return cm.guarded(label, fn, *args, **kwargs)[1]


def drv() -> KRLDriver:
    return ss.driver


def fmt_bool(v):
    if v is None:
        return "–"
    return "🟢" if str(v).upper() == "TRUE" else "⚪"


# ============================================================== sidebar
with st.sidebar:
    st.header("🦾 Connection")
    cm.connection_form()
    cm.control_sidebar()                      # monitor-only after connecting: control is explicit
    if cm.connected():
        refresh = st.slider("Live refresh [s]", 0.2, 5.0, 0.5, 0.1)
        live_on = st.checkbox("Live update", True)
    st.divider()
    st.caption(f"Model: {CFG.robot.model} · caps: OV ≤ {S.max_override}% · "
               f"VEL_CP ≤ {S.max_vel_cp} m/s · VEL_PTP ≤ {S.max_vel_ptp}%")

st.title("KUKA KR C4 — virtual pendant")
if not cm.connected():
    st.info("Connect to a controller (sidebar). Use **Test network** first if the connection fails, "
            "or **Mock** to try the pendant without hardware.")
    cm.messages_panel()
    st.stop()

if ss.info:
    st.caption(" · ".join(f"**{monitor.INFO_VARS[k]}**: {v}" for k, v in ss.info.items() if v))


# ============================================================== live status
def live_status():
    stt = cm.poll_status()
    if stt is None:
        st.error("No answer from the controller. Retrying…")
        return
    cm.status_metrics(stt)
    if kt.try_bool(stt["raw"].get("$STOPMESS")):
        st.warning("Stop message active on the controller — acknowledge it on the smartPAD.")

    a, b = st.columns(2)
    if stt["axes"]:
        lim = S.axis_limits
        df = pd.DataFrame({"Axis": kt.AXES, "Angle [°]": stt["axes"],
                           "Min": [l[0] for l in lim], "Max": [l[1] for l in lim]})
        df["Range"] = [(v - lo) / (hi - lo) for v, (lo, hi) in zip(stt["axes"], lim)]
        a.dataframe(df, hide_index=True, width="stretch", column_config={
            "Angle [°]": st.column_config.NumberColumn(format="%.2f"),
            "Range": st.column_config.ProgressColumn(min_value=0.0, max_value=1.0, format=" ")})
    if stt["pose"]:
        names = ["X [mm]", "Y [mm]", "Z [mm]", "A [°]", "B [°]", "C [°]"]
        df = pd.DataFrame({"": names, "TCP ($POS_ACT)": stt["pose"],
                           "$BASE": stt["base"] or [None] * 6, "$TOOL": stt["tool"] or [None] * 6})
        b.dataframe(df, hide_index=True, width="stretch",
                    column_config={c: st.column_config.NumberColumn(format="%.2f")
                                   for c in ("TCP ($POS_ACT)", "$BASE", "$TOOL")})


st.fragment(run_every=refresh if live_on else None)(live_status)()

ctrl = ss.control
motion_ok = cm.motion_enabled()          # control + KRLDRIVER running: needed for every KRLDRIVER command
if not ctrl:
    st.info("Monitor-only mode. **Take control** in the sidebar to send commands.")
elif not motion_ok:
    st.caption(cm.why_no_motion())

tabs = st.tabs(["🕹️ Jog", "🎯 Motion", "⚙️ Settings", "🔌 I/O", "🔎 Variables", "📈 Recorder", "📜 Log"])

# ============================================================== jog
with tabs[0]:
    c = st.columns([2, 2, 3])
    mode = c[0].radio("Jog mode", ["Axis (PTP_REL)", "Cartesian LIN_REL", "Cartesian PTP_REL"])
    if mode.startswith("Axis"):
        steps = cm.STEPS_DEG
        step = c[1].select_slider("Step [°]", steps, value=1.0 if 1.0 in steps else steps[0])
    else:
        steps = cm.STEPS_MM
        step = c[1].select_slider("Step [mm] (A/B/C in °)", steps, value=1.0 if 1.0 in steps else steps[0])
    c[2].button("🏠 Home " + str(CFG.robot.home_axes), disabled=not motion_ok,
                on_click=cm.start_move, args=("Home", drv().home))
    c[2].caption("Step-jog: each click = one relative move (exact stop), refused while the robot still moves. "
                 "Hold-to-jog is not possible over KUKAVARPROXY. Speed: override slider in the sidebar.")

    labels = kt.AXES if mode.startswith("Axis") else kt.POSE
    grid = st.columns(6)
    for i, lab in enumerate(labels):
        with grid[i]:
            st.markdown(f"**{lab}**")
            for sign, txt in ((-1, "➖"), (+1, "➕")):
                if mode.startswith("Axis"):
                    args = (f"A{i + 1} {sign * step:+g}°", drv().jog_axis, i + 1, sign * step)
                else:
                    stp = min(step, S.max_rel_rot_deg) if lab in "ABC" else step
                    args = (f"{lab} {sign * stp:+g}", drv().jog_cart, lab, sign * stp, mode.endswith("LIN_REL"))
                st.button(txt, key=f"jog_{lab}_{sign}", disabled=not motion_ok, width="stretch",
                          on_click=cm.start_move, args=args)

# ============================================================== motion
with tabs[1]:
    st.caption("Moves are sent without blocking the page; a new move is refused while the previous one runs.")

    def fill(prefix, values):
        for k, v in zip(kt.AXES if prefix == "ax" else kt.POSE, values):
            ss[f"{prefix}_{k}"] = round(float(v), 3)

    for i, k in enumerate(kt.AXES):
        ss.setdefault(f"ax_{k}", float(CFG.robot.home_axes[i]))
    for k in kt.POSE:
        ss.setdefault(f"ps_{k}", 0.0)
        ss.setdefault(f"aux_{k}", 0.0)

    st.subheader("PTP — axis target")
    cols = st.columns(7)
    for i, k in enumerate(kt.AXES):
        cols[i].number_input(k, -360.0, 360.0, step=1.0, key=f"ax_{k}")
    cols[6].button("⟲ current", key="fill_ax", help="Load current $AXIS_ACT",
                   on_click=lambda: do("read axes", lambda: fill("ax", drv().read_axes())))
    st.button("Send PTP axis", disabled=not motion_ok, type="primary",
              on_click=lambda: cm.start_move("PTP axis", drv().ptp_axis, [ss[f"ax_{k}"] for k in kt.AXES]))

    st.subheader("Cartesian target (current $BASE / $TOOL)")
    cols = st.columns(7)
    for i, k in enumerate(kt.POSE):
        cols[i].number_input(k, -5000.0, 5000.0, step=1.0, key=f"ps_{k}")
    cols[6].button("⟲ current", key="fill_ps", help="Load current $POS_ACT",
                   on_click=lambda: do("read pose", lambda: fill("ps", kt.pose_of(drv().read_pose()))))
    pose = lambda: [ss[f"ps_{k}"] for k in kt.POSE]  # noqa: E731
    b = st.columns(4)
    b[0].button("Send PTP pos", disabled=not motion_ok,
                on_click=lambda: cm.start_move("PTP pos", drv().ptp_pos, pose()))
    b[1].button("Send LIN", disabled=not motion_ok, on_click=lambda: cm.start_move("LIN", drv().lin, pose()))

    with st.expander("CIRC (auxiliary point → target above)"):
        cols = st.columns(6)
        for i, k in enumerate(kt.POSE):
            cols[i].number_input(f"aux {k}", -5000.0, 5000.0, step=1.0, key=f"aux_{k}")
        st.button("Send CIRC", disabled=not motion_ok,
                  on_click=lambda: cm.start_move("CIRC", drv().circ, [ss[f"aux_{k}"] for k in kt.POSE], pose()))

    st.subheader("Misc")
    b = st.columns(4)
    secs = b[0].number_input("WAIT SEC", 0.0, 60.0, 1.0, 0.5)
    b[1].button("Send WAIT", disabled=not motion_ok, on_click=lambda: do("WAIT", drv().wait_sec, secs))
    b[2].button("Handshake test (NOOP)", disabled=not motion_ok, on_click=lambda: do("NOOP", drv().noop))
    if b[3].button("Diagnose KRLDRIVER"):
        try:
            st.json({"globals_present": drv().globals_present(), "driver_running": drv().driver_running(),
                     **drv().driver_state()})
        except KVPError as exc:
            st.error(str(exc))

# ============================================================== settings
with tabs[2]:
    st.caption("Sent through KRLDRIVER: needs control and KRLDRIVER running.")
    l, r = st.columns(2)
    with l:
        st.subheader("BASE")
        src = st.radio("Source", ["Explicit frame", "BASE_DATA[n]"], horizontal=True, key="base_src")
        if src == "Explicit frame":
            fr = [st.number_input(f"base {k}", value=0.0, key=f"b_{k}") for k in kt.POSE]
            n = None
        else:
            n, fr = st.number_input("n", 1, 32, 1, key="base_n"), None
        off = [st.number_input(f"offset {k}", value=0.0, key=f"bo_{k}") for k in kt.POSE] \
            if st.checkbox("Offset (BASE:OFFSET)") else None
        st.button("Set BASE", disabled=not motion_ok, on_click=lambda: do("BASE", drv().set_base, fr, off, n))
    with r:
        st.subheader("TOOL")
        src = st.radio("Source", ["Explicit frame", "TOOL_DATA[n]"], horizontal=True, key="tool_src")
        if src == "Explicit frame":
            tf = [st.number_input(f"tool {k}", value=0.0, key=f"t_{k}") for k in kt.POSE]
            tn = None
        else:
            tn, tf = st.number_input("n", 1, 16, 1, key="tool_n"), None
        st.button("Set TOOL", disabled=not motion_ok, on_click=lambda: do("TOOL", drv().set_tool, tf, tn))
        st.subheader("Velocity / approximation")
        vcp = st.number_input("VEL_CP [m/s]", 0.001, float(S.max_vel_cp), min(0.05, S.max_vel_cp), 0.01, format="%.3f")
        st.button("Set VEL_CP", disabled=not motion_ok, on_click=lambda: do("VEL_CP", drv().set_vel_cp, vcp))
        vptp = st.number_input("VEL_PTP [%]", 1.0, float(S.max_vel_ptp), float(S.max_vel_ptp), 1.0)
        st.button("Set VEL_PTP", disabled=not motion_ok, on_click=lambda: do("VEL_PTP", drv().set_vel_ptp, vptp))
        apo = st.number_input("APO (C_DIS mm / C_PTP %)", 0.0, 100.0, 0.0, 1.0,
                              help="Stored only: KRLDRIVER v2 moves are exact-stop.")
        st.button("Set APO", disabled=not motion_ok, on_click=lambda: do("APO", drv().set_apo, apo))

# ============================================================== I/O
with tabs[3]:
    nin, nout = CFG.robot.n_inputs, CFG.robot.n_outputs

    def io_panel():
        try:
            ins = monitor.read_io(ss.client, "IN", nin)
            outs = monitor.read_io(ss.client, "OUT", nout)
        except KVPError as exc:
            st.error(str(exc))
            return
        st.markdown("**Inputs $IN[ ]**")
        cols = st.columns(8)
        for i, v in enumerate(ins):
            cols[i % 8].markdown(f"{fmt_bool(v if v is None else str(v))} IN {i + 1}")
        st.markdown("**Outputs $OUT[ ]** (state)")
        cols = st.columns(8)
        for i, v in enumerate(outs):
            cols[i % 8].markdown(f"{fmt_bool(v if v is None else str(v))} OUT {i + 1}")

    st.fragment(run_every=max(1.0, refresh) if live_on else None)(io_panel)()
    st.markdown("**Set output** (direct KVP write — works without KRLDRIVER / without arm)")
    c = st.columns(4)
    port_no = c[0].number_input("OUT n", 1, 4096, 1)
    c[1].button("Set TRUE", disabled=not ctrl, on_click=lambda: do(f"$OUT[{port_no}]=TRUE", drv().set_output, port_no, True))
    c[2].button("Set FALSE", disabled=not ctrl, on_click=lambda: do(f"$OUT[{port_no}]=FALSE", drv().set_output, port_no, False))
    c[3].caption("⚠️ Real outputs may drive real equipment (grippers, spindles…).")

# ============================================================== variables
with tabs[4]:
    l, r = st.columns(2)
    with l:
        st.subheader("Read")
        name = st.text_input("Variable", "$AXIS_ACT")
        if st.button("Read"):
            val = do(f"read {name}", ss.client.read, name)
            if val is not None:
                st.code(val)
                try:
                    st.json(kt.parse_aggregate(val)[1])
                except ValueError:
                    pass
        st.subheader("Write")
        wname = st.text_input("Variable ", "KRLD_SLEEP")
        wval = st.text_input("Value (KRL syntax, e.g. 10, TRUE, {X 1.0})", "0.0")
        sure = st.checkbox("I understand this writes directly to the controller")
        st.button("Write", disabled=not (ctrl and sure),
                  on_click=lambda: do(f"write {wname}", drv().write_variable, wname, wval))
        st.caption(f"Blocked prefixes: {', '.join(S.write_denylist)}")
    with r:
        st.subheader("Watch list")
        ss.watch = st.text_area("One variable per line", ss.watch, height=140)

        def watch_panel():
            names = [n.strip() for n in ss.watch.splitlines() if n.strip()]
            try:
                vals = monitor.read_many(ss.client, names)
            except KVPError as exc:
                st.error(str(exc))
                return
            st.dataframe(pd.DataFrame({"Variable": list(vals), "Value": [v if v is not None else "⟂ unknown"
                                                                         for v in vals.values()]}),
                         hide_index=True, width="stretch")

        st.fragment(run_every=max(1.0, refresh) if live_on else None)(watch_panel)()

# ============================================================== recorder
with tabs[5]:
    st.caption("Records $AXIS_ACT / $POS_ACT over a **separate** connection (read-only, works in monitor mode) "
               "and derives velocities/accelerations (Savitzky-Golay). Rate ≈ 20-50 Hz on a real KR C4; "
               "for IPO-rate data use the controller's Diagnosis → Trace.")
    rec: Recorder | None = ss.get("rec")
    busy = rec is not None and rec.running
    c = st.columns(4)
    period_ms = c[0].number_input("Period [ms] (0 = max rate)", 0, 1000, 10, 5, disabled=busy)
    duration = c[0].number_input("Duration [s] (0 = until stop)", 0.0, 3600.0, 0.0, 1.0, disabled=busy)
    wait_start = c[1].checkbox("Wait for program start", False, disabled=busy)
    until_end = c[1].checkbox("Stop when program ends", False, disabled=busy)
    measured = c[1].checkbox("Measured values (_MEAS/_MES)", False, disabled=busy,
                             help="$AXIS_ACT_MEAS / $POS_ACT_MES instead of setpoints, if available on your KSS")
    prog = c[2].text_input("Program name filter", "", disabled=busy, help="e.g. KRLDRIVER (empty = any)")
    extras = c[2].text_input("Extra variables (space separated)", "", disabled=busy,
                             help="e.g. $OV_PRO $VEL_ACT — each one lowers the sampling rate")
    smooth = c[3].number_input("Smoothing window [s]", 0.02, 2.0, 0.1, 0.02)
    b1, b2 = c[3].columns(2)
    if b1.button("⏺ Start", disabled=busy, type="primary", width="stretch"):
        ss.rec = Recorder(ss.client.host, ss.client.port, period_s=period_ms / 1000.0, measured=measured,
                          extra_vars=extras.split(), wait_program_start=wait_start,
                          stop_on_program_end=until_end, duration_s=duration or None,
                          program_name=prog or None, timeout_s=CFG.connection.timeout_s).start()
        ss.rec_was_running = True
        st.rerun()
    if b2.button("⏹ Stop", disabled=not busy, width="stretch"):
        rec.stop()
        st.rerun()

    def rec_status():
        r: Recorder | None = ss.get("rec")
        if r is None:
            st.info("No recording yet.")
            return
        s = st.columns(4)
        s[0].metric("Status", r.status)
        s[1].metric("Samples", r.n_samples)
        s[2].metric("Elapsed", f"{r.elapsed_s:.1f} s")
        s[3].metric("Rate", f"{r.rate_hz:.1f} Hz")
        if r.error:
            st.error(r.error)
        if ss.get("rec_was_running") and not r.running:   # auto-stopped -> refresh results
            ss.rec_was_running = False
            st.rerun()

    st.fragment(run_every=1.0)(rec_status)()

    if rec is not None and not rec.running and rec.n_samples >= 3:
        st.subheader("Summary")
        cm.recording_results(rec, smooth, key="pendant_rec")

# ============================================================== log
with tabs[6]:
    st.subheader("Messages")
    cm.messages_panel(height=220)
    hist = list(ss.client.history)[-300:][::-1]
    df = pd.DataFrame([{"time": datetime.fromtimestamp(h.t).strftime("%H:%M:%S.%f")[:-3], "op": h.op,
                        "variable": h.name, "sent": h.value, "reply": h.reply, "ok": h.ok,
                        "ms": round(h.ms, 1)} for h in hist])
    ev = pd.DataFrame([{"time": datetime.fromtimestamp(t).strftime("%H:%M:%S"), "level": lv, "event": msg}
                       for t, lv, msg in list(drv().events)[::-1]])
    st.subheader("Driver events")
    st.dataframe(ev, hide_index=True, width="stretch", height=180)
    st.subheader("KVP transactions (last 300)")
    st.dataframe(df, hide_index=True, width="stretch", height=300)
    if not df.empty:
        st.download_button("Download CSV", df.to_csv(index=False), f"kvp_log_{int(time.time())}.csv")
