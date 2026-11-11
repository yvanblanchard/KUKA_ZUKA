# kuka_zuka — Python 3 KR C4 control + Streamlit virtual pendant

Python 3 port of the online-control part of [ZuKa](https://github.com/mnourgwad/zuka)
(`codes/krl-driver`), driving a KUKA **KR C4** over **KUKAVARPROXY** (TCP 7000).
Robot: **KR10 R1100 sixx**. Plan & design decisions: [`PLAN_kuka_zuka.md`](PLAN_kuka_zuka.md).

```
kuka_zuka/            library: protocol, client, krltypes, driver, monitor, krlprog, compat, probe, mock/ (stdlib only)
                      + recorder, analysis (numpy/pandas): data recording & velocity/acceleration
krl/                  KRLDRIVER.src v2 + config_dat_globals.txt  -> to load on the controller
app/jog_app.py        Streamlit jog app: joint jog, Cartesian target, KRL program runner + joint recording
app/pendant.py        Streamlit virtual pendant (full feature set)
app/common.py         shared UI code of both apps (connection, control, status, messages, recording results)
examples/             demo_mock.py, zuka_example2_port.py
tests/                pytest (protocol, driver, recorder/analysis — all against the mock)
config.yaml           IP, timeouts, KR10 R1100 limits, safety caps
zuka_src/             upstream clone (reference only, not used at runtime)
```

## Quick start (PC)

```powershell
cd C:\YVAN\RepoTest\kuka_zuka
.venv\Scripts\python -m pytest -q                         # 41 tests, no hardware
.venv\Scripts\python -m kuka_zuka.probe --mock            # probe the simulator
.venv\Scripts\streamlit run app\pendant.py                # pendant -> choose "Mock" or "Real KR C4"
.venv\Scripts\streamlit run app\jog_app.py                # simple app: connect, joint jog +/-, speed,
                                                          # Cartesian target, KRL program, message log
```
(Recreate the env if needed: `python -m venv .venv; .venv\Scripts\pip install -r requirements.txt`.)

## What works on the KR C4 **without the arm**

| Feature | Bare controller | Needs drives (arm connected) |
|---|---|---|
| Connect, status, axis/TCP readout, controller info | ✅ | |
| Read / watch / write variables (ShowVar-like) | ✅ | |
| `$IN` / `$OUT` (direct KVP write) | ✅ | |
| Override / soft stop (`$OV_PRO`) | ✅ | |
| KRLDRIVER commands: jog, PTP/LIN/CIRC, BASE/TOOL/VEL via driver, NOOP | | ✅ (program must run in AUT) |

Without the arm, the RDC/motors are missing and drives cannot be switched on, so KRLDRIVER cannot be started.
The pendant then reports *"KRLDRIVER is not running"* instead of hanging. Motion is developed and tested
against the mock (sidebar → **Mock**, untick *"manipulator connected"* to mimic the bare controller).

## Controller setup checklist (KR C4, Windows XP ⇒ KSS 8.1/8.2)

> Do this with an expert who knows the cell. Back up the controller first (Main menu → File → Archive → USB → All).

1. **Find the KSS version**: smartPAD Main menu → Help → Info → *Robot controller* tab.
   WinXP-based KR C4 = KSS 8.1/8.2. The original KUKAVARPROXY **6.1** targets this generation.
2. **Log in as Expert** (Main menu → Configuration → User group → Expert, default password `kuka`).
   For Windows access, **Administrator** user group.
3. **Install KUKAVARPROXY**
   - Get `KUKAVARPROXY rev 6.1.0.101` (JOpenShowVar repo: `aauc-mechlab/JOpenShowVar/KUKAVARPROXY rev 6.1.0.101`).
   - Copy it to a USB stick → plug into the **controller's** USB port (not the smartPAD).
   - Main menu → Start-up → Service → **Minimize HMI** → copy the folder to e.g. `C:\KUKAVARPROXY`.
   - If `cswsk32.ocx` errors: `regsvr32 C:\KUKAVARPROXY\cswsk32.ocx`.
   - Run `KUKAVARPROXY.exe`. For auto-start, add a shortcut to *Start → All Programs → Startup*.
4. **Open port 7000**: Main menu → Start-up → Network configuration → *Advanced* → **NAT** →
   *Add port* 7000, protocols tcp/udp. Note the KLI IP (often `172.31.1.147`, mask 255.255.0.0).
   Do a **cold restart**.
5. **PC network**: Ethernet on the KLI (X66) port, same subnet (e.g. `172.31.1.10/16`).
   `Test-NetConnection 172.31.1.147 -Port 7000` must succeed.
6. **Smoke test, no motion**: `.venv\Scripts\python -m kuka_zuka.probe 172.31.1.147`
   (add `--write-test` after step 7; `--out N` pulses a real output — careful).
7. **Declare the globals**: paste [`krl/config_dat_globals.txt`](krl/config_dat_globals.txt) into
   `KRC:\R1\System\$config.dat`, in the *USER GLOBALS* fold. Use the smartPAD editor, or with **WorkVisual**:
   load the project from the controller → edit → deploy.
8. **Load `krl/KRLDRIVER.src`** into `KRC:\R1\Program` (WorkVisual or smartPAD navigator from USB).
   Check that it compiles: no red error marks in the navigator.
9. **Run KRLDRIVER** (once the arm is connected): select the program, T1 for a first BCO check, then **AUT**,
   drives ON, start. The pendant shows *KRLDRIVER: idle*. Test with *Handshake test (NOOP)* first,
   then step-jog at ≤ 10 % override.

Safety without arm: the X11 safety circuit (E-STOP, operator safety) must be closed/jumpered, otherwise
the controller stays in safety stop. Expect RDC/drive-related messages while the arm is absent.

## KRL program (jog app, tab **📜 KRL program**)

KUKAVARPROXY only reads/writes variables: it **cannot copy a `.src` file to the controller, nor select or
start a program**. Two ways to run pasted KRL code:

1. **Run on robot** (PC-side interpreter, `kuka_zuka/krlprog.py`): each line becomes one KRLDRIVER command,
   executed in a background thread with the PC-side caps of `config.yaml`, while a `Recorder` samples
   `$AXIS_ACT` / `$POS_ACT` over a second connection. Results: joint/TCP charts, velocities, accelerations,
   CSV download. Supported subset: `PTP`/`LIN`/`CIRC`/`PTP_REL`/`LIN_REL` with literal aggregates or points
   declared in the snippet (`DECL E6POS P1={...}`), `PTP HOME`, `$VEL.CP`, `BAS(#VEL_PTP,n)`, `$OV_PRO`,
   `$APO.CDIS/CPTP`, `$BASE`/`$TOOL` (`BASE_DATA[n]`, literal frame, `$NULLFRAME`), `$OUT[n]=TRUE/FALSE`,
   `WAIT SEC`. No control flow (use **Repeat**), no calls, no expressions. Moves are exact-stop
   (`C_DIS`/`C_PTP` ignored). **Stop** ends the run after the current instruction (a motion already sent
   can only be paused with SOFT STOP / stopped with the E-STOP).
2. **Native run** (expander *.src export*): download `NAME.src` + `NAME.dat`, copy them to `KRC:\R1\Program`
   (USB / WorkVisual), arm **Record next run**, then select and start the program on the smartPAD.
   Full KRL is available, but no PC-side caps apply.

Both need a robot that can actually move: drives ON, i.e. the **manipulator connected** (see below). On the bare
controller, use the **Mock** to try the workflow (the mock has no kinematics: Cartesian moves change `$POS_ACT`
only, axis moves `$AXIS_ACT` only).

## Library usage

```python
from kuka_zuka import KVPClient, KRLDriver, Config

cfg = Config.load()                                  # config.yaml
with KVPClient("172.31.1.147") as c:
    print(c.read("$AXIS_ACT"))                       # works on the bare controller
    drv = KRLDriver(c, cfg)
    drv.take_control()                               # PRO_ID ownership (ZuKa guard)
    drv.set_override(10)
    drv.ptp_axis([0, -90, 90, 0, 0, 0])              # needs KRLDRIVER running
    drv.lin([500, 0, 800, None, None, None])         # None = keep current component
    drv.jog_axis(1, +1.0)                            # step-jog
    drv.release_control()
```
ZuKa-style functions (`PTP(AXIS, ...)`, `LIN_REL("", 10)`, …): `from kuka_zuka.compat import *; init(host)`.

## Data recording (joints, speeds, accelerations — axes & TCP)

Records from a **running program** (any program, not only KRLDRIVER) over its own connection, read-only:

```powershell
# wait for the program to start, stop when it ends, save CSV + JSON
.venv\Scripts\python -m kuka_zuka.recorder 172.31.1.147 --wait-start --until-end --out recordings
un1
.venv\Scripts\python -m kuka_zuka.recorder 172.31.1.147 --duration 30 --extra $OV_PRO $VEL_ACT
.venv\Scripts\python -m kuka_zuka.recorder --mock-demo          # try it on the simulator
```
or in the pendant: tab **📈 Recorder** (charts + CSV download), or from Python:
```python
from kuka_zuka.recorder import Recorder
rec = Recorder("172.31.1.147", wait_program_start=True, stop_on_program_end=True).start()
rec.join()                                   # or rec.stop()
joints, tcp, summary = rec.process(smooth_s=0.1)
rec.save("recordings/run1")
```
Outputs: `*_raw.csv` (timestamped samples), `*_joints.csv` (A1..A6 + `_vel` [°/s] + `_acc` [°/s²]),
`*_tcp.csv` (X/Y/Z, vx..vz, `tcp_speed` [mm/s], `tcp_acc` & `tcp_acc_tangential` [mm/s²], wx..wz,
`tcp_ang_speed` [°/s], `tcp_ang_acc` [°/s²]), `*_meta.json` (program, $BASE/$TOOL, override, summary, sampling stats).

Method & limits:
* Polling through KUKAVARPROXY: one round trip per variable, PC timestamps (mid-request). Expect ~20–50 Hz on a
  real KR C4 with 2 variables, with jitter. Every `--extra` variable lowers the rate.
* Derivatives: uniform resampling + Savitzky–Golay (cubic, window `--smooth` s). Velocities are robust;
  accelerations are noisier (≈ a few % on clean data) and smoothed by the window, so short peaks are underestimated.
* TCP angular velocity from rotation matrices (KUKA ABC = Rz·Ry·Rx): no ±180° wrap or B=±90° singularity problems.
* `$AXIS_ACT`/`$POS_ACT` are **setpoints**; `--measured` uses `$AXIS_ACT_MEAS`/`$POS_ACT_MES` if your KSS has them.
  `$POS_ACT` is in the current `$BASE`/`$TOOL`.
* For exact IPO-cycle data (12 ms or faster, incl. motor currents/torques), use the controller's
  **Diagnosis → Trace** and analyse it in WorkVisual. This recorder is for convenient PC-side capture.

## Changes vs upstream ZuKa (see PLAN §2.1)
Py3; framed recv and per-instance socket; exceptions instead of `SystemExit`; status byte checked;
CIRC aux point fixed; full aggregates sent (no stale components); seq/ack/done handshake with timeouts
(KRLDRIVER v2, params consumed before ack); tech-package INI calls removed; optional home on start
(default: zero-length BCO move); PC-side caps (override, velocities, soft axis limits, step size,
write deny-list). Limitation: v2 moves are exact-stop (no blending).

**Not a safety device.** The physical E-STOP and the controller safety configuration remain authoritative.
