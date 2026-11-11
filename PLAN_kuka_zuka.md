# PLAN — `kuka_zuka`: Python 3 KRC4 driver (from ZuKa) + Streamlit virtual pendant

Status: **APPROVED 2026-11-08 — P1, P3, P4 implemented and tested on the mock; P0/P2/P5 pending hardware.**

### Decisions (2026-11-08)
| # | Decision |
|---|---|
| 1 | KR C4 runs **Windows XP**, so KSS 8.1/8.2 (exact version to be read from Help → Info). KUKAVARPROXY not installed yet → use the original **KVP 6.1.0.101**. |
| 2 | Robot **KR10 R1100 sixx** (same axis limits as the KR6 R900 sixx: ±170 / −190..45 / −120..156 / ±185 / ±120 / ±350). |
| 3 | No OfficeLite (WorkVisual only) → motion validated on the **mock** until the arm is connected. WorkVisual is used to deploy `$config.dat` + `KRLDRIVER.src`. |
| 4 | **KRLDRIVER v2** (fixed) adopted. |
| 5 | Not answered → default applied: arbitrary writes allowed only in Control mode, with confirmation + deny-list. |
| 6 | Mock kept **simple** (no opw kinematics). |
| 7 | Local venv `kuka_zuka/.venv`. |

### Implementation status
- P1 library: `kuka_zuka/{protocol,client,krltypes,config,monitor}.py`. ✅ 17 pytest pass.
- P2 probe CLI: `python -m kuka_zuka.probe <ip>`. ✅ on mock; ⏳ on the real KR C4 after P0.
- P3 driver + KRL v2 + mock: `driver.py`, `compat.py`, `krl/`, `mock/`. ✅ examples run on the mock. ⚠️ `KRLDRIVER.src` has **not been compiled on a controller yet**.
- P4 pendant: `app/pendant.py`. ✅ AppTest-driven check on the mock (connect, control, jog, no-drives refusal).
- P5 real motion: ⏳ needs the arm.

---

## 1. Goal

1. Turn the ZuKa online-control code (`zuka_src/codes/krl-driver/`) into a clean, tested **Python 3 library**.
2. Drive a **real KUKA KR C4** over TCP/IP (KUKAVARPROXY, port 7000), **including when no manipulator is connected**.
3. Provide a small **Streamlit "virtual smartPAD"**: status, variables, I/O, override, motion/jog.

---

## 2. What the upstream repo actually contains (analysis)

| Part | File | Verdict |
|---|---|---|
| Wire protocol (KVP client) | `codes/krl-driver/kukavarproxy.py` (same file as `kukavarproxy-msg-format/python/`) | ~110 lines. **Python 2 only.** Reuse the protocol, rewrite the code. |
| High-level API | `codes/krl-driver/krldriver.py` | KRL-style functions (`PTP`, `LIN`, `CIRC`, `BASE`, `TOOL`, `OUT`…) that write `KRLD_*` variables and then a command code into `KRLD_COM`. Port it to a class. |
| Resident robot program | `codes/krl-driver/KRLDRIVER.src` | `WHILE TRUE / SWITCH KRLD_COM` loop with 15 commands. Keep the design but fix the race conditions. |
| ROS/Kinect, Inkscape, ZUKA-CNC | others | Out of scope. |

Protocol (request): `msgID u16 | len u16 | mode u8 (0=read,1=write) | nameLen u16 | name | [valLen u16 | value]`. All fields are big-endian and values are ASCII.
Response: `msgID u16 | len u16 | mode u8 | valLen u16 | value | 3 trailing bytes`. The last trailing byte is the success flag, as documented by OpenShowVar. **I'll confirm this on the real controller.**

### 2.1 Defects found in upstream (all fixed in the port)

| # | Where | Problem | Consequence |
|---|---|---|---|
| D1 | `kukavarproxy.py` | Py2 `encode('hex')` read path; Py3 variant commented out | Crashes on Py 3.11 |
| D2 | `kukavarproxy.py` | **Module-level global socket**; `recv(1024)` with no framing | Only one connection per process. Responses can be truncated or merged. Can't reconnect. |
| D3 | `kukavarproxy.py` / `krldriver.py` | Errors call `raise SystemExit` | Would kill the Streamlit server |
| D4 | `kukavarproxy.py` | Success flag in response never checked | Failed writes (bad var name, read-only var) pass silently |
| D5 | `krldriver.py` | `CIRC` writes the aux point into `KRLD_POS` and then overwrites it with the destination. `KRLD_POS_AUX` is never written. | CIRC is broken |
| D6 | `krldriver.py` | Hard-coded IP and connection at import time; every helper re-reads `PRO_ID` (≈3 round-trips per command) | Slow and not configurable |
| D7 | `krldriver.py` | `while int(read("KRLD_COM")): continue` loops with no timeout and no sleep | UI freezes forever if KRLDRIVER is not running |
| D8 | `krldriver.py` | Axis-limit check commented out and buggy (`v5` used twice); `INI()` default `VEL_CP=3` (= **3 m/s**, the max) | Safety |
| D9 | `KRLDRIVER.src` | `KRLD_COM=0` is cleared **before** the motion consumes `KRLD_POS`, and `$ADVANCE=1` pre-evaluates the loop | The PC can overwrite `KRLD_POS` before it's used, so points get lost or skipped |
| D10 | `KRLDRIVER.src` | Only reports "accepted", never "motion done" or "error" | The pendant can't show busy/done |
| D11 | repo | **The `KRLD_*` / `PRO_ID` global declarations (`$config.dat`) are missing** from the repo, the thesis and the manual | KRLDRIVER won't compile as shipped |

---

## 3. Key constraint: KR C4 **without the manipulator**

What works with the controller alone depends on which layer you use:

| Capability | Without robot | Why |
|---|---|---|
| KVP connect, read/write **any** variable (`$AXIS_ACT`, `$POS_ACT`, `$MODE_OP`, `$OV_PRO`, user globals…) | ✅ | KVP talks to the KSS variable server and doesn't need drives |
| Digital I/O `$IN[]`/`$OUT[]` (read/write) | ✅ | Needs the I/O bus configured. With no bus, they still work as internal flags. |
| Logic in the **Submit interpreter** (`SPS.SUB`) | ✅ | Runs without drives |
| Selecting/starting **KRLDRIVER** and executing **motion** (PTP/LIN/CIRC) | ⚠️ **Most likely not possible.** With no motors/RDC on X21 the KR C4 raises RDC/KCB/drive errors and won't switch drives on. A robot program can't do its BCO run without drives. | **Needs checking on your KSS version.** Possible ways out: a simulation option on the controller (if your KSS/WorkVisual allows it), KUKA.OfficeLite, or wait until the arm is connected. |
| External motion in **T1/T2** | ❌ | Needs the physical enabling switch on the smartPAD. External control runs in **AUT / AUT EXT** only. |
| Safety circuit | ⚠️ | E-STOP and operator safety (X11 / smartPAD) must be closed. Otherwise you get permanent "safety stop" acknowledgements. |

**Proposed strategy (layered):**

- **Tier A, works on the bare controller from day 1:** connection, monitoring, ShowVar-style read/write, I/O, override, and controller/program state.
- **Tier B, motion:** a full **mock KR C4** in Python (a KVP server plus a simulated KRLDRIVER loop) for development and UI testing. On the real KR C4 I'll test the handshake down to the point where the drives are refused, and get the error messages back cleanly. Real motion gets validated once the arm is connected, or with OfficeLite or a simulation mode if you have one.

---

## 4. Target layout

```
kuka_zuka/
├── PLAN_kuka_zuka.md                 (this file)
├── zuka_src/                         upstream clone, untouched, reference only
├── pyproject.toml / requirements.txt streamlit, pyyaml, pytest
├── config.yaml                       IP, port, timeouts, robot limits, safety caps
├── kuka_zuka/                        ← the library (Python ≥3.10, no deps besides stdlib)
│   ├── protocol.py    pure encode/decode of KVP frames (struct, exact-length framing)
│   ├── client.py      KVPClient: own socket, timeout, threading.Lock, msg-id counter,
│   │                  reconnect, success-flag check, typed exceptions (KVPError, …)
│   ├── krltypes.py    parse/format KRL aggregates: E6AXIS, E6POS, FRAME, BOOL, REAL
│   │                  "{E6POS: X 1.0, Y …}" ⇄ dataclasses
│   ├── driver.py      KRLDriver class = OO port of krldriver.py (PRO_ID ownership,
│   │                  ptp/lin/circ/rel, base/tool/vel/apo/out, wait_idle(timeout),
│   │                  soft limits + caps from config) — fixes D5–D8
│   ├── monitor.py     batched reads of the standard status variables (see §5)
│   └── mock/
│       ├── server.py  fake KUKAVARPROXY (asyncio TCP :7000) with a variable store
│       └── krc.py     simulated KRLDRIVER loop + simple kinematics (axis moves update
│                      $AXIS_ACT; Cartesian via optional py_opw_kinematics FK/IK for
│                      KR6 R900 sixx, else straight assignment)
├── krl/
│   ├── KRLDRIVER.src  v2 (fixes D9/D10, see §6)
│   ├── KRLDRIVER.dat
│   └── config_dat_globals.txt   declarations to paste into $config.dat USER GLOBALS (fixes D11)
├── app/
│   └── pendant.py     Streamlit virtual smartPAD
├── examples/          ports of example.py / example2.py (py3, class API)
└── tests/             protocol round-trip, aggregate parsing, driver vs mock server
```

---

## 5. Streamlit virtual pendant (`app/pendant.py`)

The client is stored in `st.session_state`, one connection per browser session, and guarded by a lock. Live panels use `@st.fragment(run_every=0.5s)` (Streamlit 1.49 is installed). Every command has a timeout, so nothing in the UI can block forever.

| Area | Content | Tier |
|---|---|---|
| **Sidebar** | IP / port / **Real vs Mock** toggle, Connect/Disconnect, latency, **Monitor-only vs Control** mode (writes disabled in monitor mode) | A |
| **Status** | `$ROBNAME[]`, KSS version (`$RCV_INFO[]`), `$MODE_OP`, `$PRO_STATE1`, `$PRO_NAME1[]`, `$STOPMESS`, `$PERI_RDY` (drives), `$OV_PRO`, `$POS_ACT`, `$AXIS_ACT`, `$BASE`, `$TOOL`, KRLDRIVER heartbeat/owner | A |
| **Variables** | ShowVar clone: read any variable, watch list with auto-refresh, write with confirmation | A |
| **I/O** | `$IN[1..N]` LEDs, `$OUT[1..N]` toggles (range from config) | A |
| **Override** | `$OV_PRO` slider (capped by config), **"Soft STOP" = `$OV_PRO=0`**, clearly labelled *not a safety function* | A |
| **Jog / Motion** | Step-jog per axis (A1–A6 ±) and Cartesian (X/Y/Z/A/B/C ±) in BASE, step size selector. "Go to" PTP-axis / PTP-pos / LIN / CIRC forms. Home button. Base/Tool from explicit frame or `BASE_DATA[n]`/`TOOL_DATA[n]`. VEL_CP/VEL_PTP/APO. Only enabled when KRLDRIVER is detected, ownership is held and the mode is Control. | B |
| **Log** | Timestamped TX/RX and error history, exportable CSV | A |

Jogging is **step-jog**: each click sends one small relative move. KVP can't do true continuous hold-to-jog. That would need RSI or EKI, which are licensed options.

---

## 6. `KRLDRIVER.src` v2 (changes vs upstream)

- At the start of each cycle, copy the parameters into local variables, run the command, then acknowledge. This fixes D9. `$ADVANCE` is handled so that `KRLD_COM` is not pre-read for queued moves (`WAIT FOR` / `$ADVANCE=0` in the dispatch).
- New globals: `KRLD_STATE` (0 idle / 1 busy / 2 error), `KRLD_ACK` (echo of the last command sequence number) and `KRLD_HEARTBEAT` (incremented each loop). These fix D10 and let the UI detect "driver running".
- Startup move to home becomes **optional** (`KRLD_HOME_ON_START`), so selecting the program doesn't immediately move the arm.
- Commands 1–15 keep their numbering, so the upstream semantics are preserved.
- Optional `SPS.SUB` snippet for the heartbeat and I/O mirroring, so the bare controller also shows "alive".

The `$config.dat` globals I'll provide (to be checked against your KSS):
```
DECL INT KRLD_COM=0, KRLD_STATE=0, KRLD_ACK=0, KRLD_SEQ=0, KRLD_HEARTBEAT=0, PRO_ID=0
DECL E6AXIS KRLD_AXIS
DECL E6POS  KRLD_POS, KRLD_POS_AUX
DECL FRAME  KRLD_BASE, KRLD_BASE_OFFSET, KRLD_TOOL
DECL REAL   KRLD_APO=0.0, KRLD_VEL_CP=0.1, KRLD_SLEEP=0.0
DECL INT    KRLD_VEL_PTP=10, KRLD_IO=1
DECL BOOL   KRLD_SIGNAL=FALSE, KRLD_HOME_ON_START=FALSE
```

---

## 7. Phases & acceptance criteria

| Phase | Work | Done when |
|---|---|---|
| **P0 — Controller prep** (you, on the KR C4, with my checklist) | Install KUKAVARPROXY. Open port 7000 (Network config → NAT). Set the KLI IP. Paste the globals into `$config.dat`. Load KRLDRIVER v2. Close the safety circuit / acknowledge messages. | `Test-NetConnection <ip> -Port 7000` succeeds |
| **P1 — Library core** | `protocol`, `client`, `krltypes` + unit tests + mock KVP server | pytest green, round-trip against the mock |
| **P2 — Bare-controller smoke test** | CLI `python -m kuka_zuka.probe <ip>`: reads ROBNAME, KSS version, `$MODE_OP`, `$AXIS_ACT`, writes/reads back a user global, toggles a `$OUT` | Works on your real KR C4 with no arm |
| **P3 — Driver + KRL v2 + simulated KRC** | `driver.py`, `krl/` files, `mock/krc.py`; ports of the examples | Examples run end-to-end against the mock (including CIRC and the ownership guard) |
| **P4 — Streamlit pendant** | `app/pendant.py` | All Tier-A panels work on the real controller; Tier-B panels work on the mock |
| **P5 — Motion on hardware** | Real motion validation at low override in AUT | Only once drives can be enabled (arm connected / sim mode / OfficeLite) |

---

## 8. Safety design (built in, not optional)

- **Monitor-only is the default mode.** Control has to be explicitly enabled per session.
- Caps from `config.yaml`: max `$OV_PRO` (default 10 %), max VEL_CP (default 0.1 m/s), max VEL_PTP (default 10 %), max jog step, and soft axis limits (KR6 R900 sixx limits are in upstream; to be replaced with **your robot's** limits).
- Ownership via `PRO_ID`: only one PC drives at a time, and the UI shows who owns it.
- Arbitrary variable writes need a confirmation step, and a deny-list protects system variables (e.g. `$MACHINE`-type data).
- The UI states clearly that it is **not** a safety device. The physical E-STOP and the controller's own safety stay authoritative.

---

## 9. Decisions / information needed from you

1. **KSS version and Windows of the KR C4** (8.3 / Win7e or 8.5–8.7 / Win10 IoT)? This decides which KUKAVARPROXY build to use: the original 6.1 (needs `cswsk32.ocx`) or the maintained rewrite (ImtsSrl KukaVarProxy). Is **KVP already installed**?
2. **Which robot model** is configured on the controller (axis limits, home pose)? Upstream assumes a KR6 R900 sixx.
3. **Motion without the arm:** do you have KUKA.OfficeLite, or a known simulation option on this controller? If not, do you accept that Tier B is validated on the mock until the arm is connected (§3)?
4. **KRLDRIVER:** OK to use the **fixed v2** (§6, same command numbers) rather than the original as-is? (Recommended.)
5. **Write access in the UI:** should the ShowVar-style "write any variable" stay in (with confirmation and deny-list), or only the predefined controls?
6. **Mock kinematics:** use `../py_opw_kinematics` (already in your workspace) for realistic FK/IK in the simulator, or keep the mock simple (no IK)?
7. **Packaging:** a standalone package in `kuka_zuka/` with its own venv (recommended), or install into your existing Python 3.11?
