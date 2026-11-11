"""Batched reads of standard KRC status variables (all work on the bare controller)."""
from __future__ import annotations

from . import krltypes as kt
from .client import KVPClient, KVPVariableError

# name -> label. Each is read individually; unknown ones (KSS-version dependent) show None.
STATUS_VARS = {
    "$MODE_OP": "Operating mode",
    "$PRO_STATE1": "Robot interpreter",
    "$PRO_NAME1[]": "Selected program",
    "$PRO_STATE0": "Submit interpreter",
    "$PERI_RDY": "Drives ready",
    "$USER_SAF": "Operator safety",
    "$ALARM_STOP": "No E-stop (ALARM_STOP)",
    "$STOPMESS": "Stop message",
    "$OV_PRO": "Override %",
}

INFO_VARS = {
    "$ROBNAME[]": "Robot name",
    "$TRAFONAME[]": "Kinematics",
    "$MODEL_NAME[]": "Model",
    "$KR_SERIALNO": "Serial no.",
    "$RCV_INFO[]": "KSS version",
}


def read_many(client: KVPClient, names) -> dict:
    out = {}
    for n in names:
        try:
            out[n] = client.read(n)
        except KVPVariableError:
            out[n] = None
    return out


def read_status(client: KVPClient) -> dict:
    """Return a dict with raw status strings plus parsed axes/pose (None if unavailable)."""
    raw = read_many(client, list(STATUS_VARS) + ["$AXIS_ACT", "$POS_ACT", "$BASE", "$TOOL"])
    res = {"raw": raw, "axes": None, "pose": None, "base": None, "tool": None}
    for key, var, fn in (("axes", "$AXIS_ACT", kt.axes_of), ("pose", "$POS_ACT", kt.pose_of),
                         ("base", "$BASE", kt.pose_of), ("tool", "$TOOL", kt.pose_of)):
        try:
            if raw.get(var):
                res[key] = fn(raw[var])
        except ValueError:
            pass
    return res


def read_info(client: KVPClient) -> dict:
    return {k: (kt.parse_str(v) if v is not None else None)
            for k, v in read_many(client, INFO_VARS).items()}


def read_io(client: KVPClient, kind: str, n: int) -> list[bool | None]:
    vals = []
    for i in range(1, n + 1):
        try:
            vals.append(kt.parse_bool(client.read(f"${kind}[{i}]")))
        except (KVPVariableError, ValueError):
            vals.append(None)
    return vals


# ====================================================================== status -> messages
PRO_STATES = {"P_FREE": "no program selected", "P_RESET": "program selected, not started",
              "P_ACTIVE": "program running", "P_STOP": "program stopped", "P_END": "program ended"}
KRLD_STATE_ERROR = 2


def krldriver_running(raw: dict) -> bool:
    """KRLDRIVER is the selected program and the robot interpreter is active (from raw status)."""
    active = kt.parse_enum(raw.get("$PRO_STATE1") or "").upper() == "P_ACTIVE"
    return active and "KRLDRIVER" in kt.parse_str(raw.get("$PRO_NAME1[]") or "").upper()


def controller_events(prev: dict, cur: dict, max_override: float) -> list[tuple[str, str]]:
    """Turn changes of raw status variables (``read_status()["raw"]`` + ``KRLD_STATE``) into
    ``(level, message)`` pairs, level = INFO / WARN / ERROR. ``prev`` empty = first poll."""
    first = not prev
    ev: list[tuple[str, str]] = []

    def new(k):
        return cur.get(k) is not None and cur.get(k) != prev.get(k)

    for var, lvl, bad, good in (
        ("$ALARM_STOP", "ERROR", "EMERGENCY STOP pressed or safety circuit open ($ALARM_STOP = FALSE)",
         "Emergency stop released"),
        ("$USER_SAF", "WARN", "Operator safety open: safety gate / guard ($USER_SAF = FALSE)",
         "Operator safety closed"),
        ("$PERI_RDY", "WARN", "Drives are OFF ($PERI_RDY = FALSE)", "Drives ON"),
    ):
        if new(var):
            v = kt.try_bool(cur[var])
            if v is False:
                ev.append((lvl, bad))
            elif v and not first:
                ev.append(("INFO", good))
    if new("$STOPMESS"):
        v = kt.try_bool(cur["$STOPMESS"])
        if v:
            ev.append(("ERROR", "Stop message active on the controller: read and acknowledge it on the smartPAD"))
        elif v is False and not first:
            ev.append(("INFO", "Stop messages acknowledged"))
    if new("$MODE_OP"):
        mode = kt.parse_enum(cur["$MODE_OP"]).upper()
        ev.append(("INFO", f"Operating mode: {mode}"))
        if mode in ("T1", "T2"):
            ev.append(("WARN", f"{mode}: the robot only moves while the enabling switch and Start key are "
                               "held on the smartPAD. Use AUT for PC-driven motion."))
    if new("$PRO_NAME1[]"):
        ev.append(("INFO", f"Selected program: {kt.parse_str(cur['$PRO_NAME1[]']) or '(none)'}"))
    if new("$PRO_STATE1"):
        s = kt.parse_enum(cur["$PRO_STATE1"]).upper()
        was = kt.parse_enum(prev.get("$PRO_STATE1") or "").upper()
        ev.append(("WARN" if was == "P_ACTIVE" else "INFO", f"Robot interpreter: {PRO_STATES.get(s, s)}"))
    if new("KRLD_STATE") and kt.try_int(cur["KRLD_STATE"]) == KRLD_STATE_ERROR:
        ev.append(("ERROR", "KRLDRIVER reported an error on the last command"))
    if first and cur.get("KRLD_STATE") is None:
        ev.append(("WARN", "KRLD_* globals not found on the controller: paste krl/config_dat_globals.txt "
                           "into $config.dat (motion impossible until then)"))
    if new("$OV_PRO") and (kt.try_int(cur["$OV_PRO"]) or 0) > max_override:
        ev.append(("WARN", f"Override is {cur['$OV_PRO']} %, above the PC cap of {max_override:g} % "
                           "(changed on the smartPAD?)"))
    return ev
