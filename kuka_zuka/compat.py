"""ZuKa-style functional API (``PTP(AXIS, ...)``, ``LIN(...)``) on top of :class:`KRLDriver`.

Eases porting of upstream scripts:

    from kuka_zuka.compat import *
    init("172.31.1.147")
    VEL_PTP(10); OV_PRO(10)
    PTP(AXIS, 0, -90, 90, 0, 0, 0)
    LIN_REL("", 10)

Empty strings ("") mean "unchanged" (absolute) or "0" (relative), as in upstream.
"""
from __future__ import annotations

from .client import KVPClient
from .config import Config
from .driver import KRLDriver

AXIS, POS = "a", "p"
TRUE, FALSE = True, False
_drv: KRLDriver | None = None


def _n(v):
    return None if v == "" or v is None else float(v)


def init(host: str | None = None, port: int | None = None, config: Config | None = None) -> KRLDriver:
    global _drv
    cfg = config or Config.load()
    client = KVPClient(host or cfg.connection.host, port or cfg.connection.port,
                       timeout=cfg.connection.timeout_s).connect()
    _drv = KRLDriver(client, cfg)
    _drv.take_control()
    return _drv


def driver() -> KRLDriver:
    if _drv is None:
        raise RuntimeError("call init(host) first")
    return _drv


def PTP(interpolation, *v):
    vals = [_n(x) for x in v]
    if interpolation == AXIS:
        return driver().ptp_axis(vals)
    return driver().ptp_pos(vals)


def PTP_REL(interpolation, *v):
    vals = [_n(x) for x in v]
    if interpolation == AXIS:
        return driver().ptp_rel_axis(vals)
    return driver().ptp_rel_pos(vals)


def LIN(*v):
    return driver().lin([_n(x) for x in v])


def LIN_REL(*v):
    return driver().lin_rel([_n(x) for x in v])


def CIRC(x="", y="", z="", a="", b="", c="", xAux="", yAux="", zAux="", aAux="", bAux="", cAux=""):
    return driver().circ([_n(q) for q in (xAux, yAux, zAux, aAux, bAux, cAux)],
                         [_n(q) for q in (x, y, z, a, b, c)])


def PTP_HOME():
    return driver().home()


def WAIT(sec):
    return driver().wait_sec(sec)


def BASE(x=0, y=0, z=0, a=0, b=0, c=0, offX=0, offY=0, offZ=0, offA=0, offB=0, offC=0, BASE_DATA=-1):
    if BASE_DATA > 0:
        return driver().set_base(offset=[offX, offY, offZ, offA, offB, offC], base_data=BASE_DATA)
    return driver().set_base([x, y, z, a, b, c], [offX, offY, offZ, offA, offB, offC])


def TOOL(x=0, y=0, z=0, a=0, b=0, c=0, TOOL_DATA=-1):
    if TOOL_DATA > 0:
        return driver().set_tool(tool_data=TOOL_DATA)
    return driver().set_tool([x, y, z, a, b, c])


def APO(v):
    return driver().set_apo(v)


def VEL_CP(v):
    return driver().set_vel_cp(v)


def VEL_PTP(v):
    return driver().set_vel_ptp(v)


def OV_PRO(p):
    return driver().set_override(p)


def OUT(port, signal):
    return driver().set_out_via_driver(port, bool(signal))


def PAL_MODE(signal):
    return driver().pal_mode(bool(signal))
