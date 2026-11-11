"""KRL snippet parser / runner / .src export (runner against the mock)."""
import pytest

from kuka_zuka import Config, KRLDriver, KVPClient, krlprog
from kuka_zuka.mock import MockKRC, MockKVPServer

CFG = Config()

PROGRAM = """\
&ACCESS RVP
DEF TEST( )
;FOLD INI
  BAS (#INITMOV,0 )
;ENDFOLD (INI)
DECL E6AXIS P0={A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0, E1 0}
DECL E6POS P1={X 500, Y 0, Z 800, A 0, B 90, C 0, S 2, T 35}
BAS(#VEL_PTP, 10)
$VEL.CP = 0.05
PTP P0
PTP {A1 10}
PTP_REL {A6 -5}  ; comment
LIN P1 C_DIS
LIN_REL {Z -10}
CIRC {X 510, Y 10}, {X 520, Y 0}
$OUT[3] = TRUE
$BASE = BASE_DATA[1]
$TOOL = $NULLFRAME
WAIT SEC 0.2
PTP HOME
END
"""


def test_parse_program():
    prog = krlprog.parse(PROGRAM, CFG)
    assert prog.ok, prog.errors
    ops = [s.op for s in prog.steps]
    assert ops == ["set_vel_ptp", "set_vel_cp", "ptp_axis", "ptp_axis", "ptp_rel_axis", "lin", "lin_rel",
                   "circ", "set_out_via_driver", "set_base", "set_tool", "wait_sec", "home"]
    assert prog.steps[2].args == ([0.0, -90.0, 90.0, 0.0, 0.0, 0.0],)
    assert prog.steps[3].args == ([10.0, None, None, None, None, None],)       # missing axes kept
    assert prog.steps[4].args == ([0.0, 0.0, 0.0, 0.0, 0.0, -5.0],)
    assert prog.steps[5].args[0]["S"] == 2 and prog.steps[5].args[0]["X"] == 500.0
    assert prog.steps[9].kwargs == {"base_data": 1}
    assert prog.steps[5].line == 13 and prog.warnings and prog.warnings[0][0] == 13   # C_DIS ignored
    assert prog.n_motions == 7


@pytest.mark.parametrize("line, msg", [
    ("FOR I = 1 TO 3", "control flow"),
    ("PTP P9", "unknown point"),
    ("LIN {A1 10}", "Cartesian"),
    ("PTP {A1 10, X 5}", "mixes"),
    ("PTP {A1 175}", "soft limits"),
    ("PTP_REL {A1 30}", "max_rel_deg"),
    ("LIN_REL {X 80}", "max_rel_mm"),
    ("$VEL.CP = 2", "$VEL.CP"),
    ("$OV_PRO = 50", "$OV_PRO"),
    ("PTP {A1 X}", "not a number"),
    ("MYSUB()", "unsupported"),
])
def test_parse_errors(line, msg):
    prog = krlprog.parse(line, CFG)
    assert not prog.ok
    assert msg in prog.errors[0][1], prog.errors


def test_to_src():
    src, dat = krlprog.to_src("DECL E6POS P1={X 1}\nPTP P1\n", "my test")
    assert "DEF MY_TEST( )" in src and "  PTP P1" in src and "DECL E6POS" not in src
    assert src.rstrip().endswith("END")
    assert "DEFDAT MY_TEST" in dat and "DECL E6POS P1={X 1}" in dat
    assert krlprog.to_src(PROGRAM, "x") == (PROGRAM, "")                    # complete program kept
    assert krlprog.program_name("1abc") == "P1ABC"


@pytest.fixture()
def drv():
    srv = MockKVPServer(port=0, krc=MockKRC(time_scale=0.05)).start()
    srv.krc.write("$OV_PRO", "100")
    client = KVPClient(srv.host, srv.port).connect()
    d = KRLDriver(client, CFG)
    d.take_control()
    yield d
    client.close()
    srv.stop()


class FakeRecorder:
    def __init__(self):
        self.status, self.error, self.started, self.stopped = "idle", None, False, False

    @property
    def running(self):
        return self.started and not self.stopped

    def start(self):
        self.started, self.status = True, "recording"
        return self

    def stop(self):
        self.stopped = True


def test_runner_on_mock(drv):
    prog = krlprog.parse("PTP {A1 10, A2 -80}\nPTP_REL {A1 5}\n$OUT[2]=TRUE\nPTP {A1 0}", CFG)
    rec = FakeRecorder()
    r = krlprog.ProgramRunner(drv, prog, repeat=2, recorder=rec, tail_s=0).start()
    r.join(30)
    assert r.status == "finished", r.error
    assert r.n_done == r.total == 8
    assert drv.read_axes()[:2] == pytest.approx([0.0, -80.0], abs=1e-3)
    assert drv.read_output(2)
    assert rec.started and rec.stopped
    assert r.events[-1][0] == "INFO"


def test_runner_reports_refused_step(drv):
    prog = krlprog.parse("PTP {A1 10}\nPTP_REL {A1 8}\nPTP_REL {A1 8}\nPTP_REL {A1 8}", CFG)
    drv.cfg.safety.axis_limits[0] = [-30, 30]        # 3rd relative move (A1 = 34) is outside the limits
    try:
        r = krlprog.ProgramRunner(drv, prog).start()
        r.join(30)
    finally:
        drv.cfg.safety.axis_limits[0] = [-170, 170]
    assert r.status == "error" and "line 4" in r.error and "soft limits" in r.error
    assert r.n_done == 3
