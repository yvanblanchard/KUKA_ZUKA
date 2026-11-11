import time

import numpy as np
import pytest

from kuka_zuka import Config, KRLDriver, KVPClient
from kuka_zuka import analysis as an
from kuka_zuka.mock import MockKRC, MockKVPServer
from kuka_zuka.recorder import Recorder


def test_savgol_exact_on_cubic():
    t = np.linspace(0, 2, 201)
    dt = t[1] - t[0]
    y = 2 * t ** 3 - t ** 2 + 3 * t + 1
    for d, ref in ((0, y), (1, 6 * t ** 2 - 2 * t + 3), (2, 12 * t - 2)):
        np.testing.assert_allclose(an.savgol(y, 11, 3, d, dt), ref, atol=1e-6)   # incl. edges


def test_joint_kinematics_nonuniform_sampling():
    rng = np.random.default_rng(0)
    t = np.cumsum(rng.uniform(0.015, 0.025, 400))                   # jittery ~50 Hz
    a1 = 30 * np.sin(t)
    df = an.joint_kinematics(t, {k: (a1 if k == "A1" else np.zeros_like(t)) for k in an.AXES}, smooth_s=0.15)
    tt = df["t"].to_numpy() + t[0]
    inner = slice(10, -10)
    np.testing.assert_allclose(df["A1_vel"].to_numpy()[inner], 30 * np.cos(tt)[inner], atol=0.05)
    np.testing.assert_allclose(df["A1_acc"].to_numpy()[inner], -30 * np.sin(tt)[inner], atol=1.0)   # ~3 % of peak: interpolation of jittery samples


def test_tcp_linear_and_angular_velocity_through_wrap():
    t = np.linspace(0, 4, 401)
    pose = {"X": 100 + 50 * t, "Y": 0 * t, "Z": 20 * t,
            "A": ((170 + 30 * t + 180) % 360) - 180,                # rotates about Z, wraps at ±180
            "B": 90.0 + 0 * t, "C": 0 * t}                          # B = 90°: Euler singularity
    df = an.tcp_kinematics(t, pose, smooth_s=0.1)
    np.testing.assert_allclose(df["tcp_speed"], np.hypot(50, 20), rtol=1e-6)
    np.testing.assert_allclose(df["tcp_acc"], 0, atol=1e-6)
    np.testing.assert_allclose(df["tcp_ang_speed"][5:-5], 30, atol=1e-3)
    np.testing.assert_allclose(df["wz"][5:-5], 30, atol=1e-3)


@pytest.fixture
def server():
    srv = MockKVPServer(port=0, krc=MockKRC()).start()
    srv.krc.write("$OV_PRO", "100")
    srv.krc.write("KRLD_VEL_PTP", "50")          # mock: 120 deg/s * 50 % = 60 deg/s
    yield srv
    srv.stop()


def test_record_motion_on_mock(server, tmp_path):
    rec = Recorder(server.host, server.port, period_s=0.01, extra_vars=["$OV_PRO", "$BASE"]).start()
    with KVPClient(server.host, server.port) as c:
        d = KRLDriver(c, Config())
        d.take_control()
        time.sleep(0.2)
        d.ptp_axis([30, -90, 90, 0, 0, 0])
        time.sleep(0.2)
    rec.stop()
    assert rec.error is None and rec.n_samples > 50
    joints, tcp, summary = rec.process(smooth_s=0.1)
    v = joints["A1_vel"].abs()
    assert v[v > 30].median() == pytest.approx(60, rel=0.1)       # plateau speed (mock = rectangular profile)
    assert summary["axes"]["A2"]["max_abs_vel_deg_s"] < 1
    assert joints["A1"].iloc[-1] == pytest.approx(30, abs=0.5)
    assert tcp is not None and "$BASE.X" in rec.raw_frame()
    paths = rec.save(tmp_path / "run")
    assert {p.name for p in paths} == {"run_raw.csv", "run_joints.csv", "run_tcp.csv", "run_meta.json"}


def test_wait_start_and_stop_on_program_end(server):
    server.krc.set_drives(True, driver_running=False)              # program not running yet
    rec = Recorder(server.host, server.port, period_s=0.01,
                   wait_program_start=True, stop_on_program_end=True).start()
    time.sleep(0.3)
    assert rec.status == "waiting for program start" and rec.n_samples == 0
    server.krc.set_drives(True, driver_running=True)               # program starts
    time.sleep(0.5)
    assert rec.status == "recording" and rec.n_samples > 0
    server.krc.set_drives(True, driver_running=False)              # program ends
    rec.join(3)
    assert not rec.running and rec.status == "stopped (program ended)"
