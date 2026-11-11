import pytest

from kuka_zuka import (Config, DriverNotRunning, KRLDriver, KVPClient, KVPConnectionError,
                       KVPVariableError, OwnershipError, SafetyLimitError)
from kuka_zuka import krltypes as kt
from kuka_zuka import monitor
from kuka_zuka.mock import MockKRC, MockKVPServer


@pytest.fixture
def cfg():
    c = Config()
    c.connection.ack_timeout_s = 1.0
    c.connection.motion_timeout_s = 10.0
    c.connection.poll_period_s = 0.01
    return c


@pytest.fixture
def server():
    srv = MockKVPServer(port=0, krc=MockKRC(time_scale=0.02)).start()
    srv.krc.write("$OV_PRO", "100")
    srv.krc.write("KRLD_VEL_PTP", "100")
    yield srv
    srv.stop()


@pytest.fixture
def drv(server, cfg):
    c = KVPClient(server.host, server.port).connect()
    d = KRLDriver(c, cfg)
    d.take_control()
    yield d
    c.close()


def test_client_read_write(server):
    with KVPClient(server.host, server.port) as c:
        assert c.read("$MODE_OP") == "#AUT"
        assert c.write("$OV_PRO", 5) == "5"
        assert c.read("$ov_pro") == "5"          # KRL is case-insensitive
        with pytest.raises(KVPVariableError):
            c.read("NOT_A_VAR")
        with pytest.raises(KVPVariableError):
            c.write("$AXIS_ACT", "{A1 0}")       # read-only
        assert len(c.history) == 5


def test_connection_error():
    with pytest.raises(KVPConnectionError):
        KVPClient("127.0.0.1", 1, timeout=0.5).connect()


def test_ptp_axis_and_home(drv, server):
    drv.ptp_axis([10, None, None, None, None, 5])
    ax = drv.read_axes()
    assert ax[0] == pytest.approx(10) and ax[1] == pytest.approx(-90) and ax[5] == pytest.approx(5)
    drv.home()
    assert drv.read_axes() == pytest.approx([0, -90, 90, 0, 0, 0])


def test_relative_and_jog(drv):
    drv.jog_axis(1, 5.0)
    drv.jog_axis(1, 5.0)
    assert drv.read_axes()[0] == pytest.approx(10)
    z0 = drv.read_pose()["Z"]
    drv.jog_cart("Z", -20)
    assert drv.read_pose()["Z"] == pytest.approx(z0 - 20)


def test_lin_and_circ_send_aux(drv, server):
    drv.lin([500, 0, 800, None, None, None])
    assert drv.read_pose()["X"] == pytest.approx(500)
    drv.circ([510, 10, 800, None, None, None], [520, 0, 800, None, None, None])
    aux = kt.parse_aggregate(server.krc.read("KRLD_POS_AUX"))[1]
    dest = kt.parse_aggregate(server.krc.read("KRLD_POS"))[1]
    assert aux["X"] == 510 and aux["Y"] == 10 and dest["X"] == 520   # ZuKa bug D5 fixed
    assert aux["S"] == 2 and aux["T"] == 35                          # S/T kept from $POS_ACT


def test_settings_and_io(drv, server):
    drv.set_base([525, 0, 890, 0, 0, 0])
    assert kt.pose_of(server.krc.read("$BASE"))[0] == 525
    drv.set_out_via_driver(3, True)
    assert drv.read_output(3) is True
    drv.set_output(3, False)            # direct KVP write
    assert drv.read_output(3) is False
    server.krc.set_input(2, True)
    assert drv.read_input(2) is True
    drv.noop()
    drv.wait_sec(0.5)


def test_safety_caps(drv):
    with pytest.raises(SafetyLimitError):
        drv.set_override(50)            # cap 10 %
    with pytest.raises(SafetyLimitError):
        drv.ptp_axis([0, 50, 90, 0, 0, 0])   # A2 max 45 - margin
    with pytest.raises(SafetyLimitError):
        drv.jog_axis(1, 30)             # > max_rel_deg
    with pytest.raises(SafetyLimitError):
        drv.lin_rel([100, 0, 0, 0, 0, 0])    # > max_rel_mm
    with pytest.raises(SafetyLimitError):
        drv.set_vel_cp(3.0)             # ZuKa default 3 m/s now refused
    with pytest.raises(SafetyLimitError):
        drv.write_variable("$MACHINE_X", "1")
    drv.set_override(10)
    drv.soft_stop()
    assert drv.client.read("$OV_PRO") == "0"


def test_ownership_guard(drv, server, cfg):
    with KVPClient(server.host, server.port) as c2:
        other = KRLDriver(c2, cfg)
        other.take_control()
        with pytest.raises(OwnershipError):
            drv.noop()
        other.noop()


def test_no_drives_like_bare_krc4(cfg):
    with MockKVPServer(port=0, krc=MockKRC(drives_available=False)) as srv:
        with KVPClient(srv.host, srv.port) as c:
            d = KRLDriver(c, cfg)
            d.take_control()
            assert d.globals_present() and not d.driver_running()
            st = monitor.read_status(c)          # monitoring still works
            assert st["axes"] == [0, -90, 90, 0, 0, 0]
            d.set_output(1, True)                # I/O still works
            assert d.read_output(1)
            with pytest.raises(DriverNotRunning):
                d.home()
