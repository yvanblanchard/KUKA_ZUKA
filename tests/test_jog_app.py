"""Headless check of app/jog_app.py against the built-in mock (Streamlit AppTest)."""
import time
from pathlib import Path

import pytest

pytest.importorskip("streamlit")
from streamlit.testing.v1 import AppTest  # noqa: E402

APP = str(Path(__file__).resolve().parent.parent / "app" / "jog_app.py")


def _msgs(at):
    return [m for _, _, m in at.session_state["msgs"]]


def _wait_idle(at, timeout=15.0):
    drv = at.session_state["driver"]
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if drv.client.read("KRLD_COM") == "0" and drv.client.read("KRLD_STATE") == "0":
            return
        time.sleep(0.05)
    raise AssertionError("mock robot did not become idle")


def test_unreachable_robot_is_reported():
    at = AppTest.from_file(APP, default_timeout=30).run()
    at.sidebar.text_input(key="host").set_value("127.0.0.1")
    at.sidebar.number_input(key="port").set_value(1).run()
    at.sidebar.button[0].click().run()                     # Test network
    at.sidebar.button[1].click().run()                     # Connect
    assert not at.exception
    errors = [m for _, lvl, m in at.session_state["msgs"] if lvl == "ERROR"]
    assert any("unreachable" in m for m in errors), errors
    assert any("Cannot reach KUKAVARPROXY" in m for m in errors), errors
    assert at.session_state["client"] is None


def test_jog_app_on_mock():
    at = AppTest.from_file(APP, default_timeout=30).run()
    assert not at.exception
    at.sidebar.radio(key="kind").set_value("Mock (simulator)").run()
    at.sidebar.button[0].click().run()                     # Connect
    assert not at.exception
    ss = at.session_state
    assert ss["client"] is not None and ss["control"], _msgs(at)
    assert any("Connected to mock" in m for m in _msgs(at))

    # speed
    at.sidebar.slider(key="speed").set_value(10).run()
    assert ss["driver"].client.read("$OV_PRO") == "10"

    # jog A1 +
    a1 = ss["driver"].read_axes()[0]
    at.button(key="jog_p0").click().run()
    assert not at.exception
    _wait_idle(at)
    assert ss["driver"].read_axes()[0] == pytest.approx(a1 + 1.0, abs=1e-3), _msgs(at)

    # cartesian target (LIN)
    at.number_input(key="tgt_X").set_value(ss["tgt_X"] + 20.0)
    at.radio(key="mtype").set_value("LIN")
    at.checkbox(key="tgt_ok").check().run()
    move = at.button(key="go_target")
    assert not move.disabled
    x_target = ss["tgt_X"]
    move.click().run()
    assert not at.exception
    _wait_idle(at)
    assert ss["driver"].read_pose()["X"] == pytest.approx(x_target, abs=1e-2), _msgs(at)
    assert not ss["tgt_ok"]

    assert not any(lvl == "ERROR" for _, lvl, _ in ss["msgs"]), _msgs(at)

    # jog beyond the soft limit of A5 (±118°) is refused with a warning, no crash
    at.sidebar.select_slider(key="step").set_value(10.0).run()
    ss["driver"].client.write("$OV_PRO", 100)              # fast mock moves for the loop below
    for _ in range(14):
        at.button(key="jog_p4").click().run()
        _wait_idle(at)
    assert not at.exception
    assert any("outside soft limits" in m for m in _msgs(at)), _msgs(at)

    # bare controller (arm unplugged): KRLDRIVER stops -> warning + jog disabled
    at.sidebar.checkbox[0].uncheck().run()
    try:
        assert at.button(key="jog_p0").disabled
        assert any("Robot interpreter" in m for lvl, m in
                   [(lvl, m) for _, lvl, m in ss["msgs"] if lvl == "WARN"]), _msgs(at)
    finally:
        at.sidebar.checkbox[0].check().run()

    at.sidebar.button[1].click().run()                     # Disconnect
    assert ss["client"] is None


def test_krl_program_run_and_record():
    at = AppTest.from_file(APP, default_timeout=30).run()
    at.sidebar.radio(key="kind").set_value("Mock (simulator)").run()
    at.sidebar.button[0].click().run()                     # Connect
    ss = at.session_state
    ss["driver"].client.write("$OV_PRO", 100)              # fast mock moves
    _wait_idle(at)
    at.text_area(key="krl_code").set_value(
        "BAS(#VEL_PTP, 10)\nPTP {A1 0, A2 -90, A3 90}\nPTP {A1 15, A2 -80}\nPTP_REL {A6 5}\nPTP {A1 0}").run()
    assert not at.error, [e.value for e in at.error]
    assert at.button(key="krl_run").disabled               # path-clear confirmation missing
    at.checkbox(key="krl_ok").check().run()
    at.button(key="krl_run").click().run()
    assert not at.exception
    runner = ss["runner"]
    runner.join(30)
    assert runner.status == "finished", (runner.error, _msgs(at))
    ss["rec"].join(10)
    at.run()                                               # refresh: messages + results
    assert any("KRL program finished" in m for m in _msgs(at)), _msgs(at)
    assert ss["rec"].n_samples > 10
    a1 = ss["rec"].raw_frame()["A1"]
    assert a1.max() == pytest.approx(15.0, abs=0.5)
    assert not at.exception
    assert any(b.label == "Joints CSV" for b in at.get("download_button")), "results not shown"

    # syntax errors are shown and block the run
    at.text_area(key="krl_code").set_value("PTP {A1 0}\nFOR I=1 TO 3").run()
    assert any("Line 2" in e.value for e in at.error)
    assert at.button(key="krl_run").disabled
