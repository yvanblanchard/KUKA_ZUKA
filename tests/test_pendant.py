"""Headless check of app/pendant.py against the built-in mock (Streamlit AppTest)."""
import time
from pathlib import Path

import pytest

pytest.importorskip("streamlit")
from streamlit.testing.v1 import AppTest  # noqa: E402

APP = str(Path(__file__).resolve().parent.parent / "app" / "pendant.py")


def test_pendant_on_mock():
    at = AppTest.from_file(APP, default_timeout=30).run()
    assert not at.exception
    at.sidebar.radio(key="kind").set_value("Mock (simulator)").run()
    at.sidebar.button[0].click().run()                     # Connect
    assert not at.exception
    ss = at.session_state
    assert ss["client"] is not None and not ss["control"]  # monitor-only by default
    assert at.button(key="jog_A1_1").disabled

    at.sidebar.button[2].click().run()                     # Take control
    assert ss["control"]
    drv = ss["driver"]
    drv.client.write("$OV_PRO", 100)
    a1 = drv.read_axes()[0]
    at.button(key="jog_A1_1").click().run()                # non-blocking jog
    assert not at.exception
    t_end = time.monotonic() + 15
    while drv.is_busy() and time.monotonic() < t_end:
        time.sleep(0.05)
    assert drv.read_axes()[0] == pytest.approx(a1 + 1.0, abs=1e-3)
    assert any("A1 +1°" in m for _, _, m in ss["msgs"])
