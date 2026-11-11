"""Port of ZuKa ``example2.py`` (axis ping-pong) using the ZuKa-style compat API.

    .venv\Scripts\python examples\zuka_example2_port.py --host 172.31.1.147 --cycles 2
    .venv\Scripts\python examples\zuka_example2_port.py --mock

Speeds are capped by config.yaml (upstream used VEL_CP 3 m/s / OV 50 % — refused here).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kuka_zuka.compat import AXIS, OV_PRO, PTP, VEL_PTP, driver, init

ap = argparse.ArgumentParser()
ap.add_argument("--host")
ap.add_argument("--mock", action="store_true")
ap.add_argument("--cycles", type=int, default=1)
a = ap.parse_args()

srv = None
if a.mock:
    from kuka_zuka.mock import MockKRC, MockKVPServer
    srv = MockKVPServer(port=0, krc=MockKRC(time_scale=0.1)).start()
    init(srv.host, srv.port)
else:
    init(a.host)

try:
    VEL_PTP(10)
    OV_PRO(10)
    for _ in range(a.cycles):
        PTP(AXIS, 0, -90, 90, 0, 0, 0)
        PTP(AXIS, 90, -90, 90, 0, 0, 0)
        PTP(AXIS, -90, -90, 90, 0, 0, 0)
        PTP(AXIS, 0, -90, 90, 0, 0, 0)
        print("cycle done, axes:", driver().read_axes())
finally:
    driver().release_control()
    if srv:
        srv.stop()
