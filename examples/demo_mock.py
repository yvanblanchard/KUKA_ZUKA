"""Run a short motion sequence against the built-in simulator (no hardware needed).

    .venv\Scripts\python examples\demo_mock.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kuka_zuka import Config, KRLDriver, KVPClient
from kuka_zuka.mock import MockKRC, MockKVPServer

with MockKVPServer(port=0, krc=MockKRC(time_scale=0.1)) as srv:
    with KVPClient(srv.host, srv.port) as client:
        drv = KRLDriver(client, Config.load())
        drv.take_control()
        drv.set_override(10)
        drv.set_vel_ptp(10)
        for axes in ([0, -90, 90, 0, 0, 0], [45, -90, 90, 0, 0, 0], [-45, -80, 100, 0, 20, 0]):
            drv.ptp_axis(axes)
            print("axes", drv.read_axes())
        drv.lin([500, 0, 800, None, None, None])
        drv.lin_rel([0, 20, 0, 0, 0, 0])
        print("pose", drv.read_pose())
        drv.home()
        drv.release_control()
