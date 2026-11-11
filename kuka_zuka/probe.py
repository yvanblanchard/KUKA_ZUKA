"""Bare-controller smoke test (phase P2) — no motion, works without the manipulator.

    python -m kuka_zuka.probe 172.31.1.147
    python -m kuka_zuka.probe 172.31.1.147 --write-test --out 1
    python -m kuka_zuka.probe --mock            # against the built-in simulator
"""
from __future__ import annotations

import argparse
import sys
import time

from . import monitor
from .client import KVPClient, KVPError, KVPVariableError
from .driver import KRLDriver


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7000)
    ap.add_argument("--mock", action="store_true", help="start a mock controller (no drives) and probe it")
    ap.add_argument("--write-test", action="store_true", help="write/read back PRO_ID (needs config.dat globals)")
    ap.add_argument("--out", type=int, default=0, help="pulse $OUT[n] TRUE->FALSE (CAUTION: real outputs!)")
    a = ap.parse_args(argv)

    srv = None
    if a.mock:
        from .mock import MockKRC, MockKVPServer
        srv = MockKVPServer(port=0, krc=MockKRC(drives_available=False)).start()
        a.host, a.port = srv.host, srv.port

    ok = True
    try:
        with KVPClient(a.host, a.port) as c:
            print(f"[OK] connected to {a.host}:{a.port}  (ping {c.ping():.1f} ms)")
            print("\n-- controller info --")
            for k, v in monitor.read_info(c).items():
                print(f"  {monitor.INFO_VARS[k]:<16} {k:<16} {v}")
            print("\n-- status --")
            st = monitor.read_status(c)
            for k, label in monitor.STATUS_VARS.items():
                print(f"  {label:<24} {k:<14} {st['raw'][k]}")
            print(f"  {'Axes':<24} $AXIS_ACT      {st['axes']}")
            print(f"  {'TCP':<24} $POS_ACT       {st['pose']}")

            drv = KRLDriver(c)
            print("\n-- KRLDRIVER --")
            print(f"  globals in $config.dat : {drv.globals_present()}")
            print(f"  KRLDRIVER running      : {drv.driver_running()}")
            print(f"  driver vars            : {drv.driver_state()}")

            if a.write_test:
                pid = int(time.time()) % 30000
                c.write("PRO_ID", pid)
                back = int(float(c.read("PRO_ID")))
                print(f"\n[{'OK' if back == pid else 'FAIL'}] write/read PRO_ID: wrote {pid}, read {back}")
                ok &= back == pid
            if a.out:
                print(f"\n-- pulsing $OUT[{a.out}] --")
                c.write(f"$OUT[{a.out}]", True)
                v1 = c.read(f"$OUT[{a.out}]")
                time.sleep(0.5)
                c.write(f"$OUT[{a.out}]", False)
                v2 = c.read(f"$OUT[{a.out}]")
                print(f"  after TRUE: {v1}   after FALSE: {v2}")
                ok &= v1.upper() == "TRUE" and v2.upper() == "FALSE"
    except KVPVariableError as exc:
        print(f"[FAIL] {exc}")
        ok = False
    except KVPError as exc:
        print(f"[FAIL] {exc}")
        ok = False
    finally:
        if srv:
            srv.stop()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
