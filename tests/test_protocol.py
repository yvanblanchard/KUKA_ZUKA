import pytest

from kuka_zuka import krltypes as kt
from kuka_zuka import protocol as p


def test_read_request_matches_zuka_layout():
    # ZuKa: msgID(0) | len | 0 | nameLen | name
    frame = p.encode_request(0, "PRO_ID")
    assert frame == b"\x00\x00\x00\x09\x00\x00\x06PRO_ID"


def test_write_request_matches_zuka_layout():
    frame = p.encode_request(0x0102, "$OV_PRO", 10)
    body = b"\x01\x00\x07$OV_PRO\x00\x0210"
    assert frame == b"\x01\x02" + len(body).to_bytes(2, "big") + body


def test_request_roundtrip():
    frame = p.encode_request(7, "KRLD_POS", "{X 1.0}")
    mid, blen = p.decode_header(frame[:4])
    assert (mid, blen) == (7, len(frame) - 4)
    assert p.decode_request_body(frame[4:]) == (1, "KRLD_POS", "{X 1.0}")


def test_response_roundtrip_and_status():
    ok = p.encode_response(5, 0, "TRUE", ok=True)
    mid, blen = p.decode_header(ok[:4])
    r = p.decode_response_body(mid, ok[4:4 + blen])
    assert r.ok and r.value == "TRUE" and r.msg_id == 5
    bad = p.encode_response(6, 0, "", ok=False)
    r = p.decode_response_body(6, bad[4:])
    assert not r.ok and r.value == ""


def test_bad_inputs():
    with pytest.raises(p.ProtocolError):
        p.encode_request(0, "")
    with pytest.raises(p.ProtocolError):
        p.encode_request(0, "X", "")
    with pytest.raises(p.ProtocolError):
        p.encode_request(0, "ÄX")


def test_parse_aggregates():
    t, d = kt.parse_aggregate("{E6AXIS: A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0, E1 0.0}")
    assert t == "E6AXIS" and d["A2"] == -90.0
    assert kt.axes_of("{A1 1, A2 2, A3 3, A4 4, A5 5, A6 6}") == [1, 2, 3, 4, 5, 6]
    t, d = kt.parse_aggregate("{E6POS: X 485.5, Y 0.0, Z 650.0, A 0.0, B 90.0, C 0.0, S 2, T 35}")
    assert d["S"] == 2 and isinstance(d["S"], int) and d["X"] == 485.5
    assert kt.parse_aggregate("{X 1}") == (None, {"X": 1})


def test_format_aggregates():
    assert kt.axis_agg([0, -90, 90.5, 0, 0, 0]) == "{A1 0, A2 -90, A3 90.5, A4 0, A5 0, A6 0}"
    assert kt.format_aggregate({"X": 1.0, "Y": None, "Z": -2.25}) == "{X 1.0, Z -2.25}"
    assert kt.pose_agg([1, 2, 3, 0, 0, 0], {"S": 2, "T": 35}).endswith("S 2, T 35}")


def test_scalars():
    assert kt.parse_bool("TRUE") and not kt.parse_bool("false")
    assert kt.parse_str('"KRLDRIVER  "') == "KRLDRIVER"
    assert kt.parse_enum("#P_ACTIVE") == "P_ACTIVE"
