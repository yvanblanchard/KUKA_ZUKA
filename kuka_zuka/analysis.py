"""Post-processing of recorded robot data: joint & TCP velocities / accelerations.

Pipeline (per signal group, each with its own timestamps):
  1. resample onto a uniform time grid (linear interpolation) at the median sampling rate
  2. Savitzky-Golay smoothing + analytic derivatives (polynomial fit per window, proper edges)
  3. TCP orientation: KUKA ABC -> rotation matrix R = Rz(A)·Ry(B)·Rx(C); angular velocity from
     the skew part of dR/dt · R^T (no Euler-angle wrap / singularity issues), deg/s.

Units: axes deg, deg/s, deg/s² ; TCP mm, mm/s, mm/s² ; angular deg/s, deg/s².
"""
from __future__ import annotations

import math

import numpy as np
import pandas as pd

AXES = ["A1", "A2", "A3", "A4", "A5", "A6"]
XYZ = ["X", "Y", "Z"]
ABC = ["A", "B", "C"]


# ------------------------------------------------------------------ Savitzky-Golay (numpy only)
def savgol(y: np.ndarray, window: int, poly: int, deriv: int = 0, dt: float = 1.0) -> np.ndarray:
    """Savitzky-Golay filter / derivative with polynomial-fit edges (no padding artefacts)."""
    y = np.asarray(y, dtype=float)
    n = len(y)
    if window % 2 == 0:
        window += 1
    if n < window:  # too short: plain finite differences
        out = y.copy()
        for _ in range(deriv):
            out = np.gradient(out, dt) if len(out) > 1 else np.zeros_like(out)
        return out
    poly = min(poly, window - 1)
    h = window // 2
    x = np.arange(-h, h + 1, dtype=float)
    pinv = np.linalg.pinv(np.vander(x, poly + 1, increasing=True))     # (poly+1, window)

    out = np.empty(n)
    if deriv <= poly:
        c = pinv[deriv] * math.factorial(deriv)
        out[h:n - h] = np.convolve(y, c[::-1], mode="valid")
    else:
        out[h:n - h] = 0.0

    def poly_deriv_at(coefs: np.ndarray, xs: np.ndarray) -> np.ndarray:
        res = np.zeros_like(xs)
        for j in range(deriv, len(coefs)):
            res += coefs[j] * math.factorial(j) / math.factorial(j - deriv) * xs ** (j - deriv)
        return res

    a0 = pinv @ y[:window]
    out[:h] = poly_deriv_at(a0, np.arange(h, dtype=float) - h)
    a1 = pinv @ y[n - window:]
    out[n - h:] = poly_deriv_at(a1, np.arange(1, h + 1, dtype=float))
    return out / dt ** deriv


def default_window(fs: float, smooth_s: float) -> int:
    w = max(5, int(round(smooth_s * fs)))
    return w if w % 2 else w + 1


# ------------------------------------------------------------------ helpers
def resample(t: np.ndarray, cols: dict[str, np.ndarray], fs: float | None = None):
    """Uniform resampling. Returns (t_uniform, {name: values}, fs)."""
    t = np.asarray(t, float)
    order = np.argsort(t)
    t = t[order]
    keep = np.concatenate([[True], np.diff(t) > 1e-9])
    t = t[keep]
    if fs is None:
        dt = np.median(np.diff(t)) if len(t) > 1 else 1.0
        fs = 1.0 / dt
    n = max(2, int(math.floor((t[-1] - t[0]) * fs)) + 1)
    tu = t[0] + np.arange(n) / fs
    out = {k: np.interp(tu, t, np.asarray(v, float)[order][keep]) for k, v in cols.items()}
    return tu, out, fs


def euler_kuka_to_R(a_deg, b_deg, c_deg) -> np.ndarray:
    """KUKA ABC (A about Z, B about Y, C about X) -> (n, 3, 3) rotation matrices."""
    a, b, c = (np.radians(np.asarray(v, float)) for v in (a_deg, b_deg, c_deg))
    ca, sa, cb, sb, cc, sc = np.cos(a), np.sin(a), np.cos(b), np.sin(b), np.cos(c), np.sin(c)
    R = np.empty((len(a), 3, 3))
    R[:, 0, 0] = ca * cb
    R[:, 0, 1] = ca * sb * sc - sa * cc
    R[:, 0, 2] = ca * sb * cc + sa * sc
    R[:, 1, 0] = sa * cb
    R[:, 1, 1] = sa * sb * sc + ca * cc
    R[:, 1, 2] = sa * sb * cc - ca * sc
    R[:, 2, 0] = -sb
    R[:, 2, 1] = cb * sc
    R[:, 2, 2] = cb * cc
    return R


# ------------------------------------------------------------------ main entry points
def joint_kinematics(t, axes: dict[str, np.ndarray], smooth_s: float = 0.1, poly: int = 3) -> pd.DataFrame:
    tu, cols, fs = resample(t, axes)
    w = default_window(fs, smooth_s)
    dt = 1.0 / fs
    df = pd.DataFrame({"t": tu - tu[0]})
    for k in AXES:
        if k not in cols:
            continue
        df[f"{k}"] = savgol(cols[k], w, poly, 0, dt)
        df[f"{k}_vel"] = savgol(cols[k], w, poly, 1, dt)
        df[f"{k}_acc"] = savgol(cols[k], w, poly, 2, dt)
    df.attrs.update(fs=fs, window=w, poly=poly)
    return df


def tcp_kinematics(t, pose: dict[str, np.ndarray], smooth_s: float = 0.1, poly: int = 3) -> pd.DataFrame:
    tu, cols, fs = resample(t, pose)
    w = default_window(fs, smooth_s)
    dt = 1.0 / fs
    df = pd.DataFrame({"t": tu - tu[0]})
    v = np.zeros((len(tu), 3))
    acc = np.zeros((len(tu), 3))
    for i, k in enumerate(XYZ):
        df[k] = savgol(cols[k], w, poly, 0, dt)
        v[:, i] = savgol(cols[k], w, poly, 1, dt)
        acc[:, i] = savgol(cols[k], w, poly, 2, dt)
        df[f"v{k.lower()}"] = v[:, i]
        df[f"a{k.lower()}"] = acc[:, i]
    speed = np.linalg.norm(v, axis=1)
    df["tcp_speed"] = speed                                         # mm/s
    df["tcp_acc"] = np.linalg.norm(acc, axis=1)                     # mm/s² (vector norm)
    with np.errstate(invalid="ignore", divide="ignore"):
        tang = np.where(speed > 1e-3, np.einsum("ij,ij->i", v, acc) / speed, 0.0)
    df["tcp_acc_tangential"] = tang                                 # mm/s² (along path)

    for k in ABC:
        df[k] = cols[k]
    R = euler_kuka_to_R(cols["A"], cols["B"], cols["C"])
    Rs = np.empty_like(R)
    Rd = np.empty_like(R)
    for i in range(3):
        for j in range(3):
            Rs[:, i, j] = savgol(R[:, i, j], w, poly, 0, dt)
            Rd[:, i, j] = savgol(R[:, i, j], w, poly, 1, dt)
    W = Rd @ np.transpose(Rs, (0, 2, 1))
    W = 0.5 * (W - np.transpose(W, (0, 2, 1)))
    omega = np.degrees(np.stack([W[:, 2, 1], W[:, 0, 2], W[:, 1, 0]], axis=1))  # deg/s, base frame
    alpha = np.stack([savgol(omega[:, i], w, poly, 1, dt) for i in range(3)], axis=1)
    for i, k in enumerate(("wx", "wy", "wz")):
        df[k] = omega[:, i]
    df["tcp_ang_speed"] = np.linalg.norm(omega, axis=1)            # deg/s
    df["tcp_ang_acc"] = np.linalg.norm(alpha, axis=1)              # deg/s²
    df.attrs.update(fs=fs, window=w, poly=poly)
    return df


def summarize(joints: pd.DataFrame | None, tcp: pd.DataFrame | None) -> dict:
    s: dict = {}
    if joints is not None and len(joints):
        s["duration_s"] = float(joints["t"].iloc[-1])
        s["joint_fs_hz"] = float(joints.attrs.get("fs", float("nan")))
        s["axes"] = {k: {"min_deg": float(joints[k].min()), "max_deg": float(joints[k].max()),
                         "max_abs_vel_deg_s": float(joints[f"{k}_vel"].abs().max()),
                         "max_abs_acc_deg_s2": float(joints[f"{k}_acc"].abs().max())}
                     for k in AXES if k in joints}
    if tcp is not None and len(tcp):
        s["tcp_fs_hz"] = float(tcp.attrs.get("fs", float("nan")))
        s["tcp"] = {"max_speed_mm_s": float(tcp["tcp_speed"].max()),
                    "max_acc_mm_s2": float(tcp["tcp_acc"].max()),
                    "max_ang_speed_deg_s": float(tcp["tcp_ang_speed"].max()),
                    "max_ang_acc_deg_s2": float(tcp["tcp_ang_acc"].max()),
                    "path_length_mm": float(np.sum(np.linalg.norm(np.diff(tcp[XYZ].to_numpy(), axis=0), axis=1)))}
    return s
