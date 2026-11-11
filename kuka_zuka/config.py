"""Configuration (robot limits + safety caps), loaded from ``config.yaml``."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


@dataclass
class SafetyConfig:
    max_override: float = 10.0          # % cap on $OV_PRO written from the PC
    max_vel_cp: float = 0.1             # m/s cap on $VEL.CP
    max_vel_ptp: float = 10.0           # % cap on PTP axis velocity
    max_rel_deg: float = 10.0           # max single relative axis move / jog step [deg]
    max_rel_mm: float = 50.0            # max single relative cartesian translation [mm]
    max_rel_rot_deg: float = 10.0       # max single relative cartesian rotation [deg]
    axis_limits: list[list[float]] = field(default_factory=lambda: [
        [-170, 170], [-190, 45], [-120, 156], [-185, 185], [-120, 120], [-350, 350]])
    axis_margin_deg: float = 2.0        # soft-limit margin inside axis_limits
    write_denylist: list[str] = field(default_factory=lambda: [
        "$MACHINE", "$ROBCOR", "$CUSTOM", "$RAT_MOT", "$DEF_", "$AXIS_CAL", "$MAMES",
        "$SOFTN_END", "$SOFTP_END", "$SAFE", "$TORQUE"])


@dataclass
class RobotConfig:
    model: str = "KR10 R1100 sixx"
    home_axes: list[float] = field(default_factory=lambda: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
    n_inputs: int = 16
    n_outputs: int = 16


@dataclass
class ConnectionConfig:
    host: str = "172.31.1.147"
    port: int = 7000
    timeout_s: float = 2.0
    ack_timeout_s: float = 3.0          # KRLDRIVER must accept a command within this
    motion_timeout_s: float = 120.0     # a motion must finish within this
    poll_period_s: float = 0.05
    mock_port: int = 7001


@dataclass
class Config:
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Config":
        path = Path(path) if path else DEFAULT_CONFIG_PATH
        if not path.exists():
            return cls()
        import yaml
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return cls(
            connection=ConnectionConfig(**data.get("connection", {})),
            robot=RobotConfig(**data.get("robot", {})),
            safety=SafetyConfig(**data.get("safety", {})),
        )
