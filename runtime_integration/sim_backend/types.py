"""Self-contained simulation dataclasses copied into the new architecture package."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any


SIM_MODULE_IDS: tuple[str, str, str] = ("tip", "joint1", "joint2")


@dataclass(slots=True)
class CouplingState:
    """Discrete contact state between neighboring modules."""

    tip_joint1_coupled: bool = True
    joint1_joint2_coupled: bool = True


@dataclass(slots=True)
class SimState:
    """Minimal kinematic simulator state stored inside `mpc_control_new`."""

    g: float = 0.0
    c1: float = 0.0
    c2: float = 0.0
    theta1: float = 0.0
    theta2: float = 0.0
    psi1: float = 0.0
    psi2: float = 0.0
    tip_joint1_coupled: bool = True
    joint1_joint2_coupled: bool = True
    sim_time_s: float = 0.0
    seq: int = 0
    tip_joint1_distance_mm: float | None = None
    joint1_joint2_distance_mm: float | None = None
    tip_joint1_orientation_error_deg: float | None = None
    joint1_joint2_orientation_error_deg: float | None = None

    def copy(self) -> "SimState":
        """Return a defensive copy."""
        return replace(self)

    def coupling_state(self) -> CouplingState:
        """Return the coupling flags as a dedicated dataclass."""
        return CouplingState(
            tip_joint1_coupled=bool(self.tip_joint1_coupled),
            joint1_joint2_coupled=bool(self.joint1_joint2_coupled),
        )

    def to_dict(self) -> dict[str, Any]:
        """Convert the state into a JSON-friendly dictionary."""
        return {
            "g": float(self.g),
            "c1": float(self.c1),
            "c2": float(self.c2),
            "theta1": float(self.theta1),
            "theta2": float(self.theta2),
            "psi1": float(self.psi1),
            "psi2": float(self.psi2),
            "tip_joint1_coupled": bool(self.tip_joint1_coupled),
            "joint1_joint2_coupled": bool(self.joint1_joint2_coupled),
            "sim_time_s": float(self.sim_time_s),
            "seq": int(self.seq),
            "tip_joint1_distance_mm": None if self.tip_joint1_distance_mm is None else float(self.tip_joint1_distance_mm),
            "joint1_joint2_distance_mm": None if self.joint1_joint2_distance_mm is None else float(self.joint1_joint2_distance_mm),
            "tip_joint1_orientation_error_deg": None
            if self.tip_joint1_orientation_error_deg is None
            else float(self.tip_joint1_orientation_error_deg),
            "joint1_joint2_orientation_error_deg": None
            if self.joint1_joint2_orientation_error_deg is None
            else float(self.joint1_joint2_orientation_error_deg),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "SimState":
        """Build a simulator state from a dictionary-like payload."""
        payload = dict(data or {})
        return cls(
            g=float(payload.get("g", 0.0)),
            c1=float(payload.get("c1", 0.0)),
            c2=float(payload.get("c2", 0.0)),
            theta1=float(payload.get("theta1", 0.0)),
            theta2=float(payload.get("theta2", 0.0)),
            psi1=float(payload.get("psi1", 0.0)),
            psi2=float(payload.get("psi2", 0.0)),
            tip_joint1_coupled=bool(payload.get("tip_joint1_coupled", True)),
            joint1_joint2_coupled=bool(payload.get("joint1_joint2_coupled", True)),
            sim_time_s=float(payload.get("sim_time_s", 0.0)),
            seq=int(payload.get("seq", 0)),
            tip_joint1_distance_mm=_optional_float(payload.get("tip_joint1_distance_mm")),
            joint1_joint2_distance_mm=_optional_float(payload.get("joint1_joint2_distance_mm")),
            tip_joint1_orientation_error_deg=_optional_float(payload.get("tip_joint1_orientation_error_deg")),
            joint1_joint2_orientation_error_deg=_optional_float(payload.get("joint1_joint2_orientation_error_deg")),
        )


@dataclass(slots=True)
class SimCommand:
    """Persistent command state in physical units."""

    tip_growth_mm_s: float = 0.0
    joint1_crawl_mm_s: float = 0.0
    joint1_bend_deg_s: float = 0.0
    joint1_rotate_deg_s: float = 0.0
    joint2_crawl_mm_s: float = 0.0
    joint2_bend_deg_s: float = 0.0
    joint2_rotate_deg_s: float = 0.0
    tip_electromagnet_on: bool | None = None
    joint1_electromagnet_on: bool | None = None
    joint2_electromagnet_on: bool | None = None

    def copy(self) -> "SimCommand":
        """Return a defensive copy."""
        return replace(self)

    def to_dict(self) -> dict[str, Any]:
        """Convert the command into a JSON-friendly dictionary."""
        return {
            "tip_growth_mm_s": float(self.tip_growth_mm_s),
            "joint1_crawl_mm_s": float(self.joint1_crawl_mm_s),
            "joint1_bend_deg_s": float(self.joint1_bend_deg_s),
            "joint1_rotate_deg_s": float(self.joint1_rotate_deg_s),
            "joint2_crawl_mm_s": float(self.joint2_crawl_mm_s),
            "joint2_bend_deg_s": float(self.joint2_bend_deg_s),
            "joint2_rotate_deg_s": float(self.joint2_rotate_deg_s),
            "tip_electromagnet_on": self.tip_electromagnet_on,
            "joint1_electromagnet_on": self.joint1_electromagnet_on,
            "joint2_electromagnet_on": self.joint2_electromagnet_on,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "SimCommand":
        """Build a command from a dictionary-like payload."""
        payload = dict(data or {})
        return cls(
            tip_growth_mm_s=float(payload.get("tip_growth_mm_s", 0.0)),
            joint1_crawl_mm_s=float(payload.get("joint1_crawl_mm_s", 0.0)),
            joint1_bend_deg_s=float(payload.get("joint1_bend_deg_s", 0.0)),
            joint1_rotate_deg_s=float(payload.get("joint1_rotate_deg_s", 0.0)),
            joint2_crawl_mm_s=float(payload.get("joint2_crawl_mm_s", 0.0)),
            joint2_bend_deg_s=float(payload.get("joint2_bend_deg_s", 0.0)),
            joint2_rotate_deg_s=float(payload.get("joint2_rotate_deg_s", 0.0)),
            tip_electromagnet_on=_optional_bool(payload.get("tip_electromagnet_on")),
            joint1_electromagnet_on=_optional_bool(payload.get("joint1_electromagnet_on")),
            joint2_electromagnet_on=_optional_bool(payload.get("joint2_electromagnet_on")),
        )


@dataclass(slots=True)
class SimConfig:
    """Minimal simulation timing and initial-state configuration."""

    dt: float = 0.05
    initial_state: SimState = field(default_factory=SimState)

    def initial_state_copy(self) -> SimState:
        """Return a fresh initial state instance."""
        return self.initial_state.copy()


def _optional_float(value: Any) -> float | None:
    """Convert a scalar-like value into float while preserving missing data."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_bool(value: Any) -> bool | None:
    """Convert a scalar-like value into bool while preserving missing data."""
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return None
