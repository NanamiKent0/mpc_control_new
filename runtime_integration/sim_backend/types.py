"""Self-contained simulation dataclasses owned by `mpc_control_new`."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any


SIM_MODULE_IDS: tuple[str, ...] = ("tip", "joint1", "joint2", "joint3", "joint4", "joint5")
SIM_JOINT_IDS: tuple[str, ...] = tuple(module_id for module_id in SIM_MODULE_IDS if module_id != "tip")
SIM_ADJACENT_PAIRS: tuple[tuple[str, str], ...] = (
    ("joint1", "tip"),
    ("joint2", "joint1"),
    ("joint3", "joint2"),
    ("joint4", "joint3"),
    ("joint5", "joint4"),
)


def joint_index(module_id: str) -> int | None:
    """Return the numeric index for one `jointN` module id."""
    if not str(module_id).startswith("joint"):
        return None
    suffix = str(module_id)[5:]
    return int(suffix) if suffix.isdigit() else None


def pair_key(active_module: str, passive_module: str) -> str:
    """Return the stable directed pair key."""
    return f"{active_module}->{passive_module}"


def pair_field_prefix(active_module: str, passive_module: str) -> str:
    """Return the stable state-field prefix for one adjacent pair."""
    return f"{passive_module}_{active_module}"


def pair_distance_attr(active_module: str, passive_module: str) -> str:
    """Return the `SimState` distance attribute for one adjacent pair."""
    return f"{pair_field_prefix(active_module, passive_module)}_distance_mm"


def pair_orientation_attr(active_module: str, passive_module: str) -> str:
    """Return the `SimState` orientation attribute for one adjacent pair."""
    return f"{pair_field_prefix(active_module, passive_module)}_orientation_error_deg"


def pair_coupling_attr(active_module: str, passive_module: str) -> str:
    """Return the `SimState` coupling attribute for one adjacent pair."""
    return f"{pair_field_prefix(active_module, passive_module)}_coupled"


def module_position_attr(module_id: str) -> str:
    """Return the `SimState` position attribute for one module."""
    if module_id == "tip":
        return "g"
    index = joint_index(module_id)
    if index is None:
        raise ValueError(f"unsupported_module_id:{module_id}")
    return f"c{index}"


def module_orientation_attr(module_id: str) -> str | None:
    """Return the `SimState` orientation attribute for one module."""
    if module_id == "tip":
        return None
    index = joint_index(module_id)
    if index is None:
        raise ValueError(f"unsupported_module_id:{module_id}")
    return f"psi{index}"


def module_bend_attr(module_id: str) -> str | None:
    """Return the `SimState` bend attribute for one joint module."""
    if module_id == "tip":
        return None
    index = joint_index(module_id)
    if index is None:
        raise ValueError(f"unsupported_module_id:{module_id}")
    return f"theta{index}"


def command_linear_attr(module_id: str) -> str:
    """Return the `SimCommand` linear-velocity attribute for one module."""
    if module_id == "tip":
        return "tip_growth_mm_s"
    return f"{module_id}_crawl_mm_s"


def command_rotate_attr(module_id: str) -> str | None:
    """Return the `SimCommand` rotate-velocity attribute for one module."""
    if module_id == "tip":
        return None
    return f"{module_id}_rotate_deg_s"


def command_bend_attr(module_id: str) -> str | None:
    """Return the `SimCommand` bend-velocity attribute for one module."""
    if module_id == "tip":
        return None
    return f"{module_id}_bend_deg_s"


def command_electromagnet_attr(module_id: str) -> str:
    """Return the `SimCommand` electromagnet attribute for one module."""
    if module_id == "tip":
        return "tip_electromagnet_on"
    return f"{module_id}_electromagnet_on"


@dataclass(slots=True)
class CouplingState:
    """Discrete contact state between neighboring modules."""

    tip_joint1_coupled: bool = True
    joint1_joint2_coupled: bool = True
    joint2_joint3_coupled: bool = True
    joint3_joint4_coupled: bool = True
    joint4_joint5_coupled: bool = True

    def to_dict(self) -> dict[str, bool]:
        """Return the coupling state as a JSON-friendly mapping."""
        return {
            "tip_joint1_coupled": bool(self.tip_joint1_coupled),
            "joint1_joint2_coupled": bool(self.joint1_joint2_coupled),
            "joint2_joint3_coupled": bool(self.joint2_joint3_coupled),
            "joint3_joint4_coupled": bool(self.joint3_joint4_coupled),
            "joint4_joint5_coupled": bool(self.joint4_joint5_coupled),
        }


@dataclass(slots=True)
class SimState:
    """Kinematic simulator state stored inside `mpc_control_new`."""

    g: float = 0.0
    c1: float = 0.0
    c2: float = 0.0
    c3: float = 0.0
    c4: float = 0.0
    c5: float = 0.0
    theta1: float = 0.0
    theta2: float = 0.0
    theta3: float = 0.0
    theta4: float = 0.0
    theta5: float = 0.0
    psi1: float = 0.0
    psi2: float = 0.0
    psi3: float = 0.0
    psi4: float = 0.0
    psi5: float = 0.0
    tip_joint1_coupled: bool = True
    joint1_joint2_coupled: bool = True
    joint2_joint3_coupled: bool = True
    joint3_joint4_coupled: bool = True
    joint4_joint5_coupled: bool = True
    sim_time_s: float = 0.0
    seq: int = 0
    tip_joint1_distance_mm: float | None = None
    joint1_joint2_distance_mm: float | None = None
    joint2_joint3_distance_mm: float | None = None
    joint3_joint4_distance_mm: float | None = None
    joint4_joint5_distance_mm: float | None = None
    tip_joint1_orientation_error_deg: float | None = None
    joint1_joint2_orientation_error_deg: float | None = None
    joint2_joint3_orientation_error_deg: float | None = None
    joint3_joint4_orientation_error_deg: float | None = None
    joint4_joint5_orientation_error_deg: float | None = None

    def copy(self) -> "SimState":
        """Return a defensive copy."""
        return replace(self)

    def coupling_state(self) -> CouplingState:
        """Return the coupling flags as a dedicated dataclass."""
        return CouplingState(
            tip_joint1_coupled=bool(self.tip_joint1_coupled),
            joint1_joint2_coupled=bool(self.joint1_joint2_coupled),
            joint2_joint3_coupled=bool(self.joint2_joint3_coupled),
            joint3_joint4_coupled=bool(self.joint3_joint4_coupled),
            joint4_joint5_coupled=bool(self.joint4_joint5_coupled),
        )

    def to_dict(self) -> dict[str, Any]:
        """Convert the state into a JSON-friendly dictionary."""
        payload: dict[str, Any] = {
            "sim_time_s": float(self.sim_time_s),
            "seq": int(self.seq),
        }
        payload[module_position_attr("tip")] = float(self.g)
        for module_id in SIM_JOINT_IDS:
            position_attr = module_position_attr(module_id)
            bend_attr = module_bend_attr(module_id)
            orientation_attr = module_orientation_attr(module_id)
            assert bend_attr is not None
            assert orientation_attr is not None
            payload[position_attr] = float(getattr(self, position_attr))
            payload[bend_attr] = float(getattr(self, bend_attr))
            payload[orientation_attr] = float(getattr(self, orientation_attr))
        for active_module, passive_module in SIM_ADJACENT_PAIRS:
            payload[pair_coupling_attr(active_module, passive_module)] = bool(
                getattr(self, pair_coupling_attr(active_module, passive_module))
            )
            payload[pair_distance_attr(active_module, passive_module)] = _optional_float_to_payload(
                getattr(self, pair_distance_attr(active_module, passive_module))
            )
            payload[pair_orientation_attr(active_module, passive_module)] = _optional_float_to_payload(
                getattr(self, pair_orientation_attr(active_module, passive_module))
            )
        return payload

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "SimState":
        """Build a simulator state from a dictionary-like payload."""
        payload = dict(data or {})
        kwargs: dict[str, Any] = {
            "g": float(payload.get("g", 0.0)),
            "sim_time_s": float(payload.get("sim_time_s", 0.0)),
            "seq": int(payload.get("seq", 0)),
        }
        for module_id in SIM_JOINT_IDS:
            position_attr = module_position_attr(module_id)
            bend_attr = module_bend_attr(module_id)
            orientation_attr = module_orientation_attr(module_id)
            assert bend_attr is not None
            assert orientation_attr is not None
            kwargs[position_attr] = float(payload.get(position_attr, 0.0))
            kwargs[bend_attr] = float(payload.get(bend_attr, 0.0))
            kwargs[orientation_attr] = float(payload.get(orientation_attr, 0.0))
        for active_module, passive_module in SIM_ADJACENT_PAIRS:
            kwargs[pair_coupling_attr(active_module, passive_module)] = bool(
                payload.get(pair_coupling_attr(active_module, passive_module), True)
            )
            kwargs[pair_distance_attr(active_module, passive_module)] = _optional_float(
                payload.get(pair_distance_attr(active_module, passive_module))
            )
            kwargs[pair_orientation_attr(active_module, passive_module)] = _optional_float(
                payload.get(pair_orientation_attr(active_module, passive_module))
            )
        return cls(**kwargs)


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
    joint3_crawl_mm_s: float = 0.0
    joint3_bend_deg_s: float = 0.0
    joint3_rotate_deg_s: float = 0.0
    joint4_crawl_mm_s: float = 0.0
    joint4_bend_deg_s: float = 0.0
    joint4_rotate_deg_s: float = 0.0
    joint5_crawl_mm_s: float = 0.0
    joint5_bend_deg_s: float = 0.0
    joint5_rotate_deg_s: float = 0.0
    tip_electromagnet_on: bool | None = None
    joint1_electromagnet_on: bool | None = None
    joint2_electromagnet_on: bool | None = None
    joint3_electromagnet_on: bool | None = None
    joint4_electromagnet_on: bool | None = None
    joint5_electromagnet_on: bool | None = None

    def copy(self) -> "SimCommand":
        """Return a defensive copy."""
        return replace(self)

    def to_dict(self) -> dict[str, Any]:
        """Convert the command into a JSON-friendly dictionary."""
        payload: dict[str, Any] = {
            "tip_growth_mm_s": float(self.tip_growth_mm_s),
            "tip_electromagnet_on": self.tip_electromagnet_on,
        }
        for module_id in SIM_JOINT_IDS:
            linear_attr = command_linear_attr(module_id)
            bend_attr = command_bend_attr(module_id)
            rotate_attr = command_rotate_attr(module_id)
            assert bend_attr is not None
            assert rotate_attr is not None
            payload[linear_attr] = float(getattr(self, linear_attr))
            payload[bend_attr] = float(getattr(self, bend_attr))
            payload[rotate_attr] = float(getattr(self, rotate_attr))
            payload[command_electromagnet_attr(module_id)] = getattr(self, command_electromagnet_attr(module_id))
        return payload

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "SimCommand":
        """Build a command from a dictionary-like payload."""
        payload = dict(data or {})
        kwargs: dict[str, Any] = {
            "tip_growth_mm_s": float(payload.get("tip_growth_mm_s", 0.0)),
            "tip_electromagnet_on": _optional_bool(payload.get("tip_electromagnet_on")),
        }
        for module_id in SIM_JOINT_IDS:
            linear_attr = command_linear_attr(module_id)
            bend_attr = command_bend_attr(module_id)
            rotate_attr = command_rotate_attr(module_id)
            assert bend_attr is not None
            assert rotate_attr is not None
            kwargs[linear_attr] = float(payload.get(linear_attr, 0.0))
            kwargs[bend_attr] = float(payload.get(bend_attr, 0.0))
            kwargs[rotate_attr] = float(payload.get(rotate_attr, 0.0))
            kwargs[command_electromagnet_attr(module_id)] = _optional_bool(payload.get(command_electromagnet_attr(module_id)))
        return cls(**kwargs)


@dataclass(slots=True)
class SimConfig:
    """Minimal simulation timing and standalone topic configuration."""

    dt: float = 0.05
    state_topic: str = "sim/state"
    supported_namespaces: tuple[str, ...] = SIM_MODULE_IDS
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


def _optional_float_to_payload(value: float | None) -> float | None:
    """Normalize optional float payloads for serialization."""
    return None if value is None else float(value)


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


__all__ = [
    "SIM_ADJACENT_PAIRS",
    "SIM_JOINT_IDS",
    "SIM_MODULE_IDS",
    "CouplingState",
    "SimCommand",
    "SimConfig",
    "SimState",
    "command_bend_attr",
    "command_electromagnet_attr",
    "command_linear_attr",
    "command_rotate_attr",
    "joint_index",
    "module_bend_attr",
    "module_orientation_attr",
    "module_position_attr",
    "pair_coupling_attr",
    "pair_distance_attr",
    "pair_field_prefix",
    "pair_key",
    "pair_orientation_attr",
]
