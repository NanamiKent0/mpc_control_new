"""Helpers that translate legacy-like estimates into legacy-compatible state models.

These helpers remain available for compatibility tests and transition tooling, but
the new runtime-session main path should prefer runtime-frame inputs instead.
"""

from __future__ import annotations

from typing import Any

from ...control_core.models.geometry_observation import GeometryObservation
from ...control_core.models.module_state import ModuleState, make_joint_module_state, make_tip_module_state
from ...control_core.topology.pair_registry import PairExtractorRegistry
from ...control_core.topology.relation_state import RelationState, RelationType


def _path_get(source: Any, *path: str) -> Any:
    """Safely traverse dictionaries and attribute-style objects."""
    current = source
    for key in path:
        if current is None:
            return None
        if isinstance(current, dict):
            current = current.get(key)
        else:
            current = getattr(current, key, None)
    return current


def _safe_float(value: Any, default: float | None = None) -> float | None:
    """Convert a scalar-like object into float with a nullable fallback."""
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _note_missing_fields(*field_names: str) -> list[str]:
    """Return a normalized missing-field note list."""
    names = [field_name for field_name in field_names if field_name]
    if not names:
        return []
    return [f"missing_geometry_fields:{','.join(names)}"]


def _source_path(path: str | None) -> str | None:
    """Normalize a legacy source-field identifier into a readable schema path."""
    if path is None:
        return None
    if path.startswith(("geometry.", "control.")):
        return path
    if path.endswith("_fallback"):
        base = path.removesuffix("_fallback")
        if base.startswith("d_") or base.startswith("d_t"):
            return f"control.{path}"
        return f"geometry.{path}"
    return f"geometry.{path}"


def _normalize_bool(value: Any) -> bool | None:
    """Convert a bool-like object into `True`, `False`, or `None`."""
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    return None


def parse_joint_index(module_id: str) -> int | None:
    """Extract a numeric joint index when the identifier follows `jointN`."""
    if not module_id.startswith("joint"):
        return None
    suffix = module_id.removeprefix("joint")
    if not suffix.isdigit():
        return None
    return int(suffix)


def is_joint_module(module_id: str) -> bool:
    """Return whether the identifier follows the generic `jointN` pattern."""
    return parse_joint_index(module_id) is not None


def _module_type(module_id: str) -> str | None:
    """Infer the coarse module family from the module identifier."""
    if module_id == "tip":
        return "tip"
    if is_joint_module(module_id):
        return "joint"
    return None


def infer_module_type(module_id: str, observed_module_type: str | None = None) -> str | None:
    """Resolve the effective module type for legacy or runtime-facing builders."""
    normalized_observed = None if observed_module_type is None else str(observed_module_type).strip().lower()
    if normalized_observed in {"tip", "joint"}:
        return normalized_observed
    return _module_type(module_id)


def infer_relation_type(active_module: str, passive_module: str) -> RelationType | None:
    """Infer the relation type from active/passive module identifiers."""
    module_types = {
        infer_module_type(active_module),
        infer_module_type(passive_module),
    }
    if module_types == {"tip", "joint"}:
        return "tip_joint"
    if module_types == {"joint"}:
        return "joint_joint"
    return None


def _sorted_joint_modules(left: str, right: str) -> tuple[str, str]:
    """Return a stable joint pair order using numeric indices when available."""
    left_index = parse_joint_index(left)
    right_index = parse_joint_index(right)
    if left_index is not None and right_index is not None:
        return (left, right) if left_index <= right_index else (right, left)
    return tuple(sorted((left, right)))


def _tip_joint_prefix(active_module: str, passive_module: str) -> tuple[str, str]:
    """Return the legacy field prefix and joint module for a tip-joint pair."""
    if active_module == "tip" and is_joint_module(passive_module):
        return (f"tip_{passive_module}", passive_module)
    if passive_module == "tip" and is_joint_module(active_module):
        return (f"tip_{active_module}", active_module)
    raise ValueError(f"unsupported_tip_joint_pair:{active_module}->{passive_module}")


def _joint_joint_prefix(active_module: str, passive_module: str) -> tuple[str, str, str]:
    """Return the legacy field prefix and ordered joint identifiers."""
    left_joint, right_joint = _sorted_joint_modules(active_module, passive_module)
    return (f"{left_joint}_{right_joint}", left_joint, right_joint)


def _resolved_tip_joint_orientation(
    estimate: Any,
    *,
    prefix: str,
    joint_module: str,
) -> tuple[float | None, str | None]:
    """Resolve the best available signed orientation signal for a tip-joint pair."""
    signed_field = f"{prefix}_orientation_error_signed_deg"
    magnitude_field = f"{prefix}_orientation_error_deg"
    imu_field = f"{joint_module}_imu_z_to_base_deg"
    signed = _safe_float(_path_get(estimate, "geometry", signed_field), None)
    magnitude = _safe_float(_path_get(estimate, "geometry", magnitude_field), None)
    imu_fallback = _safe_float(_path_get(estimate, "geometry", imu_field), None)
    if signed is not None:
        return signed, signed_field
    if imu_fallback is not None:
        return imu_fallback, f"{imu_field}_fallback"
    if magnitude is not None:
        return magnitude, magnitude_field
    return None, None


def _resolved_joint_joint_orientation(
    estimate: Any,
    *,
    prefix: str,
    active_module: str,
    passive_module: str,
) -> tuple[float | None, str | None]:
    """Resolve a best-effort signed orientation signal for a joint-joint pair."""
    signed_field = f"{prefix}_orientation_error_signed_deg"
    magnitude_field = f"{prefix}_orientation_error_deg"
    signed = _safe_float(_path_get(estimate, "geometry", signed_field), None)
    magnitude = _safe_float(_path_get(estimate, "geometry", magnitude_field), None)
    if signed is not None:
        return signed, signed_field
    for module_id in (active_module, passive_module):
        imu_field = f"{module_id}_imu_z_to_base_deg"
        imu_fallback = _safe_float(_path_get(estimate, "geometry", imu_field), None)
        if imu_fallback is not None:
            return imu_fallback, f"{imu_field}_fallback"
    if magnitude is not None:
        return magnitude, magnitude_field
    return None, None


def _tip_joint_control_key(joint_module: str) -> str | None:
    """Return the legacy fallback control key for a tip-joint coupling flag."""
    joint_index = parse_joint_index(joint_module)
    if joint_index is None:
        return None
    return f"d_t{joint_index}"


def _joint_joint_control_key(left_joint: str, right_joint: str) -> str | None:
    """Return the legacy fallback control key for a joint-joint coupling flag."""
    left_index = parse_joint_index(left_joint)
    right_index = parse_joint_index(right_joint)
    if left_index is None or right_index is None:
        return None
    return f"d_{min(left_index, right_index)}{max(left_index, right_index)}"


def _coupling_state(estimate: Any, left_module: str, right_module: str) -> tuple[bool | None, str | None]:
    """Resolve the best available coupled flag for one tip/joint or joint/joint edge."""
    module_types = {_module_type(left_module), _module_type(right_module)}
    if module_types == {"tip", "joint"}:
        prefix, joint_module = _tip_joint_prefix(left_module, right_module)
        coupled_field = f"{prefix}_coupled"
        coupled = _normalize_bool(_path_get(estimate, "geometry", coupled_field))
        if coupled is not None:
            return coupled, f"geometry.{coupled_field}"
        control_key = _tip_joint_control_key(joint_module)
        if control_key is None:
            return None, None
        fallback = _normalize_bool(_path_get(estimate, "control", control_key))
        if fallback is None:
            return None, None
        return fallback, f"control.{control_key}_fallback"
    if module_types == {"joint"}:
        prefix, left_joint, right_joint = _joint_joint_prefix(left_module, right_module)
        coupled_field = f"{prefix}_coupled"
        coupled = _normalize_bool(_path_get(estimate, "geometry", coupled_field))
        if coupled is not None:
            return coupled, f"geometry.{coupled_field}"
        control_key = _joint_joint_control_key(left_joint, right_joint)
        if control_key is None:
            return None, None
        fallback = _normalize_bool(_path_get(estimate, "control", control_key))
        if fallback is None:
            return None, None
        return fallback, f"control.{control_key}_fallback"
    return None, None


def _joint_neighbor_ids(joint_index: int) -> tuple[str, str]:
    """Return the predecessor and successor identifiers for a generic joint."""
    previous_neighbor = "tip" if joint_index == 1 else f"joint{joint_index - 1}"
    next_neighbor = f"joint{joint_index + 1}"
    return previous_neighbor, next_neighbor


def _extract_tip_module_state(estimate: Any, module_id: str) -> ModuleState:
    """Extract the tip module using the legacy growth fields when present."""
    growth_field = "g"
    growth_velocity_field = "g_dot"
    tip_coupled, coupling_source = _coupling_state(estimate, module_id, "joint1")
    growth_mm = _safe_float(_path_get(estimate, "control", growth_field), None)
    growth_mm_s = _safe_float(_path_get(estimate, "control", growth_velocity_field), None)
    missing_fields = [
        field_name
        for field_name, value in (
            (growth_field, growth_mm),
            (growth_velocity_field, growth_mm_s),
        )
        if value is None
    ]
    notes: list[str] = []
    if missing_fields:
        notes.append(f"tip legacy fields unavailable: {','.join(missing_fields)}")
    if coupling_source is None:
        notes.append("tip coupling state unavailable for joint1")
    return make_tip_module_state(
        module_id=module_id,
        growth_mm=growth_mm,
        growth_mm_s=growth_mm_s,
        attach_state={"joint1": tip_coupled},
        metadata={
            "extractor_path": "tip_legacy",
            "legacy_fields_missing": ",".join(missing_fields) or None,
            "coupling_signal_source": coupling_source,
        },
        notes=notes,
    )


def _extract_joint_module_state(estimate: Any, module_id: str, joint_index: int) -> ModuleState:
    """Extract a generic `jointN` module snapshot from legacy-like fields when present."""
    field_names = {
        "crawl_mm": f"c{joint_index}",
        "rotate_deg": f"psi{joint_index}",
        "bend_deg": f"theta{joint_index}",
        "crawl_mm_s": f"c{joint_index}_dot",
        "rotate_deg_s": f"psi{joint_index}_dot",
        "bend_deg_s": f"theta{joint_index}_dot",
    }
    field_values = {
        key: _safe_float(_path_get(estimate, "control", field_name), None)
        for key, field_name in field_names.items()
    }
    missing_dof_fields = [
        field_name
        for key, field_name in field_names.items()
        if field_values[key] is None
    ]
    previous_neighbor, next_neighbor = _joint_neighbor_ids(joint_index)
    previous_coupled, previous_source = _coupling_state(estimate, module_id, previous_neighbor)
    next_coupled, next_source = _coupling_state(estimate, module_id, next_neighbor)
    attach_state = {
        previous_neighbor: previous_coupled,
        next_neighbor: next_coupled,
    }
    attach_sources = {
        f"attach_{previous_neighbor}_source": previous_source,
        f"attach_{next_neighbor}_source": next_source,
    }
    missing_attach_fields = [
        neighbor
        for neighbor, source in (
            (previous_neighbor, previous_source),
            (next_neighbor, next_source),
        )
        if source is None
    ]
    notes: list[str] = []
    if missing_dof_fields:
        notes.append(f"{module_id} legacy dof fields unavailable: {','.join(missing_dof_fields)}")
    if missing_attach_fields:
        notes.append(f"{module_id} attach states unavailable: {','.join(missing_attach_fields)}")
    return make_joint_module_state(
        module_id=module_id,
        crawl_mm=field_values["crawl_mm"],
        rotate_deg=field_values["rotate_deg"],
        bend_deg=field_values["bend_deg"],
        crawl_mm_s=field_values["crawl_mm_s"],
        rotate_deg_s=field_values["rotate_deg_s"],
        bend_deg_s=field_values["bend_deg_s"],
        attach_state=attach_state,
        metadata={
            "extractor_path": "generic_jointN",
            "joint_index": joint_index,
            "legacy_dof_fields_missing": ",".join(missing_dof_fields) or None,
            "legacy_attach_fields_missing": ",".join(missing_attach_fields) or None,
            **attach_sources,
        },
        notes=notes,
    )


def extract_module_state(estimate: Any, module_id: str) -> ModuleState:
    """Extract one module-level state snapshot from a legacy-like estimate."""
    if module_id == "tip":
        return _extract_tip_module_state(estimate, module_id)
    joint_index = parse_joint_index(module_id)
    if joint_index is not None:
        return _extract_joint_module_state(estimate, module_id, joint_index)
    return make_tip_module_state(
        module_id=module_id,
        metadata={
            "extractor_path": "fallback_unknown_module",
            "legacy_fields_missing": "module_type_unknown",
        },
        notes=[f"unknown module id {module_id}; returning empty tip-like module state"],
    )


def extract_tip_joint_geometry_observation(
    estimate: Any,
    active_module: str = "joint1",
    passive_module: str = "tip",
) -> GeometryObservation:
    """Extract a tip-joint geometry observation from a legacy-like estimate."""
    prefix, joint_module = _tip_joint_prefix(active_module, passive_module)
    distance_field = f"{prefix}_distance_mm"
    orientation_field = f"{prefix}_orientation_error_deg"
    coupled_field = f"{prefix}_coupled"
    distance_mm = _safe_float(_path_get(estimate, "geometry", distance_field), None)
    orientation_error_deg, orientation_source = _resolved_tip_joint_orientation(
        estimate,
        prefix=prefix,
        joint_module=joint_module,
    )
    coupled, coupled_source = _coupling_state(estimate, active_module, passive_module)
    missing_fields = [distance_field] if distance_mm is None else []
    notes = _note_missing_fields(*missing_fields)
    if orientation_source is None:
        notes.append("orientation_signal_unavailable")
    if coupled_source is None:
        notes.append("coupled_signal_unavailable")
    return GeometryObservation(
        observation_kind="tip_joint",
        pair=(active_module, passive_module),
        relation_type="tip_joint",
        distance_mm=distance_mm,
        orientation_error_deg=orientation_error_deg,
        coupled=coupled,
        observation_valid=distance_mm is not None,
        source_schema="legacy.geometry.v1",
        source_fields={
            "distance_mm": f"geometry.{distance_field}",
            "orientation_error_deg": _source_path(orientation_source),
            "coupled": _source_path(coupled_source),
        },
        metrics={
            "distance_mm": distance_mm,
            "orientation_error_deg": orientation_error_deg,
            "coupled": coupled,
            "missing_field_count": float(len(missing_fields)),
        },
        notes=notes,
        diagnostics={
            "pair_prefix": prefix,
            distance_field: distance_mm,
            orientation_field: _safe_float(_path_get(estimate, "geometry", orientation_field), None),
            "resolved_orientation_error_deg": orientation_error_deg,
            "orientation_signal_source": orientation_source,
            coupled_field: coupled,
            "coupled_signal_source": coupled_source,
            "missing_fields": ",".join(missing_fields) or None,
        },
    )


def extract_joint_joint_geometry_observation(
    estimate: Any,
    active_module: str = "joint2",
    passive_module: str = "joint1",
) -> GeometryObservation:
    """Extract a joint-joint geometry observation from a legacy-like estimate."""
    prefix, left_joint, right_joint = _joint_joint_prefix(active_module, passive_module)
    distance_field = f"{prefix}_distance_mm"
    orientation_field = f"{prefix}_orientation_error_deg"
    coupled_field = f"{prefix}_coupled"
    distance_mm = _safe_float(_path_get(estimate, "geometry", distance_field), None)
    orientation_error_deg, orientation_source = _resolved_joint_joint_orientation(
        estimate,
        prefix=prefix,
        active_module=active_module,
        passive_module=passive_module,
    )
    coupled, coupled_source = _coupling_state(estimate, left_joint, right_joint)
    missing_fields = [distance_field] if distance_mm is None else []
    notes = _note_missing_fields(*missing_fields)
    if orientation_source is None:
        notes.append("orientation_signal_unavailable")
    if coupled_source is None:
        notes.append("coupled_signal_unavailable")
    return GeometryObservation(
        observation_kind="joint_joint",
        pair=(active_module, passive_module),
        relation_type="joint_joint",
        distance_mm=distance_mm,
        orientation_error_deg=orientation_error_deg,
        coupled=coupled,
        observation_valid=distance_mm is not None,
        source_schema="legacy.geometry.v1",
        source_fields={
            "distance_mm": f"geometry.{distance_field}",
            "orientation_error_deg": _source_path(orientation_source),
            "coupled": _source_path(coupled_source),
        },
        metrics={
            "distance_mm": distance_mm,
            "orientation_error_deg": orientation_error_deg,
            "coupled": coupled,
            "missing_field_count": float(len(missing_fields)),
        },
        notes=notes,
        diagnostics={
            "pair_prefix": prefix,
            distance_field: distance_mm,
            orientation_field: _safe_float(_path_get(estimate, "geometry", orientation_field), None),
            "resolved_orientation_error_deg": orientation_error_deg,
            "orientation_signal_source": orientation_source,
            coupled_field: coupled,
            "coupled_signal_source": coupled_source,
            "missing_fields": ",".join(missing_fields) or None,
        },
    )


def relation_state_from_observation(observation: GeometryObservation) -> RelationState:
    """Convert one intermediate geometry observation into a relation state."""
    return observation.to_relation_state()


def geometry_observation_to_relation_state(observation: GeometryObservation) -> RelationState:
    """Backward-compatible alias for converting observations into relation state."""
    return relation_state_from_observation(observation)


def build_geometry_observation_from_legacy_pair(
    estimate: Any,
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
) -> GeometryObservation:
    """Build one geometry observation from a legacy-like estimate and pair request."""
    if relation_type == "tip_joint":
        return extract_tip_joint_geometry_observation(
            estimate,
            active_module=active_module,
            passive_module=passive_module,
        )
    if relation_type == "joint_joint":
        return extract_joint_joint_geometry_observation(
            estimate,
            active_module=active_module,
            passive_module=passive_module,
        )
    raise ValueError(f"unsupported_relation_type:{relation_type}")


def extract_tip_joint_relation_state(
    estimate: Any,
    active_module: str = "joint1",
    passive_module: str = "tip",
) -> RelationState:
    """Extract a tip-joint relation state from a legacy-like estimate."""
    return relation_state_from_observation(
        build_geometry_observation_from_legacy_pair(
            estimate,
            active_module=active_module,
            passive_module=passive_module,
            relation_type="tip_joint",
        )
    )


def extract_joint_joint_relation_state(
    estimate: Any,
    active_module: str = "joint2",
    passive_module: str = "joint1",
) -> RelationState:
    """Extract a joint-joint relation state from a legacy-like estimate."""
    return relation_state_from_observation(
        build_geometry_observation_from_legacy_pair(
            estimate,
            active_module=active_module,
            passive_module=passive_module,
            relation_type="joint_joint",
        )
    )


def extract_tip_joint1_relation_state(
    estimate: Any,
    active_module: str = "joint1",
    passive_module: str = "tip",
) -> RelationState:
    """Backwards-compatible wrapper for the tip-joint1 extractor."""
    return extract_tip_joint_relation_state(
        estimate,
        active_module=active_module,
        passive_module=passive_module,
    )


def extract_joint1_joint2_relation_state(
    estimate: Any,
    active_module: str = "joint2",
    passive_module: str = "joint1",
) -> RelationState:
    """Backwards-compatible wrapper for the joint1-joint2 extractor."""
    return extract_joint_joint_relation_state(
        estimate,
        active_module=active_module,
        passive_module=passive_module,
    )


def _matches_tip_joint_template(active_module: str, passive_module: str, relation_type: RelationType) -> bool:
    """Return whether the pair matches the generic tip-joint extractor template."""
    module_types = {_module_type(active_module), _module_type(passive_module)}
    return relation_type == "tip_joint" and module_types == {"tip", "joint"}


def _matches_joint_joint_template(active_module: str, passive_module: str, relation_type: RelationType) -> bool:
    """Return whether the pair matches the generic joint-joint extractor template."""
    return (
        relation_type == "joint_joint"
        and _module_type(active_module) == "joint"
        and _module_type(passive_module) == "joint"
    )


def build_default_pair_extractor_registry() -> PairExtractorRegistry:
    """Build the default registry used by the legacy-compatible adapter path."""
    registry = PairExtractorRegistry(source_name="default_pair_extractor_registry")
    registry.register_template(
        extractor_key="template:tip_joint",
        relation_type="tip_joint",
        matcher=_matches_tip_joint_template,
        extractor=extract_tip_joint_relation_state,
    )
    registry.register_template(
        extractor_key="template:joint_joint",
        relation_type="joint_joint",
        matcher=_matches_joint_joint_template,
        extractor=extract_joint_joint_relation_state,
    )
    return registry


def extract_relation_state_via_registry(
    estimate: Any,
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    registry: PairExtractorRegistry | None = None,
) -> RelationState:
    """Resolve and execute a relation extractor through the registry."""
    resolved_registry = registry or build_default_pair_extractor_registry()
    resolution = resolved_registry.resolve(
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
    )
    if resolution is None:
        raise LookupError(f"no_pair_extractor:{active_module}->{passive_module}:{relation_type}")
    extracted = resolution.extractor(
        estimate,
        active_module=active_module,
        passive_module=passive_module,
    )
    if isinstance(extracted, GeometryObservation):
        relation_state = relation_state_from_observation(extracted)
    else:
        relation_state = extracted
    descriptor = resolution.descriptor
    relation_state.diagnostics = {
        **relation_state.diagnostics,
        "pair_extractor_key": descriptor.extractor_key,
        "pair_resolution_kind": resolution.resolution_kind,
        "pair_resolution_specificity": float(resolution.specificity),
        "pair_resolution_registration_index": float(resolution.registration_index),
        "pair_resolution_descriptor": (
            f"{descriptor.extractor_key}:{descriptor.active_module}->{descriptor.passive_module}:{descriptor.relation_type}"
        ),
        "pair_registry_source": resolution.registry_source,
    }
    return relation_state
