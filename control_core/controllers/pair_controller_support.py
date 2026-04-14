"""Shared result types and helpers for generic pair-controller templates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillSpec
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import DiagnosticValue, RelationState


@dataclass(slots=True)
class PairControllerResult:
    """Structured output emitted by one pair-controller evaluation."""

    family: str
    stage: str
    status: str
    active_module: str
    passive_module: str
    relation_state: RelationState
    selected_controller: str
    selected_mpc: str | None = None
    primitive_references: list[PrimitiveReference] = field(default_factory=list)
    enabled_axes: tuple[str, ...] = ()
    completion_reason: str | None = None
    returnable_to_free_growth: bool = False
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


class PairControllerSupportMixin:
    """Utility methods shared across generic pair-controller templates."""

    @staticmethod
    def safe_float(value: Any, default: float | None = None) -> float | None:
        """Convert one scalar-like value into float with a nullable fallback."""
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def normalize_bool(value: Any) -> bool | None:
        """Normalize one bool-like value while preserving unknown as None."""
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

    @staticmethod
    def push_note(notes: list[str], message: str | None) -> None:
        """Append one note without duplicates."""
        if message and message not in notes:
            notes.append(message)

    def _resolve_topology(
        self,
        *,
        topology: ChainTopology | None,
        context: ExecutionContext | None,
    ) -> tuple[ChainTopology | None, bool]:
        """Resolve the active topology and whether it came from the context."""
        if context is not None and context.topology is not None:
            return context.topology, True
        return topology, False

    def _evaluate_topology_gate(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        topology: ChainTopology | None,
        context: ExecutionContext | None,
    ) -> dict[str, object]:
        """Evaluate pair legality against the available topology."""
        resolved_topology, used_context_topology = self._resolve_topology(
            topology=topology,
            context=context,
        )
        if resolved_topology is None:
            return {
                "topology": None,
                "used_context_topology": used_context_topology,
                "pair_allowed": True,
                "block_reason": None,
                "allow_off_frontier": False,
                "topology_snapshot": {},
                "frontier_snapshot": {},
                "support_snapshot": {},
            }
        allow_off_frontier = self._context_or_spec_bool(
            spec,
            context,
            "allow_off_frontier",
            default=False,
        )
        pair_allowed, block_reason = resolved_topology.pair_allowed(
            relation_state.active_module,
            relation_state.passive_module,
            allow_off_frontier=allow_off_frontier,
        )
        return {
            "topology": resolved_topology,
            "used_context_topology": used_context_topology,
            "pair_allowed": pair_allowed,
            "block_reason": block_reason,
            "allow_off_frontier": allow_off_frontier,
            "topology_snapshot": resolved_topology.snapshot(),
            "frontier_snapshot": resolved_topology.frontier_snapshot(),
            "support_snapshot": resolved_topology.support_snapshot(),
        }

    def _active_module_type(self, relation_state: RelationState, spec: SkillSpec) -> str:
        """Resolve the active module family from metadata or module naming."""
        metadata_type = str(spec.metadata.get("active_module_type", "")).strip().lower()
        if metadata_type in {"joint", "tip"}:
            return metadata_type
        if relation_state.active_module == "tip":
            return "tip"
        if relation_state.active_module.startswith("joint"):
            return "joint"
        return "unknown"

    def _spec_or_diagnostic_bool(
        self,
        spec: SkillSpec,
        relation_state: RelationState,
        name: str,
        default: bool = False,
    ) -> bool:
        """Resolve one boolean from params first, then diagnostics, then fallback."""
        if name in spec.params:
            value = self.normalize_bool(spec.params.get(name))
            if value is not None:
                return value
        value = self.normalize_bool(relation_state.diagnostics.get(name))
        if value is not None:
            return value
        return default

    def _spec_or_diagnostic_float(
        self,
        spec: SkillSpec,
        relation_state: RelationState,
        name: str,
        default: float | None = None,
    ) -> float | None:
        """Resolve one float from params first, then diagnostics, then fallback."""
        if name in spec.params:
            value = self.safe_float(spec.params.get(name), None)
            if value is not None:
                return value
        value = self.safe_float(relation_state.diagnostics.get(name), None)
        if value is not None:
            return value
        return default

    def _context_or_spec_bool(
        self,
        spec: SkillSpec,
        context: ExecutionContext | None,
        name: str,
        *,
        default: bool = False,
    ) -> bool:
        """Resolve one boolean from context metadata first, then spec params."""
        if context is not None and name in context.metadata:
            value = self.normalize_bool(context.metadata.get(name))
            if value is not None:
                return value
        value = self.normalize_bool(spec.params.get(name))
        if value is not None:
            return value
        return default

    @staticmethod
    def _merge_diagnostics(*parts: Mapping[str, object]) -> dict[str, DiagnosticValue]:
        """Merge multiple diagnostic mappings into a serializable flat dict."""
        merged: dict[str, DiagnosticValue] = {}
        for part in parts:
            for key, value in part.items():
                merged[str(key)] = PairControllerSupportMixin._normalize_diagnostic_value(value)
        return merged

    @staticmethod
    def _normalize_diagnostic_value(value: object) -> DiagnosticValue:
        """Normalize arbitrary values into the supported diagnostic scalar set."""
        if value is None or isinstance(value, (bool, float, str)):
            return value
        if isinstance(value, int):
            return float(value)
        return str(value)

    def _joint_hold_references(
        self,
        module_id: str,
        *,
        include_rotate: bool = True,
        include_bend: bool = True,
        semantic: str = "nominal_hold",
    ) -> list[PrimitiveReference]:
        """Return nominal joint hold references for axes not actively driven."""
        references: list[PrimitiveReference] = []
        if include_rotate:
            references.append(
                PrimitiveReference(
                    module_id=module_id,
                    primitive_name="joint_rotate_hold",
                    axis="rotate",
                    reference_kind="velocity",
                    reference_value=0.0,
                    units="deg/s",
                    primary=False,
                    semantic=semantic,
                    target_value=0.0,
                    metadata={"role": "secondary"},
                )
            )
        if include_bend:
            references.append(
                PrimitiveReference(
                    module_id=module_id,
                    primitive_name="joint_bend_hold",
                    axis="bend",
                    reference_kind="velocity",
                    reference_value=0.0,
                    units="deg/s",
                    primary=False,
                    semantic=semantic,
                    target_value=0.0,
                    metadata={"role": "secondary"},
                )
            )
        return references

    def _joint_zero_hold_references(self, module_id: str) -> list[PrimitiveReference]:
        """Return explicit zero-hold references for all joint axes."""
        return [
            PrimitiveReference(
                module_id=module_id,
                primitive_name="joint_crawl",
                axis="crawl",
                reference_kind="velocity",
                reference_value=0.0,
                units="mm/s",
                primary=True,
                semantic="zero_hold",
                target_value=0.0,
                metadata={"role": "primary"},
            ),
            PrimitiveReference(
                module_id=module_id,
                primitive_name="joint_rotate",
                axis="rotate",
                reference_kind="velocity",
                reference_value=0.0,
                units="deg/s",
                primary=True,
                semantic="zero_hold",
                target_value=0.0,
                metadata={"role": "primary"},
            ),
            PrimitiveReference(
                module_id=module_id,
                primitive_name="joint_bend",
                axis="bend",
                reference_kind="velocity",
                reference_value=0.0,
                units="deg/s",
                primary=True,
                semantic="zero_hold",
                target_value=0.0,
                metadata={"role": "primary"},
            ),
        ]

    def _tip_zero_hold_references(self, module_id: str) -> list[PrimitiveReference]:
        """Return explicit zero-hold references for tip growth."""
        return [
            PrimitiveReference(
                module_id=module_id,
                primitive_name="tip_growth",
                axis="growth",
                reference_kind="velocity",
                reference_value=0.0,
                units="mm/s",
                primary=True,
                semantic="zero_hold",
                target_value=0.0,
                metadata={"role": "primary"},
            )
        ]

    def _blocked_result(
        self,
        *,
        family: str,
        stage: str,
        relation_state: RelationState,
        controller_name: str,
        selected_mpc: str | None,
        enabled_axes: tuple[str, ...],
        diagnostics: Mapping[str, object],
        notes: list[str],
        block_reason: str,
    ) -> PairControllerResult:
        """Build a blocked result with a stable diagnostic surface."""
        return PairControllerResult(
            family=family,
            stage=stage,
            status="blocked",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller=controller_name,
            selected_mpc=selected_mpc,
            primitive_references=[],
            enabled_axes=enabled_axes,
            completion_reason=block_reason,
            diagnostics=self._merge_diagnostics(diagnostics, {"block_reason": block_reason}),
            notes=list(notes),
        )
