"""Continuous tip free-growth skill."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillCheckResult, SkillCompletionResult, SkillSpec
from ..topology.relation_state import RelationState
from .base import RelationSkill


class TipFreeGrowthSkill(RelationSkill):
    """Emit continuous tip growth command for TIP_FREE_GROWTH mode."""

    skill_key = "tip_free_growth"

    def build_relation_state(
        self,
        source: object,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> RelationState:
        del context
        return self._coerce_relation_state(source, spec)

    def check_preconditions(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCheckResult:
        del relation_state
        notes: list[str] = []
        diagnostics = {
            "active_module": spec.active_module,
            "passive_module": spec.passive_module,
            "relation_type": spec.relation_type,
        }
        if spec.active_module != "tip":
            return SkillCheckResult(
                passed=False,
                blocking_reason="tip_free_growth_requires_active_tip",
                diagnostics=diagnostics,
                notes=notes,
            )
        return SkillCheckResult(
            passed=True,
            diagnostics=diagnostics,
            notes=notes,
        )

    def generate_primitive_references(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> list[PrimitiveReference]:
        del relation_state
        speed_mm_s = 0.0
        if context is not None:
            speed_mm_s = float(
                context.metadata_value(
                    "tip_growth_speed_mm_s",
                    context.metadata_value("speed_mm_s", 0.0),
                ) or 0.0
            )
        if abs(speed_mm_s) < 1e-9:
            speed_mm_s = float(
                spec.float_param(
                    "tip_growth_speed_mm_s",
                    spec.float_param("speed_mm_s", 0.0),
                ) or 0.0
            )

        return [
            PrimitiveReference(
                module_id="tip",
                primitive_name="tip_growth",
                axis="growth",
                reference_kind="velocity",
                reference_value=speed_mm_s,
                units="mm/s",
                primary=True,
                semantic="tip_free_growth",
                target_value=None,
                metadata={
                    "tip_growth_speed_mm_s": speed_mm_s,
                    "skill_key": self.skill_key,
                },
            )
        ]

    def check_completion(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCompletionResult:
        del relation_state, spec, context
        return SkillCompletionResult(
            done=False,
            completion_reason="tip_free_growth_continuous",
        )