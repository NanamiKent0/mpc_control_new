from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class OperatorViewState:
    mode: str = "-"

    tip_heading_deg: float = 0.0
    tip_growth_mm: float = 0.0
    target_heading_delta_deg: float = 0.0

    current_tip_heading_deg: float = 0.0
    current_tip_growth_mm: float = 0.0
    current_target_heading_deg: float = 0.0

    turn_completed: bool = False

    planner_mode: str = "-"
    selected_joint_id: str = "-"
    active_pair: str = "-"
    current_skill_node: str = "-"
    last_log: str = "-"

    diagnostics: dict[str, object] = field(default_factory=dict)


class OperatorViewModel:
    """Map RuntimeSession state into GUI-friendly status/diagnostics objects."""

    def __init__(self, runtime_session: Any) -> None:
        self.runtime_session = runtime_session
        self.last_submit_info: str = "-"
        self.last_submit_error: str | None = None

    # ------------------------------------------------------------------
    # Intent submission
    # ------------------------------------------------------------------
    def submit_free_growth(self, speed_mm_s: float) -> None:
        payload = {
            "intent_kind": "TIP_FREE_GROWTH",
            "speed_mm_s": float(speed_mm_s),
        }
        self._submit(payload, label=f"TIP_FREE_GROWTH(speed_mm_s={float(speed_mm_s):.3f})")

    def submit_turn(self, target_heading_delta_deg: float) -> None:
        payload = {
            "intent_kind": "TIP_TURN",
            "target_heading_delta_deg": float(target_heading_delta_deg),
        }
        self._submit(
            payload,
            label=f"TIP_TURN(target_heading_delta_deg={float(target_heading_delta_deg):.3f})",
        )

    def submit_stop(self) -> None:
        payload = {
            "intent_kind": "TIP_FREE_GROWTH",
            "speed_mm_s": 0.0,
        }
        self._submit(payload, label="STOP -> TIP_FREE_GROWTH(speed_mm_s=0.0)")

    def submit_estop(self) -> None:
        payload = {
            "intent_kind": "TIP_FREE_GROWTH",
            "speed_mm_s": 0.0,
        }
        self._submit(payload, label="ESTOP -> TIP_FREE_GROWTH(speed_mm_s=0.0)")

    def _submit(self, payload: dict[str, object], *, label: str) -> None:
        self.last_submit_error = None
        try:
            if hasattr(self.runtime_session, "submit_intent"):
                self.runtime_session.submit_intent(payload)
                self.last_submit_info = label
            else:
                self.last_submit_error = "runtime_session has no submit_intent()"
        except Exception as exc:
            self.last_submit_error = f"{type(exc).__name__}: {exc}"

    def refresh(
        self,
        *,
        last_step_result: Any | None = None,
        runtime_error: Exception | None = None,
    ) -> OperatorViewState:
        snapshot = self._safe_snapshot()

        scheduler_diag = self._to_dict(snapshot.get("scheduler_diagnostics"))
        provider_status = self._to_dict(snapshot.get("provider_status"))
        dispatcher_status = self._to_dict(snapshot.get("dispatcher_status"))
        graph_metadata = self._to_dict(snapshot.get("graph_metadata"))
        backend_state = self._extract_backend_state(provider_status)
        frame_metadata = self._extract_frame_metadata(provider_status, snapshot)

        step_diag = self._to_dict(getattr(last_step_result, "diagnostics", None))

        merged_diag: dict[str, object] = {}
        merged_diag.update(scheduler_diag)
        merged_diag.update(step_diag)

        current_task = self._safe_text(
            self._first_not_none(
                snapshot.get("high_level_task_kind"),
                graph_metadata.get("high_level_task_kind"),
                snapshot.get("planner_mode"),
                merged_diag.get("compiled_task_kind"),
                "-",
            )
        )

        planner_mode = self._safe_text(
            self._first_not_none(
                snapshot.get("planner_mode"),
                merged_diag.get("planner_mode"),
                current_task,
                "-",
            )
        )

        selected_joint = self._safe_text(
            self._first_not_none(
                snapshot.get("selected_joint_id"),
                merged_diag.get("selected_joint_id"),
                graph_metadata.get("selected_joint_id"),
                "-",
            )
        )

        active_pair = self._safe_text(
            self._first_not_none(
                snapshot.get("current_active_pair"),
                merged_diag.get("current_active_pair"),
                graph_metadata.get("current_active_pair"),
                "-",
            )
        )

        current_skill_node = self._safe_text(
            self._first_not_none(
                snapshot.get("current_node_id"),
                merged_diag.get("current_skill_node"),
                merged_diag.get("node_id"),
                "-",
            )
        )

        tip_heading_deg = self._safe_float(
            self._first_not_none(
                backend_state.get("current_tip_heading_deg"),
                backend_state.get("tip_heading_deg"),
                frame_metadata.get("current_tip_heading_deg"),
                frame_metadata.get("tip_heading_deg"),
                merged_diag.get("tip_heading_current_deg"),
                merged_diag.get("current_tip_heading_deg"),
                merged_diag.get("tip_heading_deg"),
                0.0,
            )
        )
        current_tip_heading_deg = tip_heading_deg

        tip_growth_mm = self._safe_float(
            self._first_not_none(
                backend_state.get("current_tip_growth_mm"),
                backend_state.get("tip_extension_mm"),
                backend_state.get("tip_growth_mm"),
                backend_state.get("g"),
                frame_metadata.get("current_tip_growth_mm"),
                frame_metadata.get("tip_extension_mm"),
                frame_metadata.get("tip_growth_mm"),
                merged_diag.get("tip_growth_current_mm"),
                merged_diag.get("current_tip_growth_mm"),
                merged_diag.get("tip_growth_mm"),
                0.0,
            )
        )
        current_tip_growth_mm = tip_growth_mm

        target_heading_delta_deg = self._safe_float(
            self._first_not_none(
                snapshot.get("requested_heading_delta_deg"),
                snapshot.get("target_heading_delta_deg"),
                frame_metadata.get("requested_heading_delta_deg"),
                frame_metadata.get("target_heading_delta_deg"),
                merged_diag.get("requested_heading_delta_deg"),
                merged_diag.get("target_heading_delta_deg"),
                graph_metadata.get("target_heading_delta_deg"),
                0.0,
            )
        )

        turn_completed = self._safe_bool(
            self._first_not_none(
                snapshot.get("turn_completed"),
                merged_diag.get("turn_completed"),
                False,
            )
        )

        current_target_heading_deg = self._safe_float(
            self._first_not_none(
                snapshot.get("current_target_heading_deg"),
                frame_metadata.get("current_target_heading_deg"),
                merged_diag.get("current_target_heading_deg"),
                tip_heading_deg + target_heading_delta_deg,
            )
        )

        last_log = self._build_last_log(
            last_step_result=last_step_result,
            runtime_error=runtime_error,
            snapshot=snapshot,
        )

        diagnostics = {
            "session_started": snapshot.get("started"),
            "graph_id": snapshot.get("graph_id"),
            "current_node_id": snapshot.get("current_node_id"),
            "last_skill_key": snapshot.get("last_skill_key"),
            "high_level_task_kind": snapshot.get("high_level_task_kind"),
            "provider_kind": snapshot.get("provider_kind"),
            "dispatcher_kind": snapshot.get("dispatcher_kind"),
            "dispatch_ready": snapshot.get("dispatch_ready"),
            "input_source": snapshot.get("input_source"),
            "dispatch_target": snapshot.get("dispatch_target"),
            "last_step_reason": snapshot.get("last_step_reason"),
            "last_step_accepted": snapshot.get("last_step_accepted"),
            "selected_joint_id": selected_joint,
            "current_active_pair": active_pair,
            "requested_heading_delta_deg": target_heading_delta_deg,
            "turn_completed": turn_completed,
            "tip_heading_deg": tip_heading_deg,
            "tip_growth_mm": tip_growth_mm,
            "current_tip_heading_deg": current_tip_heading_deg,
            "current_tip_growth_mm": current_tip_growth_mm,
            "current_target_heading_deg": current_target_heading_deg,
            "topology_source": merged_diag.get("topology_source"),
            "state_builder_source": merged_diag.get("state_builder_source"),
            "provider_source": provider_status.get("provider_source"),
            "provider_reason": provider_status.get("reason"),
            "dispatcher_reason": dispatcher_status.get("reason"),
            "last_submit_info": self.last_submit_info,
            "last_submit_error": self.last_submit_error,
            "runtime_error": None if runtime_error is None else f"{type(runtime_error).__name__}: {runtime_error}",
        }

        return OperatorViewState(
            mode=current_task,
            tip_heading_deg=tip_heading_deg,
            tip_growth_mm=tip_growth_mm,
            target_heading_delta_deg=target_heading_delta_deg,
            current_tip_heading_deg=current_tip_heading_deg,
            current_tip_growth_mm=current_tip_growth_mm,
            current_target_heading_deg=current_target_heading_deg,
            turn_completed=turn_completed,
            planner_mode=planner_mode,
            selected_joint_id=selected_joint,
            active_pair=active_pair,
            current_skill_node=current_skill_node,
            last_log=last_log,
            diagnostics=diagnostics,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _safe_snapshot(self) -> dict[str, object]:
        if not hasattr(self.runtime_session, "snapshot_status"):
            return {}
        try:
            result = self.runtime_session.snapshot_status()
            return self._to_dict(result)
        except Exception as exc:
            return {"snapshot_error": f"{type(exc).__name__}: {exc}"}

    @staticmethod
    def _to_dict(value: Any) -> dict[str, object]:
        if isinstance(value, dict):
            return value
        return {}

    @staticmethod
    def _first_not_none(*values: Any) -> Any:
        for value in values:
            if value is not None:
                return value
        return None

    @staticmethod
    def _extract_backend_state(provider_status: dict[str, object]) -> dict[str, object]:
        for key in ("backend_state", "latest_backend_state"):
            value = provider_status.get(key)
            if isinstance(value, dict):
                return value
        return {}
    
    @staticmethod
    def _extract_frame_metadata(
        provider_status: dict[str, object],
        snapshot: dict[str, object],
    ) -> dict[str, object]:
        latest_frame = provider_status.get("latest_frame")
        if isinstance(latest_frame, dict):
            metadata = latest_frame.get("metadata")
            if isinstance(metadata, dict):
                return metadata

        latest_frame_metadata = snapshot.get("latest_frame_metadata")
        if isinstance(latest_frame_metadata, dict):
            return latest_frame_metadata

        return {}

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "y", "done"}
        return bool(value)

    @staticmethod
    def _safe_text(value: Any, default: str = "-") -> str:
        if value is None:
            return default
        text = str(value).strip()
        return text if text else default

    def _build_last_log(
        self,
        *,
        last_step_result: Any | None,
        runtime_error: Exception | None,
        snapshot: dict[str, object],
    ) -> str:
        if runtime_error is not None:
            return f"runtime_error: {type(runtime_error).__name__}: {runtime_error}"
        if self.last_submit_error:
            return f"submit_error: {self.last_submit_error}"
        if last_step_result is not None:
            reason = getattr(last_step_result, "reason", None)
            accepted = getattr(last_step_result, "accepted", None)
            return f"step_reason={reason}, accepted={accepted}, last_submit={self.last_submit_info}"
        snapshot_error = snapshot.get("snapshot_error")
        if snapshot_error is not None:
            return f"snapshot_error: {snapshot_error}"
        return f"last_submit={self.last_submit_info}"