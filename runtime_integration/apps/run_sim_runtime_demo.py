"""Internal sim runtime demo/diagnostic entry."""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import argparse
import json

from ...runtime_integration.runtime_session import build_sim_runtime_session


def run_sim_runtime_demo(*, step_count: int = 4, dt: float = 1.0) -> list[dict[str, object]]:
    """Run the internal sim runtime demo and return compact per-step summaries."""
    session = build_sim_runtime_session(dt=dt)
    summaries: list[dict[str, object]] = []
    for index in range(step_count):
        result = session.step()
        summaries.append(
            {
                "step_index": index,
                "graph_id": result.graph_id,
                "current_node_id": result.current_node_id,
                "active_skill_key": result.active_skill_key,
                "accepted": result.accepted,
                "reason": result.reason,
                "provider_kind": result.provider_kind,
                "dispatcher_kind": result.dispatcher_kind,
                "dispatch_target": result.dispatch_target,
                "legacy_path_used": result.diagnostics.get("legacy_path_used"),
            }
        )
    session.stop()
    return summaries


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for the internal sim runtime demo."""
    parser = argparse.ArgumentParser(description="Run the internal self-contained sim runtime demo.")
    parser.add_argument("--steps", type=int, default=4)
    parser.add_argument("--dt", type=float, default=1.0)
    args = parser.parse_args(argv)
    for summary in run_sim_runtime_demo(step_count=args.steps, dt=args.dt):
        print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
