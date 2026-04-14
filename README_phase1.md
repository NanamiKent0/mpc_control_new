# MPC Refactor Progress Note

The goal of `mpc_control_new` is to separate reusable relation-level control logic from legacy runtime wiring so the MPC stack can evolve without rewriting every mode controller at once.

The refactor still follows the same three-layer idea:

1. Core state models define module snapshots, relation snapshots, and chain topology as plain dataclasses.
2. Reusable relation skills implement the control semantics for a relation pair, independent of ROS and the legacy controller classes.
3. Thin adapters translate current legacy-like estimates into the new state models and invoke the skills, while small deterministic solver wrappers stand in for future MPC backends.

## Phase 1

Phase 1 established the scaffold:

- `ModuleState`, `RelationState`, `ChainTopology`, and lightweight skill result dataclasses.
- `CoarseApproachSkill` for M1-like and M5-like distance-closing behavior.
- `FineDockSkill` for M2-like fine docking with distance and orientation closure.
- Placeholder scalar and pair docking solver wrappers.
- Legacy-semantic extraction helpers and a smoke-test adapter.
- Plain-Python fake tests with no ROS dependency.

## Phase 2

Phase 2 makes the scaffold meaningfully more generic without touching legacy runtime integration:

- Pair extraction is now registry-based. The adapter no longer hardcodes supported pairs inline and instead resolves extractors through a `PairExtractorRegistry`.
- The default extractor registry supports template-style relation resolution for `tip_joint` and `joint_joint`, while still allowing exact-pair overrides later.
- `CoarseApproachSkill` is now active-module-type aware. Joint-led execution emits `joint_crawl`, while tip-led execution emits `tip_growth`.
- `ChainTopology` now participates in execution rather than acting as passive storage. Skills call topology legality checks during preconditions and completion, and topology-blocked requests return structured blocked results with explicit reasons.
- `FineDockSkill` now requires a topology-legal pair before emitting docking references and keeps zero-hold completion semantics only for legal pairs.
- New fake tests cover registry resolution, topology-aware blocking, and type-aware coarse approach primitive selection.

## Phase 3

Phase 3 turns the Phase-2 transitional structure into a more explicit and extensible intermediate architecture:

- Fine docking is now pair-generic at the skill layer. `FineDockSkill` no longer hard-requires a joint active module, derives supported docking axes from module type, reads docking targets from generic `SkillSpec.params`, and reports unsupported axis requests explicitly instead of faking control.
- Module extraction now has a generic `jointN` path. `extract_module_state()` parses arbitrary `joint3`, `joint4`, and future `jointN` identifiers, maps them to legacy-like field names when available, and returns normalized `ModuleState` objects with metadata/notes when fields are missing.
- `ChainTopology` now carries richer execution semantics: active frontier, support-protected modules, blocked edges, and coupled edges. Skills call topology legality checks through explicit frontier/support policy gates and report topology blocks with structured diagnostics.
- Skill execution now accepts an explicit `ExecutionContext` carrying topology, dt, target namespace, and metadata. The adapter builds this context and passes it into skills so topology is no longer only an implicit object member.
- Pair extractor registry resolution now exposes specificity and resolution-kind diagnostics, while preserving the Phase-2 exact-over-template override behavior.

## Deferred To Phase 4

The following work is still intentionally deferred beyond Phase 3:

- Full task-graph scheduling and supervisor orchestration.
- ROS/runtime dispatch integration.
- Legacy runtime hook rewrites.
- M3/M4/M7/M8 migration and broader M0~M8 parity.
- Arbitrary-pair kinematics beyond the current legacy-derived extraction structure.
- Optimized MPC formulations and full runtime/ROS dispatch parity.

## Phase 4

Phase 4 adds the first orchestration layer inside `mpc_control_new` without touching legacy runtime hooks:

- `TaskNode`, `TaskGraphSpec`, `SchedulerState`, and `SchedulerStepResult` now provide an explicit task-graph state model for skill chaining.
- `TaskGraph` validates graph connectivity and resolves linear success transitions while leaving room for future failure branches.
- `DefaultTransitionPolicy` defines the first transparent transition rule set: no jump on missing/blocked preconditions, jump on completion, finish when the graph ends.
- `SkillScheduler` now owns the active graph/node, injects scheduler metadata into `ExecutionContext`, invokes `SkillControllerAdapter.execute_skill(...)`, and updates scheduler state from skill completion.
- `build_pair_approach_then_dock_graph(...)` demonstrates one reusable pair-template graph that can drive both `joint1 -> tip` and `joint2 -> joint1` coarse-then-dock chains.
- New fake/smoke tests verify graph validation, scheduler transitions, context/topology propagation, and two reusable chain examples analogous to M1->M2 and M5->M6.

## Deferred To Phase 5

The following work remains intentionally deferred after Phase 4:

- Branch-heavy graph policies, recovery logic, and richer failure routing.
- Supervisor-level orchestration across multiple graphs or concurrent branches.
- Live ROS/runtime dispatch integration and legacy runtime handoff.
- M3/M4/M7/M8 migration and broader production-path parity.
- Solver upgrades beyond the current placeholder MPC wrappers.

## Phase 5

Phase 5 turns the linear Phase-4 scheduler into a more capable orchestration middle layer while still staying away from legacy runtime takeover:

- `SkillRegistry` is now a standalone dependency. The adapter no longer hardcodes skill routing and instead resolves `coarse_approach`, `fine_dock`, and a minimal `terminal_noop` through an explicit registry surface.
- `TaskNode` and `SchedulerState` now carry retry limits, transition policy hints, attempt counters, and last-failure metadata so retry/fallback behaviour stays transparent and testable.
- `DefaultTransitionPolicy` now supports explicit success branches, failure branches, bounded retries, fallback routing, and metadata-driven forced failure via `force_failure_after_n_attempts`.
- `SkillScheduler` injects graph/node metadata into `ExecutionContext`, resolves skills through the registry before adapter execution, tracks per-node attempt counts, and exposes retry/fallback decisions in its step result diagnostics.
- `TaskGraphBuilder` and richer graph factories now support reusable non-linear templates, including failure sinks and retryable coarse-then-dock chains.
- Legacy relation extraction now goes through a small `GeometryObservation` abstraction before becoming `RelationState`, which creates a cleaner boundary for future geometry schema replacement without rewriting skills.

## Deferred To Phase 6

The following work remains intentionally deferred after Phase 5:

- Live ROS/runtime dispatch integration and legacy runtime hook changes.
- Production-grade supervisor semantics across multiple graphs, concurrent branches, or complex recovery trees.
- Migration of M3/M4/M7/M8 and broader full-runtime mode parity.
- Replacement of the legacy-derived geometry schema with a true runtime-facing observation layer.
- Solver and dispatch upgrades beyond the current fake/smoke orchestration scaffold.

## Phase 6

Phase 6 tightens the orchestration abstraction before any real runtime takeover:

- Transition semantics are now explicitly normalized around attempts. `TaskNode.retry_limit` keeps its legacy field name for compatibility, but the effective meaning is now "maximum attempts including the first execution". `max_attempts` and `retry_budget` helpers expose that meaning directly, and scheduler diagnostics/state now record the latest retry, fallback, and failure outcome transparently.
- Transition policies are no longer wired as one implicit scheduler dependency. A standalone `TransitionPolicyRegistry` resolves policy instances by key, the scheduler reports structured errors when a requested policy key is missing, and diagnostics expose both the resolved policy key and where it came from (`node`, `graph`, or scheduler default).
- `DefaultTransitionPolicy` now cleanly applies the normalized attempt semantics, while `FailFastTransitionPolicy` provides a minimal no-retry sample policy for test coverage and future policy switching.
- Task-graph construction now supports reusable `GraphFragment` composition. Common coarse-approach / fine-dock and terminal sink fragments can be added to a `TaskGraphBuilder`, prefixed, and reused without hand-writing every node dictionary.
- Graph factories now use fragment composition rather than only inline node assembly, and node metadata is promoted into real execution hints such as `allow_off_frontier`, `requires_support_stability`, and forced-failure thresholds.
- `GeometryObservation` is now a clearer observation-boundary object rather than only an intermediate dataclass. It carries `observation_kind`, `source_schema`, `source_fields`, `metrics`, `notes`, and flattened diagnostics so the legacy path is explicitly `legacy estimate -> GeometryObservation -> RelationState`.
- The scheduler can now emit a runtime-bridge snapshot without depending on ROS. `ScheduledSkillCommand` and `SchedulerDispatchEnvelope` define the stable protocol for future runtime dispatch, while keeping runtime logic outside the scheduler itself.
- `TerminalNoopSkill` has been moved out of the skill registry module and into `control_core/skills/terminal.py`, so the registry is once again only responsible for registration and resolution.

## Deferred To Phase 7

The following work remains intentionally deferred after Phase 6:

- Real ROS/runtime dispatch integration using the new bridge protocol.
- Legacy runtime hook rewrites and live orchestration handoff.
- Supervisor semantics across multiple graphs, concurrent branches, or production recovery trees.
- Migration of M3/M4/M7/M8 and broader end-to-end controller parity.
- Richer observation schemas beyond the current legacy-derived geometry bridge.
- More advanced graph execution semantics such as parallel execution, branch joins, and production-grade supervision.
- Solver upgrades beyond the current fake/smoke orchestration scaffold.

## Phase 7

Phase 7 establishes the first runtime-facing main path inside `mpc_control_new` without modifying legacy runtime hooks:

- `RuntimeObservationFrame`, `ModuleObservation`, and `PairObservation` now define the new architecture's upstream runtime input boundary. The scheduler main path no longer has to start from a legacy estimate.
- `runtime_state_builder.py` is now the default state-construction entrypoint for the runtime-facing path. It converts runtime frames directly into `ModuleState`, `GeometryObservation`, `RelationState`, and a topology snapshot for session execution.
- `SkillControllerAdapter` now supports `execute_skill_from_frame(...)`, so skills can run directly from runtime observations while keeping the old legacy-estimate entrypoint intact for compatibility tests.
- `SkillScheduler` now diagnoses its input source explicitly, accepts runtime frames as a first-class input, and emits bridge envelopes that are ready to feed directly into a runtime session loop.
- `RuntimeSession` and `RuntimeStepResult` now provide the first explicit observation -> scheduler -> dispatch main loop owned entirely by the new architecture.
- `ObservationProvider` and `CommandDispatcher` define the stable runtime integration contracts. A sim-backed provider/dispatcher pair can now execute the minimal coarse-approach -> fine-dock chain, while live-runtime skeleton adapters keep the future hardware boundary explicit without requiring online hardware.
- `build_runtime_demo_pair_graph(...)` gives the runtime session and sim smoke tests a reusable graph factory instead of hand-built node dictionaries.

## Phase 8

Phase 8 turns the runtime-facing scaffold into a self-contained portable base architecture:

- The sim path is now fully internal to `mpc_control_new`. A minimal backend was copied into `runtime_integration/sim_backend/`, and the sim observation provider / command dispatcher no longer import the legacy `sim` package.
- `SimRuntimeBackend`, `SimState`, and `SimCommand` now live under the new package boundary and provide the minimum kinematic updates needed for tip growth, joint crawl, joint rotate, terminal noop, and pair distance/orientation closure.
- The live runtime entry is now explicitly ROS2-shaped. `ros2_interfaces.py`, `ros2_message_adapters.py`, and `ros2_runtime_node.py` define the internal ROS2 contract, message adaptation layer, and node/resource wrapper used by the live provider/dispatcher.
- `LiveRuntimeObservationProvider` and `LiveRuntimeCommandDispatcher` now expose a real start/stop/get_latest_frame/dispatch contract. They degrade cleanly to structured unavailable results when ROS2 resources are absent or intentionally disabled, and they can run through a fake ROS2 node shim in tests without hardware.
- `RuntimeSession` now exposes explicit `build_sim_runtime_session(...)` and `build_ros2_runtime_session(...)` constructors, records `runtime_frame:sim` / `runtime_frame:ros2` input provenance, and surfaces provider/dispatcher diagnostics in `RuntimeStepResult`.
- `SkillScheduler`, `SchedulerDispatchEnvelope`, `GeometryObservation`, and `runtime_state_builder.py` now carry clearer provenance diagnostics such as `state_builder_source`, `legacy_path_used`, `provider_source`, and `frame_origin`, so the runtime-frame main path is explicit and debuggable.
- `self_containment_checks.py` adds a static relocation guard that scans `mpc_control_new` for forbidden legacy imports, and new tests verify both the scan and a temporary-directory package relocation assumption.
- `build_runtime_demo_pair_graph(...)` now carries runtime metadata suitable for the sim and ROS2 smoke paths, including the default `joint1 -> tip` and `joint2 -> joint1` demo pair catalog.

## Deferred To Phase 9

The following work remains intentionally deferred after Phase 8:

- Real ROS2 custom message types, real topic wiring, and live hardware validation beyond the current fake/shim-backed contract.
- Supervisor-grade lifecycle management, recovery trees, and multi-graph orchestration.
- Migration of M3/M4/M7/M8 and broader production-path controller parity.
- Richer topology reconstruction and multi-pair observation synthesis beyond the current minimal runtime-frame scaffold.
- Production-quality sim/live command semantics, safety gates, actuator acknowledgements, and recovery handling.
- More advanced graph execution semantics such as branch joins, parallel execution, and production supervision.
- Solver upgrades beyond the current fake/smoke runtime scaffold.

## Phase 9

Phase 9 closes the standalone runtime path so `mpc_control_new` can keep evolving as the new trunk:

- ROS2 runtime integration is now a concrete minimal path rather than only a contract shell. `ros2_topic_config.py`, `ros2_interfaces.py`, `ros2_message_adapters.py`, `ros2_runtime_node.py`, `live_runtime_provider.py`, and `live_command_dispatcher.py` now define topic semantics, JSON/string-compatible payload adaptation, a shared runtime node wrapper, and provider/dispatcher objects that either run through `rclpy` or degrade cleanly with structured diagnostics.
- `RuntimeSession` is now the formal main entry for both sim and ROS2. It exposes reset/snapshot helpers, stable step diagnostics, explicit `build_sim_runtime_session(...)` / `build_ros2_runtime_session(...)` constructors, and a runtime-step result rich enough to drive CLI demos, GUI state projection, and smoke tests without falling back to legacy runtime hooks.
- The self-contained sim path now exposes enough state for later GUI/ROS2 topic bridging. The backend tracks initial state, reset, last envelope/skill metadata, and visualization snapshots. The sim observation provider and command dispatcher surface backend state, command state, and dispatch counters in runtime diagnostics.
- `runtime_integration/apps/` now provides standalone entry points for sim and ROS2 runtime demos. These launch paths only depend on `mpc_control_new`, so the package can be exercised from its own independent root.
- Self-containment checks now scan the new GUI, sim, ROS2, and app paths and reject both old import statements and hard-coded legacy absolute paths.

## Deferred To Phase 10

The following work remains intentionally deferred after Phase 9:

- Real ROS2 custom message definitions and validation against an actual ROS2 graph or hardware topics beyond the current JSON/string-compatible minimal path.
- Canonical GUI convergence onto the user-owned `gui_ros2.py` topic contract.
- Supervisor-grade lifecycle management, recovery trees, and multi-graph orchestration.
- Migration of M3/M4/M7/M8 and broader production-path controller parity.
- More advanced observation synthesis, richer topology reconstruction, and production-quality actuator acknowledgement / safety handling.
- More advanced graph execution semantics such as branch joins, parallel execution, and production supervision.
- Solver upgrades beyond the current fake/smoke runtime scaffold.

## Phase 10

Phase 10 closes the GUI/runtime baseline around the user-owned ROS2 GUI:

- `runtime_integration/gui/gui_ros2.py` is now the only formal GUI entry inside `mpc_control_new`. The temporary debug GUI stack created in the previous phase was removed instead of being maintained in parallel.
- `runtime_integration/common/encoder_protocol.py` now carries the minimal protocol surface needed by the GUI and the new runtime adapters, so the package no longer depends on an external top-level `encoder_protocol.py`.
- The sim and live ROS2 boundaries now both speak the same compact topic contract used by the GUI: `tip|jointN/motor_command` and `tip|jointN/motor_feedback`. `live_runtime_provider.py`, `live_command_dispatcher.py`, `sim_observation_provider.py`, and `sim_command_dispatcher.py` all report explicit `gui_ros2_compatible` diagnostics.
- `ros2_runtime_node.py` now includes a loopback topic broker and GUI-topic-aware publisher/subscription helpers, so fake/shim tests can exercise the real topic contract without online hardware.
- `runtime_integration/gui/gui_ros2.py` is now the only formal GUI launch entry. It owns backend discovery, embedded sim bootstrap, live ROS2 probing, command fan-out, dual-feedback aggregation, and headless launch diagnostics directly.
- 正式 GUI 启动方式固定为：
  `cd /home/cty/mpc_control_new/runtime_integration/gui`
  `python3 gui_ros2.py`
- 从工程根目录运行同一文件路径也等价；用户不再需要根据 sim / live ROS2 场景切换不同 GUI 启动脚本。
- `runtime_integration/apps/run_sim_runtime_demo.py` 和 `runtime_integration/apps/run_ros2_runtime_demo.py` 仅保留为内部 demo / 诊断脚本，不是正式 GUI 入口。
- Self-containment checks now guard against legacy imports, stale absolute paths, and top-level `encoder_protocol` imports.

## Phase 11

Phase 11 restores the old working style of "independent sim visualizer + independent GUI control panel" inside the new package:

- `runtime_integration/sim_backend/ros2_backend.py` now exposes a standalone simulator backend that subscribes to the same compact ROS2 motor-command topics used by `runtime_integration/gui/gui_ros2.py`.
- `runtime_integration/sim_backend/visualizer.py` now provides a separate visualizer process that subscribes to `sim/state` and renders the tip / joint1 / joint2 chain in its own window.
- `runtime_integration/sim_backend/run_sim_backend.py`, `runtime_integration/sim_backend/run_sim_visualizer.py`, and `runtime_integration/sim_backend/start_gui_sim_stack.sh` now provide the independent sim launch path fully inside `mpc_control_new`.
- The canonical GUI is still only `runtime_integration/gui/gui_ros2.py`. The visualizer is not embedded into the GUI.
- The migrated sim stack no longer imports legacy `sim` modules or legacy project paths.

## Current Launch Path

正式 GUI 入口固定为：

- `runtime_integration/gui/gui_ros2.py`

推荐使用方式：

第一终端：

- `bash /home/cty/mpc_control_new/runtime_integration/sim_backend/start_gui_sim_stack.sh`

第二终端：

- `python3 /home/cty/mpc_control_new/runtime_integration/gui/gui_ros2.py`

说明：

- sim visualizer 是独立窗口/独立进程，不嵌入 `gui_ros2.py`
- sim backend、sim visualizer、GUI 通过当前 compact ROS2 topic 语义联动
- 新架构下的 sim 栈只依赖 `mpc_control_new` 内部路径，可在 `/home/cty/mpc_control_new` 下独立继续开发
