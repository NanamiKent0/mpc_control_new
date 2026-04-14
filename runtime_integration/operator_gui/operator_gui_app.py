from __future__ import annotations

import tkinter as tk
from typing import Any

from .operator_controls import OperatorControls
from .operator_diagnostics_panel import OperatorDiagnosticsPanel
from .operator_status_panel import OperatorStatusPanel
from .operator_view_model import OperatorViewModel


class OperatorGUIApp:
    """High-level operator GUI: the user only controls tip growth / tip turn intent."""

    def __init__(self, runtime_session: Any, refresh_ms: int = 250) -> None:
        self.runtime_session = runtime_session
        self.refresh_ms = refresh_ms

        self.root = tk.Tk()
        self.root.title("Tip-Level Operator GUI")

        self.view_model = OperatorViewModel(runtime_session)

        self.controls = OperatorControls(
            self.root,
            on_growth_start=self._growth_start,
            on_growth_stop=self._growth_stop,
            on_turn=self._turn,
            on_stop=self._stop,
            on_estop=self._estop,
        )
        self.status_panel = OperatorStatusPanel(self.root)
        self.diag_panel = OperatorDiagnosticsPanel(self.root)

        self.controls.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self.status_panel.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        self.diag_panel.grid(row=2, column=0, sticky="nsew", padx=8, pady=8)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._bootstrap_runtime()
        self._schedule_refresh()

    def _bootstrap_runtime(self) -> None:
        """
        Start RuntimeSession and load a sane default high-level intent.
        Default mode is TIP_FREE_GROWTH with a nominal growth speed.
        """
        try:
            if hasattr(self.runtime_session, "start"):
                self.runtime_session.start()
        except Exception:
            pass

        try:
            self.view_model.submit_free_growth(8.0)
        except Exception:
            pass

    def _growth_start(self, speed: float) -> None:
        self.view_model.submit_free_growth(speed)

    def _growth_stop(self) -> None:
        self.view_model.submit_stop()

    def _turn(self, delta: float) -> None:
        self.view_model.submit_turn(delta)

    def _stop(self) -> None:
        self.view_model.submit_stop()

    def _estop(self) -> None:
        self.view_model.submit_estop()

    def _schedule_refresh(self) -> None:
        step_result = None
        runtime_error = None

        try:
            if hasattr(self.runtime_session, "step"):
                step_result = self.runtime_session.step()
        except Exception as exc:
            runtime_error = exc

        try:
            state = self.view_model.refresh(
                last_step_result=step_result,
                runtime_error=runtime_error,
            )
            self.status_panel.update_from_state(state)
            self.diag_panel.update_from_state(state)
        finally:
            self.root.after(self.refresh_ms, self._schedule_refresh)

    def _on_close(self) -> None:
        try:
            if hasattr(self.runtime_session, "stop"):
                self.runtime_session.stop()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()