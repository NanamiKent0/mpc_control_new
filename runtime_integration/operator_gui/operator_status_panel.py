from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .operator_view_model import OperatorViewState


class OperatorStatusPanel(ttk.LabelFrame):
    """Tip-centric runtime status display."""

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master, text="Tip 状态区")
        self.mode_var = tk.StringVar(value="idle")
        self.heading_var = tk.StringVar(value="0.0")
        self.growth_var = tk.StringVar(value="0.0")
        self.target_var = tk.StringVar(value="0.0")
        self.done_var = tk.StringVar(value="False")
        self._build()

    def _build(self) -> None:
        rows = [
            ("当前模式", self.mode_var),
            ("当前 Tip Heading (deg)", self.heading_var),
            ("当前 Tip 生长量 (mm)", self.growth_var),
            ("当前目标转向角 (deg)", self.target_var),
            ("转向完成", self.done_var),
        ]
        for i, (label, var) in enumerate(rows):
            ttk.Label(self, text=label).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(self, textvariable=var).grid(row=i, column=1, sticky="w", padx=4, pady=2)

    def update_from_state(self, state: OperatorViewState) -> None:
        self.mode_var.set(state.mode)
        self.heading_var.set(f"{state.tip_heading_deg:.2f}")
        self.growth_var.set(f"{state.tip_growth_mm:.2f}")
        self.target_var.set(f"{state.target_heading_delta_deg:.2f}")
        self.done_var.set(str(state.turn_completed))