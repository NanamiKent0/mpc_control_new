from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .operator_view_model import OperatorViewState


class OperatorDiagnosticsPanel(ttk.LabelFrame):
    """Show planner/scheduler diagnostics without exposing low-level control widgets."""

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master, text="诊断区")
        self.planner_mode_var = tk.StringVar(value="-")
        self.selected_joint_var = tk.StringVar(value="-")
        self.active_pair_var = tk.StringVar(value="-")
        self.skill_node_var = tk.StringVar(value="-")

        self.log_text = tk.Text(self, width=72, height=10)
        self.log_text.configure(state="disabled")
        self._build()

    def _build(self) -> None:
        rows = [
            ("Planner 模式", self.planner_mode_var),
            ("当前选中 Joint", self.selected_joint_var),
            ("当前 Active Pair", self.active_pair_var),
            ("当前 Skill Node", self.skill_node_var),
        ]
        for i, (label, var) in enumerate(rows):
            ttk.Label(self, text=label).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(self, textvariable=var).grid(row=i, column=1, sticky="w", padx=4, pady=2)

        self.log_text.grid(row=len(rows), column=0, columnspan=2, padx=4, pady=6, sticky="nsew")
        self.columnconfigure(1, weight=1)

    def update_from_state(self, state: OperatorViewState) -> None:
        self.planner_mode_var.set(state.planner_mode)
        self.selected_joint_var.set(state.selected_joint_id)
        self.active_pair_var.set(state.active_pair)
        self.skill_node_var.set(state.current_skill_node)

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, state.last_log + "\n")
        for k, v in sorted(state.diagnostics.items()):
            self.log_text.insert(tk.END, f"{k}: {v}\n")
        self.log_text.configure(state="disabled")