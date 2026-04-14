from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable


class OperatorControls(ttk.LabelFrame):
    """High-level tip-centric controls."""

    def __init__(
        self,
        master: tk.Misc,
        on_growth_start: Callable[[float], None],
        on_growth_stop: Callable[[], None],
        on_turn: Callable[[float], None],
        on_stop: Callable[[], None],
        on_estop: Callable[[], None],
    ) -> None:
        super().__init__(master, text="高层控制区")
        self.on_growth_start = on_growth_start
        self.on_growth_stop = on_growth_stop
        self.on_turn = on_turn
        self.on_stop = on_stop
        self.on_estop = on_estop

        self.growth_speed_var = tk.StringVar(value="8.0")
        self.turn_angle_var = tk.StringVar(value="20.0")

        self._build()

    def _build(self) -> None:
        ttk.Label(self, text="生长速度 (mm/s)").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(self, textvariable=self.growth_speed_var, width=10).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(self, text="开始生长", command=self._growth_start).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(self, text="停止生长", command=self.on_growth_stop).grid(row=0, column=3, padx=4, pady=4)

        ttk.Label(self, text="转向角 (deg, 正负表示方向)").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(self, textvariable=self.turn_angle_var, width=10).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(self, text="执行转向", command=self._turn).grid(row=1, column=2, padx=4, pady=4)

        ttk.Button(self, text="停止", command=self.on_stop).grid(row=2, column=2, padx=4, pady=6)
        ttk.Button(self, text="急停", command=self.on_estop).grid(row=2, column=3, padx=4, pady=6)

    def _growth_start(self) -> None:
        try:
            speed = float(self.growth_speed_var.get())
        except ValueError:
            speed = 8.0
        self.on_growth_start(speed)

    def _turn(self) -> None:
        try:
            delta = float(self.turn_angle_var.get())
        except ValueError:
            delta = 0.0
        self.on_turn(delta)