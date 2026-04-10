"""Self-contained simulation backend used by the new runtime integration path."""

from .backend import SIM_BACKEND_VERSION, SimRuntimeBackend
from .types import SIM_MODULE_IDS, CouplingState, SimCommand, SimState

__all__ = [
    "SIM_BACKEND_VERSION",
    "SIM_MODULE_IDS",
    "CouplingState",
    "SimCommand",
    "SimRuntimeBackend",
    "SimState",
]
