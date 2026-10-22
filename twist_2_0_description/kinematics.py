"""Exact leg kinematics for Twist 2.0, taken directly from twist_2_0.urdf.

Every leg is hip_yaw (Z) -> hip_pitch -> knee, with pitch and knee axes parallel and
lying horizontally at 45 deg to the body axes (legs radiate diagonally from the corners).
The foot point is the centre of the `foot_tip` collision sphere on each shin.

Frames: base_link, X forward, Y left, Z up. Foot positions are expressed in base_link.
"""

import math

import numpy as np
from numpy.typing import NDArray

LEGS = ("FL", "FR", "RL", "RR")

FOOT_RADIUS = 0.0075

# Per leg: joint origins (xyz in parent frame) and axes, copied verbatim from the URDF.
_GEOMETRY = {
    "FL": {
        "yaw_origin": (0.0809099, 0.0809099, 0.0),
        "pitch_origin": (0.0282843, 0.0282843, 0.0),
        "pitch_axis": (0.707107, -0.707107, 0.0),
        "knee_origin": (0.0385, 0.0385, 0.0544472),
        "foot": (0.0258498, 0.0219084, -0.131914),
    },
    "FR": {
        "yaw_origin": (0.0810867, -0.0810867, 0.0),
        "pitch_origin": (0.0282843, -0.0282843, 0.0),
        "pitch_axis": (-0.707107, -0.707107, 0.0),
        "knee_origin": (0.0385, -0.0385, 0.0544472),
        "foot": (0.0223142, -0.0254439, -0.131914),
    },
    "RL": {
        "yaw_origin": (-0.0810867, 0.0810867, 0.0),
        "pitch_origin": (-0.0282843, 0.0282843, 0.0),
        "pitch_axis": (0.707107, 0.707107, 0.0),
        "knee_origin": (-0.0385, 0.0385, 0.0544472),
        "foot": (-0.0223142, 0.0254439, -0.131914),
    },
    "RR": {
        "yaw_origin": (-0.0810867, -0.0810867, 0.0),
        "pitch_origin": (-0.0282843, -0.0282843, 0.0),
        "pitch_axis": (-0.707107, 0.707107, 0.0),
        "knee_origin": (-0.0385, -0.0385, 0.0544472),
        "foot": (-0.0254439, -0.0223142, -0.131914),
    },
}

# Recommended working range (URDF <safety_controller> soft limits): (hip_yaw, hip_pitch, knee).
SOFT_LOWER = np.array([-1.5708, -1.5708, -0.3491])
SOFT_UPPER = np.array([1.5708, 0.8727, 2.6180])

_Z = np.array([0.0, 0.0, 1.0])


def _unit(v) -> NDArray:
    a = np.asarray(v, dtype=float)
    return a / np.linalg.norm(a)


def _rot(axis: NDArray, angle: float) -> NDArray:
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


class _Leg:
    def __init__(self, g: dict) -> None:
        self.p_yaw = np.asarray(g["yaw_origin"], dtype=float)
        self.p_pitch = np.asarray(g["pitch_origin"], dtype=float)
        self.p_knee = np.asarray(g["knee_origin"], dtype=float)
        self.p_foot = np.asarray(g["foot"], dtype=float)
        self.a_pitch = _unit(g["pitch_axis"])

    def chain(self, q) -> tuple[NDArray, list[NDArray], list[NDArray]]:
        """Return foot position plus world joint positions and axes (for the Jacobian)."""
        r1 = _rot(_Z, q[0])
        o2 = self.p_yaw + r1 @ self.p_pitch
        a2 = r1 @ self.a_pitch
        r2 = r1 @ _rot(self.a_pitch, q[1])
        o3 = o2 + r2 @ self.p_knee
        r3 = r2 @ _rot(self.a_pitch, q[2])
        foot = o3 + r3 @ self.p_foot
        return foot, [self.p_yaw, o2, o3], [_Z, a2, a2]


_LEG_MODELS = {leg: _Leg(g) for leg, g in _GEOMETRY.items()}


def hip_position(leg: str) -> NDArray:
    return _LEG_MODELS[leg].p_yaw.copy()


def forward_kinematics(leg: str, hip_yaw: float, hip_pitch: float, knee: float) -> NDArray:
    """Foot (sphere centre) position in base_link."""
    foot, _, _ = _LEG_MODELS[leg].chain((hip_yaw, hip_pitch, knee))
    return foot


def jacobian(leg: str, q) -> NDArray:
    foot, origins, axes = _LEG_MODELS[leg].chain(q)
    return np.column_stack([np.cross(a, foot - o) for a, o in zip(axes, origins)])


def inverse_kinematics(
    leg: str,
    target,
    q_init,
    tol: float = 1e-5,
    max_iter: int = 30,
) -> tuple[NDArray, float]:
    """Damped least-squares IK, warm-started from q_init and kept within soft limits.

    Returns (q, residual_error_m). Warm-starting from the previous solution keeps the
    solver on the same (knee-out) branch the stand pose uses.
    """
    model = _LEG_MODELS[leg]
    target = np.asarray(target, dtype=float)
    q = np.clip(np.asarray(q_init, dtype=float), SOFT_LOWER, SOFT_UPPER)
    lam2 = 1e-4
    err = float("inf")
    for _ in range(max_iter):
        foot, origins, axes = model.chain(q)
        e = target - foot
        err = float(np.linalg.norm(e))
        if err < tol:
            break
        j = np.column_stack([np.cross(a, foot - o) for a, o in zip(axes, origins)])
        dq = j.T @ np.linalg.solve(j @ j.T + lam2 * np.eye(3), e)
        step = float(np.max(np.abs(dq)))
        if step > 0.3:
            dq *= 0.3 / step
        q = np.clip(q + dq, SOFT_LOWER, SOFT_UPPER)
    return q, err


# Joint-space seed close to the standing branch (thigh down/out, shin vertical).
DEFAULT_SEED = (0.0, -1.2, 0.6)
