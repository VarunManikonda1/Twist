"""Unit tests for leg kinematics and the trot gait planner."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from twist_2_0_description.gait import GaitController, GaitParams
from twist_2_0_description.joints import JOINT_NAMES, LEG_INDICES
from twist_2_0_description.kinematics import (
    DEFAULT_SEED,
    LEGS,
    SOFT_LOWER,
    SOFT_UPPER,
    forward_kinematics,
    inverse_kinematics,
    jacobian,
)

URDF = os.path.join(os.path.dirname(__file__), "..", "urdf", "twist_2_0.urdf")
MAX_JOINT_SPEED = 4.71  # rad/s, URDF velocity limit


def _urdf_fk(leg: str, q) -> np.ndarray:
    """Independent FK straight from the URDF XML."""
    root = ET.parse(URDF).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}

    def vec(el, attr):
        return np.array([float(v) for v in el.get(attr).split()])

    def rot(axis, a):
        axis = axis / np.linalg.norm(axis)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k

    r, p = np.eye(3), np.zeros(3)
    for name, qi in zip(("hip_yaw", "hip_pitch", "knee"), q):
        j = joints[f"{leg}_{name}"]
        p = p + r @ vec(j.find("origin"), "xyz")
        r = r @ rot(vec(j.find("axis"), "xyz"), qi)
    shin = next(link for link in root.findall("link") if link.get("name") == f"{leg}_shin")
    tip = next(c for c in shin.findall("collision") if c.get("name") == "foot_tip")
    return p + r @ vec(tip.find("origin"), "xyz")


@pytest.mark.parametrize("leg", LEGS)
@pytest.mark.parametrize("q", [(0, 0, 0), (0.3, -1.0, 0.5), (-0.4, -1.3, 1.0)])
def test_fk_matches_urdf(leg, q):
    assert np.allclose(forward_kinematics(leg, *q), _urdf_fk(leg, q), atol=1e-6)


@pytest.mark.parametrize("leg", LEGS)
def test_jacobian_matches_finite_difference(leg):
    q = np.array([0.1, -0.9, 0.4])
    num = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = 1e-6
        num[:, i] = (forward_kinematics(leg, *(q + d)) - forward_kinematics(leg, *(q - d))) / 2e-6
    assert np.allclose(jacobian(leg, q), num, atol=1e-6)


@pytest.mark.parametrize("leg", LEGS)
def test_ik_round_trip(leg):
    q_true = np.array([0.2, -0.95, 0.45])
    target = forward_kinematics(leg, *q_true)
    q, err = inverse_kinematics(leg, target, DEFAULT_SEED)
    assert err < 1e-4
    assert np.allclose(forward_kinematics(leg, *q), target, atol=1e-4)


def test_stand_pose_feet_level_and_within_limits():
    g = GaitController()
    q = np.array(g.stand_positions())
    assert len(q) == len(JOINT_NAMES) == 12
    heights = []
    for leg, idx in LEG_INDICES.items():
        qi = q[list(idx)]
        assert np.all(qi >= SOFT_LOWER) and np.all(qi <= SOFT_UPPER)
        foot = forward_kinematics(leg, *qi)
        assert np.allclose(foot, g.nominal_foot(leg), atol=1e-4)
        heights.append(foot[2])
    assert max(heights) - min(heights) < 1e-4


def test_idle_command_keeps_standing():
    g = GaitController()
    q0 = g.stand_positions()
    for _ in range(100):
        q = g.update(0.02, 0.0, 0.0, 0.0)
    assert not g.active
    assert np.allclose(q, q0, atol=1e-6)


COMMANDS = [
    (0.15, 0.0, 0.0),
    (-0.15, 0.0, 0.0),
    (0.0, 0.12, 0.0),
    (0.0, -0.12, 0.0),
    (0.0, 0.0, 0.8),
    (0.0, 0.0, -0.8),
    (0.1, 0.05, 0.4),
]


@pytest.mark.parametrize("style,period", [("trot", 0.6), ("crawl", 1.2)])
@pytest.mark.parametrize("cmd", COMMANDS)
def test_gait_is_reachable_smooth_and_stops(style, period, cmd):
    g = GaitController(GaitParams(gait_style=style, cycle_period=period))
    prev = np.array(g.stand_positions())
    dt = 0.01
    for _ in range(500):
        q = np.array(g.update(dt, *cmd))
        assert np.all(np.isfinite(q))
        assert np.max(np.abs(q - prev)) / dt < MAX_JOINT_SPEED * 1.15
        for leg, idx in LEG_INDICES.items():
            qi = q[list(idx)]
            assert np.all(qi >= SOFT_LOWER - 1e-9) and np.all(qi <= SOFT_UPPER + 1e-9)
            # IK actually reaches the planned foot position.
            assert np.linalg.norm(forward_kinematics(leg, *qi) - g.feet[leg]) < 1e-3
        prev = q
    assert g.active
    for _ in range(300):
        g.update(dt, 0.0, 0.0, 0.0)
    assert not g.active
    for leg in LEGS:
        assert np.linalg.norm(g.feet[leg] - g.nominal_foot(leg)) < 0.005


def test_stance_feet_move_opposite_to_body():
    """Feet on the ground must slide backwards under a forward-walking body."""
    g = GaitController()
    g.stand_positions()
    for _ in range(200):
        g.update(0.01, 0.1, 0.0, 0.0)
    before = {leg: g.feet[leg].copy() for leg in LEGS}
    g.update(0.01, 0.1, 0.0, 0.0)
    for leg in LEGS:
        if before[leg][2] == g.feet[leg][2] == -g.p.body_height:  # stance
            assert g.feet[leg][0] < before[leg][0]


def test_trot_pairs_diagonal_legs():
    g = GaitController()
    g.stand_positions()
    lifted = {leg: 0 for leg in LEGS}
    together = 0
    for _ in range(300):
        g.update(0.01, 0.1, 0.0, 0.0)
        up = {leg: g.feet[leg][2] > -g.p.body_height + 1e-4 for leg in LEGS}
        for leg in LEGS:
            lifted[leg] += up[leg]
        assert up["FL"] == up["RR"] and up["FR"] == up["RL"]
        together += up["FL"] and up["FR"]
    assert all(v > 0 for v in lifted.values())
    assert together == 0
