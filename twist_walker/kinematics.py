"""Leg kinematics built directly from the URDF.

Nothing about the robot's geometry is hard-coded here. Every joint origin, axis
and limit comes from the URDF, so the joint directions can never disagree with
what Gazebo simulates.

Concepts:
- Forward kinematics (FK): joint angles -> foot position, by multiplying the
  chain of 4x4 transforms from base_link down to the foot.
- Inverse kinematics (IK): foot position -> joint angles. Solved numerically with
  damped least squares, using the geometric Jacobian (axis x lever arm).
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis, angle):
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _homog(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def _floats(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, dtype=float)


class Joint:
    def __init__(self, el):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        origin = el.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None, [0, 0, 0])
        rpy = _floats(origin.get("rpy") if origin is not None else None, [0, 0, 0])
        self.origin = _homog(_rpy_to_matrix(*rpy), xyz)
        axis = el.find("axis")
        self.axis = _floats(axis.get("xyz") if axis is not None else None, [1, 0, 0])
        lim = el.find("limit")
        if self.type == "revolute" and lim is not None:
            self.lower = float(lim.get("lower", -math.pi))
            self.upper = float(lim.get("upper", math.pi))
        else:
            self.lower, self.upper = -math.inf, math.inf

    @property
    def movable(self):
        return self.type in ("revolute", "continuous")


class Leg:
    """One leg: base_link -> hip_yaw -> hip_pitch -> knee -> foot point."""

    def __init__(self, chain, foot_point, joint_names):
        self.chain = chain                    # all joints root->foot link (fixed ones included)
        self.foot_point = foot_point          # foot contact point in the last link's frame
        self.joint_names = joint_names        # the 3 actuated joints, in order
        movable = [j for j in chain if j.movable]
        self.lower = np.array([j.lower for j in movable])
        self.upper = np.array([j.upper for j in movable])
        # Position of the first actuated joint (hip_yaw) in the base frame, at zero pose
        t = np.eye(4)
        for j in chain:
            t = t @ j.origin
            if j.movable:
                self.hip_position = t[:3, 3].copy()
                break

    def fk(self, q):
        """Foot position (base frame) plus joint positions and world axes for the Jacobian."""
        t = np.eye(4)
        i = 0
        joint_pos, joint_axes = [], []
        for j in self.chain:
            t = t @ j.origin
            if j.movable:
                axis_world = t[:3, :3] @ (j.axis / np.linalg.norm(j.axis))
                joint_pos.append(t[:3, 3].copy())
                joint_axes.append(axis_world)
                t = t @ _homog(_axis_angle(j.axis, q[i]), np.zeros(3))
                i += 1
        foot = (t @ np.append(self.foot_point, 1.0))[:3]
        return foot, joint_pos, joint_axes

    def foot(self, q):
        return self.fk(q)[0]

    def ik(self, target, seed, iters=30, damping=1e-3, tol=1e-5):
        """Damped least squares IK. Returns (q, residual_error_m)."""
        q = np.clip(np.array(seed, dtype=float), self.lower, self.upper)
        err = np.inf
        for _ in range(iters):
            foot, jp, ja = self.fk(q)
            e = target - foot
            err = float(np.linalg.norm(e))
            if err < tol:
                break
            jac = np.column_stack([np.cross(a, foot - p) for a, p in zip(ja, jp)])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(3), e)
            # limit the step so a single iteration can't fling the leg to another branch
            n = np.linalg.norm(dq)
            if n > 0.3:
                dq *= 0.3 / n
            q = np.clip(q + dq, self.lower, self.upper)
        return q, err


def _find_foot(joints_by_parent, links, start_link, start_tf=np.eye(4)):
    """Search start_link and anything rigidly attached to it for a sphere collision (the foot)."""
    link = links.get(start_link)
    if link is not None:
        for col in link.findall("collision"):
            if col.find("geometry/sphere") is not None:
                o = col.find("origin")
                xyz = _floats(o.get("xyz") if o is not None else None, [0, 0, 0])
                return (start_tf @ np.append(xyz, 1.0))[:3]
    for j in joints_by_parent.get(start_link, []):
        if j.type == "fixed":
            found = _find_foot(joints_by_parent, links, j.child, start_tf @ j.origin)
            if found is not None:
                return found
    return None


def load_legs(urdf_xml, leg_prefixes=("FL", "FR", "RL", "RR"), foot_overrides=None):
    """Parse URDF text and build one Leg per prefix ({prefix}_hip_yaw/_hip_pitch/_knee)."""
    root = ET.fromstring(urdf_xml)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = [Joint(j) for j in root.findall("joint")]
    by_child = {j.child: j for j in joints}
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.parent, []).append(j)

    legs = {}
    for p in leg_prefixes:
        names = [f"{p}_hip_yaw", f"{p}_hip_pitch", f"{p}_knee"]
        knee = next((j for j in joints if j.name == names[2]), None)
        if knee is None:
            raise ValueError(f"joint {names[2]} not found in URDF")
        chain, link = [], knee.child
        while link in by_child:
            j = by_child[link]
            chain.insert(0, j)
            link = j.parent
        actuated = [j.name for j in chain if j.movable]
        if actuated != names:
            raise ValueError(f"leg {p}: expected movable joints {names}, URDF chain has {actuated}")
        foot = None
        if foot_overrides and p in foot_overrides:
            foot = np.array(foot_overrides[p], dtype=float)
        if foot is None:
            foot = _find_foot(by_parent, links, knee.child)
        if foot is None:
            raise ValueError(
                f"leg {p}: no sphere collision found on {knee.child}; set a foot offset manually")
        legs[p] = Leg(chain, foot, names)
    return legs
