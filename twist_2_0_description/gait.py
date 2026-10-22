"""Velocity-driven trot gait: Cartesian foot trajectories + exact IK.

The planner works in base_link. Stance feet slide backwards under the body at exactly the
commanded body twist (so they do not slip on the ground); swing feet lift and are placed
symmetrically about their nominal spot (Raibert heuristic) so each stance sweep is centred.
When the command goes to zero the robot finishes one full cycle so every foot lands back on
its nominal spot, then stops stepping and stands still.
"""

import math
from dataclasses import dataclass

import numpy as np

from twist_2_0_description.joints import LEG_INDICES
from twist_2_0_description.kinematics import DEFAULT_SEED, LEGS, inverse_kinematics

# Diagonal trot: FL+RR together, FR+RL half a cycle later.
TROT_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
# Four-beat crawl (always >= 3 feet down when swing_fraction <= 0.25).
CRAWL_OFFSET = {"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75}

_CORNER = {"FL": (1.0, 1.0), "FR": (1.0, -1.0), "RL": (-1.0, 1.0), "RR": (-1.0, -1.0)}


@dataclass
class GaitParams:
    body_height: float = 0.14  # foot-sphere centre below base_link while standing (m)
    foot_x: float = 0.14  # nominal |x| of each foot (m)
    foot_y: float = 0.14  # nominal |y| of each foot (m)
    step_height: float = 0.025  # swing clearance (m)
    cycle_period: float = 0.6  # one full gait cycle (s)
    swing_fraction: float = 0.45  # per leg; < 0.5 gives a short 4-foot support phase
    gait_style: str = "trot"  # trot | crawl
    max_vx: float = 0.15  # m/s
    max_vy: float = 0.12  # m/s
    max_wz: float = 0.8  # rad/s
    max_lin_accel: float = 0.6  # m/s^2
    max_ang_accel: float = 3.0  # rad/s^2
    max_foot_offset: float = 0.05  # keep feet inside the reachable box around nominal (m)
    max_swing_speed: float = 0.3  # peak swing-foot speed (m/s); STS3215 tops out ~4.7 rad/s


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def _approach(cur: float, tgt: float, step: float) -> float:
    if tgt > cur:
        return min(tgt, cur + step)
    return max(tgt, cur - step)


class GaitController:
    def __init__(self, params: GaitParams | None = None) -> None:
        self.p = params or GaitParams()
        self._q = {leg: np.array(DEFAULT_SEED, dtype=float) for leg in LEGS}
        self.cmd = np.zeros(3)  # rate-limited (vx, vy, wz) actually being walked
        self.phase = 0.0
        self.active = False
        self._idle_time = 0.0
        self._in_swing = {leg: False for leg in LEGS}
        self._liftoff = {}
        self.feet = {leg: self.nominal_foot(leg) for leg in LEGS}

    # --- geometry -------------------------------------------------------------------
    def nominal_foot(self, leg: str) -> np.ndarray:
        sx, sy = _CORNER[leg]
        return np.array([sx * self.p.foot_x, sy * self.p.foot_y, -self.p.body_height])

    def _offsets(self) -> dict[str, float]:
        return CRAWL_OFFSET if self.p.gait_style == "crawl" else TROT_OFFSET

    def _leg_phase(self, leg: str) -> float:
        return (self.phase - self._offsets()[leg]) % 1.0

    def _swing_fraction(self) -> float:
        # A crawl only has an all-feet-down moment (needed to stop cleanly) below 0.25.
        if self.p.gait_style == "crawl":
            return min(self.p.swing_fraction, 0.22)
        return self.p.swing_fraction

    def _stance_duration(self) -> float:
        return self.p.cycle_period * (1.0 - self._swing_fraction())

    def _feasible_twist(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        """Scale the twist down so no foot's stance sweep leaves its reachable box."""
        t_stance = self._stance_duration()
        t_swing = self.p.cycle_period - t_stance
        # Stance sweep must fit in the reachable box, and the swing (smoothstep peak speed is
        # 1.5x average) must stay within what the servos can track.
        sweep_limit = min(
            1.6 * self.p.max_foot_offset / t_stance,
            self.p.max_swing_speed * t_swing / (1.5 * t_stance),
        )
        fastest = 0.0
        for leg in LEGS:
            x, y, _ = self.nominal_foot(leg)
            fastest = max(fastest, math.hypot(vx - wz * y, vy + wz * x))
        if fastest > sweep_limit:
            k = sweep_limit / fastest
            return vx * k, vy * k, wz * k
        return vx, vy, wz

    def _touchdown_target(self, leg: str) -> np.ndarray:
        """Place the foot so the coming stance sweep is centred on the nominal spot."""
        vx, vy, wz = self.cmd
        half = 0.5 * self._stance_duration()
        nom = self.nominal_foot(leg)
        c, s = math.cos(wz * half), math.sin(wz * half)
        x = c * nom[0] - s * nom[1] + vx * half
        y = s * nom[0] + c * nom[1] + vy * half
        return np.array([x, y, nom[2]])

    def _limit(self, leg: str, foot: np.ndarray) -> np.ndarray:
        nom = self.nominal_foot(leg)
        lim = self.p.max_foot_offset
        foot[0] = nom[0] + _clamp(foot[0] - nom[0], lim)
        foot[1] = nom[1] + _clamp(foot[1] - nom[1], lim)
        return foot

    # --- main update ----------------------------------------------------------------
    def set_params(self, params: GaitParams) -> None:
        self.p = params

    def update(self, dt: float, vx: float, vy: float, wz: float) -> list[float]:
        """Advance the gait by dt seconds towards body twist (vx, vy, wz); return 12 joints."""
        p = self.p
        dt = max(0.0, min(dt, 0.05))
        target = self._feasible_twist(
            _clamp(vx, p.max_vx), _clamp(vy, p.max_vy), _clamp(wz, p.max_wz)
        )
        self.cmd[0] = _approach(self.cmd[0], target[0], p.max_lin_accel * dt)
        self.cmd[1] = _approach(self.cmd[1], target[1], p.max_lin_accel * dt)
        self.cmd[2] = _approach(self.cmd[2], target[2], p.max_ang_accel * dt)

        wants_motion = max(abs(target[0]), abs(target[1]), abs(target[2])) > 1e-3
        moving = float(np.max(np.abs(self.cmd))) > 1e-3
        if wants_motion or moving:
            self.active = True
            self._idle_time = 0.0
        elif self.active:
            self._idle_time += dt

        if self.active:
            self.phase = (self.phase + dt / p.cycle_period) % 1.0

        vx_c, vy_c, wz_c = self.cmd
        swing_fraction = self._swing_fraction()
        any_swing = False
        for leg in LEGS:
            u = self._leg_phase(leg)
            swing = self.active and u < swing_fraction
            foot = self.feet[leg]
            if swing:
                any_swing = True
                if not self._in_swing[leg]:
                    self._liftoff[leg] = foot.copy()
                s = u / swing_fraction
                start = self._liftoff[leg]
                goal = self._touchdown_target(leg)
                blend = s * s * (3.0 - 2.0 * s)
                foot = start + (goal - start) * blend
                foot[2] = goal[2] + p.step_height * math.sin(math.pi * s)
            else:
                # Ground point under a body moving with twist (v, w): dp/dt = -(v + w x p).
                fx, fy = foot[0], foot[1]
                foot = foot.copy()
                foot[0] -= (vx_c - wz_c * fy) * dt
                foot[1] -= (vy_c + wz_c * fx) * dt
                foot[2] = -p.body_height
            self._in_swing[leg] = swing
            self.feet[leg] = self._limit(leg, foot)

        # Stop stepping once a full cycle at zero command has re-centred every foot.
        if self.active and self._idle_time >= p.cycle_period and not any_swing:
            self.active = False
            self._idle_time = 0.0

        return self.joint_positions()

    def joint_positions(self) -> list[float]:
        q = [0.0] * 12
        for leg in LEGS:
            sol, _err = inverse_kinematics(leg, self.feet[leg], self._q[leg])
            self._q[leg] = sol
            for idx, val in zip(LEG_INDICES[leg], sol):
                q[idx] = float(val)
        return q

    def stand_positions(self) -> list[float]:
        """Joint angles for the nominal stance (resets the planner to standing)."""
        self.feet = {leg: self.nominal_foot(leg) for leg in LEGS}
        self._q = {leg: np.array(DEFAULT_SEED, dtype=float) for leg in LEGS}
        self.active = False
        self.cmd[:] = 0.0
        return self.joint_positions()
