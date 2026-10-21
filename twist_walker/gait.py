"""Trot gait planner in foot (Cartesian) space. Pure Python, no ROS, so it can be unit-tested.

Concepts:
- Trot: diagonal pairs move together. FL+RR swing while FR+RL push, then they swap.
- Each foot cycles through two halves of the period:
    swing  (phase 0.0-0.5): foot lifts and moves forward from -step/2 to +step/2
    stance (phase 0.5-1.0): foot on the ground, slides back from +step/2 to -step/2,
                            which pushes the body forward.
- Step vector per foot comes from the body velocity command (vx, vy, wz):
      foot velocity = v + w x r   ->   step = that velocity * (time in stance)
- A cycloid profile makes the swing start and end with zero velocity, so feet
  touch down softly instead of slapping the floor (a big source of shaking).
"""

import math

import numpy as np

# Phase offset per leg: diagonals share a phase
TROT_OFFSETS = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


def cycloid(s):
    """0->1 with zero velocity at both ends."""
    return s - math.sin(2.0 * math.pi * s) / (2.0 * math.pi)


class TrotGait:
    def __init__(self, nominal_feet, period=0.8, step_height=0.02, max_step=0.05):
        self.nominal = {k: np.array(v, dtype=float) for k, v in nominal_feet.items()}
        self.period = period
        self.step_height = step_height
        self.max_step = max_step
        self.phase = 0.0          # 0..1, global clock of the gait
        self.walking = False

    def step_vector(self, leg, vx, vy, wz):
        n = self.nominal[leg]
        stance_time = self.period / 2.0
        vel = np.array([vx - wz * n[1], vy + wz * n[0]])
        step = vel * stance_time
        norm = float(np.linalg.norm(step))
        if norm > self.max_step:
            step *= self.max_step / norm
        return step

    def update(self, dt, vx, vy, wz, moving_eps=1e-3):
        """Advance the gait clock and return {leg: foot target (base frame)}."""
        commanded = abs(vx) > moving_eps or abs(vy) > moving_eps or abs(wz) > moving_eps
        if commanded:
            self.walking = True
        if self.walking:
            prev = self.phase
            self.phase = (self.phase + dt / self.period) % 1.0
            if not commanded:
                # Finish the current half-cycle, then stop with all feet on the ground
                crossed = int(prev * 2.0) != int(self.phase * 2.0) or self.phase < prev
                if crossed:
                    self.phase = round(self.phase * 2.0) / 2.0 % 1.0
                    self.walking = False
        return {leg: self.foot_target(leg, vx, vy, wz) for leg in self.nominal}

    def foot_target(self, leg, vx, vy, wz):
        n = self.nominal[leg]
        if not self.walking:
            return n.copy()
        step = self.step_vector(leg, vx, vy, wz)
        ph = (self.phase + TROT_OFFSETS[leg]) % 1.0
        target = n.copy()
        if ph < 0.5:                                   # swing
            s = ph / 0.5
            target[:2] += step * (cycloid(s) - 0.5)
            target[2] += self.step_height * math.sin(math.pi * s)
        else:                                          # stance
            s = (ph - 0.5) / 0.5
            target[:2] += step * (0.5 - s)
        return target
