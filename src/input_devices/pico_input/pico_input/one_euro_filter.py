#!/usr/bin/env python3
"""
One-Euro Filter - Adaptive low-pass filter (Casiez et al., CHI 2012)

Suitable for pose smoothing in teleoperation:
  - At rest: low cutoff frequency -> strong smoothing (suppresses sensor jitter)
  - During fast motion: high cutoff frequency -> weak smoothing (low latency, responsive)

Supports:
  - 3D vector filtering (position, elbow direction)
  - Quaternion filtering (SLERP-based, orientation)

Dependencies: numpy (pure math, no ROS2 dependency)

Reference: https://cristal.univ-lille.fr/~casiez/1euro/
"""

import numpy as np


def _smoothing_factor(rate: float, cutoff: float) -> float:
    """Compute EMA smoothing coefficient alpha from sampling rate and cutoff frequency"""
    tau = 1.0 / (2.0 * np.pi * cutoff)
    return 1.0 / (1.0 + tau * rate)


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """
    Spherical Linear Interpolation (SLERP)

    Args:
        q0: Start quaternion [qx, qy, qz, qw]
        q1: Target quaternion [qx, qy, qz, qw] (already ensured same hemisphere)
        t: Interpolation parameter [0, 1], 0->q0, 1->q1

    Returns:
        Normalized interpolated quaternion
    """
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # Nearly identical -> linear interpolation to avoid sin(0) numerical issues
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    result = s0 * q0 + s1 * q1
    return result / np.linalg.norm(result)


class OneEuroFilter:
    """
    One-Euro Filter (3D vector)

    Algorithm:
      1. dx = (x - x_prev) * rate          # derivative (velocity)
      2. dx_hat = LPF(dx, d_cutoff)         # smoothed derivative
      3. cutoff = min_cutoff + beta * |dx_hat|  # adaptive cutoff frequency
      4. x_hat = LPF(x, cutoff)             # smoothed value

    Parameters:
        rate: Sampling rate (Hz), e.g. 90.0
        min_cutoff: Minimum cutoff frequency (Hz), controls smoothing strength at rest
                    Lower -> smoother at rest, but higher latency on fast startup
        beta: Speed response coefficient, controls speed's influence on cutoff frequency
              Higher -> tighter tracking during fast motion, but may miss jitter
        d_cutoff: Derivative filter cutoff frequency (Hz), usually fixed at 1.0
    """

    def __init__(self, rate: float, min_cutoff: float = 1.0,
                 beta: float = 0.5, d_cutoff: float = 1.0):
        self.rate = rate
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_prev = None
        self._dx_prev = None

    def __call__(self, x: np.ndarray) -> np.ndarray:
        """Filter a 3D vector, return filtered result"""
        x = np.asarray(x, dtype=np.float64)

        if self._x_prev is None:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            return x.copy()

        # 1. Compute derivative
        dx = (x - self._x_prev) * self.rate

        # 2. Low-pass filter derivative
        a_d = _smoothing_factor(self.rate, self.d_cutoff)
        self._dx_prev = a_d * dx + (1.0 - a_d) * self._dx_prev

        # 3. Adaptive cutoff frequency = min_cutoff + beta * speed
        speed = np.linalg.norm(self._dx_prev)
        cutoff = self.min_cutoff + self.beta * speed

        # 4. Low-pass filter value
        a = _smoothing_factor(self.rate, cutoff)
        self._x_prev = a * x + (1.0 - a) * self._x_prev

        return self._x_prev.copy()

    def reset(self):
        """Reset filter state"""
        self._x_prev = None
        self._dx_prev = None


class OneEuroFilterQuat:
    """
    One-Euro Filter (quaternion, SLERP-based)

    Uses inter-frame rotation angle as velocity metric, SLERP replaces linear interpolation.
    Automatically handles quaternion double cover (q and -q represent the same rotation).

    Parameters have the same meaning as OneEuroFilter.
    """

    def __init__(self, rate: float, min_cutoff: float = 1.0,
                 beta: float = 0.5, d_cutoff: float = 1.0):
        self.rate = rate
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._q_prev = None
        self._speed_prev = 0.0

    def __call__(self, q: np.ndarray) -> np.ndarray:
        """Filter a quaternion [qx, qy, qz, qw], return filtered result"""
        q = np.asarray(q, dtype=np.float64)

        # Normalize
        qn = np.linalg.norm(q)
        if qn < 1e-10:
            return self._q_prev.copy() if self._q_prev is not None else q
        q = q / qn

        # Double cover: ensure same hemisphere as previous frame (shortest path)
        if self._q_prev is not None and np.dot(q, self._q_prev) < 0.0:
            q = -q

        if self._q_prev is None:
            self._q_prev = q.copy()
            self._speed_prev = 0.0
            return q.copy()

        # 1. Compute angular velocity (rad/s)
        dot = np.clip(np.dot(q, self._q_prev), -1.0, 1.0)
        angle = 2.0 * np.arccos(abs(dot))
        speed = angle * self.rate

        # 2. Low-pass filter speed
        a_d = _smoothing_factor(self.rate, self.d_cutoff)
        self._speed_prev = a_d * speed + (1.0 - a_d) * self._speed_prev

        # 3. Adaptive cutoff frequency
        cutoff = self.min_cutoff + self.beta * self._speed_prev

        # 4. SLERP filtering (alpha=1->track, alpha=0->hold)
        a = _smoothing_factor(self.rate, cutoff)
        self._q_prev = _slerp(self._q_prev, q, a)

        return self._q_prev.copy()

    def reset(self):
        """Reset filter state"""
        self._q_prev = None
        self._speed_prev = 0.0
