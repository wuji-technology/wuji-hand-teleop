#!/usr/bin/env python3
"""
One-Euro Filter - 自适应低通滤波器 (Casiez et al., CHI 2012)

适用于遥操作中的位姿平滑:
  - 静止时: 低截止频率 → 强平滑 (抑制传感器抖动)
  - 快速运动时: 高截止频率 → 弱平滑 (低延迟, 跟手)

支持:
  - 3D 向量滤波 (位置, 肘部方向)
  - 四元数滤波 (基于 SLERP, 姿态)

依赖: numpy (纯数学, 无 ROS2 依赖)

参考: https://cristal.univ-lille.fr/~casiez/1euro/
"""

import numpy as np


def _smoothing_factor(rate: float, cutoff: float) -> float:
    """从采样率和截止频率计算 EMA 平滑系数 alpha"""
    tau = 1.0 / (2.0 * np.pi * cutoff)
    return 1.0 / (1.0 + tau * rate)


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """
    球面线性插值 (SLERP)

    Args:
        q0: 起始四元数 [qx, qy, qz, qw]
        q1: 目标四元数 [qx, qy, qz, qw] (已确保同半球)
        t: 插值参数 [0, 1], 0→q0, 1→q1

    Returns:
        归一化的插值四元数
    """
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # 几乎相同 → 线性插值避免 sin(0) 数值问题
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
    One-Euro Filter (3D 向量)

    算法:
      1. dx = (x - x_prev) * rate          # 导数 (速度)
      2. dx_hat = LPF(dx, d_cutoff)         # 平滑导数
      3. cutoff = min_cutoff + beta * |dx_hat|  # 自适应截止频率
      4. x_hat = LPF(x, cutoff)             # 平滑值

    参数:
        rate: 采样率 (Hz), 例如 90.0
        min_cutoff: 最小截止频率 (Hz), 控制静止时平滑强度
                    越小 → 静止越平滑, 但快速启动延迟越大
        beta: 速度响应系数, 控制速度对截止频率的影响
              越大 → 快速运动时跟踪越紧, 但可能漏过抖动
        d_cutoff: 导数滤波截止频率 (Hz), 通常固定 1.0
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
        """滤波一个 3D 向量, 返回滤波结果"""
        x = np.asarray(x, dtype=np.float64)

        if self._x_prev is None:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            return x.copy()

        # 1. 计算导数
        dx = (x - self._x_prev) * self.rate

        # 2. 低通滤波导数
        a_d = _smoothing_factor(self.rate, self.d_cutoff)
        self._dx_prev = a_d * dx + (1.0 - a_d) * self._dx_prev

        # 3. 自适应截止频率 = min_cutoff + beta * speed
        speed = np.linalg.norm(self._dx_prev)
        cutoff = self.min_cutoff + self.beta * speed

        # 4. 低通滤波值
        a = _smoothing_factor(self.rate, cutoff)
        self._x_prev = a * x + (1.0 - a) * self._x_prev

        return self._x_prev.copy()

    def reset(self):
        """重置滤波器状态"""
        self._x_prev = None
        self._dx_prev = None


class OneEuroFilterQuat:
    """
    One-Euro Filter (四元数, 基于 SLERP)

    使用帧间旋转角度作为速度度量, SLERP 代替线性插值。
    自动处理四元数双覆盖 (q 和 -q 表示相同旋转)。

    参数含义同 OneEuroFilter。
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
        """滤波一个四元数 [qx, qy, qz, qw], 返回滤波结果"""
        q = np.asarray(q, dtype=np.float64)

        # 归一化
        qn = np.linalg.norm(q)
        if qn < 1e-10:
            return self._q_prev.copy() if self._q_prev is not None else q
        q = q / qn

        # 双覆盖: 确保与上一帧在同一半球 (最短路径)
        if self._q_prev is not None and np.dot(q, self._q_prev) < 0.0:
            q = -q

        if self._q_prev is None:
            self._q_prev = q.copy()
            self._speed_prev = 0.0
            return q.copy()

        # 1. 计算角速度 (rad/s)
        dot = np.clip(np.dot(q, self._q_prev), -1.0, 1.0)
        angle = 2.0 * np.arccos(abs(dot))
        speed = angle * self.rate

        # 2. 低通滤波速度
        a_d = _smoothing_factor(self.rate, self.d_cutoff)
        self._speed_prev = a_d * speed + (1.0 - a_d) * self._speed_prev

        # 3. 自适应截止频率
        cutoff = self.min_cutoff + self.beta * self._speed_prev

        # 4. SLERP 滤波 (alpha=1→跟踪, alpha=0→保持)
        a = _smoothing_factor(self.rate, cutoff)
        self._q_prev = _slerp(self._q_prev, q, a)

        return self._q_prev.copy()

    def reset(self):
        """重置滤波器状态"""
        self._q_prev = None
        self._speed_prev = 0.0
