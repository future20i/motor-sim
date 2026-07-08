"""
control_algorithms.py — 3种电流环: PI (前馈解耦) / Deadbeat (预测控制) / SlidingMode (滑模控制)。统一接口: compute(Id_ref, Iq_ref, state) → (Vd, Vq)。

子系统: 控制层
依赖: motor_base.py (MotorModelParams, CurrentState)
手册对应章节: CONTROL_SETPOINTS.md §4 (电机控制参数)

3种电流环: PI (前馈解耦) / Deadbeat (预测控制) / SlidingMode (滑模控制)。统一接口: compute(Id_ref, Iq_ref, state) → (Vd, Vq)。
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np
import math


# ═══════════════════════
# 电机参数接口 (控制器需要知道 Ld, Lq, Rs, ψm)
# ═══════════════════════

@dataclass
class MotorModelParams:
    """控制器视角下的电机参数 (单极感应子, P=6, 电励磁)

    电励磁等效: ψm ≈ Lmd·If_nom/P, 其中 If 从 DC 母线取电
    500kW 电机估算: Rs≈5mΩ, Ld≈Lq≈2mH
    """
    Rs: float = 0.005
    Ld: float = 0.002
    Lq: float = 0.002
    psi_m: float = 0.08    # 等效磁链 Wb (电励磁 If=10A 时)
    P: int = 6
    Ts: float = 50e-6


@dataclass
class CurrentState:
    """当前状态快照"""
    Id: float = 0.0
    Iq: float = 0.0
    omega_m: float = 0.0  # 机械角速度 [rad/s]
    Vdc: float = 600.0
    Vd_prev: float = 0.0  # 上一周期输出电压
    Vq_prev: float = 0.0


# ═══════════════════════
# 基类
# ═══════════════════════

class ControllerBase(ABC):
    """电流控制器基类"""

    def __init__(self, params: MotorModelParams):
        self.p = params
        self.reset()

    def reset(self):
        """清空内部状态 (切换算法时调用)"""
        pass

    @abstractmethod
    def compute(self, Id_ref: float, Iq_ref: float,
                state: CurrentState, dt: float = None) -> Tuple[float, float]:
        """
        计算输出电压

        返回: (Vd_out, Vq_out) [V]
        """
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        """算法名"""
        ...

    def voltage_limit(self, Vd: float, Vq: float, Vdc: float) -> Tuple[float, float]:
        """电压限幅 (六边形→圆形近似)"""
        V_max = Vdc / math.sqrt(3) * 0.95  # 留 5% 裕量
        V_mag = math.sqrt(Vd**2 + Vq**2)
        if V_mag > V_max:
            scale = V_max / V_mag
            Vd *= scale
            Vq *= scale
        return Vd, Vq

    def current_limit(self, Id: float, Iq: float, I_max: float) -> Tuple[float, float]:
        """电流限幅"""
        I_mag = math.sqrt(Id**2 + Iq**2)
        if I_mag > I_max:
            scale = I_max / I_mag
            Id *= scale
            Iq *= scale
        return Id, Iq


# ═══════════════════════
# 1. PI 控制器 (基线)
# ═══════════════════════

class PIController(ControllerBase):
    """PI 电流控制 + 前馈解耦"""

    def __init__(self, params: MotorModelParams, bandwidth: float = 1000):
        super().__init__(params)
        self.bw = bandwidth
        self.Kp_d = params.Ld * bandwidth
        self.Ki_d = params.Rs * bandwidth
        self.Kp_q = params.Lq * bandwidth
        self.Ki_q = params.Rs * bandwidth
        self._integral_d = 0.0
        self._integral_q = 0.0

    @property
    def name(self): return f"PI (bw={self.bw}rad/s)"

    def reset(self):
        self._integral_d = 0.0
        self._integral_q = 0.0

    def compute(self, Id_ref, Iq_ref, state, dt=None):
        dt = dt or self.p.Ts
        omega_e = self.p.P * state.omega_m

        # 误差
        d_err = Id_ref - state.Id
        q_err = Iq_ref - state.Iq

        # 积分
        self._integral_d += d_err * dt
        self._integral_q += q_err * dt
        self._integral_d = np.clip(self._integral_d, -100, 100)
        self._integral_q = np.clip(self._integral_q, -100, 100)

        # PI 输出
        Vd_pi = self.Kp_d * d_err + self.Ki_d * self._integral_d
        Vq_pi = self.Kp_q * q_err + self.Ki_q * self._integral_q

        # 前馈解耦
        Vd_ff = -omega_e * self.p.Lq * state.Iq
        Vq_ff = omega_e * (self.p.Ld * state.Id + self.p.psi_m)

        Vd = Vd_pi + Vd_ff
        Vq = Vq_pi + Vq_ff

        return self.voltage_limit(Vd, Vq, state.Vdc)


# ═══════════════════════
# 2. 无差拍预测控制器
# ═══════════════════════

class DeadbeatController(ControllerBase):
    """
    无差拍预测控制

    原理: 用离散化电机方程直接算出下一周期需要的 Vd/Vq，
         使 Id/Iq 在一个控制周期内到达参考值。

    Vd(k) = Ld/Ts·(Id_ref - Id(k)) + Rs·Id(k) - ω·Lq·Iq(k)
    Vq(k) = Lq/Ts·(Iq_ref - Iq(k)) + Rs·Iq(k) + ω·(Ld·Id(k) + ψm)

    优点: 动态响应极快 (一个周期到位)
    缺点: 对参数敏感，噪声放大
    """

    def __init__(self, params: MotorModelParams, alpha: float = 0.5):
        """
        alpha: 柔化因子 (0=保守仅用当前值, 1=激进全量参考)
               Id_ref_s = alpha*Id_ref + (1-alpha)*Id_actual
        """
        super().__init__(params)
        self.alpha = alpha  # 0.5 = 折中

    @property
    def name(self): return f"Deadbeat (α={self.alpha})"

    def compute(self, Id_ref, Iq_ref, state, dt=None):
        dt = dt or self.p.Ts
        omega_e = self.p.P * state.omega_m

        # 柔化后的参考值 (避免阶跃)
        Id_ref_s = self.alpha * Id_ref + (1 - self.alpha) * state.Id
        Iq_ref_s = self.alpha * Iq_ref + (1 - self.alpha) * state.Iq

        # 无差拍方程
        Vd = (self.p.Ld / dt) * (Id_ref_s - state.Id) \
           + self.p.Rs * state.Id \
           - omega_e * self.p.Lq * state.Iq

        Vq = (self.p.Lq / dt) * (Iq_ref_s - state.Iq) \
           + self.p.Rs * state.Iq \
           + omega_e * (self.p.Ld * state.Id + self.p.psi_m)

        return self.voltage_limit(Vd, Vq, state.Vdc)


# ═══════════════════════
# 3. 滑模控制器
# ═══════════════════════

class SlidingModeController(ControllerBase):
    """
    滑模电流控制

    滑模面: s_d = Id_ref - Id,  s_q = Iq_ref - Iq
    控制律: Vd = Kp_d·sgn(s_d) + 前馈
            Vq = Kp_q·sgn(s_q) + 前馈

    优点: 对参数不敏感，鲁棒性强
    缺点: 有抖振 (chattering)，需要用饱和函数替代 sgn
    """

    def __init__(self, params: MotorModelParams,
                 Kd: float = 50.0, Kq: float = 50.0,
                 boundary: float = 0.5):
        """
        Kd, Kq: 滑模增益 (越大=越激进=抖振越大)
        boundary: 边界层厚度 (越大=抖振越小=稳态误差越大)
        """
        super().__init__(params)
        self.Kd = Kd
        self.Kq = Kq
        self.boundary = boundary

    @property
    def name(self): return f"SlidingMode (Kd={self.Kd},Kq={self.Kq},φ={self.boundary})"

    def _sat(self, err: float) -> float:
        """饱和函数 (替代 sign 减少抖振)"""
        if abs(err) < self.boundary:
            return err / self.boundary  # 边界层内: 线性
        return 1.0 if err > 0 else -1.0

    def compute(self, Id_ref, Iq_ref, state, dt=None):
        omega_e = self.p.P * state.omega_m

        # 滑模面
        s_d = Id_ref - state.Id
        s_q = Iq_ref - state.Iq

        # 滑模控制律
        Vd_sm = self.Kd * self._sat(s_d)
        Vq_sm = self.Kq * self._sat(s_q)

        # 前馈解耦
        Vd_ff = -omega_e * self.p.Lq * state.Iq
        Vq_ff = omega_e * (self.p.Ld * state.Id + self.p.psi_m)

        Vd = Vd_sm + Vd_ff
        Vq = Vq_sm + Vq_ff

        return self.voltage_limit(Vd, Vq, state.Vdc)


# ═══════════════════════
# 工厂
# ═══════════════════════

def create_controller(algo: str, params: MotorModelParams, **kwargs) -> ControllerBase:
    mapping = {
        "pi":        PIController,
        "deadbeat":  DeadbeatController,
        "sliding":   SlidingModeController,
    }
    if algo not in mapping:
        raise ValueError(f"未知算法: {algo}. 支持: {list(mapping.keys())}")
    return mapping[algo](params, **kwargs)


def list_algorithms() -> list:
    return [
        {"id": "pi",       "name": "PI 控制",       "params": "bandwidth (1000 rad/s)"},
        {"id": "deadbeat", "name": "无差拍预测控制",  "params": "alpha 柔化因子 (0.5)"},
        {"id": "sliding",  "name": "滑模控制",       "params": "Kd,Kq 增益 + boundary 边界层"},
    ]
