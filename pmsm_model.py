"""
pmsm_model.py — PMSM 电机仿真模型

支持 Level 1 (理想模型) 和 Level 2 (物理模型含齿槽转矩/温度/饱和)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from enum import Enum


class FaultType(Enum):
    NONE = 0
    PHASE_LOSS = 1
    OVERCURRENT = 2
    OVERTEMP = 3
    ENCODER_FAULT = 4
    DC_UNDERVOLTAGE = 5


@dataclass
class MotorParams:
    """PMSM 参数 — 单极感应子飞轮电机, P=6, 电励磁"""
    Rs: float = 0.005        # 定子电阻 [Ω] (500kW 级)
    Ld: float = 0.002        # d轴电感 [H] (高速电机, 低电感)
    Lq: float = 0.002        # q轴电感 [H]
    psi_m: float = 0.08      # 等效永磁磁链 [Wb] (电励磁 If=10A 等效)
    P: int = 6               # 极对数 (6对极)
    J: float = 0.04          # 转动惯量 [kg·m²]
    B: float = 0.0005        # 阻尼系数 [N·m·s]
    I_max: float = 100.0     # 最大允许电流 [A]
    Vdc_nom: float = 600.0   # 额定母线电压 [V]
    Vdc_max: float = 750.0   # 最大母线电压 [V]
    N_max: float = 10000.0   # 最大转速 [rpm]
    T_sample: float = 50e-6  # 控制周期 [s] = 50μs


@dataclass
class MotorState:
    """电机瞬时状态 — 对应 OPC UA 可读变量"""
    Id: float = 0.0
    Iq: float = 0.0
    omega_m: float = 0.0   # 机械角速度 [rad/s]
    theta_e: float = 0.0   # 电角度 [rad]
    Vdc: float = 0.0       # 母线电压 [V]
    temp: float = 25.0     # 模块温度 [°C]
    fault: FaultType = FaultType.NONE


class PMSMotor:
    """PMSM 仿真电机"""

    def __init__(self, params: MotorParams = None, fidelity: str = "level1"):
        self.p = params or MotorParams()
        self.fidelity = fidelity
        self.state = MotorState()
        self.Te = 0.0
        self.load_torque = 0.0

        self.temp_ambient = 25.0
        self._cogging_table = None

    # ── Level 2 初始化 ──
    def _init_level2(self):
        n_cog = 36
        angles = np.linspace(0, 2 * np.pi, 360)
        self._cogging_table = 0.02 * np.sin(n_cog * angles)

    # ── 单步仿真 ──
    def step(self, Vd: float, Vq: float, dt: Optional[float] = None) -> MotorState:
        dt = dt or self.p.T_sample
        s = self.state
        omega_e = self.p.P * s.omega_m

        # dq 轴微分方程
        dId_dt = (Vd - self.p.Rs * s.Id + omega_e * self.p.Lq * s.Iq) / self.p.Ld
        dIq_dt = (Vq - self.p.Rs * s.Iq - omega_e * (self.p.Ld * s.Id + self.p.psi_m)) / self.p.Lq

        if self.fidelity == "level1":
            s.Id += dId_dt * dt
            s.Iq += dIq_dt * dt
        else:
            s.Id, s.Iq = self._rk4_step(dId_dt, dIq_dt, dt, Vd, Vq)

        # 电流限幅
        I_peak = np.sqrt(s.Id**2 + s.Iq**2)
        if I_peak > self.p.I_max:
            s.Id *= self.p.I_max / I_peak
            s.Iq *= self.p.I_max / I_peak

        # 电磁转矩
        self.Te = 1.5 * self.p.P * (self.p.psi_m * s.Iq + (self.p.Ld - self.p.Lq) * s.Id * s.Iq)

        # 齿槽转矩 (Level 2)
        if self.fidelity == "level2" and self._cogging_table is not None:
            idx = int(s.theta_e / (2 * np.pi) * 360) % 360
            self.Te += self._cogging_table[idx]

        # 机械运动
        domega_dt = (self.Te - self.load_torque - self.p.B * s.omega_m) / self.p.J
        s.omega_m += domega_dt * dt
        s.omega_m = max(0, s.omega_m)

        s.theta_e += omega_e * dt
        s.theta_e %= 2 * np.pi

        # 温度 (Level 2)
        if self.fidelity == "level2":
            P_loss = self.p.Rs * (s.Id**2 + s.Iq**2)
            dtemp_dt = (P_loss - (s.temp - self.temp_ambient) / 0.05) / 500
            s.temp += dtemp_dt * dt
            self.p.Rs = self.p.Rs * (1 + 0.004 * (s.temp - 25))

        # 故障检测
        if I_peak >= 1.2 * self.p.I_max:
            s.fault = FaultType.OVERCURRENT
        if s.temp > 150:
            s.fault = FaultType.OVERTEMP

        return s

    def _rk4_step(self, dId_init, dIq_init, dt, Vd, Vq):
        def derivatives(_id, _iq):
            oe = self.p.P * self.state.omega_m
            dd_id = (Vd - self.p.Rs * _id + oe * self.p.Lq * _iq) / self.p.Ld
            dd_iq = (Vq - self.p.Rs * _iq - oe * (self.p.Ld * _id + self.p.psi_m)) / self.p.Lq
            return dd_id, dd_iq

        k1d, k1q = dId_init, dIq_init
        k2d, k2q = derivatives(self.state.Id + 0.5*dt*k1d, self.state.Iq + 0.5*dt*k1q)
        k3d, k3q = derivatives(self.state.Id + 0.5*dt*k2d, self.state.Iq + 0.5*dt*k2q)
        k4d, k4q = derivatives(self.state.Id + dt*k3d, self.state.Iq + dt*k3q)

        new_Id = self.state.Id + dt/6 * (k1d + 2*k2d + 2*k3d + k4d)
        new_Iq = self.state.Iq + dt/6 * (k1q + 2*k2q + 2*k3q + k4q)
        return new_Id, new_Iq

    def inject_fault(self, fault: FaultType):
        self.state.fault = fault

    def get_readable(self) -> dict:
        s = self.state
        return {
            "Id_actual": round(s.Id, 3),
            "Iq_actual": round(s.Iq, 3),
            "Vdc_actual": round(s.Vdc, 2),
            "speed_actual": round(s.omega_m * 60 / (2 * np.pi), 1),
            "temp_module": round(s.temp, 1),
            "temp_motor": round(s.temp - 5, 1),
            "fault_code": s.fault.value,
        }
