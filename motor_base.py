"""
motor_base.py — 电机模型基类 + 工厂函数

支持类型:
  - pmsm:     永磁同步电机 (表贴式 / 内置式)
  - induction: 鼠笼异步电机
  - synrm:     同步磁阻电机
  - pm_synrm:  永磁辅助同步磁阻电机

所有子类暴露统一接口: step(Vd, Vq, dt) → MotorState
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import numpy as np


# ═══════════════════════════════════════
# 故障枚举 + 状态结构体 (所有电机共用)
# ═══════════════════════════════════════

class FaultType(Enum):
    NONE = 0
    OVERCURRENT = 1
    OVERTEMP = 2
    PHASE_LOSS = 3
    DC_UNDERVOLTAGE = 4
    ENCODER_FAULT = 5


@dataclass
class MotorState:
    Id: float = 0.0
    Iq: float = 0.0
    omega_m: float = 0.0   # 机械角速度 [rad/s]
    theta_e: float = 0.0   # 电角度 [rad]
    Vdc: float = 0.0       # 母线电压 [V]
    temp: float = 25.0     # 温度 [°C]
    fault: FaultType = FaultType.NONE
    Te: float = 0.0        # 电磁转矩 [N·m]


# ═══════════════════════════════════════
# 基类
# ═══════════════════════════════════════

@dataclass
class MotorConfig:
    """通用电机配置 (单极感应子, 6对极, 电励磁)"""
    P: int = 6                   # 极对数 (6对极)
    Rs: float = 0.005            # 定子电阻 [Ω] (500kW 级)
    I_max: float = 1000.0        # 最大电流 [A] (500kW@750Vdc)
    Vdc_nom: float = 750.0       # 额定母线电压 [V]
    J: float = 0.04              # 转动惯量 [kg·m²] (电机转子, 远小于飞轮)
    B: float = 0.0005            # 阻尼 [N·m·s]
    T_sample: float = 50e-6      # 控制周期 [s]
    N_max: float = 10000.0       # 最大转速 [rpm]


class MotorBase(ABC):
    """电机模型基类 — 统一接口"""

    def __init__(self, config: MotorConfig, fidelity: str = "level1"):
        self.cfg = config
        self.fidelity = fidelity
        self.state = MotorState()
        self.load_torque = 0.0
        self.temp_ambient = 25.0
        self._cogging_table = None

    # ── 子类必须实现 ──
    def _electrical_dynamics(self, Vd: float, Vq: float) -> tuple:
        """
        电气动态方程的核心 (同步电机用).
        异步电机重写整个 step() 而不需要此方法.
        返回: (dId_dt, dIq_dt)
        """
        raise NotImplementedError(f"{self.type_name()} must implement _electrical_dynamics or override step()")

    @abstractmethod
    def _compute_torque(self) -> float:
        """计算电磁转矩"""
        ...

    @abstractmethod
    def type_name(self) -> str:
        """电机类型名"""
        ...

    @abstractmethod
    def param_summary(self) -> dict:
        """返回可读参数"""
        ...

    # ── 共用实现 ──
    def step(self, Vd: float, Vq: float, dt: Optional[float] = None) -> MotorState:
        dt = dt or self.cfg.T_sample
        s = self.state

        # 1. 电气动态
        dId_dt, dIq_dt = self._electrical_dynamics(Vd, Vq)

        if self.fidelity == "level1":
            s.Id += dId_dt * dt
            s.Iq += dIq_dt * dt
        else:
            s.Id, s.Iq = self._rk4_electrical(dId_dt, dIq_dt, dt, Vd, Vq)

        # 2. 电流限幅
        I_peak = np.sqrt(s.Id**2 + s.Iq**2)
        if I_peak > self.cfg.I_max:
            s.Id *= self.cfg.I_max / I_peak
            s.Iq *= self.cfg.I_max / I_peak

        # 3. 转矩
        s.Te = self._compute_torque()
        if self.fidelity == "level2" and self._cogging_table is not None:
            idx = int(s.theta_e / (2 * np.pi) * 360) % 360
            s.Te += self._cogging_table[idx]

        # 4. 机械运动
        domega_dt = (s.Te - self.load_torque - self.cfg.B * s.omega_m) / self.cfg.J
        s.omega_m += domega_dt * dt
        s.omega_m = max(0, s.omega_m)

        omega_e = self.cfg.P * s.omega_m
        s.theta_e += omega_e * dt
        s.theta_e %= 2 * np.pi

        # 5. 温度 (Level 2)
        if self.fidelity == "level2":
            P_loss = self.cfg.Rs * (s.Id**2 + s.Iq**2)
            dtemp_dt = (P_loss - (s.temp - self.temp_ambient) / 0.05) / 500
            s.temp += dtemp_dt * dt
            self.cfg.Rs *= 1 + 0.004 * (s.temp - 25)

        # 6. 故障检测
        if I_peak >= 1.2 * self.cfg.I_max:
            s.fault = FaultType.OVERCURRENT
        if s.temp > 150:
            s.fault = FaultType.OVERTEMP

        return s

    def _rk4_electrical(self, dId_init, dIq_init, dt, Vd, Vq):
        def derivs(_id, _iq):
            # 临时改 Id/Iq 算导数
            orig_id, orig_iq = self.state.Id, self.state.Iq
            self.state.Id, self.state.Iq = _id, _iq
            d_id, d_iq = self._electrical_dynamics(Vd, Vq)
            self.state.Id, self.state.Iq = orig_id, orig_iq
            return d_id, d_iq

        k1d, k1q = dId_init, dIq_init
        k2d, k2q = derivs(self.state.Id + 0.5*dt*k1d, self.state.Iq + 0.5*dt*k1q)
        k3d, k3q = derivs(self.state.Id + 0.5*dt*k2d, self.state.Iq + 0.5*dt*k2q)
        k4d, k4q = derivs(self.state.Id + dt*k3d, self.state.Iq + dt*k3q)

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
            "torque": round(s.Te, 2),
        }


# ═══════════════════════════════════════
# 1. 永磁同步电机 PMSM
# ═══════════════════════════════════════

@dataclass
class PMSMConfig(MotorConfig):
    Ld: float = 0.008     # d轴电感 [H]
    Lq: float = 0.008     # q轴电感 [H] (表贴式 Ld≈Lq, 内置式 Ld<Lq)
    psi_m: float = 0.07   # 永磁磁链 [Wb]


class PMSM(MotorBase):
    """永磁同步电机"""

    def __init__(self, config: PMSMConfig = None, fidelity: str = "level1"):
        cfg = config or PMSMConfig()
        super().__init__(cfg, fidelity)

    @property
    def Ld(self): return self.cfg.Ld
    @property
    def Lq(self): return self.cfg.Lq
    @property
    def psi_m(self): return self.cfg.psi_m

    def type_name(self): 
        ld_lq = self.Ld / self.Lq if self.Lq > 0 else 0
        if ld_lq > 0.9:
            return "PMSM (表贴式 SPM)"
        elif ld_lq > 0.5:
            return "PMSM (内置式 IPM)"
        return "PMSM"

    def param_summary(self):
        return {"Rs": self.cfg.Rs, "Ld": self.Ld, "Lq": self.Lq,
                "ψm": self.psi_m, "P": self.cfg.P, "J": self.cfg.J}

    def _electrical_dynamics(self, Vd, Vq):
        s = self.state
        omega_e = self.cfg.P * s.omega_m
        dId = (Vd - self.cfg.Rs * s.Id + omega_e * self.Lq * s.Iq) / self.Ld
        dIq = (Vq - self.cfg.Rs * s.Iq - omega_e * (self.Ld * s.Id + self.psi_m)) / self.Lq
        return dId, dIq

    def _compute_torque(self):
        s = self.state
        return 1.5 * self.cfg.P * (self.psi_m * s.Iq + (self.Ld - self.Lq) * s.Id * s.Iq)


# ═══════════════════════════════════════
# 2. 鼠笼异步电机 IM
# ═══════════════════════════════════════

@dataclass
class InductionConfig(MotorConfig):
    Ls: float = 0.03      # 定子电感 [H]
    Lr: float = 0.03      # 转子电感 [H]
    Lm: float = 0.028     # 互感 [H]
    Rr: float = 0.01      # 转子电阻 [Ω] (折算到定子侧)
    # 派生量:
    # σ = 1 - Lm²/(Ls·Lr)  → 漏磁系数
    # τr = Lr/Rr            → 转子时间常数


class InductionMotor(MotorBase):
    """鼠笼异步电机 — 转子磁场定向 (FOC) 模型

    状态变量: Id, Iq, ψr (转子磁链幅值)
    d轴对齐转子磁链: ψdr=ψr, ψqr=0

    磁链动态: dψr/dt = -ψr/τr + Lm·Id/τr
    转差频率: ωslip = (Lm/τr)·Iq/ψr
    同步频率: ωe = P·ωm + ωslip

    定子方程:
      Vd = Rs·Id + σ·Ls·dId/dt - ωe·σ·Ls·Iq + kr·dψr/dt
      Vq = Rs·Iq + σ·Ls·dIq/dt + ωe·σ·Ls·Id + kr·ωe·ψr

    转矩: Te = 1.5·P·kr·ψr·Iq

    其中: σ = 1-Lm²/(Ls·Lr), kr = Lm/Lr, τr = Lr/Rr
    """

    def __init__(self, config: InductionConfig = None, fidelity: str = "level1"):
        cfg = config or InductionConfig()
        super().__init__(cfg, fidelity)
        self._psi_r = 0.001  # 微小初值, 避免除零
        self._omega_e = 0.0  # 同步电气角速度

    @property
    def Ls(self): return self.cfg.Ls
    @property
    def Lr(self): return self.cfg.Lr
    @property
    def Lm(self): return self.cfg.Lm
    @property
    def Rr(self): return self.cfg.Rr

    def type_name(self): return "异步电机 (鼠笼式 IM, FOC模型)"
    
    def param_summary(self):
        return {"Rs": self.cfg.Rs, "Rr": self.Rr, "Ls": self.Ls,
                "Lr": self.Lr, "Lm": self.Lm, "P": self.cfg.P, "J": self.cfg.J}

    def step(self, Vd, Vq, dt=None):
        dt = dt or self.cfg.T_sample
        s = self.state
        omega_r = self.cfg.P * s.omega_m  # 转子电气角速度

        # 派生参数
        sigma = 1 - self.Lm**2 / (self.Ls * self.Lr)
        kr = self.Lm / self.Lr
        tau_r = self.Lr / self.Rr
        Rs_eq = self.cfg.Rs + self.Lm**2 * self.Rr / self.Lr**2

        # 转差 + 同步频率
        psi_safe = max(self._psi_r, 0.001)
        omega_slip = (self.Lm / tau_r) * s.Iq / psi_safe
        self._omega_e = omega_r + omega_slip

        # ── 状态导数 ──
        # 转子磁链
        dpsi_r = -self._psi_r / tau_r + self.Lm * s.Id / tau_r

        # 定子电流
        dId = (Vd - Rs_eq * s.Id + self._omega_e * sigma * self.Ls * s.Iq
               - kr * dpsi_r) / (sigma * self.Ls)
        dIq = (Vq - Rs_eq * s.Iq - self._omega_e * (sigma * self.Ls * s.Id + kr * self._psi_r)
               ) / (sigma * self.Ls)

        if self.fidelity == "level1":
            s.Id += dId * dt
            s.Iq += dIq * dt
            self._psi_r += dpsi_r * dt
        else:
            # RK4 (3状态)
            x0 = np.array([s.Id, s.Iq, self._psi_r])
            def f(x):
                oid, oiq, ops = s.Id, s.Iq, self._psi_r
                s.Id, s.Iq, self._psi_r = x[0], x[1], max(x[2], 0.001)
                di, dj, dp = dId, dIq, dpsi_r
                # 重新计算 (用新状态)
                psi_s = max(self._psi_r, 0.001)
                wsl = (self.Lm / tau_r) * s.Iq / psi_s
                we2 = omega_r + wsl
                dp2 = -self._psi_r / tau_r + self.Lm * s.Id / tau_r
                di2 = (Vd - Rs_eq * s.Id + we2 * sigma * self.Ls * s.Iq - kr * dp2) / (sigma * self.Ls)
                dj2 = (Vq - Rs_eq * s.Iq - we2 * (sigma * self.Ls * s.Id + kr * self._psi_r)) / (sigma * self.Ls)
                s.Id, s.Iq, self._psi_r = oid, oiq, ops
                return np.array([di2, dj2, dp2])
            k1 = f(x0)
            k2 = f(x0 + 0.5*dt*k1)
            k3 = f(x0 + 0.5*dt*k2)
            k4 = f(x0 + dt*k3)
            x_new = x0 + dt/6*(k1 + 2*k2 + 2*k3 + k4)
            s.Id = x_new[0]
            s.Iq = x_new[1]
            self._psi_r = max(x_new[2], 0.001)

        # 电流限幅
        I_peak = np.sqrt(s.Id**2 + s.Iq**2)
        if I_peak > self.cfg.I_max:
            s.Id *= self.cfg.I_max / I_peak
            s.Iq *= self.cfg.I_max / I_peak

        # 转矩
        s.Te = 1.5 * self.cfg.P * kr * self._psi_r * s.Iq

        # 机械
        domega_dt = (s.Te - self.load_torque - self.cfg.B * s.omega_m) / self.cfg.J
        s.omega_m += domega_dt * dt
        s.omega_m = max(0, s.omega_m)
        s.theta_e += self._omega_e * dt
        s.theta_e %= 2 * np.pi

        # 故障
        if I_peak >= 1.2 * self.cfg.I_max:
            s.fault = FaultType.OVERCURRENT

        return s

    def _compute_torque(self):
        kr = self.Lm / self.Lr
        return 1.5 * self.cfg.P * kr * self._psi_r * self.state.Iq

    def get_readable(self) -> dict:
        d = super().get_readable()
        d["psi_r"] = round(self._psi_r, 4)
        d["omega_slip"] = round(self._omega_e - self.cfg.P * self.state.omega_m, 2)
        return d


# ═══════════════════════════════════════
# 3. 同步磁阻电机 SynRM
# ═══════════════════════════════════════

@dataclass
class SynRMConfig(MotorConfig):
    Ld: float = 0.04      # d轴电感 [H] (大, 凸极性显著)
    Lq: float = 0.005     # q轴电感 [H] (小)
    # 无永磁体: ψm = 0
    # 凸极比 Ld/Lq ≈ 5-10


class SynRM(MotorBase):
    """同步磁阻电机 — 无永磁体, 纯磁阻转矩"""

    def __init__(self, config: SynRMConfig = None, fidelity: str = "level1"):
        cfg = config or SynRMConfig()
        super().__init__(cfg, fidelity)

    @property
    def Ld(self): return self.cfg.Ld
    @property
    def Lq(self): return self.cfg.Lq

    def type_name(self): return "同步磁阻电机 (SynRM)"

    def param_summary(self):
        return {"Rs": self.cfg.Rs, "Ld": self.Ld, "Lq": self.Lq,
                "凸极比": f"{self.Ld/self.Lq:.1f}", "P": self.cfg.P, "J": self.cfg.J}

    def _electrical_dynamics(self, Vd, Vq):
        s = self.state
        omega_e = self.cfg.P * s.omega_m
        dId = (Vd - self.cfg.Rs * s.Id + omega_e * self.Lq * s.Iq) / self.Ld
        dIq = (Vq - self.cfg.Rs * s.Iq - omega_e * self.Ld * s.Id) / self.Lq
        return dId, dIq

    def _compute_torque(self):
        s = self.state
        # Te = 1.5*P*(Ld-Lq)*Id*Iq  (无永磁体项)
        return 1.5 * self.cfg.P * (self.Ld - self.Lq) * s.Id * s.Iq


# ═══════════════════════════════════════
# 4. 永磁辅助同步磁阻电机 PM-SynRM
# ═══════════════════════════════════════

@dataclass
class PMSynRMConfig(MotorConfig):
    Ld: float = 0.03      # d轴电感 [H]
    Lq: float = 0.008     # q轴电感 [H]
    psi_m: float = 0.03   # 少量永磁体磁链 [Wb]


class PMSynRM(MotorBase):
    """永磁辅助同步磁阻电机 — SynRM + 少量永磁"""

    def __init__(self, config: PMSynRMConfig = None, fidelity: str = "level1"):
        cfg = config or PMSynRMConfig()
        super().__init__(cfg, fidelity)

    @property
    def Ld(self): return self.cfg.Ld
    @property
    def Lq(self): return self.cfg.Lq
    @property
    def psi_m(self): return self.cfg.psi_m

    def type_name(self): return "永磁辅助同步磁阻电机 (PM-SynRM)"

    def param_summary(self):
        return {"Rs": self.cfg.Rs, "Ld": self.Ld, "Lq": self.Lq,
                "ψm": self.psi_m, "凸极比": f"{self.Ld/self.Lq:.1f}",
                "P": self.cfg.P, "J": self.cfg.J}

    def _electrical_dynamics(self, Vd, Vq):
        s = self.state
        omega_e = self.cfg.P * s.omega_m
        dId = (Vd - self.cfg.Rs * s.Id + omega_e * self.Lq * s.Iq) / self.Ld
        dIq = (Vq - self.cfg.Rs * s.Iq - omega_e * (self.Ld * s.Id + self.psi_m)) / self.Lq
        return dId, dIq

    def _compute_torque(self):
        s = self.state
        return 1.5 * self.cfg.P * (self.psi_m * s.Iq + (self.Ld - self.Lq) * s.Id * s.Iq)


# ═══════════════════════════════════════
# 工厂函数
# ═══════════════════════════════════════

def create_motor(motor_type: str, fidelity: str = "level1", **kwargs) -> MotorBase:
    """
    工厂函数: 根据类型名创建电机

    示例:
        pmsm = create_motor("pmsm", Rs=0.015, Ld=0.008, Lq=0.008, psi_m=0.07)
        im   = create_motor("induction", Rs=0.5, Rr=0.3, Ls=0.05, Lr=0.05, Lm=0.048)
        syn  = create_motor("synrm", Rs=0.1, Ld=0.04, Lq=0.005)
        pm_syn = create_motor("pm_synrm", Rs=0.08, Ld=0.03, Lq=0.008, psi_m=0.03)
    """
    mapping = {
        "pmsm":      (PMSMConfig, PMSM),
        "induction": (InductionConfig, InductionMotor),
        "synrm":     (SynRMConfig, SynRM),
        "pm_synrm":  (PMSynRMConfig, PMSynRM),
    }

    if motor_type not in mapping:
        raise ValueError(f"未知电机类型: {motor_type}. 支持: {list(mapping.keys())}")

    config_cls, motor_cls = mapping[motor_type]
    valid_params = set(config_cls.__dataclass_fields__.keys())
    filtered = {k: v for k, v in kwargs.items() if k in valid_params}
    config = config_cls(**filtered)
    return motor_cls(config, fidelity=fidelity)


def list_motor_types() -> list:
    return [
        {"id": "pmsm",      "name": "永磁同步电机",        "params": ["Rs", "Ld", "Lq", "psi_m", "P", "J"]},
        {"id": "induction", "name": "鼠笼异步电机",        "params": ["Rs", "Rr", "Ls", "Lr", "Lm", "P", "J"]},
        {"id": "synrm",     "name": "同步磁阻电机",        "params": ["Rs", "Ld", "Lq", "P", "J"]},
        {"id": "pm_synrm",  "name": "永磁辅助同步磁阻电机",  "params": ["Rs", "Ld", "Lq", "psi_m", "P", "J"]},
    ]
