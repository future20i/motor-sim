"""
dc_bus.py — 直流母线动态模型
电容充放电 · 负载扰动 · 整流/逆变功率流 · 纹波
"""
import math
from dataclasses import dataclass


@dataclass
class DCBusConfig:
    """直流母线参数 (750VDC 系统)"""
    C: float = 0.2            # 母线电容 F (200mF, 500kW 级)
    V_nom: float = 750.0      # 标称电压 V
    V_min: float = 550.0      # 最低工作电压 V
    V_max: float = 900.0      # 最高耐压 V
    R_esr: float = 0.01       # 等效串联电阻 Ω
    P_rect_max: float = 750e3  # 整流器最大功率 W
    P_inv_max: float = 750e3   # 逆变器最大功率 W


@dataclass
class DCBusState:
    """母线实时状态"""
    Vdc: float = 0.0         # 母线电压 V
    Ic: float = 0.0          # 电容电流 A
    P_rect: float = 0.0      # 整流功率 W (电网→母线)
    P_inv: float = 0.0       # 逆变功率 W (母线→电机)
    P_load: float = 0.0      # 负载功率 W (母线→负载)
    P_balance: float = 0.0   # 功率平衡 W (正=充电)
    V_ripple: float = 0.0    # 纹波电压 V


class DCBus:
    """直流母线 — 电容动力学仿真

    C·dVdc/dt = Ic
    Ic = (P_rect - P_inv - P_load - I²R_esr) / Vdc
    """

    def __init__(self, config: DCBusConfig = None):
        self.cfg = config or DCBusConfig()
        self.state = DCBusState(Vdc=self.cfg.V_nom)
        self._Ts = 50e-6

    def step(self, P_rect: float, P_flywheel: float, P_load: float) -> DCBusState:
        """
        一个仿真步进
        P_rect:   整流器功率 W (电网→母线, 正=充电)
        P_flywheel: 飞轮功率 W (母线→飞轮, 正=充电飞轮)
        P_load:   负载功率 W (母线→负载, 正=放电)
        """
        cfg = self.cfg
        s = self.state
        Ts = self._Ts

        # 功率限幅
        P_rect = max(0, min(cfg.P_rect_max, P_rect))
        P_flywheel_abs = abs(P_flywheel)
        P_load = max(0, min(cfg.P_rect_max * 2, P_load))

        # 功率平衡: P_net = P_rect - P_flywheel_charge - P_load
        # P_rect=+ → 给母线充电
        # P_flywheel=+ → 母线→飞轮 (母线放电)
        # P_load=+ → 母线→负载 (母线放电)
        P_net = P_rect - P_flywheel - P_load

        # 电容电流: Ic = P_net / Vdc (忽略 ESR 损耗简化)
        if abs(s.Vdc) > 1.0:
            Ic = P_net / s.Vdc
        else:
            Ic = 0.0

        # 电容电压积分: dVdc = Ic/C * dt
        dV = Ic / cfg.C * Ts
        s.Vdc += dV
        s.Vdc = max(cfg.V_min * 0.5, min(cfg.V_max * 1.1, s.Vdc))

        s.Ic = Ic
        s.P_rect = P_rect
        s.P_inv = P_flywheel  # 飞轮侧
        s.P_load = P_load
        s.P_balance = P_net

        # 纹波估算 (6脉波整流典型)
        s.V_ripple = abs(Ic) * cfg.R_esr + abs(s.Vdc - cfg.V_nom) * 0.01

        return s

    def set_load(self, P_w: float):
        """设置负载功率 W"""
        self.state.P_load = max(0, P_w)

    def get_status(self) -> dict:
        s = self.state
        return {
            "Vdc": s.Vdc,
            "Vdc_pu": s.Vdc / self.cfg.V_nom,
            "Ic": s.Ic,
            "P_rect_kW": s.P_rect / 1000,
            "P_flywheel_kW": s.P_inv / 1000,
            "P_load_kW": s.P_load / 1000,
            "P_balance_kW": s.P_balance / 1000,
            "V_ripple": s.V_ripple,
            "stable": abs(s.Vdc - self.cfg.V_nom) / self.cfg.V_nom < 0.02,
        }
