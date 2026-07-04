"""
flywheel_energy.py — 飞轮储能系统模型
E = ½Jω²  ·  SOC  ·  功率流  ·  损耗  ·  热模型
泓慧能源 1.2MW/3kWh 飞轮UPS参考参数
"""
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class FlywheelConfig:
    """飞轮参数 (泓慧 500kW/3kWh 单极感应子飞轮, P=6对极, 电励磁)

    J = 2E/ω² = 2×10.8MJ/(209.4rad/s)² ≈ 492 kg·m²
    """
    J: float = 492.0           # 转动惯量 kg·m²
    omega_max: float = 2000.0  # 最大转速 rpm
    omega_min: float = 800.0   # 最小转速 rpm (放电截止, ~16% SOC)
    omega_nom: float = 1800.0  # 额定转速 rpm
    P_rated: float = 500e3     # 额定功率 W (500kW)
    Vdc_nom: float = 750.0     # 直流母线标称电压 V (750VDC)
    Vac_nom: float = 480.0     # 交流侧线电压 Vrms (480VAC)
    Poles: int = 6             # 极对数 (6对极, 单极感应子)

    # ── 电励磁参数 (励磁从 DC 母线取电) ──
    Rf: float = 75.0           # 励磁绕组电阻 Ω (750V/10A≈75Ω)
    If_nom: float = 10.0       # 额定励磁电流 A
    Lf: float = 5.0            # 励磁绕组电感 H
    # 励磁产生的等效磁链: ψf = Lf·If_nom ≈ 5×10 = 50 Wb (需除以极对数归一化)
    # 实际 d 轴磁链 ψd ≈ Lmd·If / P ≈ 0.1×10/6 ≈ 0.167 Wb

    # 损耗系数
    k_windage: float = 0.001   # 风阻系数 Nm·(s/rad)² (T_loss = k_windage·ω²)
    k_bearing: float = 0.5     # 轴承摩擦系数 Nm
    P_fixed: float = 50.0      # 固定损耗 W (控制电源/真空泵)
    R_thermal: float = 0.5     # 热阻 °C/W
    C_thermal: float = 5000.0  # 热容 J/°C
    T_ambient: float = 25.0    # 环境温度 °C
    T_max: float = 120.0       # 最高温度 °C

    @property
    def omega_max_rad(self) -> float:
        return self.omega_max * 2 * math.pi / 60

    @property
    def omega_min_rad(self) -> float:
        return self.omega_min * 2 * math.pi / 60

    @property
    def omega_nom_rad(self) -> float:
        return self.omega_nom * 2 * math.pi / 60

    @property
    def E_max_joules(self) -> float:
        """最大储能 [J]"""
        return 0.5 * self.J * self.omega_max_rad ** 2


@dataclass
class FlywheelState:
    """飞轮实时状态"""
    omega: float = 0.0         # 机械角速度 rad/s
    theta: float = 0.0         # 机械角度 rad
    Te: float = 0.0            # 电磁转矩 Nm (正=电动/充电, 负=发电/放电)
    P_mech: float = 0.0        # 机械功率 W (Te * omega)
    P_loss: float = 0.0        # 损耗功率 W
    SOC: float = 0.0           # 荷电状态 0-1
    E_stored: float = 0.0      # 当前储能 J
    temp: float = 25.0         # 温度 °C
    mode: str = "idle"         # idle / charging / discharging / floating
    If: float = 0.0            # 励磁电流 A (电励磁)
    Vdc: float = 750.0         # 直流母线电压 V


class FlywheelEnergyStorage:
    """飞轮储能系统 — 物理模型"""

    def __init__(self, config: FlywheelConfig = None):
        self.cfg = config or FlywheelConfig()
        self.state = FlywheelState()
        self._Ts = 50e-6  # 仿真步长 50μs

    @property
    def rpm(self) -> float:
        return self.state.omega * 60 / (2 * math.pi)

    def step(self, P_grid: float, Vdc: float) -> FlywheelState:
        """
        一个仿真步进
        P_grid: 电网侧需求功率 (正=吸收, 负=释放)
        Vdc: 直流母线电压

        电机侧功率 = P_grid - P_losses
        转矩 = P_motor / omega (限幅)
        """
        cfg = self.cfg
        s = self.state
        Ts = self._Ts

        # 1. 损耗计算 (含励磁损耗: P_exc = If²·Rf)
        s.If = cfg.If_nom  # 励磁建立 (简化: 直接到额定)
        P_excitation = s.If ** 2 * cfg.Rf  # 励磁铜耗
        s.P_loss = (cfg.k_windage * s.omega ** 2 +
                     cfg.k_bearing * abs(s.omega) +
                     cfg.P_fixed +
                     P_excitation)  # 励磁从 DC 母线取电

        # 2. 充放电限幅 (带 Vdc 降额: Vdc<80%时功率降至50%)
        vdc_factor = 1.0 if Vdc > 0.8 * cfg.Vdc_nom else max(0.3, Vdc / (0.8 * cfg.Vdc_nom) * 0.5)
        P_max_charge = cfg.P_rated * vdc_factor
        P_max_discharge = cfg.P_rated * vdc_factor

        # 转速边界保护: 超速只能放电, 欠速只能充电
        if s.omega >= cfg.omega_max_rad:
            P_grid = min(P_grid, 0)
        elif s.omega <= cfg.omega_min_rad:
            P_grid = max(P_grid, 0)

        P_grid = max(-P_max_discharge, min(P_max_charge, P_grid))
        # P_grid: 正值=从电网取电(充电), P_motor = P_grid - P_losses
        # 电机电磁功率 = P_grid - 机电损耗(含轴承/风阻)
        P_motor = P_grid - s.P_loss

        # 3. 动力学: J·dω/dt = Te (电磁转矩, 正=加速/充电)
        if abs(s.omega) > 0.1:
            T_motor = P_motor / s.omega
        else:
            # 低速: 使用平滑分母避免奇点
            omega_sign = 1.0 if s.omega >= 0 else -1.0
            T_motor = P_motor / (0.1 * omega_sign)

        T_max = cfg.P_rated / max(abs(s.omega), 0.1)
        T_motor = max(-T_max, min(T_max, T_motor))
        s.Te = T_motor

        # 4. 机械积分: J·dω/dt = Te
        domega = T_motor / cfg.J
        s.omega += domega * Ts
        # 下限: 不允许负转速 (飞轮不反转)
        s.omega = max(0.0, min(cfg.omega_max_rad, s.omega))
        s.theta += s.omega * Ts
        s.P_mech = T_motor * s.omega

        # 5. 能量 & SOC
        s.E_stored = 0.5 * cfg.J * s.omega ** 2
        E_max = cfg.E_max_joules
        E_min = 0.5 * cfg.J * cfg.omega_min_rad ** 2
        if E_max > E_min:
            s.SOC = max(0.0, min(1.0, (s.E_stored - E_min) / (E_max - E_min)))
        else:
            s.SOC = 0.5

        # 6. 热模型: 铜耗基于电磁功率, 不是净输出
        P_copper = abs(T_motor * s.omega) * 0.02  # 2% 铜耗 (基于电磁功率)
        P_heat = abs(s.P_loss) + P_copper
        dT = (P_heat - (s.temp - cfg.T_ambient) / cfg.R_thermal) / cfg.C_thermal
        s.temp += dT * Ts

        # 7. 模式判定 (不覆盖转速保护状态)
        if s.omega >= cfg.omega_max_rad or s.omega <= cfg.omega_min_rad:
            pass  # 保持边界状态, 不覆盖
        elif abs(P_motor) < 100:
            s.mode = "idle"
        elif P_motor > 0:
            s.mode = "charging"
        else:
            s.mode = "discharging"

        # 存储 Vdc (由逆变器决定, 这里记录)
        s.Vdc = Vdc

        return s

    def get_status(self) -> dict:
        s = self.state
        return {
            "omega_rpm": self.rpm,
            "omega_rad": s.omega,
            "SOC": s.SOC,
            "E_stored_kWh": s.E_stored / 3.6e6,
            "E_max_kWh": self.cfg.E_max_joules / 3.6e6,
            "P_mech_kW": s.P_mech / 1000,
            "P_loss_kW": s.P_loss / 1000,
            "Te_Nm": s.Te,
            "temp": s.temp,
            "mode": s.mode,
            "If_A": s.If,
            "Vdc": s.Vdc,
        }
