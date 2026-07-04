"""
flywheel_energy.py — 飞轮储能系统模型
E = ½Jω²  ·  SOC  ·  功率流  ·  损耗  ·  热模型
泓慧能源 500kW/3kWh 飞轮UPS — 30Cr42MoV钢转子, 8800rpm@33.9kg·m²
"""
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class FlywheelConfig:
    """飞轮参数 (泓慧 500kW/3kWh 单极感应子飞轮, P=6对极, 电励磁)

    转子: 30Cr42MoV 高强度合金钢, σ_y≈950MPa
    几何: D=760mm(R=380mm), L=260mm, 轮毂+辐条结构
    储能: 3kWh 可用 (8800→4400rpm)
    J = 2×10.8MJ/(ω_max² - ω_min²) = 33.9 kg·m²
    """
    # ── 物理规格 ──
    J: float = 33.9            # 转动惯量 kg·m² (能量反推)
    mass_rotor: float = 855.0  # 转子质量 kg (估算)
    R_rotor: float = 0.380     # 转子外径 m (760mm)
    L_rotor: float = 0.260     # 转子厚度 m (260mm)
    material: str = "30Cr42MoV"
    sigma_y: float = 950e6     # 芯部屈服强度 Pa

    # ── 转速 ──
    omega_max: float = 8800.0  # 工作最高转速 rpm
    omega_min: float = 4400.0  # 工作最低转速 rpm (放电截止)
    omega_nom: float = 8800.0  # 额定转速 rpm
    omega_overspeed: float = 9900.0  # 超速保护转速 rpm (112.5%)

    # ── 功率 ──
    P_rated: float = 500e3     # 额定功率 W (500kW)
    P_peak: float = 800e3      # 短时过载功率 W (800kW)

    # ── 电气接口 ──
    Vdc_nom: float = 750.0     # 直流母线标称电压 V
    Vac_nom: float = 480.0     # 交流侧线电压 Vrms
    Poles: int = 6             # 极对数 (单极感应子, 12极定子/6齿转子)

    # ── 电励磁参数 ──
    Rf: float = 75.0           # 励磁绕组电阻 Ω
    If_nom: float = 10.0       # 额定励磁电流 A
    Lf: float = 5.0            # 励磁绕组电感 H
    # 励磁功率 P_exc = If²·Rf = 7.5kW (DC 母线取电, 恒定开销)

    # ── 损耗系数 (⚠️ 需台架数据标定) ──
    k_windage: float = 0.0     # 风阻系数 (真空腔, 近似为零)
    k_bearing: float = 0.5     # 轴承摩擦系数 Nm (需标定)
    P_fixed: float = 200.0     # 固定损耗 W (控制电源/真空泵/传感器)
    R_thermal: float = 0.5     # 热阻 °C/W
    C_thermal: float = 5000.0  # 热容 J/°C
    T_ambient: float = 25.0    # 环境温度 °C
    T_max: float = 120.0       # 最高温度 °C

    @property
    def omega_max_rad(self) -> float:
        return self.omega_max * 2 * math.pi / 60

    @property
    def omega_overspeed_rad(self) -> float:
        return self.omega_overspeed * 2 * math.pi / 60

    @property
    def omega_min_rad(self) -> float:
        return self.omega_min * 2 * math.pi / 60

    @property
    def omega_nom_rad(self) -> float:
        return self.omega_nom * 2 * math.pi / 60

    @property
    def E_max_joules(self) -> float:
        """最大储能 [J] — 工作最高转速, 不包含超速余量"""
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
        P_max_charge = cfg.P_peak * vdc_factor   # 峰值功率用于瞬时限幅
        P_max_discharge = cfg.P_peak * vdc_factor

        # 转速边界保护: 超速只能放电, 欠速只能充电
        if s.omega >= cfg.omega_overspeed_rad:
            P_grid = min(P_grid, 0)  # 超速: 强制放电
        elif s.omega >= cfg.omega_max_rad:
            P_grid = min(P_grid, 0)  # 达到工作上限: 禁止充电
        elif s.omega <= cfg.omega_min_rad:
            P_grid = max(P_grid, 0)  # 欠速: 强制充电

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
        if s.omega >= cfg.omega_overspeed_rad or s.omega <= cfg.omega_min_rad:
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
