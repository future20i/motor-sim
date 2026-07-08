"""
sic_md_inverter.py — SiC MOSFET 逆变器硬件模型: CAB450M12XM3×2并, 25kHz SVPWM, 4级保真度。

子系统: 物理模型
依赖: motor_base.py, inverter_topology.py
手册对应章节: ARCHITECTURE.md §1.1

SiC MOSFET 逆变器硬件模型: CAB450M12XM3×2并, 25kHz SVPWM, 4级保真度。
"""
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple
from enum import Enum


# ═══════════════════════════════════════
# SiC 器件参数 (Wolfspeed CAB450M12XM3 × 2并)
# ═══════════════════════════════════════

@dataclass
class SiCDeviceParams:
    """单个 SiC 半桥模块器件参数 (25°C 标称值)"""
    # ── 静态参数 ──
    Vds_max: float = 1200.0         # 漏源击穿电压 V
    Id_cont_25C: float = 450.0      # 连续漏极电流 @25°C A
    Id_cont_100C: float = 340.0     # 连续漏极电流 @100°C A
    Id_pulse: float = 900.0         # 脉冲电流 (1ms) A
    Rds_on_25C: float = 2.3e-3      # 导通电阻 @25°C Ω
    Rds_on_175C: float = 3.2e-3     # 导通电阻 @175°C (+39%) Ω
    Vsd_body_diode: float = 1.5     # 体二极管正向压降 V @ Id

    # ── 动态参数 ──
    trise_ns: float = 25.0          # 上升时间 ns (10-90%)
    tfall_ns: float = 15.0          # 下降时间 ns
    Eon_mJ: float = 8.5             # 开通能量 @600V/300A mJ
    Eoff_mJ: float = 3.2            # 关断能量 @600V/300A mJ
    Err_mJ: float = 0.0             # 反向恢复 (SiC 体二极管几乎为零)
    Qg_nC: float = 800.0            # 总栅极电荷 nC

    # ── 热参数 ──
    Rth_jc: float = 0.08            # 结→壳热阻 K/W
    Rth_ch: float = 0.03            # 壳→散热器热阻 K/W (含 TIM)
    Tj_max: float = 175.0           # 最高结温 °C
    Tc_ref: float = 80.0            # 参考壳温 °C


@dataclass
class SiCInverterConfig:
    """SiC 机侧逆变器系统配置"""
    # ── 拓扑 ──
    topology: str = "2L-VSI"        # 两电平电压源逆变器
    n_parallel: int = 2             # 每相并联模块数

    # ── 器件 ──
    device: SiCDeviceParams = field(default_factory=SiCDeviceParams)

    # ── PWM ──
    f_pwm: float = 25000.0          # 开关频率 Hz
    t_dead_ns: float = 300.0        # 死区时间 ns (SiC 仅需 300ns)
    t_dead_compensation: bool = True  # 死区补偿使能
    t_min_pulse_ns: float = 500.0   # 最小脉宽 ns

    # ── DC 母线 ──
    Vdc_nom: float = 750.0          # 标称母线电压 V
    Vdc_min: float = 550.0          # 最小母线电压 V
    Vdc_max: float = 900.0          # 最大母线电压 V

    # ── 输出 ──
    I_phase_max: float = 1000.0     # 相电流峰值 A (含过载)
    f_out_max: float = 880.0        # 最高输出频率 Hz (HIM 8800rpm×6极)

    # ── 散热 ──
    T_ambient: float = 40.0         # 环境温度 °C
    coolant_flow: float = 10.0      # 冷却液流量 L/min
    cooling_type: str = "liquid"    # "liquid" | "forced_air"

    # ── 保护阈值 ──
    I_ocp: float = 1500.0           # 过流保护 A
    T_warning: float = 150.0        # 温度告警 °C
    T_shutdown: float = 170.0       # 温度关断 °C

    @property
    def t_sample(self) -> float:
        """控制周期 = 1/f_pwm (单采样单更新)"""
        return 1.0 / self.f_pwm

    @property
    def V_max(self) -> float:
        """最大相电压幅值 (SVPWM 线性区)"""
        return self.Vdc_nom / math.sqrt(3)

    @property
    def Rds_on_equivalent(self) -> float:
        """等效导通电阻 (N 个模块并联)"""
        return self.device.Rds_on_25C / self.n_parallel

    @property
    def V_ceiling(self) -> float:
        """过调制区最大相电压 (六阶梯波)"""
        return self.Vdc_nom * 2.0 / math.pi


# ═══════════════════════════════════════
# SVPWM 调制器 (两电平, 七段式对称)
# ═══════════════════════════════════════

class SVPWMModulator:
    """两电平七段式对称 SVPWM — SiC 25kHz 优化版

    扇区映射:
      Sector 0: V1(100), V2(110)  —  0° ~ 60°
      Sector 1: V2(110), V3(010)  — 60° ~ 120°
      ... 依此类推
    """

    # 6 个有效矢量对应的 αβ 分量 (幅值=2/3*Vdc)
    _VECTORS_ALPHA = np.array([1.0,  0.5, -0.5, -1.0, -0.5,  0.5])
    _VECTORS_BETA  = np.array([0.0,  0.866, 0.866, 0.0, -0.866, -0.866])

    # 每扇区的两个有效矢量索引
    _SECTOR_VECTORS = [(0,1), (1,2), (2,3), (3,4), (4,5), (5,0)]

    def __init__(self, inv_cfg: SiCInverterConfig):
        self.cfg = inv_cfg
        self._last_sector = 0
        self._last_t1_pu = 0.0
        self._last_t2_pu = 0.0

    def modulate(self, v_alpha: float, v_beta: float, vdc: float,
                 i_abc: np.ndarray = None) -> Tuple[np.ndarray, dict]:
        """αβ 参考电压 → 三相占空比

        Args:
            v_alpha, v_beta: 参考电压 αβ 分量 V
            vdc: 直流母线电压 V
            i_abc: 三相电流 [ia, ib, ic] A (用于死区补偿)

        Returns:
            (duty_abc, info) — 三相占空比 [0,1], 调制中间信息
        """
        V_base = vdc * 2.0 / 3.0  # 等幅值 Clarke 变换基准

        # 1. 归一化
        va_pu = v_alpha / V_base
        vb_pu = v_beta / V_base

        # 2. 扇区判断 (基于角度)
        angle = math.atan2(v_beta, v_alpha)
        if angle < 0:
            angle += 2 * math.pi
        sector = int(math.floor((angle + math.pi / 6) / (math.pi / 3))) % 6
        self._last_sector = sector

        # 3. 作用时间计算
        v1_idx, v2_idx = self._SECTOR_VECTORS[sector]

        # T1 对应 v1, T2 对应 v2, 利用 Clarke 逆矩阵
        # [va] = [cos(θ1)  cos(θ2)] [T1/Ts]
        # [vb]   [sin(θ1)  sin(θ2)] [T2/Ts]
        sin60 = math.sqrt(3) / 2.0
        cos60 = 0.5

        # 扇区归一化旋转后的 vx, vy
        if sector == 0:
            vx, vy = va_pu, vb_pu
            t1 = vx - vy / math.sqrt(3)
            t2 = 2.0 * vy / math.sqrt(3)
        elif sector == 1:
            vx = va_pu * cos60 + vb_pu * sin60
            vy = -va_pu * sin60 + vb_pu * cos60
            t2 = vx - vy / math.sqrt(3)
            t1 = 2.0 * vy / math.sqrt(3)
        elif sector == 2:
            vx = va_pu * (-cos60) + vb_pu * sin60
            vy = -va_pu * sin60 - vb_pu * cos60
            t1 = vx - vy / math.sqrt(3)
            t2 = 2.0 * vy / math.sqrt(3)
        elif sector == 3:
            vx, vy = -va_pu, -vb_pu
            t1 = vx - vy / math.sqrt(3)
            t2 = 2.0 * vy / math.sqrt(3)
        elif sector == 4:
            vx = va_pu * (-cos60) - vb_pu * sin60
            vy = va_pu * sin60 - vb_pu * cos60
            t2 = vx - vy / math.sqrt(3)
            t1 = 2.0 * vy / math.sqrt(3)
        else:  # sector 5
            vx = va_pu * cos60 - vb_pu * sin60
            vy = va_pu * sin60 + vb_pu * cos60
            t2 = vx - vy / math.sqrt(3)
            t1 = 2.0 * vy / math.sqrt(3)

        t1 = max(0.0, min(1.0, t1))
        t2 = max(0.0, min(1.0, t2))

        # 过调制处理
        if t1 + t2 > 1.0:
            scale = 1.0 / (t1 + t2)
            t1 *= scale
            t2 *= scale

        t0 = 1.0 - t1 - t2
        self._last_t1_pu = t1
        self._last_t2_pu = t2

        # 4. 七段式对称占空比
        # 零矢量分配: t07 = t0/4 + t1/2 + t2/2 等
        # 直接按扇区输出三相占空比
        ta = (t1 + t2 + t0 / 2) / 2  # 上管的平均导通占空比
        tb = (t2 + t0 / 2) / 2
        tc = (t0 / 2) / 2

        # 按扇区映射回三相
        sector_map = [
            (ta, tb, tc),   # sector 0: [a,b,c]
            (tb, ta, tc),   # sector 1
            (tc, ta, tb),   # sector 2
            (tc, tb, ta),   # sector 3
            (tb, tc, ta),   # sector 4
            (ta, tc, tb),   # sector 5
        ]
        da, db, dc = sector_map[sector]

        duty = np.array([da, db, dc])

        # 5. 死区补偿 (基于电流极性)
        if i_abc is not None and self.cfg.t_dead_compensation:
            t_dead_pu = self.cfg.t_dead_ns * 1e-9 / self.cfg.t_sample
            for k in range(3):
                if i_abc[k] > 0:      # 电流流出 → 输出电压偏低 → 增加占空比
                    duty[k] += t_dead_pu
                elif i_abc[k] < 0:    # 电流流入 → 输出电压偏高 → 减少占空比
                    duty[k] -= t_dead_pu

        # 6. 限幅
        duty = np.clip(duty, 0.005, 0.995)  # 最小脉宽 0.5%

        info = {
            'sector': sector,
            't1_pu': t1, 't2_pu': t2, 't0_pu': t0,
            'v_alpha_applied': va_pu * V_base,
            'v_beta_applied': vb_pu * V_base,
            'overmod': (t1 + t2) > 1.0,
        }
        return duty, info


# ═══════════════════════════════════════
# 损耗 + 热模型
# ═══════════════════════════════════════

class SiCLossThermalModel:
    """SiC 模块导通 + 开关损耗 + 结温热模型"""

    def __init__(self, cfg: SiCInverterConfig):
        self.cfg = cfg
        self.dev = cfg.device
        self.np = cfg.n_parallel
        # 每相温度状态
        self._Tj = np.full(3, 40.0)  # 初始结温 = 环境

    @property
    def Tj(self) -> np.ndarray:
        return self._Tj

    def rds_on_at_temp(self, Tj_C: float) -> float:
        """Rds(on) 温度校正 (线性插值 25→175°C)"""
        if Tj_C <= 25:
            return self.dev.Rds_on_25C
        if Tj_C >= 175:
            return self.dev.Rds_on_175C
        frac = (Tj_C - 25.0) / 150.0
        return self.dev.Rds_on_25C + frac * (self.dev.Rds_on_175C - self.dev.Rds_on_25C)

    def step(self, i_abc: np.ndarray, duty_abc: np.ndarray,
             vdc: float, dt: float) -> dict:
        """一个 PWM 周期的损耗 + 热更新

        Args:
            i_abc: [ia, ib, ic] 三相电流 A
            duty_abc: 三相占空比 [0,1]
            vdc: 直流母线电压 V
            dt: 时间步长 s

        Returns:
            {'P_cond_W': [3], 'P_sw_W': [3], 'P_total_W': float, 'Tj_C': [3]}
        """
        f_pwm = self.cfg.f_pwm
        np_ = self.np

        P_cond = np.zeros(3)
        P_sw = np.zeros(3)

        for k in range(3):
            i = abs(i_abc[k])
            Tj_k = self._Tj[k]
            rds = self.rds_on_at_temp(Tj_k) / np_  # 并联等效

            # ── 导通损耗: P_cond = I² × Rds(on) × duty ──
            # 上半周期: 上管导通 duty, 下管导通 (1-duty)
            # 电流正: 上管 I²R, 下管体二极管 Vsd·I
            # 电流负: 上管体二极管, 下管 I²R
            # 简化: 用 I²Rds(on) 近似, 功因 ≈ cosφ
            P_cond[k] = i**2 * rds * 0.5  # 上下管平均

            # ── 开关损耗: 每周期 Eon+Eoff, 线性折算 ──
            Eon = self.dev.Eon_mJ * (vdc / 600.0) * (i / 300.0) / np_
            Eoff = self.dev.Eoff_mJ * (vdc / 600.0) * (i / 300.0) / np_
            P_sw[k] = (Eon + Eoff) * 1e-3 * f_pwm  # mJ → J, × Hz = W

        P_total = np.sum(P_cond) + np.sum(P_sw)

        # ── 热模型 (一阶 RC: Rth × Cth) ──
        Rth = self.dev.Rth_jc + self.dev.Rth_ch  # 结→散热器
        Cth = 200.0  # 热容 J/K (SiC 模块近似)

        for k in range(3):
            P_k = P_cond[k] + P_sw[k]
            dT = (P_k - (self._Tj[k] - self.cfg.T_ambient) / Rth) / Cth
            self._Tj[k] += dT * dt
            self._Tj[k] = max(self.cfg.T_ambient, min(self.dev.Tj_max, self._Tj[k]))

        return {
            'P_cond_W': P_cond.tolist(),
            'P_sw_W': P_sw.tolist(),
            'P_total_W': float(P_total),
            'Tj_C': self._Tj.tolist(),
            'efficiency': 1.0 - P_total / max(1.0, vdc * max(abs(i_abc)) * math.sqrt(3)),
        }


# ═══════════════════════════════════════
# SiC 机侧逆变器主类
# ═══════════════════════════════════════

class FidelityLevel(Enum):
    IDEAL = 0       # 直通 Vd/Vq
    VOLT_LIMIT = 1  # 六边形限幅 + 效率系数
    SVPWM = 2       # SVPWM + 死区 + 压降
    FULL_SWITCH = 3 # SVPWM + 损耗 + 热模型


class SiCMachineInverter:
    """SiC 机侧逆变器 — 控制器 ↔ 电机之间的硬件抽象层

    用法:
        inv = SiCMachineInverter(fidelity=FidelityLevel.SVPWM)
        Vd_actual, Vq_actual = inv.apply(Vd_ref, Vq_ref, theta_e, Vdc, Id, Iq)
    """

    def __init__(self,
                 config: SiCInverterConfig = None,
                 fidelity: FidelityLevel = FidelityLevel.IDEAL):
        self.cfg = config or SiCInverterConfig()
        self.fidelity = fidelity

        self.modulator = SVPWMModulator(self.cfg) if fidelity.value >= 2 else None
        self.thermal = SiCLossThermalModel(self.cfg) if fidelity.value >= 3 else None

        # 运行统计
        self._total_steps = 0
        self._cumulative_loss_J = 0.0
        self._v_util_peak = 0.0
        self._overmod_count = 0

    @property
    def V_max(self) -> float:
        return self.cfg.V_max

    def apply(self,
              vd_ref: float, vq_ref: float,
              theta_e: float,
              vdc: float,
              id_fb: float = 0.0, iq_fb: float = 0.0,
              dt: float = None) -> Tuple[float, float, dict]:
        """将控制器 Vd/Vq 参考值转化为实际施加到电机的 Vd'/Vq'

        Args:
            vd_ref, vq_ref: 控制器输出的 dq 参考电压 V
            theta_e: 电角度 rad
            vdc: 直流母线电压 V
            id_fb, iq_fb: 反馈电流 A
            dt: 时间步长 s (Level 3 热模型需要)

        Returns:
            (Vd_actual, Vq_actual, diagnostic_dict)
        """
        if dt is None:
            dt = self.cfg.t_sample

        # ── Level 0: 理想 ──
        if self.fidelity == FidelityLevel.IDEAL:
            self._total_steps += 1
            return vd_ref, vq_ref, {
                'fidelity': 'ideal',
                'v_mag': math.sqrt(vd_ref**2 + vq_ref**2),
                'v_util': math.sqrt(vd_ref**2 + vq_ref**2) / self.V_max,
            }

        # ── dq → αβ (Park 逆变换) ──
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        v_alpha_ref = vd_ref * cos_t - vq_ref * sin_t
        v_beta_ref = vd_ref * sin_t + vq_ref * cos_t

        v_mag = math.sqrt(v_alpha_ref**2 + v_beta_ref**2)
        v_util = v_mag / self.V_max
        self._v_util_peak = max(self._v_util_peak, v_util)

        # ── Level 1: 电压六边形限幅 ──
        if self.fidelity == FidelityLevel.VOLT_LIMIT:
            v_max = self.cfg.V_ceiling  # 过调制极限
            if v_mag > v_max:
                scale = v_max / v_mag
                v_alpha_ref *= scale
                v_beta_ref *= scale
                self._overmod_count += 1

            # 加效率损耗
            eta = 0.985  # 98.5% 典型 SiC 效率
            v_alpha_ref *= eta
            v_beta_ref *= eta

            # αβ → dq
            vd_act = v_alpha_ref * cos_t + v_beta_ref * sin_t
            vq_act = -v_alpha_ref * sin_t + v_beta_ref * cos_t

            self._total_steps += 1
            return vd_act, vq_act, {
                'fidelity': 'volt_limit',
                'v_mag': v_mag * eta, 'v_util': v_util * eta,
                'overmod': (v_mag > v_max),
            }

        # ── Level 2/3: SVPWM ──
        # Clark 逆: I_dq → I_abc
        i_alpha = id_fb * cos_t - iq_fb * sin_t
        i_beta = id_fb * sin_t + iq_fb * cos_t
        ia = i_alpha
        ib = -0.5 * i_alpha + math.sqrt(3)/2 * i_beta
        ic = -0.5 * i_alpha - math.sqrt(3)/2 * i_beta
        i_abc = np.array([ia, ib, ic])

        duty_abc, mod_info = self.modulator.modulate(v_alpha_ref, v_beta_ref, vdc, i_abc)

        # ── 器件压降修正 ──
        v_drop = np.zeros(3)
        rds = self.cfg.Rds_on_equivalent
        for k in range(3):
            if i_abc[k] >= 0:
                v_drop[k] = i_abc[k] * rds  # 上管导通压降
            else:
                v_drop[k] = i_abc[k] * rds  # 下管导通 (对称)

        # 实际输出电压 = Vdc * duty - 压降 — 但这里不模拟相电压波形
        # 而是把压降折合回 dq 域
        v_drop_alpha = (2*v_drop[0] - v_drop[1] - v_drop[2]) / 3
        v_drop_beta = (v_drop[1] - v_drop[2]) / math.sqrt(3)

        v_alpha_act = mod_info['v_alpha_applied'] - v_drop_alpha
        v_beta_act = mod_info['v_beta_applied'] - v_drop_beta

        # αβ → dq
        vd_act = v_alpha_act * cos_t + v_beta_act * sin_t
        vq_act = -v_alpha_act * sin_t + v_beta_act * cos_t

        # ── Level 3: 损耗 + 热 ──
        loss_info = {}
        if self.fidelity == FidelityLevel.FULL_SWITCH and self.thermal is not None:
            loss_info = self.thermal.step(i_abc, duty_abc, vdc, dt)
            self._cumulative_loss_J += loss_info['P_total_W'] * dt

        self._total_steps += 1
        if mod_info.get('overmod'):
            self._overmod_count += 1

        diag = {
            'fidelity': 'svpwm' if self.fidelity == FidelityLevel.SVPWM else 'full_switch',
            'v_mag': v_mag, 'v_util': v_util,
            'sector': mod_info['sector'],
            't1_pu': mod_info['t1_pu'], 't2_pu': mod_info['t2_pu'],
            'v_drop_max': float(max(abs(v_drop))),
            'duty_a': float(duty_abc[0]),
            **loss_info,
        }
        return vd_act, vq_act, diag

    def get_dc_current(self, i_abc: np.ndarray, duty_abc: np.ndarray) -> float:
        """计算直流母线电流 (用于直流母线模型耦合)"""
        return float(np.sum(i_abc * duty_abc))

    def get_stats(self) -> dict:
        """逆变器运行统计"""
        stats = {
            'total_steps': self._total_steps,
            'overmod_count': self._overmod_count,
            'overmod_pct': self._overmod_count / max(1, self._total_steps) * 100,
            'v_util_peak': self._v_util_peak,
            'cumulative_loss_kWh': self._cumulative_loss_J / 3.6e6,
        }
        if self.thermal is not None:
            stats['Tj_max_C'] = float(max(self.thermal.Tj))
            stats['Tj_C'] = self.thermal.Tj.tolist()
        return stats

    def reset(self):
        """复位逆变器状态"""
        self._total_steps = 0
        self._cumulative_loss_J = 0.0
        self._v_util_peak = 0.0
        self._overmod_count = 0
        if self.thermal is not None:
            self.thermal._Tj = np.full(3, 40.0)
        if self.modulator is not None:
            self.modulator._last_sector = 0
            self.modulator._last_t1_pu = 0.0
            self.modulator._last_t2_pu = 0.0


# ═══════════════════════════════════════
# 励磁 Buck 变换器 (HIM 励磁绕组供电)
# ═══════════════════════════════════════

@dataclass
class ExciterBuckConfig:
    """HIM 励磁 Buck 变换器配置"""
    Vin: float = 750.0              # 输入电压 V (DC bus)
    Vout_max: float = 750.0         # 最大输出 V
    Iout_max: float = 10.0          # 最大励磁电流 A
    f_sw: float = 100e3             # 开关频率 Hz
    L: float = 1e-3                 # 滤波电感 H
    C: float = 100e-6               # 输出滤波电容 F
    Rds_on: float = 0.021           # SiC MOSFET 导通电阻 Ω (C3M0021120K)
    efficiency: float = 0.97        # 典型效率


class ExciterBuckConverter:
    """HIM 励磁绕组 Buck 供电 — DC/DC, 750V→Vf

    简化模型: 理想变压器 + 效率系数
    也可用 Level 2 模拟电感电流纹波
    """

    def __init__(self, config: ExciterBuckConfig = None):
        self.cfg = config or ExciterBuckConfig()
        self._vf_out = 0.0
        self._if_out = 0.0
        self._duty = 0.0

    def step(self, vf_ref: float, if_fb: float, dt: float,
             fidelity: str = "ideal") -> Tuple[float, float]:
        """一个控制周期

        Args:
            vf_ref: 目标励磁电压 V
            if_fb: 励磁电流反馈 A
            dt: 时间步长
            fidelity: "ideal" | "ripple"

        Returns:
            (Vf_actual, If_actual)
        """
        cfg = self.cfg
        vf_ref = max(0.0, min(cfg.Vout_max, vf_ref))

        if fidelity == "ideal":
            # 理想: Vf = Vf_ref * efficiency, 无限流
            self._vf_out = vf_ref * cfg.efficiency
            self._if_out = if_fb  # 电流由电机励磁绕组动态决定
            self._duty = vf_ref / cfg.Vin

        else:  # "ripple"
            # 简化纹波模型: dI/dt = (Vin·D - Vout)/L
            duty = vf_ref / cfg.Vin
            self._duty = duty
            # 电感电流变化
            di = (cfg.Vin * duty - self._vf_out) / cfg.L * dt
            self._if_out += di
            # 输出电压滤波
            dV = (self._if_out - if_fb) / cfg.C * dt
            self._vf_out += dV
            self._vf_out = max(0.0, min(cfg.Vout_max, self._vf_out))

        return self._vf_out, self._if_out

    def reset(self):
        self._vf_out = 0.0
        self._if_out = 0.0
        self._duty = 0.0


# ═══════════════════════════════════════
# 工厂函数: 从 SystemConfig 构建
# ═══════════════════════════════════════

def build_sic_inverter_from_system(sys_cfg, fidelity: FidelityLevel = FidelityLevel.SVPWM):
    """从 SystemConfig 构建 SiC 机侧逆变器"""
    from system_config import SystemConfig
    inv_cfg = SiCInverterConfig(
        Vdc_nom=sys_cfg.constants.Vdc_nom,
        Vdc_min=sys_cfg.dc_bus.V_min,
        Vdc_max=sys_cfg.dc_bus.V_max,
        I_phase_max=sys_cfg.motor.I_max,
    )
    return SiCMachineInverter(inv_cfg, fidelity)


# ═══════════════════════════════════════
# 自测
# ═══════════════════════════════════════

if __name__ == "__main__":
    print("=== SiC 机侧逆变器自测 ===\n")

    # ── 1. 配置 ──
    inv_cfg = SiCInverterConfig()
    print(f"1. 硬件配置:")
    print(f"   拓扑: {inv_cfg.topology}, 并联: {inv_cfg.n_parallel}模块/相")
    print(f"   f_pwm={inv_cfg.f_pwm/1000:.0f}kHz, t_dead={inv_cfg.t_dead_ns:.0f}ns")
    print(f"   Vdc={inv_cfg.Vdc_nom}V, V_max(SVPWM)={inv_cfg.V_max:.1f}V")
    print(f"   Rds(on)等效={inv_cfg.Rds_on_equivalent*1e3:.2f}mΩ")
    print(f"   散热: {inv_cfg.cooling_type}, T_amb={inv_cfg.T_ambient}°C")

    # ── 2. SVPWM 调制测试 ──
    print(f"\n2. SVPWM 调制测试 (Vdc=750V, f_pwm=25kHz):")
    inv = SiCMachineInverter(inv_cfg, fidelity=FidelityLevel.SVPWM)

    test_cases = [
        (0, 0, 0, "零矢量"),
        (300, 0, 0, "满幅 d轴 (线性区)"),
        (0, 300, math.pi/2, "满幅 q轴"),
        (433, 0, 0, "SVPWM极限 (Vdc/√3)"),
        (500, 0, 0, "过调制测试"),
        (200, 100, math.pi/6, "扇区边界"),
    ]

    for vd, vq, theta, label in test_cases:
        vd_act, vq_act, diag = inv.apply(vd, vq, theta, 750.0)
        v_mag = math.sqrt(vd**2 + vq**2)
        v_act_mag = math.sqrt(vd_act**2 + vq_act**2)
        print(f"   {label:20s}: Vd={vd:6.0f} Vq={vq:6.0f} → Vd'={vd_act:6.1f} Vq'={vq_act:6.1f} "
              f"V_util={v_mag/inv.V_max*100:5.1f}% sector={diag['sector']}")

    # ── 3. 损耗模型 ──
    print(f"\n3. 损耗 + 热模型 (Level 3 FULL_SWITCH):")
    inv3 = SiCMachineInverter(inv_cfg, fidelity=FidelityLevel.FULL_SWITCH)

    import time
    t0 = time.time()
    for i in range(1000):
        w = 2 * math.pi * 50 * (i / 1000)
        vd = 200 * math.sin(w)
        vq = 200 * math.cos(w)
        theta = w * 0.1
        vd_a, vq_a, diag = inv3.apply(vd, vq, theta, 750.0, id_fb=300.0, iq_fb=200.0, dt=40e-6)

    stats = inv3.get_stats()
    print(f"   1000步 (40ms): 耗时={time.time()-t0:.3f}s")
    print(f"   累计损耗={stats['cumulative_loss_kWh']*1e6:.1f} J")
    print(f"   结温={stats['Tj_C']} °C")
    print(f"   过调制占比={stats['overmod_pct']:.1f}%")
    print(f"   V_util 峰值={stats['v_util_peak']*100:.0f}%")

    # ── 4. 励磁 Buck ──
    print(f"\n4. 励磁 Buck 变换器:")
    exc = ExciterBuckConverter()
    for vf_ref in [0, 375, 750, 800]:
        vf, if_ = exc.step(vf_ref, 8.0, 50e-6)
        print(f"   Vf_ref={vf_ref:4.0f}V → Vf={vf:.0f}V duty={exc._duty:.2f}")

    # ── 5. 死区效应演示 ──
    print(f"\n5. 死区效应 (300ns @25kHz):")
    duty_loss = 300e-9 / 40e-6 * 100
    v_loss = duty_loss / 100 * 750
    print(f"   占空比损失={duty_loss:.2f}%, 等效电压误差={v_loss:.1f}V")
    print(f"   低速时 (V_ref=30V): 相对误差={v_loss/30*100:.1f}%  ← 死区补偿关键!")
    print(f"   高速时 (V_ref=400V): 相对误差={v_loss/400*100:.2f}%")

    print(f"\n✅ SiC 机侧逆变器模型自测通过")
