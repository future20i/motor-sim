"""
grid_sim.py — 电网仿真模型
三相电网 · 频率偏差 · 电压波动 · 相位跳变 · 故障注入
"""
import math
import random
from dataclasses import dataclass
from enum import Enum


class GridFault(Enum):
    NONE = 0
    UNDERVOLTAGE = 1    # 电压暂降
    OVERVOLTAGE = 2     # 电压暂升
    FREQ_DROP = 3       # 频率跌落
    FREQ_RISE = 4       # 频率升高
    PHASE_JUMP = 5      # 相位跳变
    BLACKOUT = 6        # 断电


@dataclass
class GridConfig:
    """电网参数 (中国 50Hz 标准)"""
    V_nom: float = 380.0        # 线电压 Vrms
    f_nom: float = 50.0         # 标称频率 Hz
    V_dc_nom: float = 600.0     # 对应直流母线电压 (整流后)
    phase_deg: float = 0.0      # 初始相位角

    # 电网强度
    X_grid: float = 0.05        # 电网等效电抗 pu (弱网)
    SCR: float = 10.0           # 短路比

    # 频率波动参数
    f_noise_std: float = 0.01   # 频率噪声标准差 Hz
    f_ramp_rate: float = 0.5    # 频率变化率 Hz/s (事故)
    f_primary_band: float = 0.2 # 一次调频死区 ±Hz


@dataclass
class GridState:
    """电网实时状态"""
    Va: float = 0.0     # 瞬时相电压 V (峰值)
    Vb: float = 0.0
    Vc: float = 0.0
    f: float = 50.0     # 当前频率 Hz
    V_rms: float = 380.0  # 当前线电压 Vrms
    theta: float = 0.0  # 当前相位角 rad
    fault: GridFault = GridFault.NONE
    t: float = 0.0


class GridSimulator:
    """电网仿真器 — 含频率/电压波动和故障"""

    def __init__(self, config: GridConfig = None):
        self.cfg = config or GridConfig()
        self.state = GridState(f=self.cfg.f_nom, V_rms=self.cfg.V_nom)
        self._f_target = self.cfg.f_nom
        self._f_ramp = 0.0       # 当前频率变化率
        self._v_scale = 1.0      # 电压标度
        self._fault_timer = 0.0  # 故障倒计时
        self._Ts = 50e-6

    def step(self, dt: float = None) -> GridState:
        """一个仿真步进"""
        if dt is None:
            dt = self._Ts
        s = self.state
        s.t += dt

        # 1. 频率更新 (含噪声, 正确的随机微分方程离散化)
        f_noise = random.gauss(0, 0.01)  # σ=0.01Hz 电压噪声
        # 白噪声积分: df_noise = σ * √dt (不是 dt!)
        s.f += self._f_ramp * dt + f_noise * math.sqrt(dt) * 0.5
        # 向目标频率回归
        f_target = getattr(self, '_f_target', self.cfg.f_nom)
        df = f_target - s.f
        s.f += df * 0.1 * dt  # τ=10s 回归
        s.f = max(49.0, min(51.0, s.f))

        # 2. 电压更新
        target_v = self.cfg.V_nom * self._v_scale
        # 快速响应电压变化 (故障时立即跳变)
        if s.fault != GridFault.NONE:
            s.V_rms = target_v  # 故障立即生效
        else:
            s.V_rms += (target_v - s.V_rms) * 20.0 * dt  # 正常恢复

        # 3. 相位累加 (归一化防止浮点精度丢失)
        s.theta += 2 * math.pi * s.f * dt
        s.theta = s.theta % (2 * math.pi)

        # 4. 瞬时三相电压 (cosine 参考, 相电压峰值 = Vrms_line * √2/√3)
        Vpk_phase = s.V_rms * math.sqrt(2.0 / 3.0)
        s.Va = Vpk_phase * math.cos(s.theta)
        s.Vb = Vpk_phase * math.cos(s.theta - 2 * math.pi / 3)
        s.Vc = Vpk_phase * math.cos(s.theta + 2 * math.pi / 3)

        # 5. 故障倒计时
        if self._fault_timer > 0:
            self._fault_timer -= dt
            if self._fault_timer <= 0:
                self.clear_fault()

        return s

    # ═══════ 电网事件 ═══════
    def set_frequency_deviation(self, df: float):
        """设置频率偏差 Hz (如 -0.3 = 49.7Hz)。偏差会维持, 不会被回归拉回标称。"""
        self._f_target = self.cfg.f_nom + df
        self.state.f = self._f_target

    def set_freq_ramp(self, dfdt: float):
        """设置频率变化率 Hz/s"""
        self._f_ramp = dfdt

    def set_voltage(self, scale: float):
        """设置电压标度 (0.8 = 80% sag)"""
        self._v_scale = scale

    def inject_fault(self, fault: GridFault, duration: float = 0.5):
        """注入电网故障"""
        s = self.state
        # 保存故障前状态以便恢复
        self._v_scale_before = self._v_scale
        self._f_ramp_before = self._f_ramp
        s.fault = fault
        self._fault_timer = duration

        if fault == GridFault.UNDERVOLTAGE:
            self._v_scale = 0.6
        elif fault == GridFault.OVERVOLTAGE:
            self._v_scale = 1.2
        elif fault == GridFault.FREQ_DROP:
            self._f_ramp = -self.cfg.f_ramp_rate
        elif fault == GridFault.FREQ_RISE:
            self._f_ramp = self.cfg.f_ramp_rate
        elif fault == GridFault.PHASE_JUMP:
            s.theta += math.radians(30)
        elif fault == GridFault.BLACKOUT:
            self._v_scale = 0.0
            s.Va = s.Vb = s.Vc = 0.0

    def clear_fault(self):
        """清除故障, 恢复到故障前运行点"""
        self.state.fault = GridFault.NONE
        self._v_scale = getattr(self, '_v_scale_before', 1.0)
        self._f_ramp = getattr(self, '_f_ramp_before', 0.0)
        # 立即恢复电压, 不等待缓慢回归
        self.state.V_rms = self.cfg.V_nom * self._v_scale

    # ═══════ 电网侧直流母线电压估算 ═══════
    def estimate_vdc(self, P_grid: float) -> float:
        """根据电网功率估算直流母线电压
        P_grid > 0: 从电网取电 (整流) → Vdc 上升
        P_grid < 0: 向电网送电 (逆变) → Vdc 取决于逆变器
        """
        # 简化模型: Vdc = Vdc_nom + k * P_grid
        # 实际是三相整流桥 + 电容
        dc_per_unit = P_grid / (self.cfg.V_dc_nom * 50)  # 标度
        vdc = self.cfg.V_dc_nom * (1.0 + 0.01 * dc_per_unit)
        return max(100.0, min(800.0, vdc))

    def get_readable(self) -> dict:
        s = self.state
        return {
            "f_Hz": s.f,
            "V_rms": s.V_rms,
            "V_abc": (s.Va, s.Vb, s.Vc),
            "theta_deg": math.degrees(s.theta) % 360,
            "df_Hz": s.f - self.cfg.f_nom,
            "fault": s.fault.name,
            "t_s": s.t,
        }
