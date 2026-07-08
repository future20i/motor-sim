"""
power_ops.py — 系统级功率调度编排器: VSG/一次调频/直流母线恒压/充放电管理。负责两套符号约定的转换。

子系统: 控制层
依赖: flywheel_energy.py, grid_sim.py, dc_bus.py, motor_base.py
手册对应章节: ARCHITECTURE.md §3 (控制架构), STATE_MACHINE.md §2-9, CONTROL_SETPOINTS.md §2 (放电触发)

系统级功率调度编排器: VSG/一次调频/直流母线恒压/充放电管理。负责两套符号约定的转换。
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional
from control_algorithms import MotorModelParams, CurrentState
from motor_base import HIMMotor, HIMConfig
from sic_md_inverter import SiCMachineInverter, FidelityLevel, ExciterBuckConverter


class PowerMode(Enum):
    IDLE = 0
    DC_BUS_CONTROL = 1    # 直流母线恒压控制
    FREQ_REGULATION = 2   # 一次调频 (droop)
    VSG = 3               # 虚拟同步发电机
    POWER_TRACKING = 4    # 功率跟踪 (充放电调度)
    INERTIA_SUPPORT = 5   # 惯量支撑 (混合模式)


@dataclass
class PowerControlState:
    """功率控制状态"""
    P_ref: float = 0.0        # 功率参考 W (正=充电/吸收, 负=放电/释放)
    P_actual: float = 0.0     # 实际功率 W
    f_grid: float = 50.0      # 电网频率 Hz
    Vdc: float = 600.0        # 直流母线电压 V
    Vdc_ref: float = 600.0    # 直流母线参考 V
    SOC: float = 0.5          # 飞轮 SOC
    omega: float = 0.0        # 转速 rad/s
    mode: PowerMode = PowerMode.IDLE
    P_freq: float = 0.0       # 调频功率分量
    P_vsg: float = 0.0        # VSG 功率分量
    P_dc: float = 0.0         # 母线控制功率分量


class VSGController:
    """虚拟同步发电机控制器 (VSG)
    模拟同步发电机的惯量和阻尼特性
    """

    def __init__(self, J_virtual: float = 10.0, D_damping: float = 50.0,
                 droop_p: float = 0.04,  # 4% 下垂
                 P_nom: float = 500e3, ts: float = 50e-6):
        self.J = J_virtual          # 虚拟惯量 kg·m² (等效)
        self.D = D_damping          # 阻尼系数 Nm·s/rad
        self.droop_p = droop_p      # 有功下垂系数
        self.P_nom = P_nom          # 额定功率
        self._omega_v = 2 * math.pi * 50  # 虚拟角速度
        self._theta_v = 0.0         # 虚拟功角
        self._Ts = ts

    def compute(self, f_grid: float, SOC: float, P_schedule: float = 0.0) -> dict:
        """
        VSG 功率计算
        f_grid: 电网频率 Hz
        SOC: 飞轮荷电状态
        P_schedule: 调度功率 (充放电指令)
        返回: {P_vsg, omega_virtual, theta_virtual}
        """
        f_nom = 50.0
        df = f_nom - f_grid

        # 1. 一次调频 (droop)
        P_droop = (df / f_nom) / self.droop_p * self.P_nom

        # 2. 惯量响应 (RoCoF — 频率变化率)
        # 实际中通过 df/dt 计算, 这里简化为比例
        P_inertia = 0.0  # 需要频率导数, 下一周期用

        # 3. VSG 功角动态
        # P_vsg = (E*V/X)*sin(delta), 简化线性
        P_vsg = P_droop + P_schedule

        # 4. SOC 约束
        if SOC < 0.05 and P_vsg > 0:
            P_vsg = max(0, P_vsg * SOC / 0.05)  # 没电不能放电
        elif SOC > 0.95 and P_vsg < 0:
            P_vsg = min(0, P_vsg * (1 - SOC) / 0.05)  # 满了不能充电

        # 5. 虚拟角速度更新 (正确的摆动方程: J·dω/dt = P/ω - D·(ω-ω_nom))
        omega_nom = 2 * math.pi * f_nom
        P_mech = P_vsg / max(abs(self._omega_v), 1.0)  # P = T·ω → T = P/ω
        domega = (P_mech - self.D * (self._omega_v - omega_nom)) / self.J
        self._omega_v += domega * self._Ts
        self._theta_v += self._omega_v * self._Ts

        return {
            "P_vsg": P_vsg,
            "P_droop": P_droop,
            "P_inertia": P_inertia,
            "omega_v": self._omega_v,
            "theta_v": self._theta_v % (2 * math.pi),
        }


class FrequencyRegulator:
    """一次调频控制器 (droop + 惯量)"""

    def __init__(self, droop: float = 0.04, deadband: float = 0.02,
                 P_max: float = 500e3, ts: float = 50e-6):
        self.droop = droop          # 4% 下垂
        self.deadband = deadband    # 死区 Hz (±0.02Hz)
        self.P_max = P_max
        self._f_last = 50.0
        self._Ts = ts

    def compute(self, f_grid: float, SOC: float) -> float:
        """返回调频功率 W (正=放电支持电网, 负=充电吸收)"""
        f_nom = 50.0
        df = f_nom - f_grid  # 正 = 频率偏低, 需要放电

        # 死区
        if abs(df) < self.deadband:
            return 0.0

        # Droop: ΔP = -Δf / (droop * f_nom) * P_max
        P_freq = (df / f_nom) / self.droop * self.P_max

        # SOC 约束
        if P_freq > 0 and SOC < 0.1:       # 需要放电但电量不足
            P_freq *= max(0, (SOC - 0.05) / 0.05)
        elif P_freq < 0 and SOC > 0.9:     # 需要充电但已满
            P_freq *= max(0, (0.95 - SOC) / 0.05)

        # 检测 RoCoF (用简单差分)
        # dfdt = (f_grid - self._f_last) / self._Ts
        self._f_last = f_grid

        return max(-self.P_max, min(self.P_max, P_freq))


class DCBusController:
    """直流母线恒压控制器 — 多算法支持
    
    模式:
    - 'pi':      PI 控制 (传统, 响应慢)
    - 'ff_pi':   功率前馈 + PI 微调 (推荐, P_fly = P_load-P_grid + PI_trim)
    - 'deadbeat': 电容能量平衡 (最快, ½C(Vref²-V²)/dt 一步到位)
    - 'adaptive': 自适应增益调度 (大偏差激进, 小偏差精细)
    """

    def __init__(self, C_bus: float = 0.2, Vdc_nom: float = 750.0,
                 mode: str = 'ff_pi', ts: float = 50e-6):
        self.C = C_bus
        self.V_nom = Vdc_nom
        self.mode = mode
        self._integral = 0.0
        self._Ts = ts
        
        # PI 增益
        self.Kp = 20.0
        self.Ki = 200.0
        # 前馈增益
        self.Kp_ff = 50.0
        self.Ki_ff = 100.0
        # Deadbeat 平滑系数 (0-1, 1=全速)
        self.db_alpha = 0.8
        # 自适应阈值
        self.adapt_high = 50.0   # >50V 大偏差
        self.adapt_low = 10.0    # <10V 小偏差
        # 功率限幅
        self.P_max = 500e3
        
        # 统计数据
        self._v_min = Vdc_nom
        self._v_max = Vdc_nom
        self._response_count = 0

    def compute(self, Vdc: float, Vdc_ref: float = None,
                P_grid: float = 0.0, P_load: float = 0.0) -> float:
        """
        计算飞轮功率需求 (正=向母线放电)
        
        参数:
        - Vdc: 当前母线电压
        - Vdc_ref: 目标电压 (默认 V_nom)
        - P_grid: 电网提供的功率 W (可选, 前馈用)
        - P_load: 负载消耗的功率 W (可选, 前馈用)
        
        返回: 飞轮功率 W (正=放电→母线, 负=充电←母线)
        """
        if Vdc_ref is None:
            Vdc_ref = self.V_nom
        
        if self.mode == 'pi':
            P = self._compute_pi(Vdc, Vdc_ref)
        elif self.mode == 'ff_pi':
            P = self._compute_ff_pi(Vdc, Vdc_ref, P_grid, P_load)
        elif self.mode == 'deadbeat':
            P = self._compute_deadbeat(Vdc, Vdc_ref, P_grid, P_load)
        elif self.mode == 'adaptive':
            P = self._compute_adaptive(Vdc, Vdc_ref, P_grid, P_load)
        else:
            P = self._compute_ff_pi(Vdc, Vdc_ref, P_grid, P_load)
        
        # 统计
        self._v_min = min(self._v_min, Vdc)
        self._v_max = max(self._v_max, Vdc)
        
        return max(-self.P_max, min(self.P_max, P))

    def _compute_pi(self, Vdc, V_ref):
        """传统 PI 控制"""
        err = V_ref - Vdc
        self._integral += err * self._Ts
        self._integral = max(-500, min(500, self._integral))
        P = self.Kp * err + self.Ki * self._integral
        return P * self.V_nom * 0.2  # 缩放到功率

    def _compute_ff_pi(self, Vdc, V_ref, P_grid, P_load):
        """功率前馈 + PI 微调
        
        P_fly = (P_load - P_grid)         ← 瞬时功率缺口
              + Kp·err + Ki·∫err           ← PI 电压微调
        """
        # 前馈: 功率缺口直接补
        P_ff = P_load - P_grid
        
        # PI 微调电压
        err = V_ref - Vdc
        self._integral += err * self._Ts
        self._integral = max(-500, min(500, self._integral))
        P_trim = (self.Kp_ff * err + self.Ki_ff * self._integral) * self.V_nom * 0.1
        
        return P_ff + P_trim

    def _compute_deadbeat(self, Vdc, V_ref, P_grid, P_load):
        """电容能量平衡 (Deadbeat)
        
        电容能量: E = ½CV²
        能量缺口: ΔE = ½C(V_ref² - Vdc²)
        所需功率: P = ΔE / τ  (τ = 控制时间常数)
        
        P_fly = -(½C(V_ref²-Vdc²)/τ + P_load - P_grid)
        """
        dE = 0.5 * self.C * (V_ref**2 - Vdc**2)
        tau = 200e-6  # 200μs 恢复时间
        P_cap = dE / tau
        
        # 平滑: 不完全补偿 (防过冲)
        P_cap = P_cap * self.db_alpha
        
        P_fly = P_cap + (P_load - P_grid)  # 正=放电→母线
        return P_fly

    def _compute_adaptive(self, Vdc, V_ref, P_grid, P_load):
        """自适应增益调度
        
        |err| > 50V:  Kp=500 (激进比例 + 前馈)
        |err| > 10V:  Kp=100, Ki=50
        |err| < 10V:  Kp=20,  Ki=200
        """
        err = V_ref - Vdc
        abs_err = abs(err)
        
        if abs_err > self.adapt_high:
            # 大偏差: 前馈 + 激进比例, 不积分 (防 windup)
            P_ff = P_load - P_grid
            P_prop = 500.0 * err * self.V_nom * 0.1
            return P_ff + P_prop
        elif abs_err > self.adapt_low:
            Kp, Ki = 100.0, 50.0
        else:
            Kp, Ki = 20.0, 200.0
        
        self._integral += err * self._Ts
        self._integral = max(-500, min(500, self._integral))
        P = Kp * err + Ki * self._integral
        
        # 加前馈
        P_ff = P_load - P_grid
        return P_ff + P * self.V_nom * 0.1

    def reset(self):
        """重置积分和统计"""
        self._integral = 0.0
        self._v_min = self.V_nom
        self._v_max = self.V_nom

    def get_stats(self) -> dict:
        return {
            "mode": self.mode,
            "integral": self._integral,
            "V_min": self._v_min,
            "V_max": self._v_max,
            "V_swing": self._v_max - self._v_min,
        }


class PowerOrchestrator:
    """功率调度编排器 — 统一管理所有功率模式
    
    支持两种模式:
    - 简化模式 (默认): 功率直接驱动飞轮惯量, 跳过电机电磁模型
    - 全链路模式: 功率→转矩→Iq_ref→电流控制器→PMSM电机→飞轮
    """

    def __init__(self,
                 flywheel,          # FlywheelEnergyStorage
                 grid=None,         # GridSimulator (可选)
                 P_nom: float = 500e3,
                 motor=None,        # MotorBase (全链路用)
                 current_ctrl=None, # ControllerBase (全链路用)
                 ts: float = 50e-6,
                 ):
        self.fw = flywheel
        self.grid = grid
        self.P_nom = P_nom
        self.motor = motor
        self.ctrl = current_ctrl
        
        # 子控制器
        self.vsg = VSGController(P_nom=P_nom, ts=ts)
        self.freq_reg = FrequencyRegulator(P_max=P_nom, ts=ts)
        self.dc_ctrl = DCBusController(Vdc_nom=750.0, ts=ts)
        
        self.state = PowerControlState()
        self._mode = PowerMode.IDLE
        self._Ts = ts
        
        # SOC 监测
        self._soc_history = []
        self._soc_alarm = 0.15  # SOC 低告警
        
    @property
    def use_full_chain(self):
        """是否走完整链路 (电机+电流控制或HIM自带控制器)"""
        if isinstance(self.motor, HIMMotor):
            return True  # HIM 自带励磁控制器
        return self.motor is not None and self.ctrl is not None

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, m: PowerMode):
        self._mode = m
        self.state.mode = m

    def step(self, Vdc: float, f_grid: float, SOC: float,
             P_schedule: float = 0.0, mode: PowerMode = None,
             P_grid: float = 0.0, P_load: float = 0.0) -> PowerControlState:
        """
        一个步进的功率调度
        
        P_grid: 电网提供功率 W (前馈用)
        P_load: 负载消耗功率 W (前馈用)
        返回更新后的 PowerControlState
        """
        if mode is not None:
            self._mode = mode

        s = self.state
        s.Vdc = Vdc
        s.f_grid = f_grid
        s.SOC = SOC
        s.omega = self.fw.state.omega

        s.P_freq = 0.0
        s.P_vsg = 0.0
        s.P_dc = 0.0

        if self._mode == PowerMode.IDLE:
            s.P_ref = 0.0

        elif self._mode == PowerMode.DC_BUS_CONTROL:
            s.P_dc = self.dc_ctrl.compute(Vdc, P_grid=P_grid, P_load=P_load)
            s.P_ref = s.P_dc

        elif self._mode == PowerMode.FREQ_REGULATION:
            s.P_freq = self.freq_reg.compute(f_grid, SOC)
            s.P_ref = s.P_freq

        elif self._mode == PowerMode.VSG:
            vsg_out = self.vsg.compute(f_grid, SOC, P_schedule)
            s.P_vsg = vsg_out["P_vsg"]
            s.P_ref = s.P_vsg

        elif self._mode == PowerMode.POWER_TRACKING:
            s.P_ref = P_schedule

        elif self._mode == PowerMode.INERTIA_SUPPORT:
            # 混合: DC母线控制 + 调频
            s.P_dc = self.dc_ctrl.compute(Vdc) * 0.3  # 30% 权重
            s.P_freq = self.freq_reg.compute(f_grid, SOC) * 0.7
            s.P_ref = s.P_dc + s.P_freq

        # 功率限幅
        s.P_ref = max(-self.P_nom, min(self.P_nom, s.P_ref))
        s.P_actual = s.P_ref  # 简化: 假设理想功率跟踪

        # ═══ 推进飞轮模型 ═══
        # 符号处理: 按模式区分
        if self._mode in (PowerMode.FREQ_REGULATION, PowerMode.VSG, PowerMode.INERTIA_SUPPORT):
            p_flywheel = -s.P_ref
        else:
            p_flywheel = s.P_ref

        if self.use_full_chain:
            self._step_full_chain(p_flywheel, Vdc)
        else:
            self._step_simple(p_flywheel, Vdc)

        s.SOC = self.fw.state.SOC
        s.omega = self.fw.state.omega
        s.P_actual = abs(self.fw.state.P_mech)

        # SOC 监测
        self._soc_history.append(s.SOC)
        if len(self._soc_history) > 1000:
            self._soc_history.pop(0)

        return s

    def _step_simple(self, P_fly: float, Vdc: float):
        """简化模式: 功率直接驱动飞轮"""
        self.fw.step(P_fly, Vdc)

    def _step_full_chain(self, P_fly: float, Vdc: float):
        """全链路模式: P→T→Iq_ref→电流控制→电机→飞轮
        
        v2.1: 支持 HIM 感应子电机 (励磁绕组 + Vf 控制)
        """
        motor = self.motor
        ctrl = self.ctrl
        ms = motor.state
        fw = self.fw

        # 1. 功率 → 转矩
        omega = max(abs(fw.state.omega), 0.1)
        T_ref = P_fly / omega
        T_max = self.P_nom / omega
        T_ref = max(-T_max, min(T_max, T_ref))

        # ── HIM 分支: 使用 HIMController 处理励磁 ──
        if isinstance(motor, HIMMotor):
            self._step_him(P_fly, Vdc, T_ref, omega)
            return

        # ── PMSM/IM 分支 (原逻辑) ──
        # 2. 转矩 → Iq 参考
        P = motor.cfg.P
        psi_m = getattr(motor.cfg, 'psi_m', 0.07) or 0.07
        Iq_ref = T_ref / (1.5 * P * psi_m)
        Iq_ref = max(-motor.cfg.I_max, min(motor.cfg.I_max, Iq_ref))
        Id_ref = 0.0

        # 3. 电流控制器
        state = CurrentState(Id=ms.Id, Iq=ms.Iq, omega_m=fw.state.omega, Vdc=Vdc)
        Vd, Vq = ctrl.compute(Id_ref, Iq_ref, state)

        # 4. 驱动电机
        motor.step(Vd, Vq)

        # 5. 同步飞轮
        fw.sync_from_motor(ms.omega_m, T_ref, Vdc)

    def _step_him(self, P_fly: float, Vdc: float, T_ref: float, omega: float):
        """HIM 感应子电机全链路步进 — 含 SiC 逆变器硬件模型

        链路: HIMController → SiC逆变器(DC→AC) → HIMMotor
              ExciterBuck → 励磁绕组 Vf
        """
        motor = self.motor
        fw = self.fw
        ms = motor.state

        # 懒加载 HIM 控制器 + SiC 逆变器 + 励磁 Buck
        if not hasattr(self, '_him_ctrl'):
            from him_controller import HIMController
            self._him_ctrl = HIMController(motor.cfg, dt=self._Ts)
            self._him_pre_excited = False
            # SiC 机侧逆变器 (SVPWM Level)
            self._sic_inv = SiCMachineInverter(fidelity=FidelityLevel.SVPWM)
            self._sic_inv.cfg.Vdc_nom = motor.cfg.Vdc_nom
            # 励磁 Buck
            self._exciter = ExciterBuckConverter()

        him = self._him_ctrl
        sic = self._sic_inv
        exc = self._exciter

        # 预励磁 (复位后重新执行)
        if not self._him_pre_excited:
            Vf_full = motor.cfg.If_nom * motor.cfg.Rf
            for _ in range(int(0.05 / self._Ts)):
                motor.step(0, 0, self._Ts, Vf=Vf_full)
            self._him_pre_excited = True

        # 转矩 → Iq 参考
        Mdf = motor.Mdf
        If_cur = max(ms.If, 0.1)
        Iq_ref = T_ref / (1.5 * motor.cfg.P * Mdf * If_cur)
        Iq_ref = max(-motor.cfg.I_max, min(motor.cfg.I_max, Iq_ref))

        # 励磁管理
        if_ext = motor.cfg.If_nom if abs(T_ref) > 0.5 else 0.0

        # ═══ HIM 控制器 → Vd/Vq 参考 ═══
        Vd_ref, Vq_ref, Vf_ref = him.update(
            omega_ref=fw.state.omega, omega_fb=fw.state.omega,
            id_fb=ms.Id, iq_fb=ms.Iq, if_fb=ms.If,
            if_ext=if_ext, iq_ext=Iq_ref,
        )

        # ═══ SiC 逆变器: Vd/Vq 参考 → 实际电压 (含 SVPWM/死区/压降) ═══
        theta_e = ms.theta_e
        Vd_act, Vq_act, inv_diag = sic.apply(
            Vd_ref, Vq_ref, theta_e, Vdc,
            id_fb=ms.Id, iq_fb=ms.Iq, dt=self._Ts,
        )
        self._inv_diag = inv_diag  # 暴露给外部读取

        # ═══ 励磁 Buck: Vf_ref → 实际 Vf ═══
        Vf_act, _ = exc.step(Vf_ref, ms.If, self._Ts)

        # ═══ 驱动电机 ═══
        motor.step(Vd_act, Vq_act, self._Ts, Vf=Vf_act)

        # 同步飞轮
        fw.sync_from_motor(ms.omega_m, T_ref, Vdc)

    # ═══════ SOC 监测 ═══════
    def reset(self):
        """复位所有内部状态 (含 HIM 预励磁标志)"""
        self._soc_history.clear()
        self._him_pre_excited = False
        if hasattr(self, '_him_ctrl'):
            self._him_ctrl.reset()
        self.dc_ctrl.reset()
        self.state = PowerControlState()
        self._mode = PowerMode.IDLE

    def get_soc_status(self) -> dict:
        """SOC 综合状态"""
        soc = self.fw.state.SOC
        fw = self.fw
        cfg = fw.cfg

        E_now = fw.state.E_stored / 3.6e6  # kWh
        E_max = cfg.E_max_joules / 3.6e6
        rpm = fw.rpm
        rpm_pct = (rpm - cfg.omega_min) / (cfg.omega_max - cfg.omega_min) * 100

        # 估算剩余放电时间 (额定功率)
        if fw.state.P_mech > 0:  # 正在放电
            t_remain = E_now * 3600 / abs(fw.state.P_mech) if abs(fw.state.P_mech) > 100 else 9999
        else:
            t_remain = 9999

        # 告警等级
        if soc < 0.05:
            level = "critical"
        elif soc < self._soc_alarm:
            level = "warning"
        elif soc > 0.95:
            level = "full"
        else:
            level = "normal"

        # SOC 变化率
        if len(self._soc_history) > 10:
            dsoc_dt = (self._soc_history[-1] - self._soc_history[-10]) / (10 * self._Ts)
        else:
            dsoc_dt = 0.0

        return {
            "SOC": soc,
            "SOC_pct": soc * 100,
            "E_kWh": E_now,
            "E_max_kWh": E_max,
            "rpm": rpm,
            "rpm_pct": rpm_pct,
            "mode": fw.state.mode,
            "P_mech_kW": fw.state.P_mech / 1000,
            "t_remain_s": t_remain,
            "level": level,
            "dsoc_dt": dsoc_dt,
            "temp": fw.state.temp,
            "alarms": self._check_soc_alarms(),
        }

    def _check_soc_alarms(self) -> list:
        """SOC 告警检查"""
        alarms = []
        soc = self.fw.state.SOC
        fw = self.fw

        if soc < 0.05:
            alarms.append({"severity": "critical", "msg": "SOC<5%: 飞轮即将停转, 立即充电或切换备用电源"})
        elif soc < self._soc_alarm:
            alarms.append({"severity": "warning", "msg": f"SOC<{self._soc_alarm*100:.0f}%: 低电量, 减少放电"})
        if soc > 0.95:
            alarms.append({"severity": "info", "msg": "SOC>95%: 接近满电, 避免继续充电"})
        if fw.state.temp > 100:
            alarms.append({"severity": "warning", "msg": f"温度 {fw.state.temp:.0f}°C: 降低功率"})
        if fw.rpm > fw.cfg.omega_max * 0.98:
            alarms.append({"severity": "warning", "msg": f"转速 {fw.rpm:.0f}rpm: 接近上限"})

        return alarms

    def get_status(self) -> dict:
        s = self.state
        return {
            "mode": s.mode.name,
            "P_ref_kW": s.P_ref / 1000,
            "P_actual_kW": s.P_actual / 1000,
            "P_freq_kW": s.P_freq / 1000,
            "P_vsg_kW": s.P_vsg / 1000,
            "P_dc_kW": s.P_dc / 1000,
            "f_grid": s.f_grid,
            "Vdc": s.Vdc,
            "Vdc_ref": s.Vdc_ref,
            "SOC": s.SOC,
        }
