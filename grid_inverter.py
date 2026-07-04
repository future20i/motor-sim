"""
grid_inverter.py — 网侧逆变器
双向 AC/DC · 预充 · VSG · PQ 控制 · 直流母线稳压 · PLL 锁相
"""
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class GridInvMode(Enum):
    OFF = 0
    PRECHARGE = 1       # 预充: 二极管整流 + 软启
    SYNC = 2            # 同步: PLL 锁相, 准备并网
    PQ_CONTROL = 3      # PQ 控制: 独立有功/无功
    VSG = 4             # 虚拟同步机: 惯量+下垂
    VDC_CONTROL = 5     # 直流母线稳压: 维持 Vdc
    FAULT = 6


@dataclass
class GridInverterConfig:
    """网侧逆变器参数 (750VDC / 480VAC 系统)"""
    # 额定值
    S_nom: float = 750e3        # 额定容量 VA (500kW + 裕量)
    Vdc_nom: float = 750.0      # 直流侧标称电压 V
    Vac_nom: float = 480.0      # 交流侧线电压 Vrms
    f_nom: float = 50.0         # 标称频率 Hz

    # 滤波器
    L_f: float = 0.5e-3         # 滤波电感 H
    R_f: float = 0.01           # 滤波电阻 Ω
    C_f: float = 20e-6          # 滤波电容 F (星接)

    # 预充
    R_precharge: float = 50.0   # 预充电阻 Ω
    Vdc_precharge_target: float = 675.0  # 预充目标 V (90% × 750V)

    # 控制参数
    Ts: float = 50e-6           # 控制周期 s
    Kp_i: float = 10.0          # 电流环 Kp
    Ki_i: float = 100.0         # 电流环 Ki
    Kp_pll: float = 2.0         # PLL Kp
    Ki_pll: float = 50.0        # PLL Ki

    # 保护
    I_max: float = 2000.0       # 最大电流 A (750kVA@480VAC)
    Vdc_max: float = 900.0      # 直流过压阈值
    Vdc_min: float = 550.0      # 直流欠压阈值


@dataclass
class GridInverterState:
    """逆变器实时状态"""
    mode: GridInvMode = GridInvMode.OFF

    # 直流侧
    Vdc: float = 0.0
    Idc: float = 0.0

    # 交流侧
    Id: float = 0.0; Iq: float = 0.0  # dq 电流 (主存储, 防止旋转帧漂移)
    Ia: float = 0.0; Ib: float = 0.0; Ic: float = 0.0  # abc (从 dq 导出)
    Va: float = 0.0; Vb: float = 0.0; Vc: float = 0.0

    # 功率
    P: float = 0.0     # 有功 W (正=逆变→电网)
    Q: float = 0.0     # 无功 var (正=感性)
    P_ref: float = 0.0
    Q_ref: float = 0.0

    # PLL
    theta_pll: float = 0.0      # PLL 角度 rad
    f_pll: float = 50.0         # PLL 频率 Hz
    Vg_amplitude: float = 0.0   # 电网电压幅值

    # VSG
    omega_vsg: float = 314.159  # VSG 虚拟角速度
    theta_vsg: float = 0.0      # VSG 虚拟功角
    P_vsg: float = 0.0

    # 状态
    contactor_closed: bool = False
    fault_code: int = 0
    t_online: float = 0.0       # 并网运行时间


class GridInverter:
    """网侧逆变器 — 双向 AC/DC 变换器
    
    控制模式:
    - PRECHARGE: 通过预充电阻给直流母线充电
    - SYNC: PLL 锁相, 准备并网
    - PQ_CONTROL: 独立有功/无功功率控制
    - VSG: 虚拟同步发电机模式
    - VDC_CONTROL: 直流母线电压控制
    """

    def __init__(self, config: GridInverterConfig = None):
        self.cfg = config or GridInverterConfig()
        self.state = GridInverterState(Vdc=self.cfg.Vdc_nom * 0.1)
        self._Ts = self.cfg.Ts

        # PLL 状态
        self._pll_integral = 0.0
        self._pll_v_q_prev = 0.0

        # 电流 PI 积分
        self._integral_d = 0.0
        self._integral_q = 0.0

        # 预充
        self._precharge_t = 0.0

        # VSG
        self._vsg_P_ref = 0.0
        self._vsg_D = 50.0       # 阻尼
        self._vsg_J = 5.0        # 虚拟惯量
        self._vsg_droop = 0.04   # 4% 下垂

        # 直流母线 PI
        self._vdc_integral = 0.0
        self._vdc_Kp = 5.0   # VDC 控制 Kp (可外部覆盖)
        self._vdc_Ki = 50.0  # VDC 控制 Ki

    # ═══════ 主仿真步进 ═══════
    def step(self, Vdc: float, grid_Va: float, grid_Vb: float, grid_Vc: float,
             P_ref: float = 0.0, Q_ref: float = 0.0,
             mode: GridInvMode = None) -> GridInverterState:
        """
        一个控制周期
        
        Vdc: 直流母线电压
        grid_Va/b/c: 电网三相瞬时电压
        P_ref, Q_ref: 功率参考 (PQ_CONTROL 模式)
        mode: 强制模式切换
        """
        s = self.state
        s.Vdc = Vdc
        if mode is not None:
            self._set_mode(mode)

        # 电网电压幅值 (αβ 坐标系峰值 = 相电压峰值)
        s.Vg_amplitude = math.sqrt(2/3 * (grid_Va**2 + grid_Vb**2 + grid_Vc**2))

        # PLL (始终运行, 用于同步)
        self._run_pll(grid_Va, grid_Vb, grid_Vc)

        # 模式执行
        if s.mode == GridInvMode.PRECHARGE:
            self._step_precharge()
        elif s.mode == GridInvMode.SYNC:
            self._step_sync()
        elif s.mode == GridInvMode.PQ_CONTROL:
            self._step_pq(P_ref, Q_ref)
        elif s.mode == GridInvMode.VSG:
            self._step_vsg()
        elif s.mode == GridInvMode.VDC_CONTROL:
            self._step_vdc()
        else:
            # OFF / FAULT: 零输出
            s.Id = s.Iq = 0.0
            s.Ia = s.Ib = s.Ic = 0.0
            s.P = s.Q = 0.0

        # 保护检查
        self._check_protection()

        s.t_online += self._Ts if s.contactor_closed else 0
        return s

    # ═══════ 预充 ═══════
    def _step_precharge(self):
        """通过预充电阻给直流母线充电
        Vdc 从 0 按 RC 曲线上升到 Vdc_precharge_target
        """
        cfg = self.cfg
        s = self.state
        self._precharge_t += self._Ts

        # RC 充电: 简化快充模型
        Vg_peak = s.Vg_amplitude * math.sqrt(2)
        dv = (Vg_peak - s.Vdc) / (cfg.R_precharge * 0.5e-3) * self._Ts  # 快充
        s.Vdc += dv * 20  # 加速因子

        s.Id = s.Iq = 0.0
        s.Ia = s.Ib = s.Ic = 0.0
        s.P = s.Q = 0.0

        # 预充完成 → 闭合接触器 → SYNC
        if s.Vdc >= cfg.Vdc_precharge_target * 0.95:
            s.contactor_closed = True
            self._set_mode(GridInvMode.SYNC)

    # ═══════ 同步 (PLL 锁相) ═══════
    def _step_sync(self):
        """等待 PLL 锁定 + 接触器闭合确认"""
        s = self.state
        s.Id = s.Iq = 0.0
        s.Ia = s.Ib = s.Ic = 0.0
        s.P = s.Q = 0.0

        # PLL 频率偏差 < 0.1Hz 且电压 > 85% → 切换
        df = abs(s.f_pll - self.cfg.f_nom)
        v_ok = s.Vg_amplitude > self.cfg.Vac_nom * 0.85

        # 同步条件满足 → 自动并网
        if df < 0.2 and v_ok and s.contactor_closed:
            # 默认进入 VDC_CONTROL 模式 (维持直流母线)
            self._set_mode(GridInvMode.VDC_CONTROL)

    # ═══════ PQ 控制 ═══════
    def _step_pq(self, P_ref: float, Q_ref: float):
        """独立有功/无功功率控制

        功率→电流:  Id_ref = 2/3 * P_ref / Vd,  Iq_ref = -2/3 * Q_ref / Vd
        电流内环:   dq PI 解耦控制 + 电网前馈
        """
        s = self.state
        cfg = self.cfg

        # 功率→电流参考 (dq 坐标系, PLL 锁定后 Vd≈Vg_amplitude)
        Vd = max(s.Vg_amplitude, 10.0)
        Id_ref = (2.0/3.0) * P_ref / Vd    # P = 3/2 * Vd * Id
        Iq_ref = -(2.0/3.0) * Q_ref / Vd   # Q = -3/2 * Vd * Iq (SVPWM 约定)

        Id_ref = max(-cfg.I_max, min(cfg.I_max, Id_ref))
        Iq_ref = max(-cfg.I_max, min(cfg.I_max, Iq_ref))

        # 电流内环
        omega = 2 * math.pi * s.f_pll
        Id, Iq, Vd_ref, Vq_ref = self._current_control(Id_ref, Iq_ref, Vd)

        s.P_ref = P_ref; s.Q_ref = Q_ref
        s.P = 1.5 * Vd * Id   # 实测功率 (dq → 三相功率)
        s.Q = -1.5 * Vd * Iq

    # ═══════ VSG 虚拟同步机 ═══════
    def _step_vsg(self):
        """VSG 模式: 模拟同步发电机惯量和下垂特性

        摆动方程: J·dω/dt = Pm - Pe - D·(ω-ω_grid)
        功率参考: P_ref + 一次调频 droop
        电流内环: dq PI 解耦
        """
        s = self.state
        cfg = self.cfg

        # 1. 功率计算
        P_ref = self._vsg_P_ref
        f_nom = cfg.f_nom
        df = f_nom - s.f_pll

        # 2. 下垂: 频率低于标称 → 增发 (正 ΔP)
        P_droop = (df / f_nom) / self._vsg_droop * cfg.S_nom

        # 3. 总功率参考 = 调度 + 下垂
        P_total = P_ref + P_droop

        # 4. 摆动方程
        omega_nom = 2 * math.pi * f_nom
        omega_grid = 2 * math.pi * s.f_pll
        Pe = s.P  # 电磁功率

        domega = (P_total - Pe) / (self._vsg_J * omega_nom)
        domega -= self._vsg_D * (s.omega_vsg - omega_grid) / self._vsg_J

        s.omega_vsg += domega * self._Ts
        s.theta_vsg += s.omega_vsg * self._Ts
        s.theta_vsg = s.theta_vsg % (2 * math.pi)

        # 5. 功角→电流参考
        Vg = max(s.Vg_amplitude, 10.0)
        Id_ref = (2.0/3.0) * P_total / Vg if Vg > 0 else 0
        Iq_ref = 0.0  # 无功暂不控

        Id_ref = max(-cfg.I_max, min(cfg.I_max, Id_ref))

        # 电流内环 (使用 VSG 频率做解耦)
        omega_vsg = s.omega_vsg
        Id, Iq, Vd_ref, Vq_ref = self._current_control(Id_ref, Iq_ref, Vg)

        s.P = 1.5 * Vg * Id
        s.Q = -1.5 * Vg * Iq
        s.P_vsg = P_total

    # ═══════ 直流母线电压控制 ═══════
    def _step_vdc(self):
        """维持直流母线电压: PI 控制 Vdc

        能量视角: E = ½C·Vdc²,  P_grid = PI(Vdc_ref² - Vdc²)
        或简化:     P_grid = PI(Vdc_ref - Vdc)  (小偏差线性近似)
        """
        s = self.state
        cfg = self.cfg

        err = s.Vdc - cfg.Vdc_nom  # Vdc>标称→正→逆变放电; Vdc<标称→负→整流充电
        self._vdc_integral += err * self._Ts
        self._vdc_integral = max(-500, min(500, self._vdc_integral))

        # PI 输出: 电网侧功率 (正: 整流, 负: 逆变)
        P_ref = self._vdc_Kp * err * cfg.Vdc_nom * 0.5 + self._vdc_Ki * self._vdc_integral * 10
        P_ref = max(-cfg.S_nom, min(cfg.S_nom, P_ref))

        # 功率→电流参考
        Vd = max(s.Vg_amplitude, 10.0)
        Id_ref = (2.0/3.0) * P_ref / Vd
        Id_ref = max(-cfg.I_max, min(cfg.I_max, Id_ref))
        Iq_ref = 0.0  # 单位功率因数 (Q=0)

        # 电流内环
        omega = 2 * math.pi * s.f_pll
        Id, Iq, Vd_ref, Vq_ref = self._current_control(Id_ref, Iq_ref, Vd)

        s.P_ref = P_ref
        s.P = 1.5 * Vd * Id
        s.Q = -1.5 * Vd * Iq

    # ═══════ 电流环 (dq 解耦 + 前馈) ═══════
    def _abc_to_dq(self, a, b, c, theta):
        """Clarke + Park 变换: abc → dq (幅值不变, cosine 参考)"""
        # Clarke: abc → αβ
        alpha = 2/3 * (a - 0.5*b - 0.5*c)
        beta = 1/math.sqrt(3) * (b - c)
        # Park: αβ → dq
        cos_t = math.cos(theta); sin_t = math.sin(theta)
        d =  cos_t * alpha + sin_t * beta
        q = -sin_t * alpha + cos_t * beta
        return d, q

    def _dq_to_abc(self, d, q, theta):
        """逆 Park + 逆 Clarke: dq → abc"""
        cos_t = math.cos(theta); sin_t = math.sin(theta)
        alpha = cos_t * d - sin_t * q
        beta  = sin_t * d + cos_t * q
        a = alpha
        b = -0.5*alpha + math.sqrt(3)/2*beta
        c = -0.5*alpha - math.sqrt(3)/2*beta
        return a, b, c

    def _current_control(self, Id_ref, Iq_ref, Vg_amplitude):
        """dq 电流内环 PI — 解耦 + 前馈 + 硬积分限幅 + L-R 动态

        电压饱和时 P 项主导瞬态响应 (di/dt=Vpi_max/L_f),
        误差减小到 Kp·err < Vpi_max 后积分自然接管。
        没有 anti-windup — 简单的硬限幅在电流控制中足够稳定。
        """
        cfg = self.cfg; s = self.state
        Ts = self._Ts

        # 1. 电流反馈 (从 dq 主存储读取, 避免旋转帧漂移)
        Id, Iq = s.Id, s.Iq

        # 2. 前馈 + 解耦
        Vd_ff = Vg_amplitude; Vq_ff = 0.0
        omega_g = 2 * math.pi * s.f_pll
        omega_L = omega_g * cfg.L_f
        Vd_decouple = -omega_L * Iq
        Vq_decouple =  omega_L * Id

        # 3. PI 可用电压
        V_pi_max = s.Vdc / math.sqrt(3) - abs(Vd_ff)
        V_pi_max = max(0.0, V_pi_max)

        err_d = Id_ref - Id
        err_q = Iq_ref - Iq

        # 4. 条件积分 (输出饱和时冻结, 防止 windup)
        Vd_pi_raw = cfg.Kp_i * err_d + cfg.Ki_i * self._integral_d
        Vq_pi_raw = cfg.Kp_i * err_q + cfg.Ki_i * self._integral_q
        Vd_pi = max(-V_pi_max, min(V_pi_max, Vd_pi_raw))
        Vq_pi = max(-V_pi_max, min(V_pi_max, Vq_pi_raw))

        # 仅当 PI 输出在限幅内才积分
        if abs(Vd_pi_raw - Vd_pi) < 0.01:
            self._integral_d += err_d * Ts
        if abs(Vq_pi_raw - Vq_pi) < 0.01:
            self._integral_q += err_q * Ts

        # 积分硬限幅
        I_lim = V_pi_max / (cfg.Ki_i + 1e-6)
        self._integral_d = max(-I_lim, min(I_lim, self._integral_d))
        self._integral_q = max(-I_lim, min(I_lim, self._integral_q))

        # 5. 电压参考 = PI(已钳位) + 前馈 + 解耦
        Vd_ref = Vd_pi + Vd_ff + Vd_decouple
        Vq_ref = Vq_pi + Vq_ff + Vq_decouple

        # 6. 逆变器电压 → abc
        s.Va, s.Vb, s.Vc = self._dq_to_abc(Vd_ref, Vq_ref, s.theta_pll)

        # 8. L-R 电流动态
        inv_factor = Ts / cfg.L_f
        Id_new = Id + inv_factor * (
            -cfg.R_f * Id + omega_g * cfg.L_f * Iq + Vd_ref - Vg_amplitude)
        Iq_new = Iq + inv_factor * (
            -cfg.R_f * Iq - omega_g * cfg.L_f * Id + Vq_ref - 0.0)

        # 10. 电流限幅 + 存储
        I_mag = math.sqrt(Id_new**2 + Iq_new**2)
        if I_mag > cfg.I_max:
            Id_new *= cfg.I_max / I_mag
            Iq_new *= cfg.I_max / I_mag
        s.Id, s.Iq = Id_new, Iq_new
        # abc 从 dq 导出 (用于显示, 不影响下次反馈)
        s.Ia, s.Ib, s.Ic = self._dq_to_abc(Id_new, Iq_new, s.theta_pll)

        return Id_new, Iq_new, Vd_ref, Vq_ref

    # ═══════ PLL ═══════
    def _run_pll(self, Va, Vb, Vc):
        """三相 SRF-PLL: 锁相电网电压。

        Clarke (幅值不变): Valpha=Vpk·sin(θ), Vbeta=-Vpk·cos(θ)
        Park (cos,sin):    Vd = Vpk·sin(θ-θ_pll), Vq = -Vpk·cos(θ-θ_pll)
        锁定条件: Vd → 0 (θ_pll = θ, Vq = -Vpk)
        
        用 Vd 做误差信号 — 锁定时 Vd=0
        """
        s = self.state
        cfg = self.cfg

        # Clarke: abc → αβ (amplitude-invariant)
        Valpha = 2/3 * (Va - 0.5*Vb - 0.5*Vc)
        Vbeta = 1/math.sqrt(3) * (Vb - Vc)

        # Park: αβ → dq
        cos_t = math.cos(s.theta_pll)
        sin_t = math.sin(s.theta_pll)
        Vd =  cos_t * Valpha + sin_t * Vbeta
        Vq = -sin_t * Valpha + cos_t * Vbeta

        # PLL: 用 Vq 做误差 (锁定后 Vq→0, Vd=Vpk, 标准 d 对齐)
        # 归一化到 pu, 钳位防止 windup
        V_norm = max(s.Vg_amplitude * math.sqrt(2/3), 10.0)
        Vq_pu = max(-2.0, min(2.0, Vq / V_norm))

        self._pll_integral += Vq_pu * self._Ts
        self._pll_integral = max(-5, min(5, self._pll_integral))
        freq_corr = cfg.Kp_pll * Vq_pu + cfg.Ki_pll * self._pll_integral

        s.f_pll = cfg.f_nom + freq_corr
        s.f_pll = max(48, min(52, s.f_pll))

        s.theta_pll += 2 * math.pi * s.f_pll * self._Ts
        s.theta_pll = s.theta_pll % (2 * math.pi)

    # ═══════ 模式切换 ═══════
    def _set_mode(self, mode: GridInvMode):
        s = self.state
        if s.mode == mode:
            return
        s.mode = mode
        if mode == GridInvMode.OFF:
            s.contactor_closed = False
        # 模式切换时重置电流和积分器 (防止跨模式 windup)
        s.Id = s.Iq = 0.0
        s.Ia = s.Ib = s.Ic = 0.0
        self._integral_d = 0.0
        self._integral_q = 0.0
        self._vdc_integral = 0.0

    # ═══════ 保护 ═══════
    def _check_protection(self):
        s = self.state
        cfg = self.cfg

        if s.Vdc > cfg.Vdc_max:
            s.fault_code = 1  # 直流过压
            self._set_mode(GridInvMode.FAULT)
        elif s.Vdc < cfg.Vdc_min and s.contactor_closed:
            s.fault_code = 2  # 直流欠压
            self._set_mode(GridInvMode.FAULT)

        I_peak = max(abs(s.Ia), abs(s.Ib), abs(s.Ic))
        if I_peak > cfg.I_max * 1.2:
            s.fault_code = 3  # 过流
            self._set_mode(GridInvMode.FAULT)

    def get_status(self) -> dict:
        s = self.state
        return {
            "mode": s.mode.name,
            "Vdc": s.Vdc,
            "P_kW": s.P / 1000,
            "Q_kvar": s.Q / 1000,
            "f_pll": s.f_pll,
            "contactor": s.contactor_closed,
            "fault": s.fault_code,
            "I_peak": max(abs(s.Ia), abs(s.Ib), abs(s.Ic)),
        }


# ═══════════════════════════════════════════════════════════
# 自测
# ═══════════════════════════════════════════════════════════
if __name__ == "__main__":
    import math, time
    from grid_sim import GridSimulator, GridConfig

    cfg = GridInverterConfig(
        L_f=5e-3, Kp_i=5.0, Ki_i=1.0,
        Kp_pll=3.0, Ki_pll=100.0,
        I_max=2500.0, Vdc_nom=750.0, Vac_nom=480.0,
    )
    inv = GridInverter(cfg)
    grid = GridSimulator(GridConfig(f_nom=50.0, V_nom=480.0))
    Vdc, dt = 750.0, cfg.Ts
    passed = 0; total = 3

    # ── Setup: precharge → VDC lock PLL ──
    for _ in range(300):
        grid.step(dt)
        inv.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                 mode=GridInvMode.PRECHARGE if Vdc < 675 else GridInvMode.VDC_CONTROL)
        Vdc += (750 - Vdc) * 0.01

    # ── Test 1: PLL lock ──
    df = abs(inv.state.f_pll - 50.0)
    ok1 = df < 0.1
    print(f"[{'PASS' if ok1 else 'FAIL'}] PLL lock: f={inv.state.f_pll:.3f}Hz (|Δf|={df:.3f}Hz)")
    if ok1: passed += 1

    # ── Test 2: PQ 200kW ──
    inv.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
             P_ref=200e3, Q_ref=0, mode=GridInvMode.PQ_CONTROL)
    t_start = time.time()
    for _ in range(4000):  # 200ms
        grid.step(dt)
        s = inv.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                     P_ref=200e3, Q_ref=0)
    elapsed = (time.time() - t_start) * 1000
    ok2 = s.P > 180e3
    print(f"[{'PASS' if ok2 else 'FAIL'}] PQ 200kW: P={s.P/1e3:.1f}kW (target >180kW, elapsed {elapsed:.0f}ms)")
    if ok2: passed += 1

    # ── Test 3: VSG droop ──
    inv._vsg_P_ref = 200e3
    for _ in range(500):
        grid.step(dt)
        s = inv.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                     mode=GridInvMode.VSG)
    p_before = s.P
    grid.set_frequency_deviation(-0.2)  # 49.8Hz
    for _ in range(500):
        grid.step(dt)
        s = inv.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc)
    dp = (s.P - p_before) / 1e3
    expected = 0.2 / 50 / 0.04 * 1.5e3  # ~150kW increase
    ok3 = dp > 50
    print(f"[{'PASS' if ok3 else 'FAIL'}] VSG droop: ΔP={dp:+.0f}kW (expected ~{expected:.0f}kW, threshold >50kW)")
    if ok3: passed += 1

    print(f"\n{passed}/{total} tests passed")
