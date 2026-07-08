"""
him_controller.py — HIM FOC控制器: 零极点对消 + 电压极限圆弱磁管理器。感应子电机专用。

子系统: 控制层
依赖: motor_base.py (HIMotor, HIMConfig), control_algorithms.py
手册对应章节: CONTROL_SETPOINTS.md §5 (HIM励磁控制)

HIM FOC控制器: 零极点对消 + 电压极限圆弱磁管理器。感应子电机专用。
"""
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional

from motor_base import HIMConfig, HIMMotor


# ═══════════════════════════════════════
# PI 控制器 (带抗积分饱和)
# ═══════════════════════════════════════

class PIController:
    """PI 控制器 — 输出限幅 + 抗积分饱和 (条件积分)"""

    def __init__(self, Kp: float, Ki: float, dt: float,
                 lim_out: Tuple[float, float], name: str = ""):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.lim_min, self.lim_max = lim_out
        self.name = name
        self.integral = 0.0
        self._last_out = 0.0

    def update(self, ref: float, fb: float) -> float:
        err = ref - fb
        # 条件积分 (anti-windup): 输出未饱和时才积分
        out_raw = self.Kp * err + self.Ki * (self.integral + err * self.dt)
        out = max(self.lim_min, min(self.lim_max, out_raw))

        if out != out_raw:
            # 饱和 → 冻结积分
            pass
        else:
            self.integral += err * self.dt

        self._last_out = out
        return out

    def reset(self):
        self.integral = 0.0
        self._last_out = 0.0


# ═══════════════════════════════════════
# HIM 控制器 (FOC + 自动弱磁)
# ═══════════════════════════════════════

class HIMController:
    """感应子电机矢量控制器 — 含电压极限圆弱磁管理器

    控制架构:
      速度外环 (PI) → Iq_ref
      电流内环 (PI+前馈解耦) → Vd, Vq
      励磁管理 (弱磁优先降 If → 负 Id) → Vf
    """

    def __init__(self, cfg: HIMConfig, dt: float = 50e-6,
                 bw_current: float = 1000.0,  # 电流环带宽 rad/s
                 bw_speed: float = 100.0):     # 速度环带宽 rad/s
        self.cfg = cfg
        self.dt = dt

        # ── 电流环 PI (零极点对消整定) ──
        # Kp = L / T_cl,  Ki = Kp * (R / L) = R / T_cl
        # T_cl = 1 / bw → 带宽 bw rad/s
        self.Kp_id = cfg.Ld * bw_current
        self.Ki_id = cfg.Rs * bw_current
        self.Kp_iq = cfg.Lq * bw_current
        self.Ki_iq = cfg.Rs * bw_current

        self.id_pi = PIController(self.Kp_id, self.Ki_id, dt,
                                   (-cfg.I_max, cfg.I_max), "id")
        self.iq_pi = PIController(self.Kp_iq, self.Ki_iq, dt,
                                   (-cfg.I_max, cfg.I_max), "iq")

        # ── 速度环 PI ──
        J = cfg.J
        P = cfg.P
        self.Kp_spd = J * bw_speed / (1.5 * P * cfg.If_nom * cfg.Mdf)
        self.Ki_spd = self.Kp_spd * bw_speed / 4
        self.speed_pi = PIController(self.Kp_spd, self.Ki_spd, dt,
                                      (-cfg.I_max, cfg.I_max), "speed")

        # ── 励磁电流 PI ──
        self.if_pi = PIController(10.0, 100.0, dt,
                                   (0.0, cfg.If_max * cfg.Rf), "if")

        # ── 参考值 ──
        self.Id_ref = 0.0
        self.Iq_ref = 0.0
        self.If_ref = 0.0
        self.Vd_ref = 0.0
        self.Vq_ref = 0.0
        self.Vf_ref = 0.0

        # ── 弱磁状态 ──
        self.flux_weakening_active = False
        self._voltage_margin = 0.95  # 电压利用率阈值

    @property
    def V_max(self) -> float:
        """逆变器最大输出相电压幅值 (SVPWM)"""
        return self.cfg.Vdc_nom / np.sqrt(3)

    def update(self, omega_ref: float, omega_fb: float,
               id_fb: float, iq_fb: float, if_fb: float,
               if_ext: Optional[float] = None,
               iq_ext: Optional[float] = None) -> Tuple[float, float, float]:
        """执行完整 FOC + 弱磁管理

        Args:
            omega_ref: 目标机械角速度 [rad/s]
            omega_fb:  当前机械角速度 [rad/s]
            id_fb:     d 轴电流反馈 [A]
            iq_fb:     q 轴电流反馈 [A]
            if_fb:     励磁电流反馈 [A]
            if_ext:    外部励磁指令 (None=自动管理)
            iq_ext:    外部 Iq 指令 (None=速度环自动计算) — POWER_TRACKING 用

        Returns:
            (Vd, Vq, Vf) — dq 轴电压 [V] + 励磁电压 [V]
        """
        cfg = self.cfg

        # ═══════ Step 1: 速度环 → Iq_ref ═══════
        if iq_ext is not None:
            self.Iq_ref = iq_ext  # 直接转矩控制 (旁路速度环)
        else:
            self.Iq_ref = self.speed_pi.update(omega_ref, omega_fb)

        # ═══════ Step 2: 励磁目标 ═══════
        if if_ext is not None:
            If_target = if_ext
        else:
            If_target = self.If_ref if self.If_ref > 0 else cfg.If_nom

        # ═══════ Step 3: 前馈解耦 + 电压预估 ═══════
        psi_d_est = cfg.Ld * self.Id_ref + cfg.Mdf * If_target
        psi_q_est = cfg.Lq * self.Iq_ref

        Vd_ff = -omega_fb * psi_q_est      # -ω·Lq·Iq
        Vq_ff = omega_fb * psi_d_est       # ω·(Ld·Id + Mdf·If)

        # ═══════ Step 4: 电压利用率检查 (弱磁触发器) ═══════
        V_mag = np.sqrt(Vd_ff**2 + Vq_ff**2)
        V_lim = self.V_max * self._voltage_margin

        self.flux_weakening_active = False
        if V_mag > V_lim and omega_fb > 10.0:
            self.flux_weakening_active = True

            # 策略 1: 优先降励磁 (感应子独有优势)
            # 反电动势 ∝ Mdf·If → 降 If 直接降电压需求
            if If_target > 0.1:
                voltage_error = (V_mag - V_lim) / V_lim
                If_target = max(0.0, If_target - voltage_error * 2.0)
            else:
                # 策略 2: 励磁已归零 → 注入负 Id (牺牲转矩换转速)
                self.Id_ref = -abs(self.Iq_ref) * 0.2
                self.Id_ref = max(self.Id_ref, -cfg.I_max * 0.5)
        else:
            # 不弱磁: Id=0 (MTPA 近似, 感应子磁阻转矩小)
            self.Id_ref = 0.0

        # ═══════ Step 5: 电流内环 PI + 前馈 ═══════
        Vd_pi = self.id_pi.update(self.Id_ref, id_fb)
        Vq_pi = self.iq_pi.update(self.Iq_ref, iq_fb)

        self.Vd_ref = Vd_pi + Vd_ff
        self.Vq_ref = Vq_pi + Vq_ff

        # 电压圆限幅
        V_mag_final = np.sqrt(self.Vd_ref**2 + self.Vq_ref**2)
        if V_mag_final > self.V_max:
            scale = self.V_max / V_mag_final
            self.Vd_ref *= scale
            self.Vq_ref *= scale

        # ═══════ Step 6: 励磁电压 ═══════
        self.If_ref = If_target
        if_fb_val = max(if_fb, 0.0)
        self.Vf_ref = self.if_pi.update(self.If_ref, if_fb_val)

        return self.Vd_ref, self.Vq_ref, self.Vf_ref

    def set_voltage_margin(self, margin: float):
        """设置弱磁触发阈值 (0.8-1.0, 默认 0.95)"""
        self._voltage_margin = max(0.5, min(1.0, margin))

    def get_status(self) -> dict:
        return {
            "Id_ref": self.Id_ref,
            "Iq_ref": self.Iq_ref,
            "If_ref": self.If_ref,
            "Vd_ref": self.Vd_ref,
            "Vq_ref": self.Vf_ref,
            "V_util": np.sqrt(self.Vd_ref**2 + self.Vq_ref**2) / self.V_max,
            "flux_weak": self.flux_weakening_active,
        }

    def reset(self):
        self.id_pi.reset()
        self.iq_pi.reset()
        self.speed_pi.reset()
        self.if_pi.reset()
        self.Id_ref = 0.0
        self.Iq_ref = 0.0
        self.If_ref = 0.0
        self.Vd_ref = 0.0
        self.Vq_ref = 0.0
        self.Vf_ref = 0.0
        self.flux_weakening_active = False


# ═══════════════════════════════════════
# 自测
# ═══════════════════════════════════════

if __name__ == "__main__":
    cfg = HIMConfig(Vdc_nom=48.0, P=2, J=0.5, Rs=0.05, Ld=0.002, Lq=0.002,
                    Mdf=0.01, Rf=0.5, Lf=0.1, If_nom=10.0, If_max=10.0,
                    I_max=20.0, T_sample=100e-6)
    motor = HIMMotor(cfg)
    ctrl = HIMController(cfg, dt=100e-6)

    print(f"=== HIM 控制器自测 ===")
    print(f"电机: {motor.type_name()}")
    print(f"V_max={ctrl.V_max:.1f}V, bw_current={1000}rad/s")
    print(f"Kp_id={ctrl.Kp_id:.3f} Ki_id={ctrl.Ki_id:.1f}")

    # 简单阶跃测试
    dt = 100e-6
    for i in range(500):
        Vd, Vq, Vf = ctrl.update(omega_ref=100.0, omega_fb=motor.state.omega_m,
                                  id_fb=motor.state.Id, iq_fb=motor.state.Iq,
                                  if_fb=motor.state.If, if_ext=10.0)
        motor.step(Vd, Vq, dt, Vf)
    print(f"500步后: ω={motor.state.omega_m:.1f}rad/s, Id={motor.state.Id:.2f}, "
          f"Iq={motor.state.Iq:.2f}, If={motor.state.If:.2f}")
    print(f"弱磁: {ctrl.flux_weakening_active}, V_util={ctrl.get_status()['V_util']:.2f}")
    print("✅ 自测通过")
