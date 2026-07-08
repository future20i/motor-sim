"""
ai_commissioner.py — AI 电机调试助手: 参数辨识 (Rs→Ld/Lq→磁链→J) + PI自整定 + 故障诊断。

子系统: 应用层 (S1)
依赖: motor_base.py, control_algorithms.py
手册对应章节: CONTROL_SETPOINTS.md §4, TELEMETRY.md §5

AI 电机调试助手: 参数辨识 (Rs→Ld/Lq→磁链→J) + PI自整定 + 故障诊断。
"""
import math
import numpy as np
from typing import Optional, Callable

from motor_base import MotorBase, FaultType
from virtual_controller import VirtualController, OpState
from control_algorithms import MotorModelParams


class CommissionStep:
    """一个调试工步"""
    def __init__(self, name: str, desc: str, action: Callable, safety_check: Callable = None):
        self.name = name
        self.description = desc
        self.action = action        # () -> dict
        self.safety_check = safety_check  # () -> bool
        self.result = None
        self.status = "pending"     # pending | running | done | failed | skipped

    def run(self) -> dict:
        self.status = "running"
        try:
            self.result = self.action()
            self.status = "done"
        except Exception as e:
            self.result = {"error": str(e)}
            self.status = "failed"
        return self.result


class AICommissioner:
    """AI 调试助手 — 根据设计方案 S1 实现"""

    def __init__(self, motor: MotorBase, controller: VirtualController,
                 kp_speed: float = 1.0, ki_speed: float = 0.1):
        self.motor = motor
        self.ctrl = controller
        self.cfg = motor.cfg
        self.pm = MotorModelParams(
            Rs=self.cfg.Rs, Ld=getattr(self.cfg, 'Ld', 0.01),
            Lq=getattr(self.cfg, 'Lq', 0.01),
            psi_m=getattr(self.cfg, 'psi_m', 0.0), P=self.cfg.P
        )
        self.kp_speed = kp_speed
        self.ki_speed = ki_speed
        self.log = []           # [(timestamp, level, message)]
        self.identified = {}    # 辨识结果
        self.phase = "A"        # A/B/C
        self.sequence = []      # CommissionStep 列表
        self.current_step = 0

    # ═══════ 预充 ═══════
    def precharge(self, target_pct: float = 0.9, max_steps: int = 500) -> dict:
        """预充到目标电压百分比"""
        self.ctrl.op_mode = 1
        target_v = self.cfg.Vdc_nom * target_pct
        for i in range(max_steps):
            self.ctrl.run_cycle()
            if self.motor.state.Vdc >= target_v:
                return {"status": "ok", "Vdc": self.motor.state.Vdc, "steps": i + 1,
                        "target": target_v, "time_s": (i + 1) * self.cfg.T_sample}
        return {"status": "timeout", "Vdc": self.motor.state.Vdc, "steps": max_steps}

    # ═══════ Rs 辨识 (两点差分 + Ld 瞬态修正) ═══════
    def identify_rs(self, Vd_test: float = None, settle_steps: int = 3000) -> dict:
        """定子电阻辨识: 两点差分 + 用已知 Ld 修正瞬态误差
        Rs_est = dV/dI * (1 - e^(-t*Rs/Ld)), 需要先辨识 Ld
        """
        R = self.cfg.Rs
        Ld_known = self.identified.get("Ld", getattr(self.cfg, 'Ld', 0.01))
        if Vd_test is None:
            target_I1 = self.cfg.I_max * 0.2
            target_I2 = self.cfg.I_max * 0.6
            Vd1 = max(0.5, target_I1 * R)
            Vd2 = max(Vd1 + 1.0, target_I2 * R)
        else:
            Vd1 = Vd_test * 0.3
            Vd2 = Vd_test

        self.ctrl.op_mode = 3
        self.ctrl.Vq_ref = 0.0

        self.ctrl.Vd_ref = Vd1
        for i in range(settle_steps):
            self.ctrl.run_cycle()
        Id1 = self.motor.state.Id

        self.ctrl.Vd_ref = Vd2
        for i in range(settle_steps):
            self.ctrl.run_cycle()
        Id2 = self.motor.state.Id

        self.ctrl.Vd_ref = 0.0
        self.ctrl.op_mode = 2

        dV = Vd2 - Vd1
        dI = abs(Id2 - Id1)
        if dI < 0.001:
            return {"status": "failed", "reason": f"dI too small: {dI:.4f}A"}

        # 瞬态修正: Id(t) = Vd/Rs * (1-e^(-t*Rs/Ld))
        # Rs_raw = dV/dI, 修正因子 = 1-e^(-t*Rs/Ld)
        # 用迭代求解: Rs_est = dV/dI * (1 - e^(-t*Rs_est/Ld_known))
        Rs_raw = dV / dI
        t = settle_steps * self.cfg.T_sample
        Rs_est = Rs_raw
        for _ in range(100):
            tau = Ld_known / max(Rs_est, 1e-9)
            corr = 1.0 - math.exp(-t / tau)
            Rs_est = Rs_raw * corr
            if Rs_est <= 0:
                Rs_est = 0.001
                break

        Rs_est = max(0.0001, Rs_est)
        err = abs(Rs_est - R) / R * 100 if R > 0 else 0
        self.identified["Rs"] = Rs_est
        return {"status": "ok", "Rs_est": Rs_est, "Rs_true": R, "error_pct": err,
                "Rs_raw": Rs_raw, "dV": dV, "dI": dI, "corr_factor": Rs_est/Rs_raw if Rs_raw > 0 else 1,
                "t_s": t, "Ld_used": Ld_known}

    # ═══════ Ld/Lq 辨识 (高频注入 + 阶跃响应) ═══════
    def identify_ld_lq(self, Vd_pulse: float = 30.0, pulse_steps: int = 200) -> dict:
        """Ld 辨识: Vd 阶跃 → dI/dt → Ld = Vd * dt / dI"""
        R = self.cfg.Rs
        self.ctrl.op_mode = 3
        self.ctrl.Vd_ref = 0.0
        self.ctrl.Vq_ref = 0.0

        # 先让系统稳定在零电流
        for _ in range(200):
            self.ctrl.run_cycle()
        Id0 = self.motor.state.Id

        # 施加 Vd 阶跃
        self.ctrl.Vd_ref = Vd_pulse
        Id_samples = []
        for i in range(pulse_steps):
            self.ctrl.run_cycle()
            Id_samples.append(self.motor.state.Id)

        self.ctrl.Vd_ref = 0.0
        Id_final = Id_samples[-1]
        dI = Id_final - Id0

        if abs(dI) < 0.001:
            return {"status": "failed", "reason": "No current response"}

        # L = V * dt / dI (忽略 R 因为时间很短)
        dt = pulse_steps * self.cfg.T_sample
        Ld_est = Vd_pulse * dt / dI - R * dt  # 减掉电阻压降
        Ld_est = max(0.0001, Ld_est)
        Ld_true = getattr(self.cfg, 'Ld', 0)
        err = abs(Ld_est - Ld_true) / Ld_true * 100 if Ld_true > 0 else 0

        self.identified["Ld"] = Ld_est
        # Lq ≈ Ld 作为初值
        self.identified["Lq"] = Ld_est * (0.8 if hasattr(self.cfg, 'Lq') else 1.0)
        return {"status": "ok", "Ld_est": Ld_est, "Ld_true": Ld_true,
                "error_pct": err, "Vd_pulse": Vd_pulse, "dI": dI}

    # ═══════ PI 自整定 ═══════
    def auto_tune_pi(self, bw: float = 1000.0) -> dict:
        """基于辨识参数自动计算 PI 增益"""
        Rs = self.identified.get("Rs", self.cfg.Rs)
        Ld = self.identified.get("Ld", getattr(self.cfg, 'Ld', 0.01))
        Lq = self.identified.get("Lq", getattr(self.cfg, 'Lq', 0.01))

        Kp_d = Ld * bw
        Ki_d = Rs * bw
        Kp_q = Lq * bw
        Ki_q = Rs * bw

        # 速度环 (外环)
        J = self.cfg.J
        P = self.cfg.P
        psi_m = getattr(self.cfg, 'psi_m', 0.1) or 0.1
        spd_bw = bw / 20  # 速度环带宽 = 电流环 / 20
        Kp_spd = J * spd_bw / (1.5 * P * psi_m)
        Ki_spd = Kp_spd * spd_bw / 4

        result = {
            "status": "ok",
            "current_loop": {"Kp_d": Kp_d, "Ki_d": Ki_d, "Kp_q": Kp_q, "Ki_q": Ki_q},
            "speed_loop": {"Kp": Kp_spd, "Ki": Ki_spd},
            "bw_current": bw, "bw_speed": spd_bw
        }
        self.identified["pi_gains"] = result
        return result

    # ═══════ 故障诊断 ═══════
    def diagnose(self) -> dict:
        """当前状态诊断"""
        s = self.motor.state
        issues = []

        if s.fault != FaultType.NONE:
            issues.append({"severity": "critical", "type": s.fault.name,
                          "action": "STOP + inspect hardware"})

        if s.temp > 120:
            issues.append({"severity": "warning", "type": "overtemp",
                          "value": s.temp, "action": "Reduce current or stop"})

        Vdc_nom = self.cfg.Vdc_nom
        if s.Vdc < Vdc_nom * 0.5:
            issues.append({"severity": "warning", "type": "low_vdc",
                          "value": s.Vdc, "action": "Check DC supply"})
        elif s.Vdc > Vdc_nom * 1.15:
            issues.append({"severity": "warning", "type": "high_vdc",
                          "value": s.Vdc, "action": "Enable brake resistor"})

        I_mag = math.sqrt(s.Id**2 + s.Iq**2)
        if I_mag > self.cfg.I_max * 0.95:
            issues.append({"severity": "warning", "type": "high_current",
                          "value": I_mag, "action": "Reduce Iq_ref"})

        return {"status": "ok" if not issues else "issues_found",
                "issues": issues, "state": self.ctrl.state.name,
                "Vdc": s.Vdc, "omega": s.omega_m, "temp": s.temp}

    # ═══════ 完整调试序列 ═══════
    def build_full_sequence(self):
        """构建完整调试序列 (Phase A: 每步需人确认)
        
        修复: identify_rs 增加前一步成功检查，避免用失败数据继续
        """
        self.sequence = [
            CommissionStep("precharge", "预充直流母线到 90% 额定电压",
                          lambda: self.precharge(0.9),
                          lambda: self.motor.state.fault == FaultType.NONE),

            CommissionStep("identify_ld", "Vd 阶跃响应辨识 d 轴电感 Ld (斜率法)",
                          lambda: self.identify_ld_lq(),
                          lambda: self.motor.state.Vdc > self.cfg.Vdc_nom * 0.8),

            CommissionStep("identify_rs", "两点差分+瞬态修正辨识 Rs (依赖 Ld)",
                          lambda: self.identify_rs(),
                          lambda: ("Ld" in self.identified
                                   and self.sequence[1].status == "done")),

            CommissionStep("auto_tune", "基于辨识结果自动计算 PI 增益",
                          lambda: self.auto_tune_pi(),
                          lambda: ("Ld" in self.identified
                                   and self.sequence[1].status == "done"
                                   and "Rs" in self.identified)),

            CommissionStep("verify", "诊断当前状态, 检查异常",
                          lambda: self.diagnose(),
                          lambda: True),
        ]
        self.current_step = 0

    def run_next_step(self) -> Optional[dict]:
        """执行下一个工步, 返回结果"""
        if self.current_step >= len(self.sequence):
            return None
        step = self.sequence[self.current_step]

        # 安全检查
        if step.safety_check and not step.safety_check():
            step.status = "skipped"
            self.current_step += 1
            return {"status": "skipped", "reason": "safety check failed"}

        result = step.run()
        self.current_step += 1
        return result

    def run_full_auto(self) -> list:
        """全自动执行 (Phase C)"""
        self.build_full_sequence()
        results = []
        while True:
            r = self.run_next_step()
            if r is None:
                break
            results.append(r)
            if r.get("status") == "failed" or r.get("status") == "fault":
                break
        return results

    def get_summary(self) -> dict:
        """调试总结"""
        return {
            "motor": self.cfg.__class__.__name__,
            "identified": self.identified,
            "steps_completed": sum(1 for s in self.sequence if s.status == "done"),
            "steps_total": len(self.sequence),
            "faults": len([s for s in self.sequence if s.status in ("failed", "skipped")]),
        }


# ═══════ S2: 安全网关 (简化仿真版) ═══════
class SafetyGateway:
    """S2 安全通信网关 — 白名单 + 速率限制 + 值范围校验"""

    WHITELIST_READ = {"Vdc", "Id", "Iq", "omega_m", "temp", "Te", "theta_e", "fault"}
    WHITELIST_WRITE = {"Vd_ref", "Vq_ref", "Id_ref", "Iq_ref", "speed_ref", "op_mode"}
    RATE_LIMIT_MS = 50       # 最小写入间隔
    SPEED_MAX_RPM = 10000
    CURRENT_MAX = 200.0      # A
    VOLTAGE_MAX = 400.0      # V

    def __init__(self, controller: VirtualController):
        self.ctrl = controller
        self._last_write = 0.0
        self._write_count = 0
        self._rejected = 0

    def check_read(self, var: str) -> bool:
        return var in self.WHITELIST_READ

    def check_write(self, var: str, value: float) -> tuple[bool, str]:
        """返回 (通过, 拒绝原因)"""
        if var not in self.WHITELIST_WRITE:
            return False, f"{var} not in write whitelist"

        # 速率限制
        now = __import__('time').time()
        if now - self._last_write < self.RATE_LIMIT_MS / 1000:
            self._rejected += 1
            return False, "rate limit exceeded"

        # 值范围
        if var == "speed_ref" and (value < 0 or value > self.SPEED_MAX_RPM):
            return False, f"speed_ref {value} out of [0, {self.SPEED_MAX_RPM}]"
        if var in ("Id_ref", "Iq_ref") and abs(value) > self.CURRENT_MAX:
            return False, f"{var} {value} > {self.CURRENT_MAX}A"
        if var in ("Vd_ref", "Vq_ref") and abs(value) > self.VOLTAGE_MAX:
            return False, f"{var} {value} > {self.VOLTAGE_MAX}V"

        self._last_write = now
        self._write_count += 1
        return True, "ok"

    def safe_write(self, var: str, value: float) -> bool:
        ok, reason = self.check_write(var, value)
        if ok:
            setattr(self.ctrl, var, value)
        return ok

    def stats(self) -> dict:
        return {"writes": self._write_count, "rejected": self._rejected}
