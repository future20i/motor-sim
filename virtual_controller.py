"""
virtual_controller.py — CODESYS 仿真替代: 六状态机 (待机/预充/就绪/辨识/运行/故障) + 电流环。

子系统: 控制层 (S3)
依赖: motor_base.py, control_algorithms.py

CODESYS 仿真替代: 六状态机 (待机/预充/就绪/辨识/运行/故障) + 电流环。
"""
import numpy as np
from enum import Enum
from motor_base import MotorBase, MotorConfig, FaultType


class OpState(Enum):
    IDLE = 0
    PRECHARGE = 1
    READY = 2
    IDENTIFY = 3
    RUN = 4
    FAULT = 5


class VirtualController:
    """虚拟控制器 — 和 CODESYS 状态机一字不差"""

    def __init__(self, motor: MotorBase):
        self.motor = motor
        self.cfg = motor.cfg
        self.state = OpState.IDLE
        self.ts = self.cfg.T_sample

        # === 电流环 PI (自动根据电机类型调谐) ===
        self._tune_pi()

        self._integral_d = 0.0
        self._integral_q = 0.0

        # === 指令值 (来自 OPC UA / AI) ===
        self.Vd_ref = 0.0
        self.Vq_ref = 0.0
        self.Id_ref = 0.0
        self.Iq_ref = 0.0
        self.speed_ref = 0.0
        self.op_mode = 0

        # === 统计 ===
        self.cycle_count = 0
        self._precharge_progress = 0.0

    def _tune_pi(self):
        """自动 PI 调谐 — 根据电机类型选电感"""
        bw = 1000  # 电流环带宽 [rad/s]

        # 用电机类型的特征电感
        if hasattr(self.cfg, 'Ld'):
            L = self.cfg.Ld  # PMSM, SynRM, PM-SynRM
        elif hasattr(self.cfg, 'Ls'):
            L = self.cfg.Ls  # 异步电机
        else:
            L = 0.01  # 回退

        self.Kp_d = L * bw
        self.Ki_d = self.cfg.Rs * bw
        self.Kp_q = L * bw
        self.Ki_q = self.cfg.Rs * bw

    def run_cycle(self) -> dict:
        """跑一个控制周期"""
        self.cycle_count += 1
        ms = self.motor.state

        # ── 状态机 ──
        self._tick_state_machine()

        # ── 安全 ──
        if ms.fault != FaultType.NONE:
            self.state = OpState.FAULT
            self.Vd_ref = 0.0
            self.Vq_ref = 0.0
            self.speed_ref = 0.0

        # ── 控制输出 ──
        Vd_out, Vq_out = 0.0, 0.0

        if self.state == OpState.RUN:
            Vd_out, Vq_out = self._current_pi()
        elif self.state in (OpState.IDLE, OpState.READY, OpState.IDENTIFY):
            Vd_out, Vq_out = self.Vd_ref, self.Vq_ref
        elif self.state == OpState.PRECHARGE:
            # 真实预充 ~2s: 50μs/2s = 0.000025 per step → 40000步到满压
            self._precharge_progress += self.ts / 2.0
            ms.Vdc = min(self.cfg.Vdc_nom, self._precharge_progress * self.cfg.Vdc_nom)

        # ── 电压限幅 ──
        V_max = max(ms.Vdc, 10) / np.sqrt(3)
        V_mag = np.sqrt(Vd_out**2 + Vq_out**2)
        if V_mag > V_max:
            scale = V_max / V_mag
            Vd_out *= scale
            Vq_out *= scale

        # ── 推给电机 ──
        self.motor.step(Vd_out, Vq_out)

        # ── 速度环 ──
        if self.state == OpState.RUN and self.speed_ref > 0:
            speed_rps = self.speed_ref * 2 * np.pi / 60
            speed_err = speed_rps - ms.omega_m
            self.Iq_ref = np.clip(10 * speed_err, -self.cfg.I_max, self.cfg.I_max)

        return self.motor.get_readable()

    def _current_pi(self) -> tuple:
        ms = self.motor.state
        d_err = self.Id_ref - ms.Id
        q_err = self.Iq_ref - ms.Iq

        self._integral_d += d_err * self.ts
        self._integral_q += q_err * self.ts
        self._integral_d = np.clip(self._integral_d, -100, 100)
        self._integral_q = np.clip(self._integral_q, -100, 100)

        Vd = self.Kp_d * d_err + self.Ki_d * self._integral_d
        Vq = self.Kp_q * q_err + self.Ki_q * self._integral_q
        return Vd, Vq

    def _tick_state_machine(self):
        ms = self.motor.state

        if self.state == OpState.IDLE:
            if self.op_mode == 1:
                self.state = OpState.PRECHARGE
                self._precharge_progress = 0.0

        elif self.state == OpState.PRECHARGE:
            if ms.Vdc >= 0.9 * self.cfg.Vdc_nom:
                self.state = OpState.READY

        elif self.state == OpState.READY:
            if self.op_mode == 3:
                self.state = OpState.IDENTIFY
            elif self.speed_ref > 0:
                self.state = OpState.RUN

        elif self.state == OpState.IDENTIFY:
            if self.op_mode != 3:
                self.state = OpState.READY

        elif self.state == OpState.RUN:
            if self.speed_ref == 0:
                self.state = OpState.READY

    def get_op_state(self) -> dict:
        return {
            "op_state": self.state.value,
            "op_state_name": self.state.name,
            "Vdc_actual": self.motor.state.Vdc,
            "fault_code": self.motor.state.fault.value,
            "cycle": self.cycle_count,
        }
