"""
s4_ai_controller.py — AI 电网调度策略: 规则/自适应/预测模式, 8维观测→6动作决策。

子系统: 应用层 (S4)
依赖: power_ops.py
手册对应章节: STATE_MACHINE.md §2-3

AI 电网调度策略: 规则/自适应/预测模式, 8维观测→6动作决策。
"""
import math
from dataclasses import dataclass
from typing import Optional
from enum import Enum

from power_ops import PowerMode


class AIStrategy(Enum):
    RULE_BASED = "rule_based"
    ADAPTIVE = "adaptive"
    PREDICTIVE = "predictive"


@dataclass
class GridSnapshot:
    """电网状态快照 — AI 决策输入"""
    f: float = 50.0         # 频率 Hz
    df: float = 0.0         # 频率偏差 Hz
    dfdt: float = 0.0       # RoCoF Hz/s
    Vdc: float = 600.0      # 母线电压 V
    dVdc: float = 0.0       # 母线电压偏差 V
    SOC: float = 0.5        # 飞轮 SOC
    dSOC: float = 0.0       # SOC 变化率 /s
    P_grid: float = 0.0     # 电网功率 W
    P_load: float = 0.0     # 负载功率 W
    f_pll: float = 50.0     # PLL 频率
    Vg_amplitude: float = 311.0  # 电网电压幅值
    mode: PowerMode = PowerMode.IDLE


@dataclass
class AIControlAction:
    """AI 控制决策"""
    mode: PowerMode = PowerMode.IDLE
    P_ref: float = 0.0          # 功率参考 W
    # 自适应参数
    droop: float = 0.04
    deadband: float = 0.02
    dc_mode: str = 'ff_pi'
    # 决策元数据
    reason: str = ""
    confidence: float = 1.0


class S4AIController:
    """S4 AI 控制器 — 功率编排层的智能大脑

    比固定模式控制器更聪明:
    - 看到 df/dt 提前响应 (不用等频率偏差超过死区)
    - SOC 低时自动降额而非硬切
    - 复合扰动时自动切混合模式
    """

    def __init__(self, strategy: AIStrategy = AIStrategy.ADAPTIVE):
        self.strategy = strategy
        self._prev_snapshot: Optional[GridSnapshot] = None
        self._action_history = []
        self._mode_switch_cooldown = 0  # 模式切换冷却计数器

        # 自适应参数范围
        self.droop_min = 0.02
        self.droop_max = 0.08
        self.deadband_min = 0.01
        self.deadband_max = 0.05

        # RoCoF 阈值
        self.rocot_warn = 0.2    # Hz/s 告警
        self.rocot_action = 0.5  # Hz/s 动作

    def decide(self, snapshot: GridSnapshot) -> AIControlAction:
        """核心决策: 根据电网状态选择控制动作"""
        if self.strategy == AIStrategy.RULE_BASED:
            return self._decide_rules(snapshot)
        elif self.strategy == AIStrategy.ADAPTIVE:
            return self._decide_adaptive(snapshot)
        elif self.strategy == AIStrategy.PREDICTIVE:
            return self._decide_predictive(snapshot)
        return AIControlAction(reason="unknown strategy")

    def _decide_rules(self, s: GridSnapshot) -> AIControlAction:
        """专家规则引擎 — 优先级: Vdc保护 > 频率+Vdc复合 > 频率 > Vdc > SOC"""

        freq_event = abs(s.dfdt) > self.rocot_action or abs(s.df) > 0.15
        vdc_event = abs(s.dVdc) > 10
        vdc_critical = abs(s.dVdc) > 50

        # 规则 0: 紧急 Vdc 保护 (最高优先级, 任何模式下触发)
        if vdc_critical:
            return AIControlAction(
                mode=PowerMode.DC_BUS_CONTROL,
                dc_mode='deadbeat',
                reason=f"⚠ Vdc 紧急 {s.dVdc:.0f}V → Deadbeat 稳压 (覆盖其他模式)",
            )

        # 规则 1: 频率 + Vdc 复合 → 惯性支撑 (同时调频+稳压)
        if freq_event and vdc_event:
            if s.SOC < 0.08 and s.df > 0:
                return AIControlAction(
                    mode=PowerMode.DC_BUS_CONTROL,
                    dc_mode='ff_pi',
                    reason=f"SOC={s.SOC:.1%} 极低, 放弃调频→稳压优先",
                )
            return AIControlAction(
                mode=PowerMode.INERTIA_SUPPORT,
                droop=0.04,
                dc_mode='ff_pi',
                reason=f"复合: df={s.df:.3f}Hz + ΔVdc={s.dVdc:.0f}V → 惯性支撑",
                confidence=0.85,
            )

        # 规则 2: 纯频率事件
        if freq_event:
            if s.SOC < 0.1 and s.df > 0:
                return AIControlAction(
                    mode=PowerMode.IDLE,
                    reason=f"频率偏低但 SOC={s.SOC:.1%} 不足，待机",
                )
            return AIControlAction(
                mode=PowerMode.FREQ_REGULATION,
                droop=0.04,
                reason=f"调频: df={s.df:.3f}Hz, RoCoF={s.dfdt:.3f}Hz/s",
                confidence=0.9,
            )

        # 规则 3: 纯 Vdc 偏离
        if vdc_event:
            return AIControlAction(
                mode=PowerMode.DC_BUS_CONTROL,
                dc_mode='ff_pi',
                reason=f"Vdc 偏差 {s.dVdc:.0f}V → FF+PI 稳压",
            )

        # 规则 4: SOC 太高/太低 → 充放电管理
        if s.SOC > 0.9:
            return AIControlAction(
                mode=PowerMode.POWER_TRACKING,
                P_ref=-500e3,
                reason=f"SOC={s.SOC:.1%} 过高 → 放电",
            )
        if s.SOC < 0.15:
            return AIControlAction(
                mode=PowerMode.POWER_TRACKING,
                P_ref=300e3,
                reason=f"SOC={s.SOC:.1%} 过低 → 充电",
            )

        # 默认: 待机
        return AIControlAction(mode=PowerMode.IDLE, reason="无事件, 待机")

    def _decide_adaptive(self, s: GridSnapshot) -> AIControlAction:
        """自适应策略 — 动态调参 + 模式选择"""

        action = self._decide_rules(s)  # 先用规则选模式

        # ── 自适应 droop ──
        if action.mode == PowerMode.FREQ_REGULATION:
            # SOC 低 → 增大 droop (少出力, 保护 SOC)
            if s.SOC < 0.3:
                action.droop = self.droop_min + (self.droop_max - self.droop_min) * (1 - s.SOC / 0.3)
                action.reason += f", droop→{action.droop:.3f} (SOC 保护)"
            # RoCoF 大 → 减小 droop (更激进响应)
            elif abs(s.dfdt) > 0.3:
                action.droop = max(self.droop_min, 0.04 * 0.5)
                action.reason += f", droop→{action.droop:.3f} (RoCoF 激进)"

        # ── 自适应 deadband ──
        if abs(s.df) < 0.05 and abs(s.dfdt) < 0.1:
            action.deadband = self.deadband_max  # 安静 → 宽死区, 防抖
        else:
            action.deadband = self.deadband_min  # 活跃 → 窄死区, 灵敏

        # ── 自适应 DC 算法 ──
        if action.mode == PowerMode.DC_BUS_CONTROL:
            if abs(s.dVdc) > 30:
                action.dc_mode = 'deadbeat'
            elif abs(s.dVdc) > 10:
                action.dc_mode = 'adaptive'
            else:
                action.dc_mode = 'ff_pi'

        # ── 混合模式检测 ──
        # 频率事件 + Vdc 偏离 → 惯性支撑模式
        if abs(s.df) > 0.1 and abs(s.dVdc) > 20:
            action.mode = PowerMode.INERTIA_SUPPORT
            action.reason = f"复合: df={s.df:.3f} + dVdc={s.dVdc:.0f} → 惯性支撑"

        self._prev_snapshot = s
        return action

    def _decide_predictive(self, s: GridSnapshot) -> AIControlAction:
        """预测策略 — 基于 RoCoF 提前动作 (比规则快 50-200ms)"""

        action = self._decide_adaptive(s)

        # ── 预测性动作 ──
        # RoCoF 正在恶化 → 不等频率跌出死区就先动
        if abs(s.df) < 0.05 and abs(s.dfdt) > self.rocot_warn:
            # 频率还没掉太多但变化率大 — 预判!
            predicted_df = s.dfdt * 0.5  # 预测 500ms 后
            if abs(predicted_df) > 0.1:
                action.mode = PowerMode.FREQ_REGULATION
                action.droop = 0.03  # 激进
                action.confidence = 0.7
                action.reason = f"预测: df(500ms)≈{s.df + predicted_df:.3f}Hz, 提前调频"
                return action

        # ── 预测性 SOC 约束 ──
        if s.SOC < 0.25 and s.df > 0.02 and s.dfdt > 0:
            # SOC 低 + 频率偏低 + 还在恶化 → 预留能量
            action.mode = PowerMode.IDLE
            action.reason = f"SOC={s.SOC:.1%} 低 + 频率恶化中 → 预留能量"
            action.confidence = 0.8

        return action

    def get_strategy_name(self) -> str:
        names = {
            AIStrategy.RULE_BASED: "规则引擎",
            AIStrategy.ADAPTIVE: "自适应增益",
            AIStrategy.PREDICTIVE: "预测+规则",
        }
        return names.get(self.strategy, self.strategy.value)

    def reset(self):
        self._prev_snapshot = None
        self._action_history.clear()
        self._mode_switch_cooldown = 0
