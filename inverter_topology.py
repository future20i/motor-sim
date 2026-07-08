"""
inverter_topology.py — 3种逆变器拓扑: 两电平VSI / 三电平NPC / 三电平T-type。统一接口: modulate(Vα,Vβ,Vdc)。

子系统: 物理模型
依赖: 无
手册对应章节: ARCHITECTURE.md §1.1

3种逆变器拓扑: 两电平VSI / 三电平NPC / 三电平T-type。统一接口: modulate(Vα,Vβ,Vdc)。
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Dict
import numpy as np
import math


# ═══════════════════════════════
# 基础数据结构
# ═══════════════════════════════

@dataclass
class SwitchState:
    """一个开关状态"""
    name: str                          # "100" / "210" 等
    states: Tuple[int, ...]            # (Sa, Sb, Sc), 每相电平索引
    V_alpha: float = 0.0               # α轴电压 [V] (以 Vdc 归一化)
    V_beta: float = 0.0                # β轴电压 [V]
    is_redundant: bool = False         # 是否冗余矢量 (3L 特有)
    i_np: float = 0.0                  # 中点电流系数 (3L 特有)


@dataclass
class ModulationResult:
    """调制输出"""
    state_a: int                       # A相电平 (0/1 或 0/1/2)
    state_b: int                       # B相电平
    state_c: int                       # C相电平
    duty_a: float                      # A相占空比
    duty_b: float
    duty_c: float
    gate_signals: np.ndarray           # 门极信号 (n_gates,)
    V_alpha_out: float                 # 实际输出的 α 电压
    V_beta_out: float


@dataclass
class DCLinkState:
    """直流母线电容状态 (3L)"""
    Vdc_total: float = 600.0           # 总电压
    Vdc_upper: float = 300.0           # 上电容电压
    Vdc_lower: float = 300.0           # 下电容电压
    C_upper: float = 1000e-6           # 上电容容值
    C_lower: float = 1000e-6           # 下电容容值
    imbalance: float = 0.0             # 中点偏移 [V]


# ═══════════════════════════════
# 基类
# ═══════════════════════════════

class TopologyBase(ABC):
    """逆变器拓扑基类"""

    @abstractmethod
    def num_levels(self) -> int:
        """电平数"""
        ...

    @abstractmethod
    def num_gates(self) -> int:
        """门极信号数 (6 for 2L, 12 for 3L)"""
        ...

    @abstractmethod
    def all_vectors(self) -> List[SwitchState]:
        """返回所有开关状态和对应矢量"""
        ...

    @abstractmethod
    def modulate(self, V_alpha: float, V_beta: float, Vdc: float,
                 dc_link: DCLinkState = None) -> ModulationResult:
        """SVPWM 调制"""
        ...

    def gate_signals(self, state_a: int, state_b: int, state_c: int) -> np.ndarray:
        """开关状态 → 门极信号"""
        raise NotImplementedError

    def dead_time_voltage_error(self, Ia: float, Ib: float, Ic: float,
                                 T_dead: float = 2e-6, Ts: float = 50e-6) -> Tuple[float, float, float]:
        """死区造成的电压误差 [V]"""
        sign = lambda x: 1 if x > 0 else (-1 if x < 0 else 0)
        V_err = T_dead / Ts  # 占空比误差
        return (
            -V_err * sign(Ia),
            -V_err * sign(Ib),
            -V_err * sign(Ic),
        )

    @staticmethod
    def clarke_transform(Ia: float, Ib: float, Ic: float) -> Tuple[float, float]:
        """Clarke 变换: abc → αβ"""
        alpha = Ia
        beta = (Ia + 2*Ib) / math.sqrt(3)
        return alpha, beta

    @staticmethod
    def park_transform(alpha: float, beta: float, theta: float) -> Tuple[float, float]:
        """Park 变换: αβ → dq"""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        d =  cos_t * alpha + sin_t * beta
        q = -sin_t * alpha + cos_t * beta
        return d, q

    @staticmethod
    def inv_park_transform(d: float, q: float, theta: float) -> Tuple[float, float]:
        """逆 Park 变换: dq → αβ"""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        alpha = cos_t * d - sin_t * q
        beta  = sin_t * d + cos_t * q
        return alpha, beta


# ═══════════════════════════════
# 两电平 VSI
# ═══════════════════════════════

class TwoLevelVSI(TopologyBase):
    """两电平电压型逆变器"""

    def num_levels(self): return 2
    def num_gates(self): return 6

    def all_vectors(self) -> List[SwitchState]:
        vectors = []
        for sa in (0, 1):
            for sb in (0, 1):
                for sc in (0, 1):
                    name = f"{sa}{sb}{sc}"
                    # 相电压 (相对于 DC 中点)
                    Va = (2*sa - 1) / 2  # -0.5 or +0.5
                    Vb = (2*sb - 1) / 2
                    Vc = (2*sc - 1) / 2
                    # Clarke → αβ
                    alpha, beta = self.clarke_transform(Va, Vb, Vc)
                    vectors.append(SwitchState(
                        name=name, states=(sa, sb, sc),
                        V_alpha=alpha, V_beta=beta
                    ))
        return vectors

    def modulate(self, V_alpha: float, V_beta: float, Vdc: float,
                 dc_link: DCLinkState = None) -> ModulationResult:
        """七段式 SVPWM"""
        # 归一化到 [-1, 1] (调制比)
        V_alpha_n = V_alpha / Vdc
        V_beta_n = V_beta / Vdc

        # 扇区判断
        theta = math.atan2(V_beta_n, V_alpha_n)
        if theta < 0:
            theta += 2 * np.pi
        sector = int(theta / (np.pi / 3)) % 6

        # 矢量作用时间 (简化: 纯正弦调制，不做 SVPWM 完整计算)
        # 实际用 SPWM 等效: 占空比 = (V_phase + 0.5)
        Va_ref = V_alpha_n
        Vb_ref = -0.5 * V_alpha_n + np.sqrt(3)/2 * V_beta_n
        Vc_ref = -0.5 * V_alpha_n - np.sqrt(3)/2 * V_beta_n

        # 三次谐波注入提高利用率
        V_offset = -(max(Va_ref, Vb_ref, Vc_ref) + min(Va_ref, Vb_ref, Vc_ref)) / 2
        duty_a = max(0, min(1, Va_ref + V_offset + 0.5))
        duty_b = max(0, min(1, Vb_ref + V_offset + 0.5))
        duty_c = max(0, min(1, Vc_ref + V_offset + 0.5))

        # 门极信号: [AH, AL, BH, BL, CH, CL]
        gates = np.array([
            1 if duty_a > 0 else 0, 1 if duty_a < 1 else 0,
            1 if duty_b > 0 else 0, 1 if duty_b < 1 else 0,
            1 if duty_c > 0 else 0, 1 if duty_c < 1 else 0,
        ], dtype=int)

        return ModulationResult(
            state_a=1 if duty_a > 0.5 else 0,
            state_b=1 if duty_b > 0.5 else 0,
            state_c=1 if duty_c > 0.5 else 0,
            duty_a=duty_a, duty_b=duty_b, duty_c=duty_c,
            gate_signals=gates,
            V_alpha_out=V_alpha, V_beta_out=V_beta
        )

    def gate_signals(self, state_a, state_b, state_c):
        return np.array([
            1-state_a, state_a,   # AH, AL (互补，加死区)
            1-state_b, state_b,
            1-state_c, state_c,
        ], dtype=int)


# ═══════════════════════════════
# 三电平 NPC
# ═══════════════════════════════

class ThreeLevelNPC(TopologyBase):
    """三电平中点箝位逆变器"""

    def __init__(self):
        # 预计算所有 27 个矢量
        self._vectors = self._build_vectors()

    def num_levels(self): return 3
    def num_gates(self): return 12

    def _build_vectors(self) -> List[SwitchState]:
        vectors = []
        for sa in (0, 1, 2):  # 0: -Vdc/2, 1: 0, 2: +Vdc/2
            for sb in (0, 1, 2):
                for sc in (0, 1, 2):
                    name = f"{sa}{sb}{sc}"
                    # 相电压 (相对于 DC 中点), 归一化
                    Va = (sa - 1) / 2  # -0.5, 0, +0.5
                    Vb = (sb - 1) / 2
                    Vc = (sc - 1) / 2
                    alpha, beta = self.clarke_transform(Va, Vb, Vc)

                    # 中点电流系数: i_np = (1-|sa-1|)*Ia + ...
                    i_np = ((1 - abs(sa - 1)) + (1 - abs(sb - 1)) + (1 - abs(sc - 1))) / 3

                    vectors.append(SwitchState(
                        name=name, states=(sa, sb, sc),
                        V_alpha=alpha, V_beta=beta,
                        is_redundant=name in self._redundant_set(),
                        i_np=i_np
                    ))
        return vectors

    def _redundant_set(self):
        """冗余矢量集 — 同一位置有多条路径可达"""
        return {"110", "001", "101", "010", "011", "100", "000", "111", "222"}

    def all_vectors(self): return self._vectors

    def _select_redundant(self, V_alpha, V_beta, dc_link: DCLinkState,
                          candidates: List[SwitchState]) -> SwitchState:
        """从冗余矢量中选出平衡中点的那个"""
        imbalance = dc_link.imbalance
        best = candidates[0]
        best_cost = float('inf')
        for v in candidates:
            # 选中点电流系数和当前不平衡方向相反的矢量
            cost = abs(v.i_np - (-imbalance * 0.001))
            if cost < best_cost:
                best_cost = cost
                best = v
        return best

    def modulate(self, V_alpha: float, V_beta: float, Vdc: float,
                 dc_link: DCLinkState = None) -> ModulationResult:
        """三电平 SVPWM + 中点平衡"""
        if dc_link is None:
            dc_link = DCLinkState(Vdc_total=Vdc)

        # 归一化
        V_alpha_n = V_alpha / Vdc
        V_beta_n = V_beta / Vdc

        # 找最近的三个矢量 (简化: 用最近邻搜索)
        # 实际实现用扇区判断+矢量表查表
        candidates = []
        min_dist = float('inf')
        for v in self._vectors:
            # 跳过不可达矢量 (超过最大调制比)
            mag = math.sqrt(v.V_alpha**2 + v.V_beta**2)
            if mag > 0.58:  # 三电平最大线性调制比 ≈ 0.577
                continue
            dist = (v.V_alpha - V_alpha_n)**2 + (v.V_beta - V_beta_n)**2
            if dist < min_dist:
                min_dist = dist
                candidates = [v]
            elif dist == min_dist:
                candidates.append(v)

        # 中点平衡选择
        chosen = self._select_redundant(V_alpha_n, V_beta_n, dc_link, candidates)

        sa, sb, sc = chosen.states

        # 门极: 每相 4 个开关 (S1,S2,S3,S4)
        def phase_gates(state):
            if state == 2:  return [1, 1, 0, 0]  # +Vdc/2
            elif state == 1: return [0, 1, 1, 0]  # 0
            else:            return [0, 0, 1, 1]  # -Vdc/2

        gates = np.array(phase_gates(sa) + phase_gates(sb) + phase_gates(sc), dtype=int)

        return ModulationResult(
            state_a=sa, state_b=sb, state_c=sc,
            duty_a=sa/2, duty_b=sb/2, duty_c=sc/2,
            gate_signals=gates,
            V_alpha_out=chosen.V_alpha * Vdc,
            V_beta_out=chosen.V_beta * Vdc
        )

    def update_dc_link(self, state_a, state_b, state_c,
                        Ia, Ib, Ic, dc_link: DCLinkState, dt: float):
        """更新母线电容电压"""
        # 中点电流
        i_np = sum(
            (1 - abs(s - 1)) * i_phase
            for s, i_phase in zip([state_a, state_b, state_c], [Ia, Ib, Ic])
        )
        # 电容电压更新
        dc_link.Vdc_upper += (-i_np / dc_link.C_upper) * dt
        dc_link.Vdc_lower += (i_np / dc_link.C_lower) * dt
        dc_link.Vdc_total = dc_link.Vdc_upper + dc_link.Vdc_lower
        dc_link.imbalance = dc_link.Vdc_upper - dc_link.Vdc_lower

    def gate_signals(self, state_a, state_b, state_c):
        def g(s):
            if s == 2: return [1, 1, 0, 0]
            elif s == 1: return [0, 1, 1, 0]
            else: return [0, 0, 1, 1]
        return np.array(g(state_a) + g(state_b) + g(state_c), dtype=int)


class ThreeLevelTNPC(ThreeLevelNPC):
    """三电平 T-type — 开关矢量同 NPC, 器件损耗模型不同"""

    def num_levels(self): return 3
    def num_gates(self): return 12

    def type_name(self): return "3L-TNPC (T-type)"
