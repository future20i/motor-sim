"""
s4_metrics.py — v2

子系统: 应用层 (S4)
依赖: power_ops.py

v2.0 评分体系: 硬约束 + 帕累托前沿 + 蒙特卡洛验证。
"""
import math
import random
import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Set
from collections import defaultdict

from s4_scenarios import ExperimentMetrics, ScenarioConfig, ScenarioID
from s4_ai_controller import AIStrategy


# ═══════════════════════════════════════════
# 0. 统一评分函数 — 全局唯一, 消除重复
# ═══════════════════════════════════════════

def compute_all_scores(m: ExperimentMetrics, s: ScenarioConfig,
                       max_df_observed: float = 0.0) -> dict:
    """统一评分函数 — s4_experiment_runner 和 s4_metrics 共用

    返回: {"vdc", "freq", "soc", "eff", "composite"} 各维度 0-100 分
    """
    FREQ_THRESHOLD = 0.06  # Hz — 低于此视为无频率事件

    # Vdc 评分: 偏差越大分越低
    vdc_score = max(0.0, 100.0 - m.vdc_max_dev * 2) if m.vdc_max_dev > 0 else 100.0

    # 频率响应评分
    if m.freq_response_t > 0:
        freq_score = 100.0 * math.exp(-m.freq_response_t / 600.0)
    elif max_df_observed < FREQ_THRESHOLD:
        freq_score = 100.0  # 无频率事件, 不响应=正确
    else:
        severity = max_df_observed - FREQ_THRESHOLD
        freq_score = max(0.0, 35.0 - severity * 300)

    # SOC 评分
    soc_score = max(0.0, 100.0 - m.soc_violations * 20)

    # 效率评分
    eff_score = m.efficiency * 100.0

    # 加权综合
    w_sum = max(s.weight_vdc_reg + s.weight_freq_support +
                s.weight_soc_mgmt + s.weight_efficiency, 1e-6)
    composite = (
        s.weight_vdc_reg      * vdc_score +
        s.weight_freq_support * freq_score +
        s.weight_soc_mgmt     * soc_score +
        s.weight_efficiency   * eff_score
    ) / w_sum

    return {
        "vdc": vdc_score,
        "freq": freq_score,
        "soc": soc_score,
        "eff": eff_score,
        "composite": composite,
    }


# ═══════════════════════════════════════════
# 1. 硬约束 — 一票否决
# ═══════════════════════════════════════════

@dataclass
class Violation:
    """违反硬约束"""
    name: str           # 约束名称
    actual: float       # 实际值
    limit: float        # 限制值
    unit: str           # 单位
    severity: str       # "critical" | "warning"


@dataclass
class HardConstraintResult:
    """硬约束检查结果"""
    passed: bool = True
    violations: List[Violation] = field(default_factory=list)

    @property
    def failed(self) -> bool:
        return not self.passed

    @property
    def reason(self) -> str:
        if self.passed:
            return "✅ 通过全部硬约束"
        return "\n".join(
            f"❌ {v.name}: {v.actual:.1f}{v.unit} > {v.limit:.1f}{v.unit} [{v.severity}]"
            for v in self.violations
        )


class HardConstraints:
    """飞轮 UPS 硬约束 — 参照 IEC 62040-3 / GB/T 标准"""

    # ── 硬约束定义 ──
    CONSTRAINTS = [
        {
            "name": "母线过压保护",
            "check": lambda m: m.vdc_max_dev < 20.0,
            "actual": lambda m: m.vdc_max_dev,
            "limit": 20.0,
            "unit": "%",
            "severity": "critical",
        },
        {
            "name": "飞轮失速保护",
            "check": lambda m: m.soc_final > 0.03,
            "actual": lambda m: m.soc_final * 100,
            "limit": 3.0,
            "unit": "%",
            "severity": "critical",
        },
        {
            "name": "飞轮过功率",
            "check": lambda m: m.freq_power_peak < 750e3,  # 500kW * 1.5
            "actual": lambda m: m.freq_power_peak / 1000,
            "limit": 750.0,
            "unit": "kW",
            "severity": "critical",
        },
        {
            "name": "SOC 越限次数过多",
            "check": lambda m: m.soc_violations <= 5,
            "actual": lambda m: m.soc_violations,
            "limit": 5,
            "unit": "次",
            "severity": "warning",
        },
    ]

    # ── 各场景额外约束 ──
    PER_SCENE_CONSTRAINTS = {
        ScenarioID.GRID_FREQ_DIP: [
            {
                "name": "频率响应太慢",
                "check": lambda m: m.freq_response_t < 2000,  # 必须 2s 内响应
                "actual": lambda m: m.freq_response_t,
                "limit": 2000,
                "unit": "ms",
                "severity": "warning",
            },
        ],
        ScenarioID.VOLTAGE_SAG: [
            {
                "name": "LVRT 母线崩溃",
                "check": lambda m: m.vdc_max_dev < 35.0,
                "actual": lambda m: m.vdc_max_dev,
                "limit": 35.0,
                "unit": "%",
                "severity": "critical",
            },
        ],
        ScenarioID.LOAD_STEP_UP: [
            {
                "name": "负载突增 Vdc 跌落过大",
                "check": lambda m: m.vdc_max_dev < 15.0,
                "actual": lambda m: m.vdc_max_dev,
                "limit": 15.0,
                "unit": "%",
                "severity": "warning",
            },
        ],
    }

    @classmethod
    def check(cls, metrics: ExperimentMetrics,
              scenario_id: Optional[ScenarioID] = None) -> HardConstraintResult:
        """检查单个实验结果是否通过硬约束"""
        result = HardConstraintResult()

        # 通用约束
        for c in cls.CONSTRAINTS:
            if not c["check"](metrics):
                result.passed = False
                result.violations.append(Violation(
                    name=c["name"],
                    actual=c["actual"](metrics),
                    limit=c["limit"],
                    unit=c["unit"],
                    severity=c["severity"],
                ))

        # 场景特定约束
        if scenario_id and scenario_id in cls.PER_SCENE_CONSTRAINTS:
            for c in cls.PER_SCENE_CONSTRAINTS[scenario_id]:
                if not c["check"](metrics):
                    result.passed = False
                    result.violations.append(Violation(
                        name=c["name"],
                        actual=c["actual"](metrics),
                        limit=c["limit"],
                        unit=c["unit"],
                        severity=c["severity"],
                    ))

        return result

    @classmethod
    def filter_valid(cls, results: List["StrategyResult"],
                     scenario_id: Optional[ScenarioID] = None) -> Tuple[
                         List["StrategyResult"], List["StrategyResult"]]:
        """分离合格/不合格策略"""
        valid, invalid = [], []
        for r in results:
            v = cls.check(r.metrics, scenario_id)
            if v.passed:
                valid.append(r)
            else:
                r.violations = v
                invalid.append(r)
        return valid, invalid


# ═══════════════════════════════════════════
# 2. 帕累托前沿分析
# ═══════════════════════════════════════════

@dataclass
class StrategyResult:
    """策略在单个场景的结果（带维度分数）"""
    name: str
    metrics: ExperimentMetrics
    vdc_score: float = 0.0
    freq_score: float = 0.0
    soc_score: float = 0.0
    eff_score: float = 0.0
    composite_score: float = 0.0
    violations: Optional[HardConstraintResult] = None
    is_pareto: bool = False
    dominated_by: List[str] = field(default_factory=list)


@dataclass
class ParetoReport:
    """帕累托分析报告"""
    frontier: List[StrategyResult]       # 帕累托前沿策略
    dominated: List[StrategyResult]      # 被支配策略
    dimensions: List[str]                # 分析维度

    def summary(self) -> str:
        lines = []
        lines.append(f"\n{'='*70}")
        lines.append(f"  帕累托前沿分析 ({' × '.join(self.dimensions)})")
        lines.append(f"{'='*70}")

        lines.append(f"\n  🟢 帕累托前沿 ({len(self.frontier)} 个):")
        for r in self.frontier:
            scores = [f"{getattr(r, d+'_score', 0):.0f}" for d in self.dimensions]
            dim_str = " | ".join(f"{d}:{s}" for d, s in zip(self.dimensions, scores))
            lines.append(f"    ✓ {r.name:16s}  [{dim_str}]")

        if self.dominated:
            lines.append(f"\n  🔴 被支配 ({len(self.dominated)} 个):")
            for r in self.dominated:
                lines.append(f"    ✗ {r.name:16s}  ← 被 {', '.join(r.dominated_by)} 支配")

        return "\n".join(lines)


class ParetoAnalyzer:
    """多目标帕累托前沿分析

    找出所有不被任何其他策略严格支配的策略。
    策略 A 支配策略 B 当且仅当:
      - A 在所有维度上 >= B
      - A 在至少一个维度上 > B
    """

    @classmethod
    def analyze(cls, results: List[StrategyResult],
                dimensions: Optional[List[str]] = None) -> ParetoReport:
        """分析帕累托前沿"""
        if dimensions is None:
            dimensions = ["vdc", "freq", "soc"]

        n = len(results)
        dominated = [False] * n
        dominated_by = [[] for _ in range(n)]

        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                if cls._dominates(results[i], results[j], dimensions):
                    dominated[j] = True
                    dominated_by[j].append(results[i].name)

        frontier = []
        rest = []
        for i in range(n):
            r = results[i]
            r.dominated_by = dominated_by[i]
            if not dominated[i]:
                r.is_pareto = True
                frontier.append(r)
            else:
                rest.append(r)

        return ParetoReport(
            frontier=frontier,
            dominated=rest,
            dimensions=dimensions,
        )

    @classmethod
    def _dominates(cls, a: StrategyResult, b: StrategyResult,
                   dims: List[str]) -> bool:
        """A 是否严格支配 B"""
        at_least_one_better = False
        for d in dims:
            score_a = getattr(a, d + "_score", 0)
            score_b = getattr(b, d + "_score", 0)
            if score_a < score_b:
                return False
            if score_a > score_b:
                at_least_one_better = True
        return at_least_one_better

    @classmethod
    def compute_dimension_scores(cls, m: ExperimentMetrics,
                                  s: ScenarioConfig,
                                  max_df_observed: float = 0.0) -> Dict[str, float]:
        """计算各维度独立分数 (0-100) — 委托给统一评分函数"""
        return compute_all_scores(m, s, max_df_observed)


# ═══════════════════════════════════════════
# 3. 蒙特卡洛鲁棒性评分
# ═══════════════════════════════════════════

@dataclass
class RobustnessStats:
    """鲁棒性统计"""
    strategy_name: str
    scenario_name: str
    n_runs: int
    mean: float
    std: float
    min_score: float
    p05: float           # 5% 分位数 — 保守承诺边界
    p95: float           # 95% 分位数
    scores: List[float] = field(default_factory=list)
    failures: int = 0    # 触发硬约束的次数

    def summary(self) -> str:
        return (
            f"{self.strategy_name:20s} "
            f"均值={self.mean:5.1f}  "
            f"σ={self.std:4.1f}  "
            f"最低={self.min_score:5.1f}  "
            f"P5={self.p05:5.1f}  "
            f"P95={self.p95:5.1f}  "
            f"失控={self.failures}/{self.n_runs}"
        )


class RobustnessScorer:
    """蒙特卡洛鲁棒性评估

    对场景参数施加 ±perturbation% 随机抖动, 跑 N 次,
    输出均值和方差 —— 均值高 + 方差低 = 真正可靠。
    """

    @staticmethod
    def perturb_scenario(scenario: ScenarioConfig,
                         perturbation: float = 0.20) -> ScenarioConfig:
        """对场景参数施加随机扰动

        perturbation: 扰动幅度, 0.20 = ±20%
        """
        import copy
        sc = copy.deepcopy(scenario)

        for ev in sc.events:
            # 扰动事件幅度
            if ev.event_type in ('freq_dip', 'freq_swell', 'freq_osc'):
                factor = 1.0 + random.uniform(-perturbation, perturbation)
                ev.magnitude *= factor
                if ev.ramp_rate > 0:
                    ev.ramp_rate *= factor

            elif ev.event_type in ('v_sag', 'v_swell'):
                factor = 1.0 + random.uniform(-perturbation, perturbation)
                ev.magnitude = max(0.1, min(1.5, ev.magnitude * factor))

            elif ev.event_type == 'load_step':
                factor = 1.0 + random.uniform(-perturbation, perturbation)
                ev.magnitude = max(50e3, ev.magnitude * factor)

            # 扰动时间
            t_shift = random.uniform(-0.1, 0.1) * sc.duration
            ev.t_start = max(0, ev.t_start + t_shift)
            ev.t_end = max(ev.t_start + 0.1, ev.t_end + t_shift)

        # 扰动初始 SOC
        sc.initial_soc = max(0.1, min(0.9,
            sc.initial_soc + random.uniform(-0.1, 0.1)))

        return sc

    @classmethod
    def monte_carlo(cls, runner, scenario: ScenarioConfig,
                    controller, strategy_name: str,
                    n: int = 100,
                    perturbation: float = 0.20,
                    verbose: bool = True) -> RobustnessStats:
        """蒙特卡洛鲁棒性跑分

        Args:
            runner: S4ExperimentRunner 实例
            scenario: 基准场景
            controller: 策略控制器
            strategy_name: 策略名称
            n: 跑分次数
            perturbation: 扰动幅度
            verbose: 是否打印进度

        Returns:
            RobustnessStats
        """
        scores = []
        failures = 0

        if verbose:
            print(f"\n  🎲 蒙特卡洛: {strategy_name} × {scenario.name} (N={n}, σ={perturbation:.0%})")

        for i in range(n):
            # 扰动场景
            sc_perturbed = cls.perturb_scenario(scenario, perturbation)

            # 跑仿真
            result = runner.run_scenario(sc_perturbed, controller, strategy_name)

            # 检查硬约束
            v = HardConstraints.check(result.metrics, sc_perturbed.scenario_id)
            if v.failed:
                failures += 1

            scores.append(result.metrics.composite_score)

            if verbose and (i + 1) % 20 == 0:
                avg_so_far = sum(scores) / len(scores)
                print(f"    [{i+1}/{n}] 当前均值={avg_so_far:.1f}  失控={failures}")

        # 统计
        scores_sorted = sorted(scores)
        mean = sum(scores) / n
        variance = sum((s - mean) ** 2 for s in scores) / n
        std = math.sqrt(variance)
        p05_idx = max(0, int(n * 0.05))
        p95_idx = min(n - 1, int(n * 0.95))

        stats = RobustnessStats(
            strategy_name=strategy_name,
            scenario_name=scenario.name,
            n_runs=n,
            mean=mean,
            std=std,
            min_score=scores_sorted[0],
            p05=scores_sorted[p05_idx],
            p95=scores_sorted[p95_idx],
            scores=scores,
            failures=failures,
        )

        if verbose:
            print(f"    → {stats.summary()}")

        return stats

    @classmethod
    def compare_strategies(cls, runner, scenario: ScenarioConfig,
                           controllers: List[Tuple[str, object]],
                           n: int = 100,
                           perturbation: float = 0.20) -> List[RobustnessStats]:
        """多个策略在同一场景的鲁棒性对比"""
        results = []
        for name, ctrl in controllers:
            stats = cls.monte_carlo(
                runner, scenario, ctrl, name,
                n=n, perturbation=perturbation, verbose=True,
            )
            results.append(stats)
        return results


# ═══════════════════════════════════════════
# 4. 综合报告 — 把三套评分串起来
# ═══════════════════════════════════════════

@dataclass
class DecisionTable:
    """最终决策表 — 给工程师的选型依据"""
    strategy_name: str
    composite_score: float
    vdc_score: float
    freq_score: float
    soc_score: float
    is_pareto: bool
    hard_pass: bool
    robustness_mean: float = 0
    robustness_std: float = 0
    weaknesses: List[str] = field(default_factory=list)

    def summary(self) -> str:
        status = "🟢" if self.hard_pass else "🔴"
        pareto = "P" if self.is_pareto else " "
        return (
            f"{status} {pareto} {self.strategy_name:18s} "
            f"综合={self.composite_score:5.1f}  "
            f"Vdc={self.vdc_score:4.0f}  "
            f"Freq={self.freq_score:4.0f}  "
            f"SOC={self.soc_score:4.0f}  "
            f"Rσ={self.robustness_std:3.1f}"
            f"{' ⚠️'+','.join(self.weaknesses) if self.weaknesses else ''}"
        )


def build_decision_table(
    all_results: Dict[str, List["StrategyResult"]],
    dimensions: Optional[List[str]] = None,
) -> List[DecisionTable]:
    """从多场景多策略结果生成决策表"""
    if dimensions is None:
        dimensions = ["vdc", "freq", "soc"]

    table = []

    for scenario_name, results in all_results.items():
        # 帕累托
        pareto = ParetoAnalyzer.analyze(results, dimensions)
        pareto_names = {r.name for r in pareto.frontier}

        for r in results:
            # 硬约束
            v = HardConstraints.check(r.metrics)
            # 弱点维度
            weaknesses = []
            if r.vdc_score < 50:
                weaknesses.append("Vdc弱")
            if r.freq_score < 50:
                weaknesses.append("频率弱")
            if r.soc_score < 50:
                weaknesses.append("SOC弱")

            table.append(DecisionTable(
                strategy_name=r.name,
                composite_score=r.composite_score,
                vdc_score=r.vdc_score,
                freq_score=r.freq_score,
                soc_score=r.soc_score,
                is_pareto=r.name in pareto_names,
                hard_pass=v.passed,
                weaknesses=weaknesses,
            ))

    return table
