"""
adversarial_scenarios.py — 对抗场景生成: 搜索AI策略的最差电网扰动组合。

子系统: 应用层 (v3.0)
依赖: power_ops.py, grid_sim.py

对抗场景生成: 搜索AI策略的最差电网扰动组合。
"""
import math
import random
import time
import copy
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any
from enum import Enum

import sys
sys.path.insert(0, '.')

from s4_scenarios import ScenarioConfig, GridEvent, ScenarioID
from s4_experiment_runner import S4ExperimentRunner
from s4_metrics import compute_all_scores


# ═══════════════════════════════════════════
# 数据结构
# ═══════════════════════════════════════════

@dataclass
class AdversarialResult:
    """对抗搜索的一次结果"""
    worst_params: dict                          # 最弱参数组合
    worst_score: float                          # 最弱参数对应的 composite_score
    worst_scenario: ScenarioConfig              # 生成的可运行场景
    convergence: List[float] = field(default_factory=list)  # 每代最优分数
    n_evals: int = 0                            # 总评估次数
    elapsed_sec: float = 0.0                    # 搜索耗时

    def summary(self) -> str:
        lines = [
            f"AdversarialResult(n_evals={self.n_evals}, worst_score={self.worst_score:.1f})",
            f"  worst_params:",
        ]
        for k, v in self.worst_params.items():
            if isinstance(v, float):
                lines.append(f"    {k}: {v:.4f}")
            else:
                lines.append(f"    {k}: {v}")
        lines.append(f"  scenario duration: {self.worst_scenario.duration:.1f}s")
        lines.append(f"  scenario events: {len(self.worst_scenario.events)}")
        return "\n".join(lines)


# ═══════════════════════════════════════════
# 参数 → 场景 构建器
# ═══════════════════════════════════════════

def params_to_scenario(params: Dict[str, float]) -> ScenarioConfig:
    """将连续参数向量转换为可运行的 ScenarioConfig。

    参数映射:
        freq_dip      → 频率骤降幅度 (负值, Hz)
        v_sag         → 电压暂降幅度 (pu)
        initial_soc   → 初始 SOC
        duration      → 总仿真时间 (s)
        load_magnitude → 负载突增功率 (W)
    """
    freq_dip = -abs(params.get('freq_dip', 0.5))  # 确保负值
    v_sag = params.get('v_sag', 0.7)
    initial_soc = params.get('initial_soc', 0.5)
    duration = params.get('duration', 3.0)
    load_mag = abs(params.get('load_magnitude', 500e3))

    # 确保时长合理
    duration = max(1.0, min(duration, 10.0))

    events = [
        # 频率骤降: t=0.3*duration → t=0.7*duration
        GridEvent(
            t_start=duration * 0.3,
            t_end=duration * 0.7,
            event_type='freq_dip',
            magnitude=freq_dip,
            ramp_rate=abs(freq_dip) * 2.0,  # 斜坡速率
        ),
        # 电压暂降: t=0.4*duration → t=0.6*duration
        GridEvent(
            t_start=duration * 0.4,
            t_end=duration * 0.6,
            event_type='v_sag',
            magnitude=v_sag,
        ),
        # 负载突增: t=0.35*duration → t=0.65*duration
        GridEvent(
            t_start=duration * 0.35,
            t_end=duration * 0.65,
            event_type='load_step',
            magnitude=load_mag,
        ),
    ]

    return ScenarioConfig(
        scenario_id=ScenarioID.COMBINED_STRESS,
        name="对抗搜索场景",
        description=f"差分进化搜索: freq_dip={freq_dip:.2f}Hz v_sag={v_sag:.2f}pu "
                    f"soc={initial_soc:.2f} load={load_mag/1e3:.0f}kW dur={duration:.1f}s",
        duration=duration,
        events=events,
        initial_soc=initial_soc,
        weight_vdc_reg=1.0,
        weight_freq_support=1.5,
        weight_soc_mgmt=0.8,
    )


# ═══════════════════════════════════════════
# 差分进化引擎 (简化实现)
# ═══════════════════════════════════════════

class DifferentialEvolution:
    """简化版差分进化算法 (不依赖 scipy)。

    算法:
        1. 初始化 popsize 个随机个体
        2. 每代: 对每个体, 随机选 3 个不同个体 a,b,c
           mutant = a + F*(b-c), 边界钳位
        3. 二项式交叉: trial = crossover(mutant, target, CR)
        4. 选择: 如果 trial 更优则替换
        5. 收敛: max_iter 或分数变化 < tol
    """

    def __init__(self, bounds: Dict[str, Tuple[float, float]],
                 popsize: int = 10, F: float = 0.8, CR: float = 0.9,
                 seed: int = 42):
        self.bounds = bounds
        self.param_names = list(bounds.keys())
        self.n_dim = len(self.param_names)
        self.popsize = popsize
        self.F = F      # 差分权重
        self.CR = CR    # 交叉概率
        self.rng = random.Random(seed)

        # 预计算边界数组
        self.lower = [bounds[n][0] for n in self.param_names]
        self.upper = [bounds[n][1] for n in self.param_names]

    def initialize_population(self) -> List[List[float]]:
        """随机初始化种群"""
        pop = []
        for _ in range(self.popsize):
            individual = [
                self.rng.uniform(self.lower[i], self.upper[i])
                for i in range(self.n_dim)
            ]
            pop.append(individual)
        return pop

    def individual_to_dict(self, individual: List[float]) -> Dict[str, float]:
        """将向量转为参数字典"""
        return {self.param_names[i]: individual[i] for i in range(self.n_dim)}

    def _clamp(self, value: float, i: int) -> float:
        """钳位到边界内"""
        return max(self.lower[i], min(self.upper[i], value))

    def mutate_and_crossover(self, pop: List[List[float]],
                              target_idx: int) -> List[float]:
        """对一个目标个体执行变异+交叉, 返回 trial 向量"""
        # 选三个不同的随机个体 (不含目标)
        candidates = [j for j in range(self.popsize) if j != target_idx]
        a_idx, b_idx, c_idx = self.rng.sample(candidates, 3)

        a = pop[a_idx]
        b = pop[b_idx]
        c = pop[c_idx]
        target = pop[target_idx]

        # 差分变异: v = a + F*(b-c)
        mutant = [
            self._clamp(a[i] + self.F * (b[i] - c[i]), i)
            for i in range(self.n_dim)
        ]

        # 二项式交叉
        j_rand = self.rng.randrange(self.n_dim)  # 确保至少一个维度被替换
        trial = []
        for i in range(self.n_dim):
            if self.rng.random() < self.CR or i == j_rand:
                trial.append(mutant[i])
            else:
                trial.append(target[i])

        return trial

    def run(self, fitness_fn, max_iter: int = 50,
            tol: float = 0.1) -> Tuple[Dict[str, float], float, List[float], int]:
        """运行差分进化。

        Args:
            fitness_fn: callable(dict) -> float, 目标是最小化
            max_iter: 最大代数
            tol: 收敛容差 (最优值变化 < tol 则停止)

        Returns:
            (best_params_dict, best_fitness, convergence_history, n_evals)
        """
        pop = self.initialize_population()
        fitness = []
        n_evals = 0
        convergence = []

        # 评估初始种群
        for ind in pop:
            params = self.individual_to_dict(ind)
            score = fitness_fn(params)
            fitness.append(score)
            n_evals += 1

        # 找当前最优
        best_idx = min(range(self.popsize), key=lambda j: fitness[j])
        best_fitness = fitness[best_idx]
        convergence.append(best_fitness)

        for gen in range(max_iter):
            for i in range(self.popsize):
                trial = self.mutate_and_crossover(pop, i)
                trial_params = self.individual_to_dict(trial)
                trial_fitness = fitness_fn(trial_params)
                n_evals += 1

                if trial_fitness < fitness[i]:
                    pop[i] = trial
                    fitness[i] = trial_fitness

            # 更新全局最优
            best_idx = min(range(self.popsize), key=lambda j: fitness[j])
            current_best = fitness[best_idx]
            convergence.append(current_best)

            # 收敛检查
            if abs(current_best - best_fitness) < tol and gen > 5:
                best_fitness = current_best
                break

            best_fitness = current_best

        # 最优参数
        best_idx = min(range(self.popsize), key=lambda j: fitness[j])
        best_params = self.individual_to_dict(pop[best_idx])

        return best_params, best_fitness, convergence, n_evals


# ═══════════════════════════════════════════
# 对抗场景生成器
# ═══════════════════════════════════════════

class AdversarialScenarioGenerator:
    """用差分进化找策略弱点。

    工作流程:
        1. 定义搜索空间 (param_bounds)
        2. 差分进化生成候选参数
        3. 参数 → ScenarioConfig → run_scenario → composite_score
        4. 最小化 composite_score → 找到策略最难受的参数
    """

    def __init__(self, runner: S4ExperimentRunner, controller,
                 strategy_name: str):
        """
        Args:
            runner: S4ExperimentRunner 实例
            controller: 策略控制器 (必须有 .decide(snapshot) 方法)
            strategy_name: 策略名称 (用于结果标记)
        """
        self.runner = runner
        self.controller = controller
        self.strategy_name = strategy_name

        # 结果缓存
        self._last_result: Optional[AdversarialResult] = None
        self._pareto_front: List[Dict] = []

    def _evaluate(self, params: Dict[str, float]) -> float:
        """评估一组参数: 构建场景 → 跑仿真 → 返回 composite_score。

        目标是最小化 composite_score, 所以差策略得高分 → 这是我们想要的对抗方向:
        分数越低 = 策略表现越差 = 越对抗。
        """
        scenario = params_to_scenario(params)

        try:
            result = self.runner.run_scenario(
                scenario, self.controller, self.strategy_name
            )
            return result.metrics.composite_score
        except Exception as e:
            # 仿真出错 → 返回高分 (表示无效参数, DE 会避开)
            return 1e9

    def search(self, param_bounds: Dict[str, Tuple[float, float]],
               max_iter: int = 50, popsize: int = 10,
               F: float = 0.8, CR: float = 0.9,
               seed: int = 42, verbose: bool = True) -> AdversarialResult:
        """运行差分进化搜索。

        Args:
            param_bounds: 参数搜索空间, 如:
                {'freq_dip': (0.1, 2.0), 'v_sag': (0.3, 0.9), ...}
            max_iter: 最大迭代代数
            popsize: 种群大小
            F: 差分权重 (0.5-1.0)
            CR: 交叉概率 (0.7-0.95)
            seed: 随机种子
            verbose: 是否打印进度

        Returns:
            AdversarialResult
        """
        t_start = time.time()

        de = DifferentialEvolution(
            bounds=param_bounds,
            popsize=popsize,
            F=F, CR=CR,
            seed=seed,
        )

        if verbose:
            print(f"\n{'='*60}")
            print(f"  对抗场景搜索: {self.strategy_name}")
            print(f"  参数空间: {len(param_bounds)} 维, 种群={popsize}, 代数={max_iter}")
            print(f"{'='*60}")

        # 用于记录帕累托前沿 (参数 → score)
        evaluated: List[Tuple[Dict, float]] = []

        def fitness_fn(params: Dict[str, float]) -> float:
            score = self._evaluate(params)
            evaluated.append((copy.deepcopy(params), score))
            return score

        best_params, best_score, convergence, n_evals = de.run(
            fitness_fn, max_iter=max_iter, tol=0.1
        )

        # 生成最弱场景
        worst_scenario = params_to_scenario(best_params)

        # 构建 result
        result = AdversarialResult(
            worst_params=best_params,
            worst_score=best_score,
            worst_scenario=worst_scenario,
            convergence=convergence,
            n_evals=n_evals,
            elapsed_sec=time.time() - t_start,
        )

        # 缓存帕累托前沿 (所有非支配的参数组合)
        self._last_result = result
        self._pareto_front = self._compute_pareto_front(evaluated)

        if verbose:
            print(f"\n  搜索完成: {n_evals} 次评估, {result.elapsed_sec:.1f}s")
            print(f"  最弱得分: {best_score:.1f}")
            print(f"  最弱参数: {best_params}")

        return result

    def _compute_pareto_front(self, evaluated: List[Tuple[Dict, float]]
                              ) -> List[Dict]:
        """从所有评估结果中提取帕累托前沿 (最小化 score)。"""
        front = []
        n = len(evaluated)
        for i in range(n):
            score_i = evaluated[i][1]
            dominated = False
            for j in range(n):
                if i == j:
                    continue
                score_j = evaluated[j][1]
                # j 支配 i: j 分数更低 (更好)
                if score_j < score_i:
                    dominated = True
                    break
            if not dominated:
                front.append({
                    'params': evaluated[i][0],
                    'score': score_i,
                })
        return front

    def generate_report(self) -> dict:
        """生成搜索报告。

        Returns:
            {
                'worst_params': {...},
                'worst_score': float,
                'pareto_front': [...],
                'convergence_history': [...],
                'n_evals': int,
                'elapsed_sec': float,
            }
        """
        if self._last_result is None:
            return {
                'error': '尚未运行搜索, 请先调用 .search()',
            }

        r = self._last_result
        return {
            'strategy': self.strategy_name,
            'worst_params': r.worst_params,
            'worst_score': r.worst_score,
            'worst_scenario': {
                'duration': r.worst_scenario.duration,
                'initial_soc': r.worst_scenario.initial_soc,
                'events': [
                    {
                        'type': ev.event_type,
                        'magnitude': ev.magnitude,
                        't_start': ev.t_start,
                        't_end': ev.t_end,
                    }
                    for ev in r.worst_scenario.events
                ],
            },
            'pareto_front': self._pareto_front,
            'convergence_history': r.convergence,
            'n_evals': r.n_evals,
            'elapsed_sec': r.elapsed_sec,
        }

    def print_report(self):
        """打印人类可读的报告"""
        report = self.generate_report()
        if 'error' in report:
            print(report['error'])
            return

        print(f"\n{'='*60}")
        print(f"  对抗场景搜索报告")
        print(f"{'='*60}")
        print(f"  策略: {report['strategy']}")
        print(f"  评估次数: {report['n_evals']}")
        print(f"  耗时: {report['elapsed_sec']:.1f}s")
        print(f"\n  ⚠ 最弱参数 (composite={report['worst_score']:.1f}):")
        for k, v in report['worst_params'].items():
            if isinstance(v, float):
                print(f"    {k:18s} = {v:.4f}")
            else:
                print(f"    {k:18s} = {v}")

        print(f"\n  📉 收敛历史 (每代最优):")
        for i, score in enumerate(report['convergence_history']):
            bar = '█' * max(1, int(50 * score / 100))
            print(f"    Gen {i:3d}: {score:6.1f} |{bar}")

        print(f"\n  🎯 帕累托前沿 ({len(report['pareto_front'])} 个非支配解):")
        for i, pf in enumerate(report['pareto_front'][:5]):
            print(f"    #{i+1}: score={pf['score']:.1f}")
        if len(report['pareto_front']) > 5:
            print(f"    ... 还有 {len(report['pareto_front'])-5} 个")


# ═══════════════════════════════════════════
# CLI 入口
# ═══════════════════════════════════════════

if __name__ == '__main__':
    import argparse
    from s4_ai_controller import S4AIController, AIStrategy

    parser = argparse.ArgumentParser(
        description='对抗场景生成器 — 用差分进化找策略弱点'
    )
    parser.add_argument('--strategy', default='AI_ADAPTIVE',
                        choices=['AI_ADAPTIVE', 'AI_RULE_BASED', 'AI_PREDICTIVE',
                                 'BASELINE_VDC', 'BASELINE_FREQ', 'BASELINE_VSG'],
                        help='目标策略')
    parser.add_argument('--max-iter', type=int, default=30,
                        help='最大迭代代数 (默认 30)')
    parser.add_argument('--popsize', type=int, default=10,
                        help='种群大小 (默认 10)')
    parser.add_argument('--seed', type=int, default=42,
                        help='随机种子')
    parser.add_argument('--quick', action='store_true',
                        help='快速模式: 5 代 × 5 个体')

    args = parser.parse_args()

    # 解析策略
    strategy_map = {
        'AI_ADAPTIVE': AIStrategy.ADAPTIVE,
        'AI_RULE_BASED': AIStrategy.RULE_BASED,
        'AI_PREDICTIVE': AIStrategy.PREDICTIVE,
    }

    from s4_experiment_runner import (
        BaselineFixedVDC, BaselineFixedFreq, BaselineFixedVSG
    )
    baseline_map = {
        'BASELINE_VDC': BaselineFixedVDC(),
        'BASELINE_FREQ': BaselineFixedFreq(),
        'BASELINE_VSG': BaselineFixedVSG(),
    }

    if args.strategy in strategy_map:
        controller = S4AIController(strategy=strategy_map[args.strategy])
    else:
        controller = baseline_map[args.strategy]

    runner = S4ExperimentRunner()
    generator = AdversarialScenarioGenerator(
        runner, controller, args.strategy
    )

    if args.quick:
        max_iter = 5
        popsize = 5
    else:
        max_iter = args.max_iter
        popsize = args.popsize

    result = generator.search(
        param_bounds={
            'freq_dip': (0.1, 2.0),
            'v_sag': (0.3, 0.9),
            'initial_soc': (0.1, 0.9),
            'duration': (0.5, 5.0),
            'load_magnitude': (100e3, 1e6),
        },
        max_iter=max_iter,
        popsize=popsize,
        seed=args.seed,
    )

    generator.print_report()
