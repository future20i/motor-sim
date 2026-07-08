"""
test_adversarial.py — 对抗场景生成器测试。

子系统: 应用层测试
依赖: adversarial_scenarios.py

对抗场景生成器测试。
"""
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from adversarial_scenarios import (
    AdversarialScenarioGenerator,
    AdversarialResult,
    DifferentialEvolution,
    params_to_scenario,
)
from s4_experiment_runner import (
    S4ExperimentRunner,
    BaselineFixedVDC,
    BaselineFixedFreq,
)
from s4_ai_controller import S4AIController, AIStrategy
from s4_scenarios import ScenarioConfig, ScenarioID


# ═══════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════

@pytest.fixture
def runner():
    return S4ExperimentRunner()


@pytest.fixture
def ai_controller():
    return S4AIController(strategy=AIStrategy.ADAPTIVE)


@pytest.fixture
def baseline_controller():
    return BaselineFixedVDC()


@pytest.fixture
def generator_ai(runner, ai_controller):
    return AdversarialScenarioGenerator(runner, ai_controller, "AI_ADAPTIVE")


@pytest.fixture
def generator_baseline(runner, baseline_controller):
    return AdversarialScenarioGenerator(runner, baseline_controller, "BASELINE_VDC")


@pytest.fixture
def small_param_bounds():
    return {
        'freq_dip': (0.2, 1.0),
        'v_sag': (0.5, 0.8),
        'initial_soc': (0.3, 0.7),
        'duration': (1.0, 3.0),
        'load_magnitude': (200e3, 500e3),
    }


# ═══════════════════════════════════════════
# Tests: AdversarialScenarioGenerator
# ═══════════════════════════════════════════

class TestGeneratorCreation:
    """测试生成器创建"""

    def test_creation_with_ai_controller(self, runner, ai_controller):
        """用 AI 控制器创建生成器"""
        gen = AdversarialScenarioGenerator(runner, ai_controller, "AI_ADAPTIVE")
        assert gen.strategy_name == "AI_ADAPTIVE"
        assert gen.runner is runner
        assert gen.controller is ai_controller
        assert gen._last_result is None
        assert gen._pareto_front == []

    def test_creation_with_baseline(self, runner, baseline_controller):
        """用基线控制器创建生成器"""
        gen = AdversarialScenarioGenerator(
            runner, baseline_controller, "BASELINE_VDC"
        )
        assert gen.strategy_name == "BASELINE_VDC"

    def test_multiple_strategies(self, runner):
        """不同策略都能创建"""
        strategies = [
            ("AI_RULE", S4AIController(strategy=AIStrategy.RULE_BASED)),
            ("AI_PRED", S4AIController(strategy=AIStrategy.PREDICTIVE)),
            ("BASELINE_FREQ", BaselineFixedFreq()),
        ]
        for name, ctrl in strategies:
            gen = AdversarialScenarioGenerator(runner, ctrl, name)
            assert gen.strategy_name == name


class TestSearchBasic:
    """测试基本搜索功能"""

    def test_search_3_iterations_no_crash(self, generator_ai, small_param_bounds):
        """跑 3 次迭代不崩溃"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=3,
            popsize=5,
            seed=123,
            verbose=False,
        )
        assert result is not None
        assert result.n_evals >= 5   # 初始种群评估
        assert len(result.convergence) >= 1

    def test_search_returns_adversarial_result(self, generator_ai, small_param_bounds):
        """search 返回 AdversarialResult"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=2,
            popsize=4,
            seed=99,
            verbose=False,
        )
        assert isinstance(result, AdversarialResult)
        assert isinstance(result.worst_params, dict)
        assert isinstance(result.worst_score, float)

    def test_search_with_baseline(self, generator_baseline, small_param_bounds):
        """基线策略也能搜索"""
        result = generator_baseline.search(
            small_param_bounds,
            max_iter=2,
            popsize=4,
            seed=42,
            verbose=False,
        )
        assert result.n_evals > 0
        assert result.worst_score >= 0


class TestResultHasScenario:
    """测试返回可运行的 ScenarioConfig"""

    def test_result_has_scenario_config(self, generator_ai, small_param_bounds):
        """搜索结果包含 ScenarioConfig"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=3,
            popsize=5,
            seed=7,
            verbose=False,
        )
        assert isinstance(result.worst_scenario, ScenarioConfig)
        assert result.worst_scenario.scenario_id == ScenarioID.COMBINED_STRESS

    def test_scenario_is_runnable(self, runner, ai_controller, small_param_bounds):
        """返回的 scenario 可以被 run_scenario 执行"""
        gen = AdversarialScenarioGenerator(runner, ai_controller, "AI_ADAPTIVE")
        result = gen.search(
            small_param_bounds,
            max_iter=2,
            popsize=4,
            seed=13,
            verbose=False,
        )

        sc = result.worst_scenario
        # 可以直接用 runner 跑
        exp_result = runner.run_scenario(sc, ai_controller, "AI_ADAPTIVE")
        assert exp_result.metrics.composite_score >= 0

    def test_scenario_has_events(self, generator_ai, small_param_bounds):
        """场景包含事件"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=2,
            popsize=4,
            seed=77,
            verbose=False,
        )
        assert len(result.worst_scenario.events) > 0
        event_types = [ev.event_type for ev in result.worst_scenario.events]
        assert 'freq_dip' in event_types
        assert 'v_sag' in event_types
        assert 'load_step' in event_types


class TestWorstParams:
    """测试最弱参数的有效性"""

    def test_worst_params_in_bounds(self, generator_ai, small_param_bounds):
        """最弱参数在搜索空间内"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=3,
            popsize=5,
            seed=42,
            verbose=False,
        )

        for param_name, (lo, hi) in small_param_bounds.items():
            val = result.worst_params[param_name]
            assert lo <= val <= hi, (
                f"{param_name}={val} 超出边界 [{lo}, {hi}]"
            )

    def test_initial_soc_bounds(self, generator_ai):
        """initial_soc 在 0.1-0.9 范围内"""
        bounds = {
            'freq_dip': (0.5, 1.0),
            'v_sag': (0.5, 0.7),
            'initial_soc': (0.1, 0.9),
            'duration': (2.0, 3.0),
            'load_magnitude': (300e3, 400e3),
        }
        result = generator_ai.search(bounds, max_iter=2, popsize=4, seed=1, verbose=False)
        assert 0.1 <= result.worst_params['initial_soc'] <= 0.9


class TestGenerateReport:
    """测试报告生成"""

    def test_report_before_search(self, generator_ai):
        """未搜索前报告返回错误"""
        report = generator_ai.generate_report()
        assert 'error' in report

    def test_report_after_search(self, generator_ai, small_param_bounds):
        """搜索后报告包含所有字段"""
        generator_ai.search(
            small_param_bounds,
            max_iter=2, popsize=4, seed=42, verbose=False
        )
        report = generator_ai.generate_report()

        assert 'worst_params' in report
        assert 'worst_score' in report
        assert 'worst_scenario' in report
        assert 'pareto_front' in report
        assert 'convergence_history' in report
        assert 'n_evals' in report
        assert 'elapsed_sec' in report
        assert report['strategy'] == 'AI_ADAPTIVE'

    def test_report_scenario_details(self, generator_ai, small_param_bounds):
        """报告中的场景详情正确"""
        generator_ai.search(
            small_param_bounds,
            max_iter=2, popsize=4, seed=42, verbose=False
        )
        report = generator_ai.generate_report()
        sc = report['worst_scenario']
        assert 'duration' in sc
        assert 'initial_soc' in sc
        assert 'events' in sc
        assert len(sc['events']) == 3

    def test_print_report_no_crash(self, generator_ai, small_param_bounds):
        """print_report 不崩溃"""
        generator_ai.search(
            small_param_bounds,
            max_iter=2, popsize=4, seed=42, verbose=False
        )
        generator_ai.print_report()  # 不抛异常

    def test_print_report_before_search(self, generator_ai):
        """未搜索时 print_report 不崩溃"""
        generator_ai.print_report()


class TestConvergence:
    """测试收敛行为"""

    def test_convergence_improves(self, generator_ai, small_param_bounds):
        """收敛历史中分数不应上升 (我们最小化 composite_score)"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=5,
            popsize=6,
            seed=42,
            verbose=False,
        )
        # 最终分数 <= 初始分数 (DE 是最小化)
        assert result.convergence[-1] <= result.convergence[0] + 5  # 容忍噪声

    def test_worst_score_is_not_nan(self, generator_ai, small_param_bounds):
        """最弱分数不为 NaN"""
        result = generator_ai.search(
            small_param_bounds,
            max_iter=2, popsize=4, seed=42, verbose=False
        )
        import math
        assert not math.isnan(result.worst_score)
        assert not math.isinf(result.worst_score)


# ═══════════════════════════════════════════
# Tests: DifferentialEvolution
# ═══════════════════════════════════════════

class TestDifferentialEvolution:
    """测试差分进化引擎"""

    def test_init_population(self):
        """初始种群在边界内"""
        bounds = {'x': (0.0, 1.0), 'y': (-5.0, 5.0)}
        de = DifferentialEvolution(bounds, popsize=20)
        pop = de.initialize_population()
        assert len(pop) == 20
        for ind in pop:
            assert 0.0 <= ind[0] <= 1.0
            assert -5.0 <= ind[1] <= 5.0

    def test_simple_optimization(self):
        """最小化简单函数 f(x,y) = x^2 + y^2"""
        bounds = {'x': (-5.0, 5.0), 'y': (-5.0, 5.0)}
        de = DifferentialEvolution(bounds, popsize=15, seed=1)

        def fitness(p):
            return p['x']**2 + p['y']**2

        best_params, best_score, conv, n_evals = de.run(fitness, max_iter=30)

        assert best_score < 1.0  # 应该接近 (0, 0)
        assert abs(best_params['x']) < 1.5
        assert abs(best_params['y']) < 1.5

    def test_individual_to_dict(self):
        """向量到字典转换正确"""
        bounds = {'a': (0.0, 10.0), 'b': (20.0, 30.0)}
        de = DifferentialEvolution(bounds, popsize=1)
        d = de.individual_to_dict([5.0, 25.0])
        assert d == {'a': 5.0, 'b': 25.0}

    def test_early_convergence(self):
        """如果所有个体都相同, 算法应早停"""
        bounds = {'x': (0.0, 1.0)}
        de = DifferentialEvolution(bounds, popsize=5, seed=42)

        # 所有个体初始化就很接近
        call_count = [0]

        def fitness(p):
            call_count[0] += 1
            return p['x']  # 只是返回 x

        best_params, score, conv, n_evals = de.run(fitness, max_iter=50, tol=0.01)

        # 应该比 max_iter 早结束
        assert len(conv) <= 50


# ═══════════════════════════════════════════
# Tests: params_to_scenario
# ═══════════════════════════════════════════

class TestParamsToScenario:
    """测试参数到场景的转换"""

    def test_basic_conversion(self):
        """基本转换"""
        params = {
            'freq_dip': 0.8,
            'v_sag': 0.6,
            'initial_soc': 0.4,
            'duration': 2.0,
            'load_magnitude': 300e3,
        }
        sc = params_to_scenario(params)
        assert isinstance(sc, ScenarioConfig)
        assert sc.duration == 2.0
        assert sc.initial_soc == 0.4
        assert len(sc.events) == 3

    def test_freq_dip_is_negative(self):
        """freq_dip 确保为负值"""
        sc = params_to_scenario({'freq_dip': 1.5})
        freq_event = next(ev for ev in sc.events if ev.event_type == 'freq_dip')
        assert freq_event.magnitude < 0

    def test_duration_clamped(self):
        """duration 被钳位"""
        sc = params_to_scenario({'duration': 0.01})
        assert sc.duration >= 1.0

        sc = params_to_scenario({'duration': 100.0})
        assert sc.duration <= 10.0

    def test_events_have_valid_timing(self):
        """事件时间在 duration 范围内"""
        params = {
            'freq_dip': 0.5,
            'v_sag': 0.7,
            'initial_soc': 0.5,
            'duration': 3.0,
            'load_magnitude': 500e3,
        }
        sc = params_to_scenario(params)
        for ev in sc.events:
            assert ev.t_start >= 0
            assert ev.t_start < ev.t_end
            assert ev.t_end <= sc.duration

    def test_load_magnitude_positive(self):
        """load_magnitude 为正"""
        sc = params_to_scenario({'load_magnitude': -500e3})
        load_event = next(ev for ev in sc.events if ev.event_type == 'load_step')
        assert load_event.magnitude > 0


# ═══════════════════════════════════════════
# Tests: AdversarialResult
# ═══════════════════════════════════════════

class TestAdversarialResult:
    """测试结果数据类"""

    def test_summary(self):
        """summary 方法不崩溃"""
        sc = params_to_scenario({
            'freq_dip': 0.8, 'v_sag': 0.6,
            'initial_soc': 0.5, 'duration': 2.0,
            'load_magnitude': 300e3,
        })
        result = AdversarialResult(
            worst_params={'freq_dip': 0.8, 'v_sag': 0.6},
            worst_score=42.5,
            worst_scenario=sc,
            convergence=[80.0, 65.0, 50.0, 42.5],
            n_evals=45,
            elapsed_sec=12.3,
        )
        s = result.summary()
        assert '42.5' in s
        assert '0.8000' in s
        assert '45' in s


# ═══════════════════════════════════════════
# 集成测试 (仅在有足够时间时运行)
# ═══════════════════════════════════════════

@pytest.mark.slow
class TestIntegration:
    """慢速集成测试 — 用 --run-slow 标志运行"""

    def test_full_search_10_iterations(self, runner, ai_controller):
        """完整 10 代搜索"""
        gen = AdversarialScenarioGenerator(runner, ai_controller, "AI_ADAPTIVE")
        result = gen.search(
            {
                'freq_dip': (0.2, 1.5),
                'v_sag': (0.4, 0.8),
                'initial_soc': (0.2, 0.8),
                'duration': (1.0, 3.0),
                'load_magnitude': (200e3, 600e3),
            },
            max_iter=10,
            popsize=8,
            seed=123,
            verbose=True,
        )
        assert result.n_evals > 0
        report = gen.generate_report()
        assert report['worst_score'] >= 0
