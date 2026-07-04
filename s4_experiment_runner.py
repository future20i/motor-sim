"""
s4_experiment_runner.py — S4: Experiment Runner

编排全系统仿真 + 场景注入 + 指标采集 + 策略对比

用法:
    python s4_experiment_runner.py              # 跑所有场景
    python s4_experiment_runner.py --scenario 1 # 单个场景
    python s4_experiment_runner.py --compare    # 三策略对比
"""
import sys
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional
from collections import defaultdict

# 项目内模块
sys.path.insert(0, '.')
from grid_sim import GridSimulator, GridConfig, GridFault
from grid_inverter import GridInverter, GridInvMode, GridInverterConfig
from power_ops import (
    PowerOrchestrator, PowerMode, DCBusController, VSGController, FrequencyRegulator,
)
from flywheel_energy import FlywheelEnergyStorage
from dc_bus import DCBus, DCBusConfig
from s4_scenarios import (
    ScenarioConfig, GridEvent, ExperimentMetrics,
    build_all_scenarios, ScenarioID,
)
from s4_ai_controller import S4AIController, AIStrategy, GridSnapshot, AIControlAction


# ═══════════════════════════════════════════
# Baseline 策略 (固定模式, 不智能)
# ═══════════════════════════════════════════

class BaselineFixedVDC:
    """基线: 始终 DC 母线恒压控制"""
    name = "基线-固定VDC"
    def decide(self, snap: GridSnapshot) -> AIControlAction:
        return AIControlAction(
            mode=PowerMode.DC_BUS_CONTROL,
            dc_mode='ff_pi',
            reason="固定 VDC 模式",
        )


class BaselineFixedFreq:
    """基线: 始终一次调频"""
    name = "基线-固定调频"
    def decide(self, snap: GridSnapshot) -> AIControlAction:
        return AIControlAction(
            mode=PowerMode.FREQ_REGULATION,
            droop=0.04,
            reason="固定调频模式",
        )


class BaselineFixedVSG:
    """基线: 始终 VSG"""
    name = "基线-固定VSG"
    def decide(self, snap: GridSnapshot) -> AIControlAction:
        return AIControlAction(
            mode=PowerMode.VSG,
            droop=0.04,
            reason="固定 VSG 模式",
        )


# ═══════════════════════════════════════════
# 实验运行器
# ═══════════════════════════════════════════

@dataclass
class ExperimentResult:
    """单次实验完整结果"""
    scenario: ScenarioConfig
    strategy_name: str
    metrics: ExperimentMetrics = field(default_factory=ExperimentMetrics)
    time_series: dict = field(default_factory=dict)
    decisions: list = field(default_factory=list)
    elapsed_ms: float = 0


class S4ExperimentRunner:
    """S4 实验运行器 — 场景驱动, 多策略对比"""

    def __init__(self):
        self.Ts = 50e-6        # 仿真步长
        self.metrics_interval = 100  # 每 100 步采集一次 (5ms)

    def _get_signed_flywheel_power(self, orchestrator) -> float:
        """计算带符号的飞轮功率 (DC 母线视角)

        正 = 母线→飞轮 (充电), 负 = 飞轮→母线 (放电)
        """
        po = orchestrator
        s = po.state
        # P_mech: 正=电动(充电, 母线→飞轮), 负=发电(放电, 飞轮→母线)
        # 返回带符号值: 正=母线→飞轮充电, 负=飞轮→母线放电
        return -s.P_actual if s.P_ref >= 0 else s.P_actual

    def run_scenario(self, scenario: ScenarioConfig, controller,
                     strategy_name: str = "unknown") -> ExperimentResult:
        """跑一个场景, 返回完整结果"""
        t_start = time.time()

        # ── 初始化系统 ──
        grid = GridSimulator(GridConfig(V_nom=480.0, f_nom=50.0))
        inv_cfg = GridInverterConfig(
            Vdc_nom=750.0,
            Kp_i=10.0, Ki_i=100.0,
            Kp_pll=3.0, Ki_pll=100.0,
        )
        inverter = GridInverter(inv_cfg)
        # 提升 VDC 控制增益
        inverter._vdc_Kp = 50.0
        inverter._vdc_Ki = 200.0
        dc_bus = DCBus(DCBusConfig(V_nom=750.0, C=0.2))
        flywheel = FlywheelEnergyStorage()
        orchestrator = PowerOrchestrator(flywheel, grid=grid)

        # 设置初始状态
        dc_bus.state.Vdc = scenario.initial_vdc
        flywheel.state.omega = flywheel.cfg.omega_min_rad + \
            (flywheel.cfg.omega_max_rad - flywheel.cfg.omega_min_rad) * scenario.initial_soc
        flywheel.state.SOC = scenario.initial_soc

        # ── 预充 + 同步 ──
        # 先跑预充让逆变器到达 VDC_CONTROL
        Vdc = 60.0  # 从低压开始
        # 手动设置逆变器状态, 加速预充
        inverter.state.Vdc = Vdc
        inverter.state.mode = GridInvMode.PRECHARGE
        inverter.state.contactor_closed = False
        for i in range(600):
            grid.step()
            if inverter.state.mode == GridInvMode.PRECHARGE:
                inverter.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                        mode=GridInvMode.PRECHARGE)
                if inverter.state.Vdc >= inverter.cfg.Vdc_precharge_target * 0.95:
                    inverter.state.contactor_closed = True
                    inverter.state.mode = GridInvMode.SYNC
            elif inverter.state.mode == GridInvMode.SYNC:
                inverter.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                        mode=GridInvMode.SYNC)
                if abs(inverter.state.f_pll - 50.0) < 0.2 and inverter.state.contactor_closed:
                    inverter.state.mode = GridInvMode.VDC_CONTROL
            else:
                inverter.step(Vdc, grid.state.Va, grid.state.Vb, grid.state.Vc,
                        mode=GridInvMode.VDC_CONTROL)
            Vdc += (scenario.initial_vdc - Vdc) * 0.02

        # 确保已进入 VDC_CONTROL
        if inverter.state.mode != GridInvMode.VDC_CONTROL:
            inverter.state.mode = GridInvMode.VDC_CONTROL
            inverter.state.contactor_closed = True
        # 预充后同步 DC 母线到目标值
        dc_bus.state.Vdc = max(dc_bus.state.Vdc, scenario.initial_vdc * 0.8)

        # ── 数据采集 ──
        ts = {'t': [], 'f': [], 'Vdc': [], 'P_fly': [], 'SOC': [],
              'mode': [], 'P_ref': [], 'P_load': [], 'P_grid': []}
        decisions = []
        metrics = ExperimentMetrics()

        total_steps = int(scenario.duration / self.Ts)
        step = 0
        P_load_active = 0.0
        prev_f = 50.0
        soc_min = scenario.initial_soc
        soc_max = scenario.initial_soc
        vdc_samples = []
        freq_responded = False
        freq_response_time = 0
        total_loss = 0.0
        max_df_observed = 0.0

        while step < total_steps:
            t = step * self.Ts

            # 处理电网事件
            P_load_active = self._apply_events(grid, scenario.events, t)

            # 步进电网
            grid.step()

            # ── 构成 AI 决策输入 ──
            current_soc = flywheel.state.SOC
            dfdt = (grid.state.f - prev_f) / max(self.Ts * self.metrics_interval, 1e-9)
            prev_f = grid.state.f

            snap = GridSnapshot(
                f=grid.state.f,
                df=grid.state.f - grid.cfg.f_nom,
                dfdt=dfdt,
                Vdc=dc_bus.state.Vdc,
                dVdc=dc_bus.state.Vdc - dc_bus.cfg.V_nom,
                SOC=current_soc,
                P_grid=inverter.state.P,
                P_load=P_load_active,
                f_pll=inverter.state.f_pll,
                Vg_amplitude=inverter.state.Vg_amplitude,
            )

            # AI 决策
            if step % self.metrics_interval == 0:
                action = controller.decide(snap)

                # 逆变器始终 VDC_CONTROL (负责电网侧整流稳压)
                # 编排器忠实执行 AI/基线选择的模式 — 不再把 DC_BUS_CONTROL /
                # INERTIA_SUPPORT 强行改写为 FREQ_REGULATION。
                # [FIX-2026-07 迭代1] 旧版此处的别名重写会让所有依赖飞轮
                # 直流母线支撑(dc_ctrl)的策略静默退化为纯调频策略。
                orchestrator.mode = action.mode

                # 应用决策参数 (按实际执行的模式下发, 而非改写后的模式)
                if action.mode == PowerMode.FREQ_REGULATION:
                    orchestrator.freq_reg.droop = action.droop
                    orchestrator.freq_reg.deadband = action.deadband
                elif action.mode == PowerMode.VSG:
                    orchestrator.vsg.droop_p = action.droop
                elif action.mode == PowerMode.DC_BUS_CONTROL:
                    orchestrator.dc_ctrl.mode = action.dc_mode
                elif action.mode == PowerMode.INERTIA_SUPPORT:
                    orchestrator.freq_reg.droop = action.droop
                    orchestrator.dc_ctrl.mode = action.dc_mode

                decisions.append({
                    't': t, 'mode': action.mode.name, 'P_ref': action.P_ref,
                    'reason': action.reason, 'confidence': action.confidence,
                    'droop': action.droop, 'dc_mode': action.dc_mode,
                })

            # ── 步进网侧逆变器 (始终 VDC_CONTROL 稳压) ──
            inverter.step(
                Vdc=dc_bus.state.Vdc,
                grid_Va=grid.state.Va,
                grid_Vb=grid.state.Vb,
                grid_Vc=grid.state.Vc,
                mode=GridInvMode.VDC_CONTROL,
            )

            # ── 步进功率调度 → 飞轮 ──
            po_state = orchestrator.step(
                Vdc=dc_bus.state.Vdc,
                f_grid=grid.state.f,
                SOC=flywheel.state.SOC,
                P_schedule=orchestrator.state.P_ref,
                P_grid=inverter.state.P,
                P_load=P_load_active,
            )

            # ── 步进直流母线 ──
            P_rect = -inverter.state.P  # 正=电网充电母线
            P_fly_signed = self._get_signed_flywheel_power(orchestrator)
            dc_bus.step(P_rect=P_rect, P_flywheel=P_fly_signed, P_load=P_load_active)

            # ── 数据采集 ──
            if step % self.metrics_interval == 0:
                ts['t'].append(t)
                ts['f'].append(grid.state.f)
                ts['Vdc'].append(dc_bus.state.Vdc)
                ts['P_fly'].append(flywheel.state.P_mech / 1000)
                ts['SOC'].append(flywheel.state.SOC)
                ts['mode'].append(orchestrator.mode.value)
                ts['P_load'].append(P_load_active / 1000)
                ts['P_grid'].append(inverter.state.P / 1000)

                vdc_samples.append(dc_bus.state.Vdc)
                soc_min = min(soc_min, current_soc)
                soc_max = max(soc_max, current_soc)
                total_loss += flywheel.state.P_loss * self.Ts * self.metrics_interval
                max_df_observed = max(max_df_observed, abs(grid.state.f - 50.0))

                # 频率响应时间检测
                if not freq_responded and abs(grid.state.f - 50.0) > 0.1:
                    if abs(po_state.P_actual) > 10e3:
                        freq_response_time = t
                        freq_responded = True

            step += 1

        # ── 计算指标 ──
        if vdc_samples:
            metrics.vdc_mean = sum(vdc_samples) / len(vdc_samples)
            metrics.vdc_std = (sum((v - metrics.vdc_mean)**2 for v in vdc_samples)
                              / len(vdc_samples)) ** 0.5
            metrics.vdc_max_dev = max(abs(v - dc_bus.cfg.V_nom) for v in vdc_samples) / dc_bus.cfg.V_nom * 100

        metrics.freq_response_t = freq_response_time * 1000 if freq_responded else -1
        metrics.freq_power_peak = max(abs(v) for v in ts['P_fly']) * 1000 if ts['P_fly'] else 0
        energy_kwh = sum(abs(ts['P_fly'][i]) * (ts['t'][i+1]-ts['t'][i]) / 3600
                        for i in range(len(ts['P_fly'])-1)) if len(ts['P_fly']) > 1 else 0
        metrics.freq_energy = energy_kwh

        metrics.soc_final = flywheel.state.SOC
        metrics.soc_range = soc_max - soc_min
        metrics.total_loss_kWh = total_loss / 3.6e6
        metrics.efficiency = 1 - metrics.total_loss_kWh / max(energy_kwh, 0.001)

        # SOC 越限检测
        metrics.soc_violations = sum(
            1 for i in range(len(ts['SOC']))
            if ts['SOC'][i] < 0.05 or ts['SOC'][i] > 0.95
        )

        # ── 综合评分 ──
        metrics.composite_score = self._compute_composite_score(metrics, scenario, max_df_observed)

        return ExperimentResult(
            scenario=scenario,
            strategy_name=strategy_name,
            metrics=metrics,
            time_series=ts,
            decisions=decisions,
            elapsed_ms=(time.time() - t_start) * 1000,
        )

    def _apply_events(self, grid: GridSimulator, events: List[GridEvent], t: float) -> float:
        """应用电网事件, 返回当前负载功率"""
        P_load = 0.0
        for ev in events:
            if t < ev.t_start or t > ev.t_end:
                continue

            if ev.event_type == 'freq_dip':
                if ev.ramp_rate > 0:
                    dt = min(t - ev.t_start, (ev.t_end - ev.t_start))
                    f_target = 50 + ev.magnitude * min(1.0, dt / abs(ev.magnitude / ev.ramp_rate))
                else:
                    f_target = 50 + ev.magnitude
                grid.set_frequency_deviation(f_target - 50.0)

            elif ev.event_type == 'freq_swell':
                if ev.ramp_rate > 0:
                    dt = min(t - ev.t_start, (ev.t_end - ev.t_start))
                    f_target = 50 + ev.magnitude * min(1.0, dt / abs(ev.magnitude / ev.ramp_rate))
                else:
                    f_target = 50 + ev.magnitude
                grid.set_frequency_deviation(f_target - 50.0)

            elif ev.event_type == 'freq_osc':
                # 正弦振荡
                freq_amp = ev.magnitude
                f_target = 50 + freq_amp * math.sin(2 * math.pi * 0.5 * (t - ev.t_start))
                grid.set_frequency_deviation(f_target - 50.0)

            elif ev.event_type == 'v_sag':
                grid.set_voltage(ev.magnitude)

            elif ev.event_type == 'v_swell':
                grid.set_voltage(ev.magnitude)

            elif ev.event_type == 'phase_jump':
                # 简化: 注入相位跳变
                grid.inject_fault(GridFault.PHASE_JUMP)

            elif ev.event_type == 'load_step':
                P_load = abs(ev.magnitude)

        return P_load

    def _compute_composite_score(self, m: ExperimentMetrics, s: ScenarioConfig,
                                  max_df_observed: float = 0.0) -> float:
        """加权综合评分 (0-100, 越高越好)

        [FIX-2026-07 迭代2] 原评分公式对响应时间线性扣分且 200ms 归零,
        而系统的决策周期(5ms采样)+ 飞轮惯量爬升 + droop 一次调频的物理响应
        时间通常在 0.5~1s 量级, 属于正常, 不应被评为 0 分。
        与此同时, 原公式对"从未响应"给出中性 50 分默认值, 造成一个从不
        参与调频支撑的策略反而比慢速但真实响应的策略得分更高的悖论
        (例如: 完全不理会频率骤降的固定VDC策略 58.1分 > 705ms内完成
        droop响应的固定调频策略 24.5分)。这会在实际验收测试中把
        "不作为"误判为优等策略, 是评估方法学上的重大缺陷。

        新公式:
          1. 用指数衰减代替线性衰减, 时间常数 τ=600ms, 更符合飞轮+droop
             的真实响应特性 (100ms→~85分, 500ms→~43分, 1s→~19分)
          2. 区分"场景本身无需响应"(max_df_observed 很小) 与
             "场景确有事件但策略未响应"两种情况: 后者按事件严重程度扣分,
             不再给中性分, 避免不作为策略被高估
        """

        # Vdc 评分: 偏差越小越好
        vdc_score = max(0, 100 - m.vdc_max_dev * 2) if m.vdc_max_dev > 0 else 100

        # 频率响应: 指数衰减模型, 响应越快分越高; 真实无响应且确有事件则按事件严重度扣分
        FREQ_EVENT_THRESHOLD = 0.06  # Hz, 低于此视为噪声/无实质事件
        if m.freq_response_t > 0:
            freq_score = 100 * math.exp(-m.freq_response_t / 600.0)  # τ=600ms
        elif max_df_observed < FREQ_EVENT_THRESHOLD:
            freq_score = 100  # 场景本身没有明显频率事件, 不响应是正确的
        else:
            # 确有事件但从未响应 → 按超出阈值的严重程度扣分, 而非给中性分
            severity = max_df_observed - FREQ_EVENT_THRESHOLD
            freq_score = max(0, 35 - severity * 300)

        # SOC: 不越限满分
        soc_score = max(0, 100 - m.soc_violations * 20)

        # 效率
        eff_score = m.efficiency * 100

        total = (s.weight_vdc_reg * vdc_score +
                s.weight_freq_support * freq_score +
                s.weight_soc_mgmt * soc_score +
                s.weight_efficiency * eff_score) / (
                    max(s.weight_vdc_reg + s.weight_freq_support +
                        s.weight_soc_mgmt + s.weight_efficiency, 0.001))

        return total

    def run_all(self) -> List[ExperimentResult]:
        """跑所有场景, 用 AI adaptive 策略"""
        controller = S4AIController(strategy=AIStrategy.ADAPTIVE)
        scenarios = build_all_scenarios()
        results = []

        print(f"\n{'='*70}")
        print(f"  S4 实验: AI Adaptive vs 基线 × {len(scenarios)} 场景")
        print(f"{'='*70}\n")

        for sc in scenarios:
            print(f"\n▸ 场景 {sc.scenario_id.value}: {sc.name}")
            print(f"  {sc.description}")

            # 跑 AI
            result = self.run_scenario(sc, controller, controller.get_strategy_name())
            results.append(result)
            m = result.metrics
            print(f"  ✓ {result.strategy_name}: "
                  f"综合={m.composite_score:.1f} | "
                  f"Vdc偏差={m.vdc_max_dev:.1f}% | "
                  f"响应={m.freq_response_t:.0f}ms | "
                  f"SOC={m.soc_final:.1%}")

        return results

    def run_compare(self):
        """三策略对比 + 两个基线"""
        strategies = [
            ("基线-固定VDC", BaselineFixedVDC()),
            ("基线-固定调频", BaselineFixedFreq()),
            ("基线-固定VSG", BaselineFixedVSG()),
            ("AI 规则引擎", S4AIController(strategy=AIStrategy.RULE_BASED)),
            ("AI 自适应", S4AIController(strategy=AIStrategy.ADAPTIVE)),
            ("AI 预测", S4AIController(strategy=AIStrategy.PREDICTIVE)),
        ]

        scenarios = build_all_scenarios()

        print(f"\n{'='*80}")
        print(f"  S4 策略对比: {len(strategies)} 策略 × {len(scenarios)} 场景")
        print(f"{'='*80}")

        # 汇总表
        all_results = {}
        for sc in scenarios:
            print(f"\n{'─'*80}")
            print(f"  场景 {sc.scenario_id.value}: {sc.name}")
            all_results[sc.name] = []
            for sname, ctrl in strategies:
                r = self.run_scenario(sc, ctrl, sname)
                all_results[sc.name].append(r)
                m = r.metrics
                print(f"    {sname:16s} │ 综合={m.composite_score:5.1f} │ "
                      f"Vdc偏差={m.vdc_max_dev:4.1f}% │ "
                      f"响应={m.freq_response_t:5.0f}ms │ "
                      f"SOC={m.soc_final:.2f}")

        # ── 排名汇总 ──
        print(f"\n{'='*80}")
        print(f"  综合排名 (平均得分)")
        print(f"{'─'*80}")
        scores = defaultdict(list)
        for sc_name, sc_results in all_results.items():
            for r in sc_results:
                scores[r.strategy_name].append(r.metrics.composite_score)

        ranked = sorted(scores.items(), key=lambda x: sum(x[1])/len(x[1]), reverse=True)
        for i, (name, sc_list) in enumerate(ranked):
            avg = sum(sc_list) / len(sc_list)
            medal = ['🥇', '🥈', '🥉', '  '][min(i, 3)]
            print(f"  {medal} {name:20s}  平均={avg:5.1f}  (min={min(sc_list):.0f}, max={max(sc_list):.0f})")

        return all_results

    def print_summary_table(self, results: List[ExperimentResult]):
        """打印结果汇总表"""
        print(f"\n{'场景':20s} │ {'Vdc偏差%':>8s} │ {'响应ms':>7s} │ {'SOC终':>6s} │ {'综合分':>6s}")
        print(f"{'─'*20}─┼─{'─'*8}─┼─{'─'*7}─┼─{'─'*6}─┼─{'─'*6}")
        for r in results:
            m = r.metrics
            print(f"{r.scenario.name:20s} │ {m.vdc_max_dev:7.1f}% │ "
                  f"{m.freq_response_t:6.0f}ms │ {m.soc_final:5.1%} │ {m.composite_score:5.1f}")


# ═══════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='S4 AI Control Experiments')
    parser.add_argument('--scenario', type=int, help='Run single scenario by ID (1-10)')
    parser.add_argument('--compare', action='store_true', help='Compare all strategies')
    parser.add_argument('--all', action='store_true', help='Run all scenarios with AI adaptive')

    args = parser.parse_args()
    runner = S4ExperimentRunner()

    if args.compare:
        runner.run_compare()
    elif args.scenario:
        scenarios = build_all_scenarios()
        sc = next((s for s in scenarios if s.scenario_id.value == args.scenario), None)
        if sc:
            ctrl = S4AIController(strategy=AIStrategy.ADAPTIVE)
            r = runner.run_scenario(sc, ctrl, ctrl.get_strategy_name())
            runner.print_summary_table([r])
            print(f"\n决策记录 ({len(r.decisions)} 次, 显示关键决策):")
            shown = 0
            for d in r.decisions:
                if d['mode'] != 'IDLE' or shown < 3:
                    print(f"  t={d['t']:.3f}s  mode={d['mode']:22s}  {d['reason']}")
                    shown += 1
                if shown >= 30:
                    print(f"  ... ({len(r.decisions)-shown} 条省略)")
                    break
        else:
            print(f"场景 ID {args.scenario} 不存在 (1-10)")
    else:
        runner.run_all()
