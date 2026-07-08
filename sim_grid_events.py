#!/usr/bin/env python3
"""
sim_grid_events.py — 电网事件批处理仿真: 遍历场景并生成汇总报告。

子系统: 应用层
依赖: grid_sim.py, s4_scenarios.py

电网事件批处理仿真: 遍历场景并生成汇总报告。
"""
import math, time
import numpy as np
from dataclasses import dataclass
from typing import List, Dict

from system_config import DEFAULT_CONFIG
from motor_base import HIMMotor, HIMConfig
from him_controller import HIMController
from flywheel_energy import FlywheelEnergyStorage
from power_ops import PowerOrchestrator, PowerMode

# ═══════════════════════════════════════
# 场景定义
# ═══════════════════════════════════════

@dataclass
class GridScenario:
    name: str
    desc: str
    duration: float       # s
    events: List[Dict]    # [{t_start, t_end, type, params}]
    init_soc: float = 0.5
    init_rpm: float = 4400  # 初始转速 (工作区间中位)


# 预计算: SOC ≈ (ω² - ω_min²) / (ω_max² - ω_min²)
# omega_min=4400rpm, omega_max=8800rpm
def rpm_to_soc(rpm):
    w = rpm; wm = 4400; wx = 8800
    return max(0.0, min(1.0, (w*w - wm*wm) / (wx*wx - wm*wm)))

SCENARIOS = [
    GridScenario(
        name="频率骤降 50→49.5Hz",
        desc="电网频率跌落 → HIM 飞轮应放电支撑电网，125kW 持续 2.5s",
        duration=4.0,
        init_soc=rpm_to_soc(6600), init_rpm=6600,
        events=[
            {"t_start": 0.5, "t_end": 3.0, "type": "freq", "f": 49.5},
        ],
    ),
    GridScenario(
        name="频率骤升 50→50.5Hz",
        desc="电网频率上升 → HIM 应吸收功率充电 125kW，测试升速弱磁",
        duration=4.0,
        init_soc=rpm_to_soc(5500), init_rpm=5500,
        events=[
            {"t_start": 0.5, "t_end": 3.0, "type": "freq", "f": 50.5},
        ],
    ),
    GridScenario(
        name="负载突增 0→800kW",
        desc="800kW 负载阶跃 1.5s → 飞轮快速放电稳压，HIM 励磁需维持转矩",
        duration=3.0,
        init_soc=rpm_to_soc(7700), init_rpm=7700,
        events=[
            {"t_start": 0.5, "t_end": 2.0, "type": "load", "P": 800e3},
        ],
    ),
    GridScenario(
        name="电压暂降 LVRT (750→525V)",
        desc="直流母线跌至 0.7pu + 300kW 持续负载 → 弱磁自动降励磁维持控制",
        duration=2.0,
        init_soc=rpm_to_soc(6600), init_rpm=6600,
        events=[
            {"t_start": 0.4, "t_end": 0.9, "type": "vdc", "vdc": 525.0},
            {"t_start": 0.4, "t_end": 1.2, "type": "load", "P": 300e3},  # 加负载才有V_util变化
        ],
    ),
    GridScenario(
        name="频率振荡 50±0.3Hz",
        desc="电网频率正弦振荡 4s → HIM VSG 阻尼 + 励磁调速跟踪",
        duration=6.0,
        init_soc=rpm_to_soc(6600), init_rpm=6600,
        events=[
            {"t_start": 1.0, "t_end": 5.0, "type": "freq_osc", "amp": 0.3, "period": 1.0},
        ],
    ),
    GridScenario(
        name="复合应力 (频率跌+负载+电压暂降)",
        desc="t=0.3s 49.6Hz → t=1s 600kW → t=2.5s 550V 三连击",
        duration=5.0,
        init_soc=rpm_to_soc(7700), init_rpm=7700,
        events=[
            {"t_start": 0.3, "t_end": 2.5, "type": "freq", "f": 49.6},
            {"t_start": 1.0, "t_end": 2.5, "type": "load", "P": 600e3},
            {"t_start": 2.5, "t_end": 3.0, "type": "vdc", "vdc": 550.0},
        ],
    ),
    GridScenario(
        name="HIM 关励磁零损耗待机",
        desc="无负载时 If→0，反电势崩溃，飞轮惰转几乎零损耗 — 感应子独有优势",
        duration=1.0,
        init_soc=rpm_to_soc(7700), init_rpm=7700,
        events=[],  # 全程无事件，IDLE 模式
    ),
]


# ═══════════════════════════════════════
# 仿真核心
# ═══════════════════════════════════════

def run_scenario(scenario: GridScenario, dt: float = 100e-6, plot: bool = True):
    """运行单个电网场景仿真"""

    # ── 构建全链路 ──
    cfg = DEFAULT_CONFIG.to_him_config()
    fw_cfg = DEFAULT_CONFIG.to_flywheel_config()

    fw = FlywheelEnergyStorage(fw_cfg)
    fw.state.omega = scenario.init_rpm * 2 * math.pi / 60

    motor = HIMMotor(cfg)
    motor.state.omega_m = fw.state.omega

    orch = PowerOrchestrator(fw, motor=motor, P_nom=500e3, ts=dt)

    steps = int(scenario.duration / dt)
    record_every = max(1, steps // 600)

    records = []
    t0 = time.time()

    # ── 事件预处理: 时间→动作映射 ──
    def get_grid_state(t: float) -> dict:
        f = 50.0
        P_load = 0.0
        Vdc = 750.0

        for ev in scenario.events:
            if t < ev["t_start"] or t > ev["t_end"]:
                continue
            if ev["type"] == "freq":
                f = ev["f"]
            elif ev["type"] == "load":
                P_load = ev["P"]
            elif ev["type"] == "vdc":
                Vdc = ev["vdc"]
            elif ev["type"] == "freq_osc":
                progress = (t - ev["t_start"]) / (ev["t_end"] - ev["t_start"])
                f = 50.0 + ev["amp"] * math.sin(progress * 2 * math.pi * (ev["t_end"] - ev["t_start"]) / ev["period"])

        return {"f": f, "P_load": P_load, "Vdc": Vdc}

    # ── 运行 ──
    for i in range(steps):
        t = i * dt
        gs = get_grid_state(t)

        # 功率调度
        # FREQ_REGULATION: p_flywheel = -P_ref (内部翻号), 所以 df<0 时 P_ref 为正
        # POWER_TRACKING:   p_flywheel = P_ref (不翻号), 负载突增→负P放电
        df = gs["f"] - 50.0
        if abs(df) > 0.02 and gs["P_load"] == 0:
            # 调频: df=-0.5Hz → P_sched=+125kW → p_flywheel=-125kW → 放电 ✅
            P_sched = -df / 50.0 / 0.04 * 500e3  # droop 4%
            mode = PowerMode.FREQ_REGULATION
        elif gs["P_load"] != 0:
            # 负载突增 800kW → 飞轮放电 → P_sched=-800kW → p_flywheel=-800kW ✅
            P_sched = -gs["P_load"]
            mode = PowerMode.POWER_TRACKING
        else:
            P_sched = 0.0
            mode = PowerMode.IDLE

        orch.step(Vdc=gs["Vdc"], f_grid=gs["f"],
                  SOC=fw.state.SOC,
                  P_schedule=P_sched, mode=mode,
                  P_load=gs["P_load"])

        # ── 记录 ──
        if i % record_every == 0:
            ms = motor.state
            him_ctrl = orch._him_ctrl if hasattr(orch, '_him_ctrl') else None
            fw_status = him_ctrl.get_status() if him_ctrl else {}

            records.append({
                "t": t,
                "f_grid": gs["f"],
                "Vdc": gs["Vdc"],
                "P_load": gs["P_load"] / 1e3,
                "P_ref": orch.state.P_ref / 1e3,
                "P_actual": orch.state.P_actual / 1e3,
                "rpm": fw.rpm,
                "SOC": fw.state.SOC * 100,
                "Te": ms.Te,
                "Id": ms.Id,
                "Iq": ms.Iq,
                "If": ms.If,
                "V_util": motor.voltage_utilization * 100,
                "fw_active": fw_status.get("flux_weak", False),
                "If_ref": fw_status.get("If_ref", 0),
            })

    elapsed = time.time() - t0

    # ── 结果汇总 ──
    rec = {k: np.array([d[k] for d in records]) for k in records[0].keys()}

    print(f"\n{'='*60}")
    print(f"  {scenario.name}")
    print(f"  {scenario.desc}")
    print(f"{'='*60}")
    print(f"  步数={steps:,} 耗时={elapsed:.2f}s")
    print(f"  rpm: {scenario.init_rpm:.0f} → {fw.rpm:.0f}")
    print(f"  SOC: {scenario.init_soc*100:.0f}% → {fw.state.SOC*100:.1f}%")
    print(f"  If: {motor.state.If:.2f}A")
    print(f"  V_util 峰值: {np.max(rec['V_util']):.0f}%")
    print(f"  弱磁触发: {'✅' if np.any(rec['fw_active']) else '—'}")
    print(f"  关励磁待机: {'✅' if np.any(rec['If'] < 0.5) else '—'}")

    # ── 绘图 ──
    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
            fig.suptitle(f"HIM 飞轮 — {scenario.name}", fontsize=14, fontweight='bold')

            # 1. 电网频率 + 负载
            ax = axes[0]
            ax.plot(rec['t'], rec['f_grid'], 'b-', lw=2, label='电网频率 Hz')
            ax2 = ax.twinx()
            ax2.fill_between(rec['t'], 0, rec['P_load'], alpha=0.15, color='red', label='负载 kW')
            ax.set_ylabel('频率 (Hz)'); ax.grid(True, alpha=0.3)
            ax.legend(loc='upper left', fontsize=8)

            # 2. 转速 + SOC
            ax = axes[1]
            ax.plot(rec['t'], rec['rpm'], 'b-', lw=2, label='转速 rpm')
            ax2 = ax.twinx()
            ax2.plot(rec['t'], rec['SOC'], 'g--', lw=1.5, label='SOC %')
            ax.set_ylabel('转速 (rpm)'); ax.grid(True, alpha=0.3)
            ax.legend(loc='upper left', fontsize=8)
            ax2.legend(loc='upper right', fontsize=8)

            # 3. 功率
            ax = axes[2]
            ax.plot(rec['t'], rec['P_ref'], 'b-', lw=1.5, label='P_ref kW')
            ax.plot(rec['t'], rec['P_actual'], 'r--', lw=1, label='P_actual kW')
            ax.axhline(0, color='gray', ls=':', alpha=0.5)
            ax.set_ylabel('功率 (kW)'); ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

            # 4. HIM 电流 Id/Iq/If + 弱磁标志
            ax = axes[3]
            ax.plot(rec['t'], rec['Iq'], 'b-', lw=1.5, label='Iq (转矩)')
            ax.plot(rec['t'], rec['Id'], 'r--', lw=1, label='Id')
            ax.plot(rec['t'], rec['If'], 'g-', lw=2, label='If (励磁)')
            # 弱磁触发背景
            fw_mask = rec['fw_active']
            if np.any(fw_mask):
                t_vals = rec['t'][fw_mask]
                if len(t_vals) > 1:
                    ax.axvspan(t_vals[0], t_vals[-1], alpha=0.1, color='orange', label='弱磁触发')
            ax.set_ylabel('电流 (A)'); ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7, ncol=4)

            # 5. 电压利用率 + 弱磁阈值
            ax = axes[4]
            ax.fill_between(rec['t'], 0, rec['V_util'], alpha=0.3, color='blue')
            ax.axhline(95, color='red', ls='--', alpha=0.5, label='弱磁阈值 (95%)')
            ax.axhline(100, color='darkred', ls=':', alpha=0.3, label='SVPWM 极限')
            ax.set_ylabel('V_util (%)'); ax.set_xlabel('时间 (秒)')
            ax.set_ylim(0, 150); ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

            plt.tight_layout()
            fname = f"/tmp/him_grid_{scenario.name[:6].replace(' ','_')}.png"
            plt.savefig(fname, dpi=120); plt.close()
            print(f"  📊 {fname}")
            return fname
        except ImportError:
            pass
    return None


# ═══════════════════════════════════════
# 主程序
# ═══════════════════════════════════════

if __name__ == "__main__":
    import sys
    args = sys.argv[1:]
    scenario_idx = int(args[0]) if args and args[0].isdigit() else None
    run_all = "all" in args

    if run_all:
        print("=== HIM 飞轮 — 电网事件联合仿真 (全部 6 场景) ===\n")
        for i, sc in enumerate(SCENARIOS):
            try:
                run_scenario(sc)
            except Exception as e:
                import traceback
                print(f"  ❌ 场景 [{i}] {sc.name} 失败: {e}")
                traceback.print_exc()
    elif scenario_idx is not None:
        run_scenario(SCENARIOS[scenario_idx])
    else:
        print("=== HIM 飞轮 — 电网事件联合仿真 ===\n")
        for i, sc in enumerate(SCENARIOS):
            print(f"  [{i}] {sc.name}")
        print(f"\n用法: python3 sim_grid_events.py <编号>")
        print(f"或:   python3 sim_grid_events.py all")
