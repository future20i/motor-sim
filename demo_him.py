"""
demo_him.py — HIM 感应子电机演示: 充放电循环。

子系统: 应用层
依赖: him_controller.py, motor_base.py (HIM)
手册对应章节: INSTALLATION.md §4.1

HIM 感应子电机演示: 充放电循环。
"""
import argparse, math, time
import numpy as np
from motor_base import HIMConfig, HIMMotor
from him_controller import HIMController


def sim_quick(cfg: HIMConfig, dt: float, T_total: float, show_plot: bool = True):
    """快速仿真: 开环直驱验证 HIM 核心物理"""
    motor = HIMMotor(cfg)
    Vf_full = cfg.If_nom * cfg.Rf      # 满励磁电压
    V_bus = cfg.Vdc_nom / np.sqrt(3)   # 最大相电压
    steps = int(T_total / dt)
    record_every = max(1, steps // 500)

    records = []
    mode = "励磁"
    t0 = time.time()

    for i in range(steps):
        t = i * dt

        # ── 模式调度 ──
        if t < 0.05:
            # Phase 0: 预励磁 (直给满Vf, 不等PI)
            Vd, Vq, Vf = 0, 0, Vf_full
            mode = "🔌 Pre-excite"

        elif t < 0.8:
            # Phase 1: 满功率加速
            Vd, Vq, Vf = 0, V_bus * 0.9, Vf_full
            mode = "⚡ Accelerate"

        elif t < 0.9:
            # Phase 2: 关励磁 → 反电势坍塌
            Vd, Vq, Vf = 0, 0, 0.0
            mode = "⏸  Field-OFF"

        elif t < 1.3:
            # Phase 3: 重励磁 + 反向制动
            Vd, Vq, Vf = 0, -V_bus * 0.6, Vf_full
            mode = "🔋 Brake"

        elif t < 1.5:
            # Phase 4: 弱磁演示 (降励磁电流)
            Vd, Vq, Vf = 0, V_bus * 0.5, Vf_full * 0.3
            mode = "🌀 Flux-Weak"

        else:
            Vd, Vq, Vf = 0, 0, 0
            mode = "⏹  Stop"

        # ── 物理步进 ──
        motor.step(Vd, Vq, dt, Vf)

        # ── 记录 ──
        if i % record_every == 0:
            Vd_est = -motor.state.omega_m * cfg.Lq * motor.state.Iq
            Vq_est = motor.state.omega_m * (cfg.Ld * motor.state.Id + cfg.Mdf * motor.state.If)
            V_mag = np.sqrt(Vd_est**2 + Vq_est**2)
            v_util = V_mag / V_bus if V_bus > 0 else 0

            records.append({
                't': t, 'mode': mode,
                'omega': motor.state.omega_m * 60 / (2 * math.pi),
                'Id': motor.state.Id, 'Iq': motor.state.Iq, 'If': motor.state.If,
                'Te': motor.state.Te, 'Vd': Vd, 'Vq': Vq, 'Vf': Vf,
                'V_util': v_util,
            })

    elapsed = time.time() - t0
    s = motor.state
    rpm = s.omega_m * 60 / (2 * math.pi)
    v_util_final = records[-1]['V_util'] if records else 0

    print(f"  步数={steps:,} 耗时={elapsed:.2f}s")
    print(f"  最终: ω={rpm:.0f}rpm Id={s.Id:.1f}A Iq={s.Iq:.1f}A If={s.If:.1f}A Te={s.Te:.1f}Nm")

    # 关键事件检测
    rpm_max = max(r['omega'] for r in records)
    if_max = max(r['If'] for r in records)
    if_off = min(r['If'] for r in records if 'Field-OFF' in r['mode'])
    vutil_peak = max(r['V_util'] for r in records)
    vutil_off = min(r['V_util'] for r in records if 'Field-OFF' in r['mode'])

    print(f"  峰值转速: {rpm_max:.0f}rpm  If_max={if_max:.1f}A  V_util_peak={vutil_peak:.0%}")
    print(f"  关磁后: If_min={if_off:.2f}A  V_util_min={vutil_off:.0%}")
    print(f"  励磁建立: {'✅' if if_max>cfg.If_nom*0.8 else '⚠️'}")
    print(f"  关磁降压: {'✅' if vutil_off<0.3 else '⚠️'} (V_util {vutil_peak:.0%}→{vutil_off:.0%})")
    print(f"  弱磁演示: {'✅ 已触发' if vutil_peak>0.85 else '(转速不够)'}")

    # ── 绘图 ──
    if show_plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            rec = {k: np.array([d[k] for d in records]) for k in records[0].keys()}
            fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
            fig.suptitle(f"HIM 感应子电机仿真 (Vdc={cfg.Vdc_nom}V, P={cfg.P}, J={cfg.J}kg·m²)",
                         fontsize=13, fontweight='bold')

            # 1. 转速
            ax = axes[0]
            ax.plot(rec['t'], rec['omega'], 'b-', lw=2)
            ax.set_ylabel('转速 (rpm)'); ax.grid(True, alpha=0.3)
            # 模式背景
            cmap = {'Pre': '#90EE90', 'Acc': '#FFB347', 'Field': '#D3D3D3',
                    'Brake': '#FF6B6B', 'Flux': '#87CEEB', 'Stop': '#FFF'}
            for mode_kw, color in [('Pre', '#90EE90'), ('Acc', '#FFB347'),
                                    ('Field', '#D3D3D3'), ('Brake', '#FF6B6B'),
                                    ('Flux', '#87CEEB')]:
                mask = [mode_kw in m for m in rec['mode']]
                if np.any(mask):
                    t_vals = rec['t'][mask]
                    if len(t_vals) > 1:
                        ax.axvspan(t_vals[0], t_vals[-1], alpha=0.08, color=color)

            # 2. 电流 Id/Iq/If
            ax = axes[1]
            ax.plot(rec['t'], rec['Iq'], 'b-', lw=2, label='Iq (转矩)')
            ax.plot(rec['t'], rec['Id'], 'r--', lw=1, label='Id')
            ax.plot(rec['t'], rec['If'], 'g-.', lw=2, label='If (励磁)')
            ax.set_ylabel('电流 (A)'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

            # 3. 电压利用率 + 弱磁阈值
            ax = axes[2]
            ax.fill_between(rec['t'], 0, rec['V_util'], alpha=0.3, color='blue')
            ax.axhline(0.85, color='red', ls='--', alpha=0.5, label='弱磁阈值')
            ax.set_ylabel('电压利用率'); ax.set_ylim(0, 1.3)
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

            # 4. 转矩
            ax = axes[3]
            ax.plot(rec['t'], rec['Te'], 'purple', lw=1.5)
            ax.set_ylabel('Te (Nm)'); ax.set_xlabel('时间 (秒)'); ax.grid(True, alpha=0.3)

            plt.tight_layout()
            out = "/tmp/him_simulation.png"
            plt.savefig(out, dpi=120); plt.close()
            print(f"\n📊 图表: {out}")
            return out
        except ImportError:
            pass
    return None


def main():
    parser = argparse.ArgumentParser(description="HIM 感应子电机仿真")
    parser.add_argument("--big", action="store_true", help="泓慧 500kW 参数")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    if args.big:
        cfg = HIMConfig(Vdc_nom=750.0, P=6, J=33.9, Rs=0.005, Ld=0.002, Lq=0.002,
                        Mdf=0.01, Rf=75.0, Lf=5.0, If_nom=10.0, If_max=10.0,
                        I_max=1000.0, N_max=8800.0, T_sample=200e-6)
        dt, T = 200e-6, 60.0
    else:
        cfg = HIMConfig(Vdc_nom=200.0, P=2, J=0.005, Rs=0.1, Ld=0.002, Lq=0.002,
                        Mdf=0.02, Rf=1.0, Lf=0.05, If_nom=8.0, If_max=8.0,
                        I_max=30.0, N_max=15000.0, T_sample=500e-6)
        dt, T = 500e-6, 1.8

    print(f"=== HIM 感应子电机仿真 ===")
    print(f"Vdc={cfg.Vdc_nom}V P={cfg.P} J={cfg.J}kg·m²")
    print(f"励磁→加速→关磁→制动→弱磁")
    sim_quick(cfg, dt, T, show_plot=not args.no_plot)


if __name__ == "__main__":
    main()
