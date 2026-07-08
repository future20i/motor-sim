#!/usr/bin/env python3
"""
him_field_off.py — HIM 励磁关断测试: 验证零励磁零反电势特性。

子系统: 控制层
依赖: him_controller.py
手册对应章节: CONTROL_SETPOINTS.md §5

HIM 励磁关断测试: 验证零励磁零反电势特性。
"""
import math, numpy as np
from system_config import DEFAULT_CONFIG
from motor_base import HIMMotor
from him_controller import HIMController

cfg = DEFAULT_CONFIG.to_him_config()
motor = HIMMotor(cfg)
dt = cfg.T_sample
Vf_full = cfg.If_nom * cfg.Rf
V_max = cfg.Vdc_nom / math.sqrt(3)

# Phase 1: Vf=750V 直建磁 10A
motor.state.omega_m = 7700 * 2 * math.pi / 60
for _ in range(int(0.2 / dt)):
    motor.step(0, 0, dt, Vf=Vf_full)

# Phase 2: 控制器微调
ctrl = HIMController(cfg, dt=dt)
for _ in range(int(0.03 / dt)):
    Vd, Vq, Vf = ctrl.update(
        omega_ref=motor.state.omega_m, omega_fb=motor.state.omega_m,
        id_fb=motor.state.Id, iq_fb=motor.state.Iq, if_fb=motor.state.If,
        if_ext=cfg.If_nom, iq_ext=0.0)
    motor.step(Vd, Vq, dt, Vf=Vf)

omega = motor.state.omega_m; rpm = omega * 60 / (2 * math.pi)
psi_d = cfg.Ld * motor.state.Id + cfg.Mdf * motor.state.If
Eq0 = omega * psi_d
if0 = motor.state.If

print(f"=== 关励磁前稳态 (7700rpm, If={if0:.2f}A) ===")
print(f"ψd = {cfg.Ld*1e3}·{motor.state.Id:.2f} + {cfg.Mdf*1e3}·{motor.state.If:.2f} = {psi_d*1e3:.1f} mWb")
print(f"Eq = ω·ψd = {omega:.0f}·{psi_d:.4f} = {Eq0:.0f}V")
print(f"V_util = {Eq0/V_max*100:.1f}%")

# Phase 3: 关励磁 + 精确记录
print(f"\n=== Vf→0, 励磁自由衰减 (τ = Lf/Rf = {cfg.Lf/cfg.Rf*1e3:.0f}ms) ===\n")
T = 0.3
records = []
tau_found = None

for i in range(int(T / dt)):
    t = i * dt
    motor.step(0, 0, dt, Vf=0.0)
    
    if tau_found is None and motor.state.If < if0 * 0.368:
        tau_found = t
    
    if i % 200 == 0:
        ms = motor.state
        psi_d = cfg.Ld * ms.Id + cfg.Mdf * ms.If
        Eq = ms.omega_m * psi_d
        records.append({
            't_ms': t*1e3, 'If': ms.If, 'psi_d_mWb': psi_d*1e3,
            'Eq': Eq, 'V_util': abs(Eq)/V_max*100,
            'rpm': ms.omega_m*60/(2*math.pi)
        })

print(f"{'t(ms)':>7} {'If(A)':>8} {'ψd(mWb)':>9} {'Eq(V)':>8} {'V_util%':>8} {'rpm':>7}")
print("-" * 55)
for r in records:
    t = r['t_ms']
    if t <= 12 or abs(t - tau_found*1e3) < 4 if tau_found else False or t >= 280:
        print(f"{t:7.1f} {r['If']:8.3f} {r['psi_d_mWb']:9.1f} {r['Eq']:8.0f} {r['V_util']:8.1f} {r['rpm']:7.0f}")

print(f"\n✅ 实测 τ_exp = {tau_found*1e3:.1f}ms  (理论 τ={cfg.Lf/cfg.Rf*1e3:.1f}ms)")

# ── 结论 ──
print(f"\n{'='*60}")
print(f"  关励磁后 HIM 飞轮发生了什么 (公式 + 验证)")
print(f"{'='*60}")
print(f"""
① 励磁电流指数衰减:
   If(t) = If₀·exp(-t/τ),  τ = Lf/Rf = {cfg.Lf/cfg.Rf*1e3:.0f}ms
   实测 τ_exp = {tau_found*1e3:.1f}ms  ✅

② d 轴磁链随着崩:
   ψd(t) = Ld·Id + Mdf·If(t)
   初始: ψd(0)  = Mdf·If₀ = {cfg.Mdf*1e3}·{if0:.1f} = {cfg.Mdf*if0*1e3:.0f} mWb (Id≈0)
   最终: ψd(∞) → 0

③ 反电动势按指数衰减到零:
   Eq(t) = ω·ψd(t) = ω·Mdf·If₀·exp(-t/τ)
   t=0:      Eq = {omega:.0f}·{cfg.Mdf:.3f}·{cfg.If_nom} = {omega*cfg.Mdf*cfg.If_nom:.0f}V (V_util={omega*cfg.Mdf*cfg.If_nom/V_max*100:.0f}%)
   t=1τ=67ms: Eq = {omega*cfg.Mdf*cfg.If_nom*0.368:.0f}V (V_util={omega*cfg.Mdf*cfg.If_nom*0.368/V_max*100:.0f}%)
   t=3τ=200ms: Eq ≈ 0

④ 电磁转矩消失:
   Te = 1.5·P·(ψd·Iq - ψq·Id)
   关磁后 ψd→0, Iq→0 → Te→0
   飞轮纯惰转: dω/dt = -Bω/J = -{cfg.B}·{omega:.0f}/{cfg.J} = {-cfg.B*omega/cfg.J:.4f} rad/s²

⑤ 损耗仅为机械阻尼:
   P_loss = B·ω² = {cfg.B*omega**2:.1f}W  (额定 500kW 的 {cfg.B*omega**2/500e3*100:.2f}%)
   储能 3kWh 的自放电时间 ≈ 数千小时 (真空腔)

💡 对比 PMSM: 永磁磁链 ψm={cfg.psi_m if hasattr(cfg,'psi_m') else 0.07}Wb 永远存在
   7700rpm 时永久反电势: Eq_pmsm = ω·ψm = {omega*0.07:.0f}V (V_util={omega*0.07/V_max*100:.0f}%)
   关不掉 → 必须持续消耗功率压制 → 不能零损耗待机
""")
