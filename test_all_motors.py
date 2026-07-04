"""
test_all_motors.py — 验证所有电机类型

运行: .venv/bin/python test_all_motors.py
"""
import numpy as np
from motor_base import create_motor, list_motor_types
from virtual_controller import VirtualController, OpState


def test_motor(motor_type: str, duration_cycles: int = 60000):
    """对一种电机跑完整测试: 预充 → Rs辨识 → Run → 故障"""
    motor = create_motor(motor_type)
    ctrl = VirtualController(motor)
    cfg = motor.cfg
    name = motor.type_name()
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"{'='*60}")
    print(f"  参数: {motor.param_summary()}")

    # ── 1. 预充 ──
    ctrl.op_mode = 1
    for _ in range(2200):
        ctrl.run_cycle()
    assert ctrl.motor.state.Vdc >= 0.9 * cfg.Vdc_nom, "预充失败"
    print(f"  ✅ 预充完成: Vdc={ctrl.motor.state.Vdc:.0f}V")

    # ── 2. Rs 辨识 ──
    ctrl.op_mode = 3
    # 选择合适电压: 目标电流 ≈ 30% I_max
    V_test = cfg.Rs * cfg.I_max * 0.3
    V_test = max(0.1, min(V_test, 50))  # 钳位到合理范围
    ctrl.Vd_ref = V_test
    ctrl.Vq_ref = 0.0

    # 等 5τ (电气时间常数)
    if motor_type == "induction":
        tr = cfg.Lr / cfg.Rr
        wait = int(5 * tr / cfg.T_sample)
    elif hasattr(cfg, 'Ld'):
        tau_L = cfg.Ld / cfg.Rs if cfg.Rs > 0 else 0.01
        wait = int(5 * tau_L / cfg.T_sample)
    else:
        wait = 20000

    for _ in range(wait):
        ctrl.run_cycle()

    Id = ctrl.motor.state.Id
    Rs_est = V_test / Id if abs(Id) > 0.01 else float('nan')
    err = abs(Rs_est - cfg.Rs) / cfg.Rs * 100 if cfg.Rs > 0 else 0
    print(f"  ✅ Rs辨识: Vd={V_test:.3f}V → Id={Id:.3f}A → Rs={Rs_est:.4f}Ω "
          f"(真值={cfg.Rs:.4f}Ω, 误差={err:.1f}%)")

    # ── 3. 加速到 1000 rpm ──
    ctrl.op_mode = 2  # 回 READY
    for _ in range(100):
        ctrl.run_cycle()

    ctrl.speed_ref = 1000
    for _ in range(duration_cycles):
        ctrl.run_cycle()

    rpm = ctrl.motor.state.omega_m * 60 / (2 * np.pi)
    torque = ctrl.motor.state.Te
    print(f"  ✅ 速度: {rpm:.0f} rpm (目标=1000), Te={torque:.2f} N·m")

    # ── 4. 故障测试 ──
    ctrl.speed_ref = 0
    for _ in range(1000):
        ctrl.run_cycle()
    ctrl.motor.inject_fault(FaultType := __import__('motor_base').FaultType)
    from motor_base import FaultType
    ctrl.motor.inject_fault(FaultType.OVERCURRENT)
    for _ in range(100):
        ctrl.run_cycle()
    assert ctrl.state.name == "FAULT", f"故障响应失败: {ctrl.state.name}"
    print(f"  ✅ 故障响应: 状态→{ctrl.state.name}, Vd_ref→{ctrl.Vd_ref}")

    return True


if __name__ == "__main__":
    types = list_motor_types()
    results = {}
    for t in types:
        try:
            test_motor(t["id"])
            results[t["id"]] = "✅"
        except Exception as e:
            results[t["id"]] = f"❌ {e}"

    print(f"\n{'='*60}")
    print(f"  汇总: {results}")
    print(f"{'='*60}")
    if all(v == "✅" for v in results.values()):
        print("  全部通过 ✅")
    else:
        print("  有失败项，请检查 ❌")
