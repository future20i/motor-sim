"""
test_identify.py — 模拟 AI 执行 PMSM 参数辨识

启动仿真后运行此脚本，将自动执行:
1. 定子电阻 Rs 辨识 (直流注入法)
2. d轴电感 Ld 辨识 (高频注入法)

输出每个步骤的结果和与真值的误差。
"""
import asyncio
import time
import sys
from asyncua import Client


async def identify_rs(client, ns):
    """定子电阻辨识 — 直流注入法"""
    print("\n=== 步骤 1: 定子电阻 Rs 辨识 (直流注入法) ===")

    # 切换到辨识模式
    op_mode = client.get_node(f"ns={ns};s=op_mode")
    await op_mode.write_value(3)
    await asyncio.sleep(0.1)

    # 飞轮电机 Rs≈0.015Ω 很小, Vd=0.2V→Id≈13A
    # 如果不知道 Rs，从 0.5V 开始保守注入
    Vd_inject = 0.5
    print(f"  注入 Vd={Vd_inject}V, Vq=0V ...")

    vd_node = client.get_node(f"ns={ns};s=Vd_ref")
    vq_node = client.get_node(f"ns={ns};s=Vq_ref")
    await vd_node.write_value(Vd_inject)
    await vq_node.write_value(0.0)

    # 等电流稳定 (电气时间常数 τ=Ld/Rs, 飞轮电机 τ≈0.53s, 需等 ~3s)
    print("  等待电流稳定 (~3s)...")
    await asyncio.sleep(3.0)

    # 读 Id
    id_node = client.get_node(f"ns={ns};s=Id_actual")
    Id = await id_node.read_value()
    print(f"  稳态 Id = {Id:.3f} A")

    # 检查是否饱和 (如果 Id 接近 I_max=100A，说明电压太大，电流被限幅了)
    if Id > 95:
        print("  ⚠ Id 接近饱和! 降低注入电压重试")
        await vd_node.write_value(0.0)
        Vd_inject = 0.1
        await vd_node.write_value(Vd_inject)
        await asyncio.sleep(0.5)
        Id = await id_node.read_value()
        print(f"  重试 Vd={Vd_inject}V → Id={Id:.3f}A")

    # 算 Rs
    Rs = Vd_inject / Id if Id > 0 else float('nan')
    print(f"  Rs_estimated = {Vd_inject}V / {Id:.3f}A = {Rs:.4f} Ω")

    # 停止注入
    await vd_node.write_value(0.0)
    await vq_node.write_value(0.0)
    await op_mode.write_value(2)  # 回 READY

    return Rs


async def identify_ld(client, ns):
    """d轴电感辨识 — 高频注入法"""
    print("\n=== 步骤 2: d轴电感 Ld 辨识 (高频注入法) ===")

    await asyncio.sleep(0.2)

    # 注入 Vd=20*sin(2π*100*t), Vq=0
    # 简化: 直接注入阶跃然后看电流上升斜率
    op_mode = client.get_node(f"ns={ns};s=op_mode")
    await op_mode.write_value(3)
    await asyncio.sleep(0.05)

    vd_node = client.get_node(f"ns={ns};s=Vd_ref")
    vq_node = client.get_node(f"ns={ns};s=Vq_ref")
    id_node = client.get_node(f"ns={ns};s=Id_actual")

    await vq_node.write_value(0.0)

    # 阶跃注入 Vd=5V
    print("  注入 Vd=5V 阶跃 ...")
    await vd_node.write_value(5.0)

    # 读 Id 上升 (等 5ms)
    Id1 = await id_node.read_value()
    t1 = time.monotonic()
    await asyncio.sleep(0.005)

    Id2 = await id_node.read_value()
    t2 = time.monotonic()
    dt = t2 - t1
    dId_dt = (Id2 - Id1) / dt

    # Ld = Vd / (dId/dt)  (忽略电阻，因为 dt 很短)
    Ld = 5.0 / dId_dt if dId_dt > 0 else float('nan')
    print(f"  Id: {Id1:.3f}A → {Id2:.3f}A (Δt={dt*1000:.1f}ms)")
    print(f"  dId/dt = {dId_dt:.1f} A/s")
    print(f"  Ld_estimated = 5V / {dId_dt:.1f} = {Ld*1000:.2f} mH")

    # 停止
    await vd_node.write_value(0.0)
    await op_mode.write_value(2)

    return Ld


async def main():
    print("=== PMSM 参数辨识测试 ===")
    print("连接 OPC UA Server ...")

    client = Client("opc.tcp://localhost:4841")
    try:
        await client.connect()
        print("已连接 ✓")
    except Exception as e:
        print(f"连接失败: {e}")
        print("\n请先启动仿真: python3 demo.py")
        sys.exit(1)

    ns = 4  # flywheel_motor

    # 心跳
    async def heartbeat():
        while True:
            try:
                # 读一个变量作为心跳
                await client.get_node(f"ns={ns};s=Id_actual").read_value()
            except:
                pass
            await asyncio.sleep(0.2)

    hb_task = asyncio.create_task(heartbeat())

    # 先读初始状态
    id_node = client.get_node(f"ns={ns};s=Id_actual")
    state_node = client.get_node(f"ns={ns};s=op_state")
    state = await state_node.read_value()
    print(f"初始状态: op_state={state}")

    # 辨识
    Rs_est = await identify_rs(client, ns)
    Ld_est = await identify_ld(client, ns)

    # 结果汇总
    print("\n" + "=" * 50)
    print("=== 参数辨识结果汇总 ===")

    # 真值 (从模型参数)
    RS_TRUE = 0.015
    LD_TRUE = 0.008

    rs_err = abs(Rs_est - RS_TRUE) / RS_TRUE * 100
    ld_err = abs(Ld_est - LD_TRUE) / LD_TRUE * 100

    print(f"{'参数':<10} {'辨识值':<12} {'真值':<12} {'误差':<10} {'判断'}")
    print("-" * 50)
    print(f"{'Rs':<10} {Rs_est:<12.4f}Ω {RS_TRUE:<12.4f}Ω {rs_err:<9.1f}% {'✅' if rs_err < 5 else '❌'}")
    print(f"{'Ld':<10} {Ld_est*1000:<12.2f}mH {LD_TRUE*1000:<12.2f}mH {ld_err:<9.1f}% {'✅' if ld_err < 5 else '❌'}")

    hb_task.cancel()
    await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
