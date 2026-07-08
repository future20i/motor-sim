"""
demo.py — 基础演示脚本: PMSM飞轮全链路。

子系统: 应用层
依赖: power_ops.py, grid_sim.py, system_config.py
手册对应章节: INSTALLATION.md §4.1 (运行仿真)

基础演示脚本: PMSM飞轮全链路。
"""
import argparse
import asyncio
from motor_base import create_motor, list_motor_types
from virtual_controller import VirtualController
from virtual_opcua_server import VirtualOpcuaServer


async def main(motor_type="pmsm", fidelity="level1"):
    types = {t["id"]: t for t in list_motor_types()}

    print(f"=== 飞轮电机 AI 调试仿真栈 ===")
    print(f"电机类型: {types[motor_type]['name']}")
    print(f"模型精度: {fidelity}")
    print()

    # S4: 电机 (工厂创建)
    motor = create_motor(motor_type, fidelity=fidelity)
    print(f"S4 电机: {motor.type_name()}")
    for k, v in motor.param_summary().items():
        print(f"  {k} = {v}")

    # S3: 控制器
    controller = VirtualController(motor)
    print(f"S3 控制器: bw=1000rad/s, Ts={motor.cfg.T_sample*1e6:.0f}μs")

    # S2: OPC UA Server
    server = VirtualOpcuaServer(controller)
    print(f"S2 OPC UA Server: 就绪")
    print()
    print(f"支持类型: {', '.join(types.keys())}")
    print(f"切换: Ctrl+C 后换 --motor <type> 重启")

    await server.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--motor", default="pmsm",
                        choices=["pmsm", "induction", "synrm", "pm_synrm"],
                        help="电机类型")
    parser.add_argument("--level2", action="store_true",
                        help="Level 2 物理模型 (RK4+齿槽+温度)")
    args = parser.parse_args()
    asyncio.run(main(args.motor, "level2" if args.level2 else "level1"))
