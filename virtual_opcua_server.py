"""
virtual_opcua_server.py — asyncua Server: 暴露 5写+15读 节点。真系统和仿真系统使用完全相同接口。

子系统: 通信层 (S2)
依赖: 无
手册对应章节: TELEMETRY.md §8 (OPC UA节点映射)

asyncua Server: 暴露 5写+15读 节点。真系统和仿真系统使用完全相同接口。
"""
import asyncio
import time
from asyncua import Server, ua
from virtual_controller import VirtualController


class VirtualOpcuaServer:
    """OPC UA Server — 仿真->真机迁移时接口零改动"""

    def __init__(self, controller: VirtualController, host="0.0.0.0", port=4841):
        self.controller = controller
        self.host = host
        self.port = port
        self.server = Server()
        self.nodes = {}
        self._running = False

        # 白名单规则 (和真机一模一样)
        self.write_rules = {
            "Vd_ref":    {"min": -100, "max": 100, "max_step": 5.0, "default": 0.0},
            "Vq_ref":    {"min": -100, "max": 100, "max_step": 5.0, "default": 0.0},
            "speed_ref": {"min": 0, "max": 11000, "max_step": 50.0, "default": 0.0},
            "Id_ref":    {"min": -100, "max": 100, "max_step": 2.0, "default": 0.0},
            "Iq_ref":    {"min": -100, "max": 100, "max_step": 2.0, "default": 0.0},
            "op_mode":   {"enum": [0, 1, 2, 3], "default": 0},
            "emergency_stop": {"write_only_true": True, "default": False},
        }

        self.last_values = {}
        self.last_write_time = {}
        self.last_heartbeat = time.monotonic()

    async def setup(self):
        await self.server.init()
        self.server.set_endpoint(f"opc.tcp://{self.host}:{self.port}")
        ns = await self.server.register_namespace("flywheel_motor")
        objects = self.server.nodes.objects

        # 可写变量
        for var_name, rule in self.write_rules.items():
            node = await objects.add_variable(ns, var_name, rule["default"])
            await node.set_writable(True)
            self.nodes[var_name] = node
            self.last_values[var_name] = rule["default"]

        # 只读变量
        read_vars = [
            "Id_actual", "Iq_actual", "Vdc_actual", "speed_actual",
            "temp_module", "temp_motor", "fault_code", "op_state"
        ]
        for var_name in read_vars:
            node = await objects.add_variable(ns, var_name, 0.0)
            await node.set_writable(False)
            self.nodes[var_name] = node

        # 写处理: 在 _sim_loop 中主动轮询可写节点 (asyncua 无 add_writerequest_callback API)
        self._running = True
        asyncio.create_task(self._sim_loop())
        asyncio.create_task(self._heartbeat_check())

    def _on_write(self, var_name, value):
        rule = self.write_rules.get(var_name, {})
        val = value.Value.Value if hasattr(value, "Value") else value

        # 急停只能写 True
        if rule.get("write_only_true") and val is not True:
            print(f"  ⚠ 急停拒绝: 写入了 {val}")
            return

        # 枚举校验
        if "enum" in rule and val not in rule["enum"]:
            print(f"  ⚠ {var_name}={val} 不在枚举 {rule['enum']} 内, 拒绝")
            return

        # 范围钳位
        if "min" in rule:
            val = max(rule["min"], min(rule["max"], val))

        # 速率限制
        if "max_step" in rule:
            last = self.last_values.get(var_name, 0)
            if abs(val - last) > rule["max_step"]:
                val = last + rule["max_step"] * (1 if val > last else -1)

        # 频率限制 (50ms)
        now = time.monotonic()
        last_t = self.last_write_time.get(var_name, 0)
        if now - last_t < 0.05:
            return

        # 心跳
        if now - self.last_heartbeat > 0.5:
            print(f"  ⚠ 心跳丢失, 拒绝写入")
            self.controller.op_mode = 0
            return

        # 通过 → 写控制器
        setattr(self.controller, var_name, val)
        self.last_values[var_name] = val
        self.last_write_time[var_name] = now

    async def _sim_loop(self):
        dt = self.controller.ts
        step = 0
        while self._running:
            # ── 轮询可写节点 (替代不存在的 add_writerequest_callback) ──
            for var_name in self.write_rules:
                try:
                    node = self.nodes[var_name]
                    val = await node.read_value()
                    last = self.last_values.get(var_name)
                    if last is not None and val != last:
                        self._on_write(var_name, val)
                        self.last_values[var_name] = val
                except Exception:
                    pass  # 节点未就绪时跳过

            # ── 更新只读变量 ──
            readable = self.controller.run_cycle()
            for var, val in readable.items():
                if var in self.nodes:
                    await self.nodes[var].write_value(
                        ua.DataValue(ua.Variant(val, ua.VariantType.Double))
                    )
            # 也更新 op_state
            if "op_state" in self.nodes:
                await self.nodes["op_state"].write_value(
                    ua.DataValue(ua.Variant(self.controller.state.value, ua.VariantType.Int32))
                )

            step += 1
            # 每 1000 步打印一次状态
            if step % 1000 == 0:
                s = self.controller.motor.state
                st = self.controller.state.name
                print(f"  [{step:6d}] {st:10s} | ω={s.omega_m*60/(2*np.pi):7.1f}rpm | "
                      f"Id={s.Id:6.2f}A Iq={s.Iq:6.2f}A | Vdc={s.Vdc:5.0f}V | T={s.temp:4.0f}°C")

            await asyncio.sleep(dt)

    async def _heartbeat_check(self):
        while self._running:
            await asyncio.sleep(0.05)
            if time.monotonic() - self.last_heartbeat > 0.5:
                self.controller.op_mode = 0

    def heartbeat(self):
        """AI 调用此方法发心跳"""
        self.last_heartbeat = time.monotonic()

    async def run(self):
        await self.setup()
        print(f"Virtual OPC UA Server running on opc.tcp://{self.host}:{self.port}")
        print(f"Namespace: ns=4 (flywheel_motor)")
        print(f"可写: {list(self.write_rules.keys())}")
        print(f"只读: Id_actual, Iq_actual, Vdc_actual, speed_actual, temp_module, temp_motor, fault_code, op_state")
        print(f"")
        print(f"等待 AI 连接... (Ctrl+C 停止)")
        import numpy as np  # for status display
        async with self.server:
            while True:
                await asyncio.sleep(1)
