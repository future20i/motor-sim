# 遥测参数字典

**文档编号:** MOTORSIM-TEL-03
**版本:** v2.5
**最后更新:** 2026-07-08
**适用范围:** 泓慧能源 500kW/3kWh 飞轮UPS全链路仿真平台

---

## 1. 三相遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 1 | 输入线电压 AB | Input Line Voltage AB | Vrms | A-B 相间输入电压测量值 | `grid_sim.py` |
| 2 | 输入线电压 BC | Input Line Voltage BC | Vrms | B-C 相间输入电压测量值 | `grid_sim.py` |
| 3 | 输入线电压 CA | Input Line Voltage CA | Vrms | C-A 相间输入电压测量值 | `grid_sim.py` |
| 4 | A相输入电流 | Current In Phase A | Arms | A相输入电流 | `grid_inverter.py` |
| 5 | B相输入电流 | Current In Phase B | Arms | B相输入电流 | `grid_inverter.py` |
| 6 | C相输入电流 | Current In Phase C | Arms | C相输入电流 | `grid_inverter.py` |
| 7 | 输入功率 | Input Power | kW | 基于三相输入电压和电流的计算值 | `grid_inverter.py` |
| 8 | A相输入千伏安 | Input kVA Phase A | kVA | 基于A相输入电压和电流的计算值 | `grid_inverter.py` |
| 9 | B相输入千伏安 | Input kVA Phase B | kVA | 基于B相输入电压和电流的计算值 | `grid_inverter.py` |
| 10 | C相输入千伏安 | Input kVA Phase C | kVA | 基于C相输入电压和电流的计算值 | `grid_inverter.py` |
| 11 | 输出线电压 AB | Output Line Voltage AB | Vrms | A-B 相间输出电压测量值 | `grid_inverter.py` |
| 12 | 输出线电压 BC | Output Line Voltage BC | Vrms | B-C 相间输出电压测量值 | `grid_inverter.py` |
| 13 | 输出线电压 CA | Output Line Voltage CA | Vrms | C-A 相间输出电压测量值 | `grid_inverter.py` |
| 14 | A相输出电流 | Current Out Phase A | Arms | A相输出电流 | `grid_inverter.py` |
| 15 | B相输出电流 | Current Out Phase B | Arms | B相输出电流 | `grid_inverter.py` |
| 16 | C相输出电流 | Current Out Phase C | Arms | C相输出电流 | `grid_inverter.py` |
| 17 | 输出功率 | Output Power | kW | 基于三相输出电压和电流的计算值 | `grid_inverter.py` |
| 18 | A相输出千伏安 | Output kVA Phase A | kVA | 基于A相输出电压和电流的计算值 | `grid_inverter.py` |
| 19 | B相输出千伏安 | Output kVA Phase B | kVA | 基于B相输出电压和电流的计算值 | `grid_inverter.py` |
| 20 | C相输出千伏安 | Output kVA Phase C | kVA | 基于C相输出电压和电流的计算值 | `grid_inverter.py` |

## 2. 单相遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 21 | 输入频率 | Input Frequency | Hz | C相上输入频率的测量值 | `grid_sim.py` |
| 22 | 输出频率 | Output Frequency | Hz | C相上输出频率的测量值 | `grid_inverter.py` |
| 23 | 总输出功率 | Total Output Power | kW | A/B/C三相输出功率总和 | `grid_inverter.py` |
| 24 | 总输入功率 | Total Input Power | kW | A/B/C三相输入功率总和 | `grid_inverter.py` |
| 25 | 总输出千伏安 | Total Output kVA | kVA | A/B/C三相输出千伏安总和 | `grid_inverter.py` |
| 26 | 总输入千伏安 | Total Input kVA | kVA | A/B/C三相输入千伏安总和 | `grid_inverter.py` |
| 27 | 输出功率因素 | Output Power Factor | — | 基于输出相电压和输出电流的计算值 | `grid_inverter.py` |
| 28 | 输入功率因素 | Input Power Factor | — | 基于输入相电压和输入电流的计算值 | `grid_inverter.py` |

## 3. 直流母线遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 29 | 正直流母线电压 | Positive DC Bus Voltage | VDC | 输入整流器产生的正直流母线电压 | `dc_bus.py` |
| 30 | 负直流母线电压 | Negative DC Bus Voltage | VDC | 输入整流器产生的负直流母线电压 | `dc_bus.py` |
| 31 | 直流母线电流 | DC Bus Current | A | 总母线电流 | `dc_bus.py` |
| 32 | 直流母线功率 | DC Bus Power | kW | 母线传输功率 (Vdc × Idc) | `dc_bus.py` |
| 33 | 直流母线电压不平衡 | DC Bus Voltage Imbalance | V | \|Vdc+\| - \|Vdc-\| | `dc_bus.py` |

## 4. 飞轮遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 34 | 飞轮转速 | Tachometer (Flywheel RPM) | RPM | 飞轮旋转速度 | `flywheel_energy.py` |
| 35 | 可用能量百分比 | Percent Usable Energy | % | 飞轮中储存的有效能百分比 (SOC) | `flywheel_energy.py` |
| 36 | 机械储能 | Stored Energy | kWh | 当前储存能量 (½Jω²) | `flywheel_energy.py` |
| 37 | 真空度 | Vacuum Gauge | mTorr | 飞轮壳体内的真空水平 | `flywheel_energy.py` |
| 38 | 下轴承承载力 | Bottom Bearing Force | N | 下轴承上的作用力数量 | `flywheel_energy.py` |
| 39 | 横向振动 | Lateral Vibration | G | 飞轮横向振动量 (轴承顺畅工作指标) | `flywheel_energy.py` |
| 40 | 轴向振动 | Axial Vibration | G | 飞轮轴向振动量 (轴承顺畅工作指标) | `flywheel_energy.py` |
| 41 | 飞轮温度 | Flywheel Temperature | °C | 飞轮整体温度 | `flywheel_energy.py` |
| 42 | 放电功率 | Discharge Power | kW | 当前放电/充电功率 (+放/-充) | `flywheel_energy.py` |
| 43 | 剩余放电时间 | Remaining Discharge Time | s | 当前SOC下可维持放电的时间估计 | `flywheel_energy.py` |

## 5. 电机遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 44 | d轴电流 | Id Current | A | d轴电流 | `motor_base.py` |
| 45 | q轴电流 | Iq Current | A | q轴电流 | `motor_base.py` |
| 46 | d轴电压 | Vd Voltage | V | d轴参考电压 | `control_algorithms.py` |
| 47 | q轴电压 | Vq Voltage | V | q轴参考电压 | `control_algorithms.py` |
| 48 | 电磁转矩 | Electromagnetic Torque | N·m | 电机输出转矩 | `motor_base.py` |
| 49 | 电机温度 | Motor Temperature | °C | 电机绕组温度估计 | `motor_base.py` |
| 50 | 上励磁线圈电流 | Top Field Coil Current | A | 飞轮上励磁线圈电流 (HIM专有) | `him_controller.py` |
| 51 | 下励磁线圈电流 | Bottom Field Coil Current | A | 飞轮下励磁线圈电流 (HIM专有) | `him_controller.py` |
| 52 | 上励磁线圈温度 | Top Field Coil Temp | °C | 飞轮上励磁线圈温度 | `him_controller.py` |
| 53 | 下励磁线圈温度 | Bottom Field Coil Temp | °C | 飞轮下励磁线圈温度 | `him_controller.py` |
| 54 | 上励磁IGBT温度 | Top Field Coil IGBT Temp | °C | 上部励磁线圈IGBT温度 | `sic_md_inverter.py` |
| 55 | 下励磁IGBT温度 | Bottom Field Coil IGBT Temp | °C | 下部励磁线圈IGBT温度 | `sic_md_inverter.py` |
| 56 | 电枢温度 | Armature Temperature | °C | 电枢绕组温度 | `motor_base.py` |
| 57 | 上轴承温度 | Top Bearing Temp | °C | 上轴承温度 | `flywheel_energy.py` |
| 58 | 下轴承温度 | Bottom Bearing Temp | °C | 下轴承温度 | `flywheel_energy.py` |

## 6. 系统环境遥测点

| # | 参数名 | 英文名称 | 单位 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|:--|
| 59 | 柜子温度 | Cabinet Temperature | °C | 柜内气温 | 环境传感器 |
| 60 | 进气口温度 | Air Inlet Temperature | °C | 柜子进气口气温 | 环境传感器 |
| 61 | 静态开关温度 | Static Switch Temperature | °C | 静态开关散热片温度 | `grid_inverter.py` |
| 62 | 室温 | Room Temperature | °C | 设备间室温 | 环境传感器 |
| 63 | 负载百分比 | Load Percentage | % | 当前负载占额定容量百分比 | `power_ops.py` |

## 7. 部件状态遥测点

| # | 参数名 | 英文名称 | 说明 | 来源模块 |
|:--|:--|:--|:--|:--|
| 64 | 输入接触器状态 | Input Contactor Status | K1 开/关 | `power_ops.py` |
| 65 | 输出接触器状态 | Output Contactor Status | K2 开/关 | `power_ops.py` |
| 66 | 旁路接触器状态 | Bypass Contactor Status | K3 开/关 | `power_ops.py` |
| 67 | 飞轮变流器状况 | Flywheel Converter Status | 正常/故障/离线 | `sic_md_inverter.py` |
| 68 | 市电变流器状况 | Utility Converter Status | 正常/故障/离线 | `grid_inverter.py` |
| 69 | 输入继电器状态 | Input Relay Status | 正常/故障 | `power_ops.py` |
| 70 | 输出继电器状态 | Output Relay Status | 正常/故障 | `power_ops.py` |
| 71 | 风扇状况 | Fan Status | 正常/故障/警告 | 环境 |
| 72 | 保险丝状况 | Fuse Status | 正常/熔断 | `power_ops.py` |
| 73 | GenSTART 状况 | GenSTART Status | 在线/离线/故障 | `power_ops.py` |

## 8. OPC UA 节点映射

所有遥测点通过虚拟 OPC UA Server (`virtual_opcua_server.py`) 暴露，节点路径:

```
ns=2;s=<参数英文名>
```

### 8.1 写权限节点 (AI/S2可写)

| OPC UA 节点 | 类型 | 范围 | 说明 |
|:--|:--|:--|:--|
| `Vd_ref` | Float | [-400, 400] | d轴电压参考 |
| `Vq_ref` | Float | [-400, 400] | q轴电压参考 |
| `speed_ref` | Float | [0, 9900] | 转速参考 rpm |
| `op_mode` | Int32 | [0, 3] | 0:待机, 1:预充, 2:辨识, 3:并网 |
| `emergency_stop` | Boolean | — | 紧急停止 |

### 8.2 读权限节点

| OPC UA 节点 | 类型 | 对应遥测 # |
|:--|:--|:--|
| `Id` | Float | #44 |
| `Iq` | Float | #45 |
| `Vdc` | Float | #29 |
| `Temp` | Float | #41 |
| `Speed` | Float | #34 |
| `fault_code` | Int32 | — |
| `SOC` | Float | #35 |
| `P_out` | Float | #23 |
| `f_grid` | Float | #21 |
| `mode` | String | — |
