# 系统架构总览

**文档编号:** MOTORSIM-ARCH-01
**版本:** v2.5
**最后更新:** 2026-07-08
**适用范围:** 泓慧能源 500kW/3kWh 飞轮UPS全链路仿真平台

---

## 1. 系统说明

CleanSource 飞轮储能仿真平台是完全集成的全链路仿真系统，使用飞轮将机械能储存为旋转质量。
当市电中断时，系统会将储存在飞轮中的机械能转变为电能，提供给外部负载，直到以下状况之一发生：

- 备用发电机承担该负载
- 市电可以重新使用
- 飞轮耗尽储存的能量

一旦市电恢复，系统会不中断地将负载转移回市电。

### 1.1 系统部件概述

| 部件 | 功能 | 对应模块 | 技术细节 |
|:--|:--|:--|:--|
| **飞轮储能本体** | 机械能↔电能转换，SOC监测 | `flywheel_energy.py` | J=33.9 kg·m², E=½Jω², 4400-8800rpm |
| **飞轮电机** | 电动/发电双向运行 | `motor_base.py` | 5种电机: PMSM/IM/SynRM/PM-SynRM/HIM |
| **直流母线** | 能量缓冲，电容动力学 | `dc_bus.py` | Vdc_nom=750V, C=0.2F |
| **网侧逆变器** | 双向 AC/DC，PLL锁相 | `grid_inverter.py` | 4种模式: VSG/PQ/VDC/PRECHARGE |
| **电网模拟器** | 三相电网+故障注入 | `grid_sim.py` | 频率/电压/相位扰动 |
| **功率编排器** | 系统级功率调度 | `power_ops.py` | 4种策略: VSG/Droop/DC恒压/Mixed |
| **HIM控制器** | 感应子电机FOC+弱磁 | `him_controller.py` | 零极点对消+电压极限圆 |
| **逆变器拓扑** | SiC功率模块仿真 | `sic_md_inverter.py` | CAB450M12XM3×2并, 25kHz SVPWM |
| **AI调试助手** | 参数辨识+PI自整定 | `ai_commissioner.py` | Rs→Ld/Lq→磁链→J 四步辨识 |
| **RL策略** | 强化学习电网调度 | `rl_policy.py` | REINFORCE+Baseline, 8D→6actions |
| **虚拟控制器** | CODESYS仿真替代 | `virtual_controller.py` | 状态机+电流环 |
| **虚拟OPC UA** | S2通信层模拟 | `virtual_opcua_server.py` | asyncua Server |

### 1.2 系统配置表

| 系统 | 所含模块 |
|:--|:--|
| **基础仿真** | `system_config` + `grid_sim` + `flywheel_energy` + `dc_bus` |
| **全链路(简化)** | 基础 + `power_ops` + `grid_inverter` |
| **全链路(完整)** | 全链路 + `motor_base` + `control_algorithms` + `virtual_controller` |
| **HIM专有** | 全链路(简化) + `him_controller` + `motor_base`(HIM) |
| **SiC逆变器** | 全链路(完整) + `sic_md_inverter` + `inverter_topology` |
| **AI调试** | 全链路(完整) + `ai_commissioner` + `virtual_opcua_server` |
| **RL训练** | 基础 + `rl_environment` + `train_rl` + `rl_policy` |
| **实验对比** | 全链路(简化) + `s4_experiment_runner` + `s4_scenarios` + `s4_metrics` |

---

## 2. 系统框图 (单线图 SLD)

```
                      ┌─────────────────────────────────────────────┐
                      │                电网模拟器                     │
                      │          grid_sim.py (Va,Vb,Vc)              │
                      └──────┬──────────────────────────────────────┘
                             │ 三相 480VAC 50Hz
                             ▼
           ┌─────────────────────────────────────────────┐
           │              静态开关 (固态)                  │
           │           SCR "先接后断" <1ms                │
           └──────┬──────────────────┬───────────────────┘
                  │                  │
                  ▼                  ▼
       ┌──────────────┐    ┌──────────────┐
       │  线式电感     │    │  旁路断路器    │
       │  (滤波+隔离)   │    │   K3/K4       │
       └──────┬───────┘    └──────┬────────┘
              │                   │
              ▼                   │
    ┌──────────────────┐          │
    │  网侧逆变器        │          │
    │  grid_inverter.py │          │
    │  AC↔DC 双向       │          │
    └────────┬─────────┘          │
             │                    │
             ▼                    │
    ┌──────────────────┐          │
    │   直流母线         │          │
    │   dc_bus.py       │          │
    │   Vdc=750V, C=0.2F│          │
    └───┬──────────────┘          │
        │                         │
        ▼                         │
    ┌──────────────────┐          │
    │  飞轮逆变器        │          │
    │  (sic_md_inverter │          │
    │  或 motor_base)    │          │
    └────────┬─────────┘          │
             │                    │
             ▼                    │
    ┌──────────────────┐          │
    │   飞轮电机+飞轮    │          │
    │   motor_base.py   │          │
    │   +flywheel_energy │          │
    └───────────────────┘          │
                                   │
                                   ▼
                          ┌──────────────┐
                          │   输出接触器   │
                          │   K1/K2       │
                          └──────┬───────┘
                                 │
                                 ▼
                          ┌──────────────┐
                          │   负载        │
                          └──────────────┘
```

---

## 3. 控制架构 (双变流器)

### 3.1 正常模式 (Online) 功率流

```
市电 ──→ 静态开关(ON) ──→ 线式电感 ──→ 市电变流器(整流) ──→ DC母线=750V ──→ 市电变流器(逆变) ──→ 负载
                                                                    │
                                             飞轮逆变器(电机驱动) ←─┘
                                                    │
                                             飞轮电机(维持待机速度)
                                             ~1kW/飞轮 维持损耗
```

### 3.2 放电模式 (Discharge) 功率流

```
市电 ✕── 静态开关(OFF) ── 输入接触器(OPEN)
                              ×
飞轮(降速) ──→ 飞轮逆变器(发电) ──→ DC母线=750V ──→ 市电变流器(逆变) ──→ 负载
```

### 3.3 旁路模式 (Bypass) 功率流

```
市电 ──→ 旁路断路器(K3) ──→ 负载  (飞轮/逆变器全部隔离)
```

---

## 4. 系统节点

这些节点决定系统的状态。输入节点和输出节点是外部可用节点。滤波器节点由系统内部使用。

| 节点 | 位置 | 作用 | 监控量 |
|:--|:--|:--|:--|
| **输入节点** | 市电入口 | 将系统连接到电源 | Vac_in, Iac_in, f_in, PF_in |
| **输出节点** | 负载出口 | 将系统连接到负载 | Vac_out, Iac_out, f_out, PF_out |
| **直流母线节点** | 变流器之间 | 能量缓冲 | Vdc(+), Vdc(-), Idc |
| **滤波器节点** | 线式电感后 | 内部滤波 | Vfilt, Ifilt |
| **飞轮节点** | 飞轮逆变器后 | 飞轮状态 | ω, SOC, Te, 真空度 |

---

## 5. 代码架构分层

```
Layer 4: 应用层     demo.py, demo_him.py, debug_ui.py, s4_experiment_runner.py
Layer 3: 实验引擎   s4_scenarios.py, s4_metrics.py, train_rl.py, rl_environment.py
Layer 2: 物理模型   flywheel_energy.py, motor_base.py, dc_bus.py, grid_sim.py
Layer 1: 控制层     control_algorithms.py, s4_ai_controller.py, him_controller.py,
                    virtual_controller.py, power_ops.py, grid_inverter.py
Layer 0: 基础层     system_config.py (单一事实来源)
```

---

## 6. 模块依赖图

```
system_config.py  ←── 全局配置 (PhysicalConstants, FlywheelMotorSpec, FlywheelMechanicalSpec, DCBusSpec)
      │
      ├── motor_base.py       ← 电机模型 (依赖 PhysicalConstants)
      ├── flywheel_energy.py  ← 飞轮物理 (依赖 FlywheelMechanicalSpec)
      ├── dc_bus.py           ← 直流母线 (依赖 DCBusSpec, PhysicalConstants)
      ├── grid_sim.py         ← 电网 (依赖 PhysicalConstants)
      │
      ├── control_algorithms.py  ← 电流环 (依赖 motor_base)
      ├── him_controller.py     ← HIM控制 (依赖 motor_base + control_algorithms)
      ├── grid_inverter.py      ← 网侧逆变器 (依赖 grid_sim + control_algorithms)
      │
      ├── power_ops.py          ← 功率调度 (依赖 flywheel_energy + grid_sim + dc_bus + motor_base)
      │
      ├── s4_ai_controller.py   ← AI策略 (依赖 power_ops)
      ├── ai_commissioner.py    ← 参数辨识 (依赖 motor_base + control_algorithms)
      │
      ├── s4_scenarios.py       ← 电网事件 (依赖 grid_sim)
      ├── s4_metrics.py         ← 评分 (依赖 power_ops)
      ├── s4_experiment_runner.py ← 实验 (依赖 s4_scenarios + s4_metrics + power_ops)
      │
      ├── rl_environment.py     ← RL环境 (依赖 power_ops + grid_sim)
      ├── rl_policy.py          ← RL策略 (依赖 rl_environment)
      └── train_rl.py           ← RL训练 (依赖 rl_environment)
```

---

## 7. 参考标准

| 标准 | 说明 |
|:--|:--|
| **IEC 62040** | UPS 性能与测试要求 |
| **IEC 62933** | 电能储存系统 (ESS) |
| **IEEE 1547** | 分布式能源并网 |
| **GB/T 34120** | 飞轮储能系统技术规范 |
| **UL 1778** | UPS 安全标准 |
| **IEC 61850** | 变电站自动化通信 (OPC UA 参考) |
