# motor-sim — 飞轮储能电机仿真平台

泓慧能源 **500kW / 3kWh 飞轮UPS** 全链路仿真系统。

## 关键参数

| 参数 | 值 |
|---|---|
| 额定功率 | 500 kW |
| 储能 | 3 kWh (800–2000 rpm) |
| DC 母线 | 750 V |
| AC 输出 | 480 V / 50 Hz |
| 电机类型 | 单极感应子 (电励磁, 6对极) |
| 励磁 | If=10A, Rf=75Ω (DC母线取电) |
| 惯量 | J=492 kg·m² |
| 逆变器 | 750 kVA, I_max=2000A |

## 模块

| 文件 | 功能 |
|---|---|
| `flywheel_energy.py` | 飞轮储能本体 (惯量/SOC/功率) |
| `pmsm_model.py` | 电机 dq 模型 |
| `motor_base.py` | 通用电机控制框架 |
| `control_algorithms.py` | FOC/VSG/PQ 控制算法 |
| `grid_inverter.py` | 并网逆变器 (PLL/SVPWM/预充) |
| `grid_sim.py` | 电网模拟器 (电压/频率/故障) |
| `dc_bus.py` | DC 母线模型 |
| `power_ops.py` | 功率调度编排器 |
| `inverter_topology.py` | 逆变器拓扑仿真 |
| `s4_ai_controller.py` | AI 策略控制器 |
| `s4_experiment_runner.py` | S4 实验框架 |
| `s4_scenarios.py` | 10 个电网扰动场景 |

## S4 实验 (6 策略 × 10 场景)

```
🥇 基线-固定VSG     43.5
🥈 AI 预测         41.4
🥉 AI 自适应       40.8
```

AI 预测在电压暂降(LVRT)场景中比基线高 11%。

## 运行

```bash
# 逆变器自测
python3 grid_inverter.py

# 单场景
python3 s4_experiment_runner.py --scenario 5

# 全对比
python3 s4_experiment_runner.py --compare
```
