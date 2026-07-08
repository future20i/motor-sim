# motor-sim — 飞轮储能电机仿真平台 v2.0

泓慧能源 **500kW / 3kWh 飞轮UPS** 全链路仿真与AI控制算法选型验收平台。

## 架构分层

```
Layer 4: 应用层    (CLI / Streamlit / Jupyter)
Layer 3: 实验引擎  (S4ExperimentRunner / SimulationEngine)
Layer 2: 物理模型  (Grid → DC Bus → Flywheel → Motor)
Layer 1: 控制层    (PI / Deadbeat / SlidingMode / AI / RL)
Layer 0: 基础层    (SystemConfig / ControllerBase / MotorBase)
```

## 模块清单

| 模块 | 功能 | 状态 |
|------|------|------|
| `system_config.py` | 统一配置层 | ✅ v2.0 |
| `motor_base.py` | 5种电机模型 (PMSM/IM/SynRM/PM-SynRM/HIM) | ✅ |
| `control_algorithms.py` | PI / Deadbeat / SlidingMode | ✅ |
| `him_controller.py` | HIM FOC控制器 + 自动弱磁管理器 | ✅ |
| `flywheel_energy.py` | 飞轮储能物理模型 | ✅ v2.0 |
| `power_ops.py` | 功率调度编排 (VSG/Droop/DC/Mixed) | ✅ |
| `dc_bus.py` | 直流母线动态 | ✅ |
| `grid_sim.py` | 电网仿真 + 故障注入 | ✅ |
| `grid_inverter.py` | 网侧逆变器 (PLL/PQ/VSG/VDC) | ✅ |
| `s4_scenarios.py` | 10个标准电网事件场景 | ✅ |
| `s4_metrics.py` | 评分 + 帕累托 + 蒙特卡洛 | ✅ v2.0 |
| `s4_experiment_runner.py` | 实验运行器 | ✅ |
| `s4_ai_controller.py` | AI 启发式策略 | ✅ |
| `ai_commissioner.py` | 电机参数辨识 | ✅ |
| `rl_environment.py` | RL Gym 环境 (8D obs, 6 actions, REINFORCE) | ✅ v2.5 |
| `train_rl.py` | RL 训练脚本 (纯 numpy, REINFORCE+Baseline) | ✅ v2.5 |
| `rl_policy.py` | RL 策略导出 + DecisionLogger (替换启发式) | ✅ v2.5 |
| `adversarial_scenarios.py` | 对抗场景生成 | 🚧 v3.0 |
| `grid_recorder.py` | 真实录波回放 | 🚧 v3.0 |
| `flywheel_farm.py` | 多飞轮协调 | 🚧 v3.5 |

## v2.5 RL 训练

```bash
# 训练 (纯 numpy, 无 torch 依赖)
python3 train_rl.py --episodes 500

# 评估
python3 train_rl.py --eval

# 使用 RL 策略
from rl_policy import RLPolicy
policy = RLPolicy("models/rl_policy_best.npz")
action = policy.decide(snapshot)  # GridSnapshot → AIControlAction
```

**50 episode 训练结果:** RL +681 vs 随机基线, 方差降低 87%

## 运行

```bash
# 全策略对比
python3 s4_experiment_runner.py --compare

# RL 训练
python3 -m motor_sim.train --algo ppo --timesteps 1000000

# 对抗场景搜索
python3 -m motor_sim.adversarial --strategy AI_ADAPTIVE
```

## 测试

```bash
pytest tests/ -v
```
