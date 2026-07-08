# 部署与安装清单

**文档编号:** MOTORSIM-INST-06
**版本:** v2.5
**最后更新:** 2026-07-08

---

## 1. 环境要求

### 1.1 硬件要求

| 项目 | 最低要求 | 推荐配置 |
|:--|:--|:--|
| **CPU** | 2核 | 4核+ |
| **内存** | 2GB | 8GB |
| **磁盘** | 500MB | 2GB+ |
| **操作系统** | Linux (Ubuntu 20.04+) / macOS | Linux |
| **Python** | 3.10+ | 3.11 |

### 1.2 软件依赖

| 包 | 版本 | 用途 |
|:--|:--|:--|
| `numpy` | ≥1.24 | 数值计算 |
| `matplotlib` | ≥3.7 | 波形可视化 |
| `pandas` | ≥2.0 | CSV 数据记录 |
| `pytest` | ≥7.0 | 单元测试 |
| `streamlit` | ≥1.28 | 调试 UI (可选) |
| `gymnasium` | ≥0.29 | RL 环境 (可选) |
| `asyncua` | ≥1.1 | OPC UA 服务器 (可选) |
| `tqdm` | ≥4.65 | 进度条 (可选) |

---

## 2. 安装程序

### 2.1 克隆仓库

```bash
git clone https://github.com/future20i/motor-sim.git
cd motor-sim
```

### 2.2 创建虚拟环境

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 2.3 安装依赖

```bash
# 基础依赖
pip install numpy pandas matplotlib pytest

# 可选依赖
pip install streamlit gymnasium asyncua tqdm
```

### 2.4 验证安装

```bash
# 运行冒烟测试
python3 -c "
from system_config import SystemConfig
cfg = SystemConfig()
print(f'Vdc_nom={cfg.physical.Vdc_nom}V, f_nom={cfg.physical.f_nom}Hz')
print(f'J={cfg.flywheel_mech.J} kg·m², P_rated={cfg.flywheel_mech.P_rated/1e3}kW')
print('[OK] system_config loaded')
"

# 运行完整测试套件
pytest tests/ -v

# 运行演示
python3 demo.py
```

---

## 3. 部署验证清单

**重要**: 首次运行前必须完成所有适用步骤。不适用的步骤标注为 "N/A"。

| # | 任务 | 验证方法 | ✓ |
|:--|:--|:--|:--|
| 1 | 仓库已克隆到正确路径 | `ls /root/workspace/motor-sim/` | ☐ |
| 2 | Python 版本 ≥3.10 | `python3 --version` | ☐ |
| 3 | 虚拟环境已创建 | `ls .venv/bin/python` | ☐ |
| 4 | 基础依赖已安装 | `pip list | grep numpy` | ☐ |
| 5 | system_config 可加载 | `python3 -c "from system_config import SystemConfig"` | ☐ |
| 6 | motor_base 可加载 (5种电机) | `python3 -c "from motor_base import create_motor, list_motor_types; print(list_motor_types())"` | ☐ |
| 7 | control_algorithms 可加载 | `python3 -c "from control_algorithms import create_controller, list_controllers"` | ☐ |
| 8 | flywheel_energy 可加载 | `python3 -c "from flywheel_energy import FlywheelEnergyStorage"` | ☐ |
| 9 | grid_sim 可加载 | `python3 -c "from grid_sim import GridSimulator"` | ☐ |
| 10 | dc_bus 可加载 | `python3 -c "from dc_bus import DCBus"` | ☐ |
| 11 | grid_inverter 可加载 | `python3 -c "from grid_inverter import GridInverter"` | ☐ |
| 12 | power_ops 可加载 | `python3 -c "from power_ops import PowerOrchestrator"` | ☐ |
| 13 | him_controller 可加载 | `python3 -c "from him_controller import HIMController"` | ☐ |
| 14 | demo.py 运行成功 | `python3 demo.py` | ☐ |
| 15 | demo_him.py 运行成功 | `python3 demo_him.py` | ☐ |
| 16 | 所有测试通过 | `pytest tests/ -v` | ☐ |
| 17 | J=33.9 kg·m² (泓慧参数) | `python3 -c "from system_config import SystemConfig; c=SystemConfig(); assert abs(c.flywheel_mech.J-33.9)<0.1"` | ☐ |
| 18 | ½Jω²≈3.0 kWh @8800rpm | `python3 -c "from flywheel_energy import FlywheelEnergyStorage; f=FlywheelEnergyStorage(); f.omega=8800*2*3.14159/60; print(f'E={f.stored_energy_kwh:.1f}kWh')"` | ☐ |
| 19 | 调频响应方向正确 (49.7Hz→放电) | `python3 -c "from power_ops import FrequencyRegulator..."` | ☐ |
| 20 | Streamlit UI 可启动 (可选) | `streamlit run debug_ui.py` | ☐ |

---

## 4. 常用命令

### 4.1 运行仿真

```bash
# 基础演示
python3 demo.py

# HIM 感应子电机演示
python3 demo_him.py

# 全策略对比实验
python3 s4_experiment_runner.py --compare

# 特定策略测试
python3 s4_experiment_runner.py --strategy VSG --scenario GRID_FAULT
```

### 4.2 RL 训练

```bash
# 训练 500 episode
python3 train_rl.py --episodes 500

# 评估
python3 train_rl.py --eval

# 使用最佳模型
python3 train_rl.py --eval --model models/rl_policy_best.npz
```

### 4.3 测试

```bash
# 全部测试
pytest tests/ -v

# 特定测试文件
pytest tests/test_rl_env.py -v

# 带覆盖率
pytest tests/ --cov=. --cov-report=term-missing
```

### 4.4 Streamlit UI

```bash
streamlit run debug_ui.py --server.port 8501
```

### 4.5 虚拟 OPC UA Server

```bash
python3 virtual_opcua_server.py
# 默认监听 opc.tcp://0.0.0.0:4840
```

---

## 5. 目录结构

```
motor-sim/
├── docs/                    ← 📖 文档体系 (本次新建)
│   ├── ARCHITECTURE.md       # 系统架构总览
│   ├── STATE_MACHINE.md      # 状态机
│   ├── TELEMETRY.md          # 遥测参数字典
│   ├── ERROR_CODES.md        # 错误码枚举
│   ├── CONTROL_SETPOINTS.md  # 控制设定值
│   ├── INSTALLATION.md       # 本文档
│   ├── CONVENTIONS.md        # 全局约定
│   └── plans/                # 实现计划
├── models/                  ← RL 模型存储
├── config/                  ← 配置文件目录
├── tests/                   ← 测试
├── scenarios/               ← 场景数据
├── system_config.py         ← 统一配置 (单一事实来源)
├── motor_base.py            ← 电机模型
├── flywheel_energy.py       ← 飞轮物理
├── dc_bus.py                ← 直流母线
├── grid_sim.py              ← 电网模拟
├── grid_inverter.py         ← 网侧逆变器
├── power_ops.py             ← 功率调度
├── control_algorithms.py    ← 控制算法
├── him_controller.py        ← HIM控制器
├── inverter_topology.py     ← 逆变器拓扑
├── sic_md_inverter.py       ← SiC逆变器模型
├── ai_commissioner.py       ← AI调试助手
├── s4_ai_controller.py      ← AI策略
├── s4_scenarios.py          ← 场景定义
├── s4_metrics.py            ← 评分
├── s4_experiment_runner.py  ← 实验运行器
├── rl_environment.py        ← RL环境
├── rl_policy.py             ← RL策略
├── train_rl.py              ← RL训练
├── virtual_controller.py    ← 虚拟控制器
├── virtual_opcua_server.py  ← 虚拟OPC UA
├── debug_ui.py              ← Streamlit UI
├── demo.py                  ← 演示脚本
├── demo_him.py              ← HIM演示
└── pyproject.toml           ← 项目配置
```

---

## 6. 故障排除

| 现象 | 含义 | 排除措施 |
|:--|:--|:--|
| ImportError: No module named 'flywheel_energy' | 未激活虚拟环境 | `source .venv/bin/activate` |
| AssertionError: J mismatch | system_config未更新为泓慧参数 | 检查 `FlywheelMechanicalSpec.J=33.9` |
| ModuleNotFoundError: numpy | 未安装依赖 | `pip install -r requirements.txt` |
| streamlit command not found | Streamlit未安装 | `pip install streamlit` |
| OSError: Address already in use | OPC UA端口被占用 | `fuser -k 4840/tcp` |
