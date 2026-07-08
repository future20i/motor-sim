# 全局约定与符号

**文档编号:** MOTORSIM-CONV-07
**版本:** v2.5
**最后更新:** 2026-07-08

---

## 1. 安全符号

| 符号 | 含义 | 使用场景 |
|:--|:--|:--|
| ⚠️ **警告** | 可能导致设备损坏或人身伤害的危险 | 代码中的危险操作注释 |
| ⚡ **注意** | 可能导致数据丢失或性能下降 | 性能敏感代码段 |
| ℹ️ **重要** | 关键信息需要特别关注 | 配置项说明 |
| ✅ | 已通过 / 正确 / 正常 | 验证清单、状态指示 |
| ❌ | 失败 / 错误 / 不可用 | 错误报告 |
| ⚠️ | 警告 / 注意 | 状态指示 |
| 🔴 | 警报状态 | 严重错误 |
| 🟡 | 注意状态 | 非关键警告 |
| 🟢 | 正常工作 | 在线/正常 |
| 🔧 | 维修/维护 | 维护相关 |
| ⚡ | 放电事件 | 飞轮供电中 |

## 2. 字首组合词与术语表

| 缩写 | 全称 | 中文 |
|:--|:--|:--|
| **AWG** | American Wire Gauge | 美国线规 |
| **BTU** | British Thermal Unit | 英热单位 |
| **EMO** | Emergency Module Off | 紧急模块关闭 |
| **EPO** | Emergency Power Off | 紧急断电 |
| **ESS** | External Source Sync | 外部电源同步 |
| **FOC** | Field-Oriented Control | 磁场定向控制 |
| **HIM** | Homopolar Inductor Motor | 单极感应子电机 |
| **IGBT** | Insulated Gate Bipolar Transistor | 绝缘栅双极型晶体管 |
| **IM** | Induction Motor | 异步电机 |
| **LED** | Light Emitting Diode | 发光二极管 |
| **MMS** | Multi-Module System | 多模块系统 |
| **MMU** | Multi-Module Unit | 多模块单元(单个飞轮柜) |
| **NEC** | National Electrical Code | 国家电气规程 |
| **NEMA** | National Electrical Manufacturers Association | 国家电子设备制造协会 |
| **OPC UA** | Open Platform Communications Unified Architecture | 开放平台通信统一架构 |
| **PCC** | Power Conversion Controller | 功率转换控制器 |
| **PF** | Power Factor | 功率因素 |
| **PLL** | Phase-Locked Loop | 锁相环 |
| **PMSM** | Permanent Magnet Synchronous Motor | 永磁同步电机 |
| **REPO** | Remote Emergency Power Off | 远程紧急断电 |
| **RMS** | Root Mean Square | 均方根 |
| **RPS** | Rotor Position Sensor | 转子位置传感器 |
| **SCR** | Silicon Controlled Rectifier | 可控硅整流器 |
| **SELV** | Safety Extra Low Voltage | 安全超低压 |
| **SIO** | System Input/Output | 系统输入/输出 |
| **SLD** | Single Line Diagram | 单线图 |
| **SMS** | Single Module System | 单模块系统 |
| **SNMP** | Simple Network Management Protocol | 简单网络管理协议 |
| **SOC** | State of Charge | 荷电状态 |
| **SRF-PLL** | Synchronous Reference Frame PLL | 同步旋转坐标系锁相环 |
| **SVPWM** | Space Vector Pulse Width Modulation | 空间矢量脉宽调制 |
| **S1-S4** | System 1-4 | AI调试四子系统架构 |
| **SynRM** | Synchronous Reluctance Motor | 同步磁阻电机 |
| **THD** | Total Harmonic Distortion | 总谐波失真 |
| **UPS** | Uninterruptible Power Supply | 不间断电源 |
| **VAC** | Volts Alternating Current | 交流电压 |
| **VDC** | Volts Direct Current | 直流电压 |
| **VSG** | Virtual Synchronous Generator | 虚拟同步发电机 |

## 3. 命名约定

### 3.1 文件命名

| 类型 | 格式 | 示例 |
|:--|:--|:--|
| Python 模块 | `snake_case.py` | `flywheel_energy.py` |
| 测试文件 | `test_<module>.py` | `test_rl_env.py` |
| 文档 | `UPPER_SNAKE_CASE.md` | `STATE_MACHINE.md` |
| 模型文件 | `name_version.ext` | `rl_policy_best.npz` |

### 3.2 变量命名

| 类型 | 前缀/格式 | 示例 |
|:--|:--|:--|
| 物理量 | 标准符号 | `omega`, `Vdc`, `Id`, `Iq` |
| 额定值 | `_nom` 或 `_rated` | `Vdc_nom`, `P_rated` |
| 最大值 | `_max` | `I_max`, `T_max` |
| 最小值 | `_min` | `V_min`, `omega_min` |
| 参考值 | `_ref` | `Id_ref`, `speed_ref` |
| 实际值 | 无后缀 | `Id`, `Vdc` |
| 配置类 | `PascalCase` | `SystemConfig`, `PhysicalConstants` |
| 枚举 | `PascalCase` | `SystemMode`, `ErrorLevel` |

### 3.3 单位约定

| 物理量 | 内部单位 | 显示单位 |
|:--|:--|:--|
| 电压 | V | V / kV |
| 电流 | A | A / kA |
| 功率 | W | W / kW / MW |
| 频率 | Hz | Hz |
| 角速度 | rad/s | RPM (显示) |
| 转矩 | N·m | N·m |
| 转动惯量 | kg·m² | kg·m² |
| 时间 | s | s / ms / μs |
| 温度 | °C | °C |
| 能量 | J | kWh (显示) |
| 角度 | rad | rad / ° (显示) |

## 4. 功率符号约定 (⚠️ 关键)

系统中有**两套符号约定**，必须严格区分：

| 模块 | 正号含义 | 调用方式 |
|:--|:--|:--|
| `flywheel_energy.step(P_grid)` | **+ = 充电** (电网→飞轮) | `fw.step(power)` |
| `FrequencyRegulator.compute()` | **+ = 放电** (飞轮→电网) | `reg.compute(...)` |
| `DCBusController.compute()` | **+ = 放电** (飞轮→母线) | `dc.compute(...)` |
| `DCBus.step(P_flywheel)` | **+ = 充电** (母线→飞轮) | `bus.step(p_fly)` |

`PowerOrchestrator.step()` 负责转换:
- `FREQ_REGULATION/VSG`: P_ref需取反 → `-s.P_ref`
- `POWER_TRACKING/DC_BUS`: 直接传递 → `s.P_ref`

**违规后果**:
- 符号反了 → 飞轮反向出力 → 母线崩溃
- 两套约定混用 → 调频响应方向错误

## 5. 代码文件头模板

```python
"""
<模块名> — <一句话描述>

子系统: <S1/S2/S3/S4/物理模型/控制层/应用层>
依赖: <主要依赖模块>
手册对应章节: <对应 docs/ 中的文档章节>

<详细说明>
"""
```

### 5.1 示例

```python
"""
flywheel_energy.py — 飞轮储能物理模型 (J=33.9 kg·m², 3.0kWh)

子系统: 物理模型
依赖: system_config.py (FlywheelMechanicalSpec)
手册对应章节:
  - docs/ARCHITECTURE.md  §1 (系统部件概述)
  - docs/TELEMETRY.md     §4 (飞轮遥测点)
  - docs/STATE_MACHINE.md §3 (ONLINE子状态)

飞轮本体模型: 包含转动惯量、风阻/轴承损耗、热模型、SOC计算。
功率符号: + = 充电 (电网→飞轮)
"""
```

## 6. 注释分级

| 级别 | 符号 | 说明 | 示例 |
|:--|:--|:--|:--|
| **⚠️ 警告** | `# ⚠️ WARNING:` | 危险操作，可能导致崩溃 | `# ⚠️ WARNING: Vd_inject 必须自适应，不能写死` |
| **⚡ 注意** | `# ⚡ NOTE:` | 非显而易见的限制 | `# ⚡ NOTE: 辨识模式下放宽 max_step 到 0.1V` |
| **🔧 坑** | `# 🔧 PITFALL:` | 已知陷阱 | `# 🔧 PITFALL: asyncua 的 value 是 DataValue 对象` |
| **📐 公式** | `# 📐 FORMULA:` | 物理/数学公式引用 | `# 📐 FORMULA: E = ½Jω²` |
| **📖 参考** | `# 📖 REF:` | 外部参考 | `# 📖 REF: docs/TELEMETRY.md §4` |
| **✅ TODO** | `# ✅ TODO:` | 待完成 | `# ✅ TODO: 添加温度传感器模型` |

## 7. 版本管理

| 版本 | 日期 | 变更 |
|:--|:--|:--|
| v1.0 | — | 初始版本: PMSM + FOC 基础仿真 |
| v1.1 | — | 添加电网模型和故障注入 |
| v2.0 | — | 统一配置层、多电机模型、S4实验框架 |
| v2.5 | 2026-07-04 | RL 强化学习训练 (REINFORCE+Baseline) |
| v2.5.1 | 2026-07-08 | 📖 文档体系建立 (本次) |

## 8. 参考文档索引

| 文档 | 内容 | 对应手册章节 |
|:--|:--|:--|
| `ARCHITECTURE.md` | 系统架构总览、SLD、模块依赖图 | §1-2 (系统说明/部件概述) |
| `STATE_MACHINE.md` | 状态机、模式转换、子状态 | §3 (系统工作模式) |
| `TELEMETRY.md` | 遥测参数字典 (73点) | §10-11 (遥测参数) |
| `ERROR_CODES.md` | 错误码枚举 (100+) | §9 (系统状态和错误消息) |
| `CONTROL_SETPOINTS.md` | 控制设定值表 (59项) | §12 (控制设定值) |
| `INSTALLATION.md` | 安装部署 + 验证清单 | §13-15 (安装/预启动/清单) |
| `CONVENTIONS.md` | 本文档 (约定/术语/符号) | §0 (安全符号/字首组合词/惯例) |
