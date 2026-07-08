# 错误码与系统消息

**文档编号:** MOTORSIM-ERR-04
**版本:** v2.5
**最后更新:** 2026-07-08

---

## 1. 消息分类

| 级别 | 符号 | 含义 | 是否保护负载 | 说明 |
|:--|:--|:--|:--|:--|
| **信息** | ℹ️ | 正常操作消息 | ✅ 是 / — | 启动/停止/切换等正常事件 |
| **注意** | 🟡 | 非关键错误 | ⚠️ 不一定 | 需要关注但不影响运行 |
| **警报** | 🔴 | 关键错误 | ❌ 否 | 需要立即处理，可能触发关机 |
| **维修** | 🔧 | 维护相关 | — | 提醒维护/校准/更换 |
| **放电** | ⚡ | 放电事件 | ✅ 是 | 飞轮正在供电 |
| **错误** | ❌ | 故障状态 | ❌ 否 | 设备异常需修复 |

---

## 2. 飞轮与电机错误

### 2.1 飞轮本体

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| FW-001 | Flywheel Overspeed at _ RPM | 🔴 ALARM | 否 | 飞轮超速 (>9900 RPM) | 紧急制动，检查逆变器 |
| FW-002 | Flywheel Underspeed at _ RPM | 🟡 NOTICE | 否 | 飞轮转速低于最低放电转速 | 检查电机驱动 |
| FW-003 | Flywheel not charging | 🟡 NOTICE | 否 | 飞轮未充电 (转速不增加) | 检查电机驱动和电源 |
| FW-004 | Flywheel Bearing Replacement is due | 🔧 SERVICE | 是 | 轴承更换到期 (2.5-3年) | 安排轴承更换 |
| FW-005 | Flywheel Lockout due to High Bottom Bearing Temp | 🔴 ALARM | 否 | 下轴承温度超限致飞轮锁定 | 更换轴承 |
| FW-006 | Flywheel Lockout due to High Vibration at _ RPM | 🔴 ALARM | 否 | 振动超限致飞轮锁定 | 检查轴承/平衡 |
| FW-007 | Flywheel Overcurrent Shutdown | 🔴 ALARM | 否 | 飞轮过流关机 | 检查负载/逆变器 |
| FW-008 | Flywheel Master Enable Bit Low Error | ❌ ERROR | 否 | 飞轮使能位错误 | 检查硬件/控制器 |
| FW-009 | Invalid Bearing Install date | ❌ ERROR | 否 | 轴承安装日期无效 | 设定正确日期 |
| FW-010 | Flywheel Preheat not required temp > 30.0 C | ℹ️ INFO | — | 飞轮超过30°C不需要预热 | 无 |
| FW-011 | Flywheel preheat complete equalizing for _ minutes | ℹ️ INFO | — | 预热完成，热量扩散均衡中 | 无 |
| FW-012 | Bearing Install Date: XX/XX/XXXX | ℹ️ INFO | — | 轴承更换记录 | 无 |

### 2.2 励磁线圈

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| FC-001 | Top Field Coil Fault | ❌ ERROR | 否 | 上励磁线圈电流错误 | 检查IGBT和接线 |
| FC-002 | Bottom Field Coil Fault | ❌ ERROR | 否 | 下励磁线圈电流错误 | 检查IGBT和接线 |
| FC-003 | Top Field Coil Timeout | ❌ ERROR | 否 | 上励磁线圈超时 | 检查IGBT响应 |
| FC-004 | Bottom Field Coil Timeout | ❌ ERROR | 否 | 下励磁线圈超时 | 检查IGBT响应 |
| FC-005 | Top Hall Error | ❌ ERROR | 否 | 上霍尔传感器错误 | 更换传感器 |
| FC-006 | Bottom Hall Error | ❌ ERROR | 否 | 下霍尔传感器错误 | 更换传感器 |

### 2.3 电机控制

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| MC-001 | Motor Phase Current Overload A:_ B:_ C:_ | 🔴 ALARM | 否 | 电机相电流过载 | 减小负载 |
| MC-002 | Motor Phase Current Imbalance | 🟡 NOTICE | 否 | 电机相电流不平衡 | 检查接线/绕组 |
| MC-003 | Motor Temperature Warning: Over limit | 🟡 NOTICE | 是 | 电机温度超限警告 | 检查冷却 |
| MC-004 | Motor Temperature Critical: Shutdown | 🔴 ALARM | 否 | 电机温度临界→关机 | 立即停机和检查 |
| MC-005 | RPS (Rotor Position Sensor) Lost | ❌ ERROR | 否 | 转子位置传感器失锁 | 检查编码器 |
| MC-006 | Encoder Pulse Loss Detected | 🔴 ALARM | 否 | 编码器丢脉冲 | 更换编码器 |
| MC-007 | FW IGBT Current Error - Phase X - Check cable | ❌ ERROR | 否 | IGBT电流低于最小值 | 检查IGBT电缆 |

---

## 3. 直流母线错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| DC-001 | DC Bus Voltage Imbalance Error (>100V) | ❌ ERROR | 否 | 正负母线不平衡 >100V | 检查母线电容 |
| DC-002 | DC High Voltage Error (>900V) | 🔴 ALARM | 否 | 直流母线过压 | 检查整流器/逆变器 |
| DC-003 | DC Low Voltage Error (<550V) | 🔴 ALARM | 否 | 直流母线欠压 | 检查电源/电容 |
| DC-004 | DC Bus Capacitor Degradation Warning | 🟡 NOTICE | 是 | 母线电容退化警告 | 计划更换电容 |

---

## 4. 电网与逆变器错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| GR-001 | Input Over Voltage: A:_ B:_ C:_ | ⚡ DISCHARGE | 是 | 输入线电压超出上限 | 自动放电→恢复正常 |
| GR-002 | Input Under Voltage: A:_ B:_ C:_ | ⚡ DISCHARGE | 是 | 输入线电压低于下限 | 自动放电→恢复正常 |
| GR-003 | Over Frequency Detected @ _ Hz | ⚡ DISCHARGE | 是 | 频率超出上限 | 自动放电→恢复正常 |
| GR-004 | Under Frequency Detected @ _ Hz | ⚡ DISCHARGE | 是 | 频率低于下限 | 自动放电→恢复正常 |
| GR-005 | Input Phase Rotation Error (CCW) | ❌ ERROR | 否 | 输入相位反转 (C,B,A) | 修正接线 |
| GR-006 | Input Freq Range Err: _ | ⚡ DISCHARGE | 是 | 脱离放电时频率仍然差→退回放电 | 等待电网恢复 |
| GR-007 | Input Voltage Range Err: A:_ B:_ C:_ | ⚡ DISCHARGE | 是 | 脱离放电时电压仍然差→退回放电 | 等待电网恢复 |
| GR-008 | PLL Frequency Lock Lost | ⚡ DISCHARGE | 是 | PLL 失锁 | 检查电网质量 |
| GR-009 | Ground Fault Detected | 🟡 NOTICE | 是 | 3线系统检测到接地故障 | 排查接地 |
| GR-010 | Ground Fault Cleared | ℹ️ INFO | — | 接地故障清除 | 无 |

---

## 5. 功率与过载错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| PO-001 | System Overload 10 Min: _ kW | 🔴 ALARM | 否 | 10分钟过载 | 减小负载 |
| PO-002 | System Overload 2 Min: _ kW | 🔴 ALARM | 否 | 2分钟过载 | 减小负载 |
| PO-003 | System Overload 30 Sec: _ kW | 🔴 ALARM | 否 | 30秒过载 | 立即减小负载 |
| PO-004 | Input Current Bypass Overload: A:_% B:_% C:_% | ❌ ERROR | 否 | 旁路电流过载 | 减小负载 |
| PO-005 | Output Current Overload 10 Min: A:_ B:_ C:_ | ❌ ERROR | 否 | 10分钟输出过载 | 减小负载 |
| PO-006 | Output Current Overload 2 Min: A:_ B:_ C:_ | ❌ ERROR | 否 | 2分钟输出过载 | 减小负载 |
| PO-007 | Output Current Overload 30 Sec: A:_ B:_ C:_ | ❌ ERROR | 否 | 30秒输出过载 | 立即减小负载 |
| PO-008 | Overload Cleared _ rated kW | ℹ️ INFO | — | 过载已清除 | 无 |
| PO-009 | Load Sharing Error Imbalance _ | 🟡 NOTICE | 是 | 多MMU负载不均衡 | 检查并联均流 |

---

## 6. 热管理错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| TH-001 | High Room Temperature Reported | 🟡 NOTICE | 是 | 室温过高 | 检查空调 |
| TH-002 | High Room Temperature Cleared | ℹ️ INFO | — | 室温恢复 | 无 |
| TH-003 | Cabinet Temperature Warning: Over limit | 🟡 NOTICE | 是 | 柜内温度警告 | 检查通风 |
| TH-004 | Static Switch Temperature Warning | 🟡 NOTICE | 是 | 静态开关温度警告 | 检查负载/通风 |
| TH-005 | Fan _ has failed | ❌ ERROR | 否 | 风扇故障 (流量指示器) | 检查/更换风扇 |
| TH-006 | Fan _ now okay | ℹ️ INFO | — | 风扇恢复 | 无 |
| TH-007 | Multiple Fan Failure: System Shutdown | 🔴 ALARM | 否 | 多重风扇故障→关机 | 立即更换风扇 |

---

## 7. 真空与轴承错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| VC-001 | Vacuum Gauge Warning: Over limit | 🟡 NOTICE | 是 | 真空度超限 (压力升高) | 检查真空泵 |
| VC-002 | System Waiting on Vacuum | 🟡 NOTICE | 否 | 等待真空建立 (冷启动正常) | 运转几小时后消失 |
| VC-003 | Vacuum Signal Lost — check connections and sender | ❌ ERROR | 否 | 真空信号丢失 | 检查连接和传感器 |
| BR-001 | Radial Bearing Vibration Warning: Over limit | 🟡 NOTICE | 是 | 径向振动警告 (>0.25G) | 检查轴承 |
| BR-002 | Axial Bearing Vibration Warning: Over limit | 🟡 NOTICE | 是 | 轴向振动警告 | 检查轴承 |
| BR-003 | Bottom Bearing Force Warning | 🟡 NOTICE | 是 | 下轴承承载力异常 | 检查轴承 |

---

## 8. 接触器与开关错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| SW-001 | K3 Circuit Breaker Over Current Trip | ❌ ERROR | 否 | 旁路断路器过流跳闸 | 检查负载/复位 |
| SW-002 | K3 Circuit Breaker Status Error | ❌ ERROR | 否 | 旁路断路器状态错误 | 检查位置传感器 |
| SW-003 | Bypass Contactor Failed Open (Low Voltage) | ❌ ERROR | 否 | 旁路接触器因低压未闭合 | 检查电压/接触器 |
| SW-004 | Bypass Static Switch Contactor Fail Open | ❌ ERROR | 否 | 静态开关接触器未闭合 | 检查SCR/接触器 |
| SW-005 | Bypass Static Switch Contactor Stuck Closed | ❌ ERROR | 否 | 静态开关接触器粘连 | 更换接触器 |
| SW-006 | Reverse Input Power In Discharge, _ kW | ❌ ERROR | 否 | 放电期间检测到反向功率流 | 检查K3熔接 |

---

## 9. 通信与系统错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| CM-001 | Node Timeout, Node Stopped Communicating | 🟡 NOTICE | 是 | 节点通信超时 | 检查网络/UPSNet |
| CM-002 | Operating Parameter Mismatch with Node _ | 🟡 NOTICE | 是 | 多飞轮参数不匹配 | 检查固件版本 |
| CM-003 | MMU _ Primary UPSNet Data Lock Lost | ❌ ERROR | 否 | MMU主数据锁丢失 | 检查电缆/终端电阻 |
| CM-004 | MMU _ Backup UPSNet Data Lock Lost | ❌ ERROR | 否 | MMU备份数据锁丢失 | 检查电缆/终端电阻 |
| CM-005 | UPSNet Master Changed Previous: _ Now: _ | ℹ️ INFO | — | 主MMU变更 | 无 |
| CM-006 | MODBUS: Connection Made | ℹ️ INFO | — | Modbus连接建立 | 无 |
| CM-007 | MODBUS: Disconnected | 🟡 NOTICE | — | Modbus断开 | 检查连接 |
| CM-008 | Email Send Failure — Check Mail Server IP | ❌ ERROR | — | 邮件发送失败 | 检查邮件服务器 |
| CM-009 | Modem: Page Failed: _ | ❌ ERROR | — | 寻呼失败 | 检查电话线/号码 |

---

## 10. 电源与保险丝错误

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| PS-001 | Both Power Supplies (1 and 2) Failed | ❌ ERROR | 否 | 双电源均故障 | 检查电源输入 |
| PS-002 | Sys PS _ Failure | ❌ ERROR | 否 | 系统电源故障 | 更换电源 |
| PS-003 | Sys PS _ OKAY | ℹ️ INFO | — | 系统电源恢复 | 无 |
| PS-004 | Sys Fuse Failure | ❌ ERROR | 否 | 控制电源保险丝熔断 | 更换保险丝 |
| PS-005 | System _ Fuse Failure | ❌ ERROR | 否 | 系统保险丝熔断 | 更换保险丝 |
| PS-006 | Bypass Static Switch Fuse has failed | ❌ ERROR | 否 | 静态旁路保险丝熔断 | 更换保险丝 |
| PS-007 | Bypass Static Switch Fuse Cleared | ℹ️ INFO | — | 静态旁路保险丝已清除 | 无 |
| PS-008 | Zigzag Fuse Blown | ❌ ERROR | 否 | 4线系统中性保险丝熔断 | 更换保险丝 |

---

## 11. 紧急事件

| 代码 | 消息 | 级别 | 是否放电 | 说明 | 措施 |
|:--|:--|:--|:--|:--|:--|
| EM-001 | EPO Activated (Front Panel) | 🔴 ALARM | 否 | 前门面板EPO激活 | 查明原因→复原 |
| EM-002 | EPO Activated (Serial Data) | 🔴 ALARM | 否 | 内部串联链路EPO激活 | 查明原因→复原 |
| EM-003 | EMO Activated | 🔴 ALARM | 否 | MMU紧急模块关闭 | 查明原因→复原 |
| EM-004 | Remote EPO Activated | 🔴 ALARM | 否 | 远程EPO按钮按下 | 查明原因→复原 |
| EM-005 | Remote EPO (Serial Data) | 🔴 ALARM | 否 | 通过串联链路远程EPO | 查明原因→复原 |
| EM-006 | System Cabinet ESO Switch Activated | 🔴 SHUTDOWN | 否 | 系统柜紧急关闭开关激活 | 查明原因→复原 |
| EM-007 | Fire Alarm Detected | 🔴 ALARM | 否 | 火灾报警激活 | 执行消防预案 |
| EM-008 | Building Alarm 1/2/3/4 Detected | ℹ️ INFO | — | 楼宇报警输入 | 检查楼宇系统 |

---

## 12. 状态枚举代码

```python
class ErrorCode(Enum):
    """错误码枚举 — 对应上表"""

    # 飞轮本体 (FW-xxx)
    FW_OVERSPEED = "FW-001"
    FW_UNDERSPEED = "FW-002"
    FW_NOT_CHARGING = "FW-003"
    FW_BEARING_REPLACE_DUE = "FW-004"
    FW_LOCKOUT_BEARING_TEMP = "FW-005"
    FW_LOCKOUT_VIBRATION = "FW-006"
    FW_OVERCURRENT_SHUTDOWN = "FW-007"
    # ...

class ErrorLevel(Enum):
    INFO = "info"       # ℹ️ 信息
    NOTICE = "notice"   # 🟡 注意
    ALARM = "alarm"     # 🔴 警报
    ERROR = "error"     # ❌ 错误
    SERVICE = "service" # 🔧 维修
    DISCHARGE = "discharge"  # ⚡ 放电
    SHUTDOWN = "shutdown"    # 🔴 关机

@dataclass
class SystemMessage:
    """系统消息 — 对应手册第 10 节消息矩阵"""
    code: ErrorCode
    level: ErrorLevel
    supports_load: bool       # 是否支持负载
    message: str              # 消息原文
    description: str          # 中文说明
    action: str               # 建议措施
```
