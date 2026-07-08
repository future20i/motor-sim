"""
s4_scenarios.py — 10个标准电网扰动场景定义。

子系统: 应用层 (S4)
依赖: grid_sim.py
手册对应章节: CONTROL_SETPOINTS.md §2 (放电触发阈值)

10个标准电网扰动场景定义。
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from enum import Enum


class ScenarioID(Enum):
    """场景 ID"""
    GRID_FREQ_DIP = 1          # 电网频率骤降 → 测试一次调频
    GRID_FREQ_SWELL = 2        # 频率骤升 → 测试吸收能力
    LOAD_STEP_UP = 3           # 负载突增 → 测试母线稳压
    LOAD_STEP_DOWN = 4         # 负载突降 → 测试过压保护
    VOLTAGE_SAG = 5            # 电压暂降 → 测试 LVRT
    VOLTAGE_SWELL = 6          # 电压暂升 → 测试过压响应
    PHASE_JUMP = 7             # 相位跳变 → 测试 PLL 鲁棒
    FREQ_OSCILLATION = 8       # 频率振荡 → 测试 VSG 阻尼
    COMBINED_STRESS = 9        # 复合扰动 → 测试综合鲁棒
    CHARGE_DISCHARGE_CYCLE = 10  # 充放电循环 → 测试 SOC 管理


@dataclass
class GridEvent:
    """单个电网事件"""
    t_start: float         # 开始时间 s
    t_end: float           # 结束时间 s (None=持续到结束)
    event_type: str        # 'freq_dip','freq_swell','v_sag','v_swell','phase_jump','freq_osc','load_step'
    magnitude: float       # 幅度 (Hz / pu / deg / W)
    ramp_rate: float = 0   # 变化率 Hz/s (0=瞬时)

    def __post_init__(self):
        if self.t_end is None:
            self.t_end = float('inf')


@dataclass
class ScenarioConfig:
    """场景配置"""
    scenario_id: ScenarioID
    name: str
    description: str
    duration: float        # 总仿真时间 s
    events: List[GridEvent] = field(default_factory=list)
    initial_soc: float = 0.5   # 初始 SOC
    initial_vdc: float = 750.0  # 初始母线电压

    # 评估权重
    weight_vdc_reg: float = 1.0    # 母线电压调节
    weight_freq_support: float = 1.0  # 调频响应
    weight_soc_mgmt: float = 0.5   # SOC 管理
    weight_efficiency: float = 0.3  # 效率


@dataclass
class ExperimentMetrics:
    """实验结果指标"""
    # 母线电压
    vdc_mean: float = 0
    vdc_std: float = 0       # 越小越好
    vdc_max_dev: float = 0   # 最大偏差 %
    vdc_settle_t: float = 0  # 恢复时间 s

    # 频率支撑
    freq_response_t: float = 0     # 响应时间 ms
    freq_power_peak: float = 0    # 峰值功率 W
    freq_energy: float = 0        # 总支撑能量 kWh

    # SOC
    soc_final: float = 0
    soc_range: float = 0          # SOC 波动范围
    soc_violations: int = 0       # SOC 越限次数

    # 效率
    total_loss_kWh: float = 0
    efficiency: float = 0

    # 综合
    composite_score: float = 0    # 加权综合分

    def to_dict(self) -> dict:
        return {
            "vdc": {"mean": self.vdc_mean, "std": self.vdc_std,
                    "max_dev_pct": self.vdc_max_dev, "settle_t_s": self.vdc_settle_t},
            "freq": {"response_ms": self.freq_response_t, "peak_kW": self.freq_power_peak/1000,
                     "energy_kWh": self.freq_energy},
            "soc": {"final": self.soc_final, "range": self.soc_range,
                    "violations": self.soc_violations},
            "efficiency": {"loss_kWh": self.total_loss_kWh, "efficiency_pct": self.efficiency*100},
            "composite": self.composite_score,
        }


# ═══════════════════════════════════════════
# 场景工厂
# ═══════════════════════════════════════════

def build_all_scenarios() -> List[ScenarioConfig]:
    """所有 S4 测试场景"""
    return [
        build_freq_dip(),
        build_freq_swell(),
        build_load_step_up(),
        build_load_step_down(),
        build_voltage_sag(),
        build_voltage_swell(),
        build_phase_jump(),
        build_freq_oscillation(),
        build_combined_stress(),
        build_charge_discharge_cycle(),
    ]


def build_freq_dip() -> ScenarioConfig:
    """电网频率骤降 50→49.5Hz → 飞轮应放电支撑"""
    return ScenarioConfig(
        scenario_id=ScenarioID.GRID_FREQ_DIP,
        name="频率骤降",
        description="t=0.5s 开始频率以 0.5Hz/s 从 50 降至 49.5Hz, 持续 2s → 飞轮应释放功率支撑电网",
        duration=5.0,
        events=[
            GridEvent(0.5, 3.5, 'freq_dip', -0.5, ramp_rate=0.5),
        ],
        weight_vdc_reg=0.5, weight_freq_support=2.0, weight_soc_mgmt=0.3,
    )


def build_freq_swell() -> ScenarioConfig:
    """频率骤升 50→50.5Hz → 飞轮应吸收功率"""
    return ScenarioConfig(
        scenario_id=ScenarioID.GRID_FREQ_SWELL,
        name="频率骤升",
        description="t=0.5s 频率从 50 升至 50.5Hz → 飞轮应吸收过剩功率",
        duration=5.0,
        events=[
            GridEvent(0.5, 3.5, 'freq_swell', 0.5, ramp_rate=0.5),
        ],
        weight_vdc_reg=0.3, weight_freq_support=2.0, weight_soc_mgmt=0.5,
    )


def build_load_step_up() -> ScenarioConfig:
    """负载突增 0→800kW → 母线电压应快速恢复"""
    return ScenarioConfig(
        scenario_id=ScenarioID.LOAD_STEP_UP,
        name="负载突增",
        description="t=1s 负载从 0 跳变到 800kW → 飞轮快速放电维持 Vdc",
        duration=5.0,
        events=[
            GridEvent(1.0, 4.0, 'load_step', 800e3),
        ],
        weight_vdc_reg=2.0, weight_freq_support=0.3, weight_soc_mgmt=0.5,
    )


def build_load_step_down() -> ScenarioConfig:
    """负载突降 800kW→0 → 母线不应过冲"""
    return ScenarioConfig(
        scenario_id=ScenarioID.LOAD_STEP_DOWN,
        name="负载突降",
        description="t=1s 负载从 800kW 跳变到 0 → 飞轮吸收避免 Vdc 过冲",
        duration=5.0,
        initial_soc=0.8,
        events=[
            GridEvent(0.0, 1.0, 'load_step', -800e3),
        ],
        weight_vdc_reg=2.0, weight_freq_support=0.3, weight_soc_mgmt=0.5,
    )


def build_voltage_sag() -> ScenarioConfig:
    """电压暂降 380→266V (0.7pu) → 测试低电压穿越"""
    return ScenarioConfig(
        scenario_id=ScenarioID.VOLTAGE_SAG,
        name="电压暂降 LVRT",
        description="t=0.5s 电网电压降至 0.7pu, 持续 0.5s → 逆变器应保持并网",
        duration=3.0,
        events=[
            GridEvent(0.5, 1.0, 'v_sag', 0.7),
        ],
        weight_vdc_reg=1.5, weight_freq_support=0.3, weight_soc_mgmt=0.3,
    )


def build_voltage_swell() -> ScenarioConfig:
    """电压暂升 380→456V (1.2pu)"""
    return ScenarioConfig(
        scenario_id=ScenarioID.VOLTAGE_SWELL,
        name="电压暂升",
        description="t=0.5s 电网电压升至 1.2pu, 持续 0.5s → 过压保护",
        duration=3.0,
        events=[
            GridEvent(0.5, 1.0, 'v_swell', 1.2),
        ],
        weight_vdc_reg=1.5, weight_freq_support=0.3, weight_soc_mgmt=0.3,
    )


def build_phase_jump() -> ScenarioConfig:
    """相位跳变 +30° → 测试 PLL 重新锁定"""
    return ScenarioConfig(
        scenario_id=ScenarioID.PHASE_JUMP,
        name="相位跳变",
        description="t=0.5s 电网相位跳变 +30° → PLL 应在 <100ms 内重新锁定",
        duration=3.0,
        events=[
            GridEvent(0.5, 2.5, 'phase_jump', 30.0),
        ],
        weight_vdc_reg=0.5, weight_freq_support=0.5, weight_soc_mgmt=0.3,
    )


def build_freq_oscillation() -> ScenarioConfig:
    """频率振荡 50±0.3Hz, 0.5Hz → 测试 VSG 阻尼"""
    return ScenarioConfig(
        scenario_id=ScenarioID.FREQ_OSCILLATION,
        name="频率振荡",
        description="电网频率在 49.7-50.3Hz 间振荡 → VSG 应提供阻尼",
        duration=8.0,
        events=[
            GridEvent(1.0, 7.0, 'freq_osc', 0.3),
        ],
        weight_vdc_reg=0.3, weight_freq_support=2.0, weight_soc_mgmt=0.5,
    )


def build_combined_stress() -> ScenarioConfig:
    """复合应力: 频率跌 + 负载突增 + 电压暂降"""
    return ScenarioConfig(
        scenario_id=ScenarioID.COMBINED_STRESS,
        name="复合应力测试",
        description="t=0.5s 频率跌至 49.6Hz + t=1.5s 负载 600kW + t=3s 电压暂降 0.8pu",
        duration=6.0,
        events=[
            GridEvent(0.5, 4.0, 'freq_dip', -0.4, ramp_rate=0.8),
            GridEvent(1.5, 3.5, 'load_step', 600e3),
            GridEvent(3.0, 3.5, 'v_sag', 0.8),
        ],
        weight_vdc_reg=1.0, weight_freq_support=1.5, weight_soc_mgmt=0.8,
    )


def build_charge_discharge_cycle() -> ScenarioConfig:
    """充放电循环: 充电→放电→充电"""
    return ScenarioConfig(
        scenario_id=ScenarioID.CHARGE_DISCHARGE_CYCLE,
        name="充放电循环",
        description="t=0 充电 1MW → t=3s 放电 1MW → t=6s 充电 500kW",
        duration=10.0,
        initial_soc=0.3,
        events=[
            GridEvent(0.0, 3.0, 'load_step', -1e6),   # 充电
            GridEvent(3.0, 6.0, 'load_step', 1e6),     # 放电
            GridEvent(6.0, 9.0, 'load_step', -500e3),  # 充电
        ],
        weight_vdc_reg=0.5, weight_freq_support=0.3, weight_soc_mgmt=2.0,
    )
