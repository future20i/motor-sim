"""
rl_environment.py — RL Gym 环境: 8D 观测 (Vdc/f/SOC/ω/P_load/P_grid/mode/event) → Discrete(6) 动作。

子系统: 应用层
依赖: power_ops.py, grid_sim.py

RL Gym 环境: 8D 观测 (Vdc/f/SOC/ω/P_load/P_grid/mode/event) → Discrete(6) 动作。
"""
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

from flywheel_energy import FlywheelEnergyStorage
from dc_bus import DCBus
from grid_sim import GridSimulator, GridConfig, GridFault
from grid_inverter import GridInverter, GridInvMode
from power_ops import PowerOrchestrator, PowerMode
from system_config import DEFAULT_CONFIG


@dataclass
class EnvConfig:
    """RL 环境配置"""
    # 仿真
    steps_per_action: int = 100     # 每个 RL step 跑多少仿真步 (100×50μs=5ms)
    max_episode_steps: int = 1000   # 最大 episode 步数

    # 奖励权重
    w_vdc: float = 1.0              # 母线电压调节
    w_freq: float = 1.0             # 频率支撑
    w_soc: float = 0.5              # SOC 管理
    w_stable: float = 0.3           # 稳定性奖励 (小偏差加分)

    # 惩罚
    fault_penalty: float = -100.0   # 故障惩罚
    soc_low_penalty: float = -50.0  # SOC 过低惩罚
    vdc_dev_penalty_scale: float = 0.5  # Vdc 偏差惩罚系数

    # 场景随机化 (提高泛化能力)
    randomize_scenario: bool = True
    freq_dip_prob: float = 0.3      # 频率跌落概率 (每 episode)
    load_step_prob: float = 0.3     # 负载突变概率
    voltage_sag_prob: float = 0.2   # 电压暂降概率


class FlywheelEnv(gym.Env):
    """飞轮 UPS 控制 RL 环境

    观测: 8维归一化连续向量
    动作: Discrete(6) 功率模式选择

    奖励设计:
      + 母线电压偏差小 → 正奖励
      + 频率快速响应   → 正奖励
      + SOC 管理好     → 正奖励
      - 触发故障       → 大负奖励
      - SOC 过低       → 负奖励
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, config: EnvConfig = None, system_config=None):
        super().__init__()
        self.env_cfg = config or EnvConfig()
        self.sys_cfg = system_config or DEFAULT_CONFIG
        Ts = self.sys_cfg.constants.Ts
        self.dt = Ts * self.env_cfg.steps_per_action  # 控制周期

        # ── 动作空间: 6 个功率模式 ──
        self.action_space = spaces.Discrete(6)
        self._action_map = [
            PowerMode.IDLE,
            PowerMode.DC_BUS_CONTROL,
            PowerMode.FREQ_REGULATION,
            PowerMode.VSG,
            PowerMode.POWER_TRACKING,
            PowerMode.INERTIA_SUPPORT,
        ]

        # ── 观测空间: 8维连续, 归一化到 [-1, 1] ──
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        # ── 系统组件 (延迟初始化) ──
        self.grid: Optional[GridSimulator] = None
        self.inverter: Optional[GridInverter] = None
        self.dc_bus: Optional[DCBus] = None
        self.flywheel: Optional[FlywheelEnergyStorage] = None
        self.orchestrator: Optional[PowerOrchestrator] = None

        self._step_count = 0
        self._episode_reward = 0.0
        self._fault_triggered = False

    def _init_system(self):
        """初始化仿真子系统"""
        Ts = self.sys_cfg.constants.Ts
        self.grid = GridSimulator(GridConfig(
            V_nom=self.sys_cfg.constants.Vac_nom,
            f_nom=self.sys_cfg.constants.f_nom,
        ), ts=Ts)
        self.inverter = GridInverter(self.sys_cfg.to_grid_inverter_config())
        self.dc_bus = DCBus(self.sys_cfg.to_dc_bus_config(), ts=Ts)
        self.flywheel = FlywheelEnergyStorage(system_config=self.sys_cfg)
        self.orchestrator = PowerOrchestrator(
            self.flywheel, grid=self.grid, ts=Ts,
        )

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, dict]:
        """重置环境"""
        super().reset(seed=seed)
        self._init_system()
        self._step_count = 0
        self._episode_reward = 0.0
        self._fault_triggered = False

        # 随机初始 SOC (30%-70%)
        init_soc = self.np_random.uniform(0.3, 0.7)
        fw = self.flywheel
        cfg = fw.cfg
        # 从 SOC 反推转速
        E_min = 0.5 * cfg.J * cfg.omega_min_rad ** 2
        E_max = cfg.E_max_joules
        E_target = E_min + init_soc * (E_max - E_min)
        omega_init = math.sqrt(2 * E_target / cfg.J)
        fw.state.omega = max(cfg.omega_min_rad, min(cfg.omega_max_rad, omega_init))
        fw.state.E_stored = 0.5 * cfg.J * fw.state.omega ** 2
        fw.state.SOC = init_soc

        # 随机初始母线电压
        self.dc_bus.state.Vdc = self.sys_cfg.constants.Vdc_nom * self.np_random.uniform(0.95, 1.05)

        # 预充逆变器到 VDC_CONTROL 模式
        self.inverter.state.Vdc = self.dc_bus.state.Vdc
        self.inverter.state.contactor_closed = True
        self.inverter._set_mode(GridInvMode.VDC_CONTROL)

        # 随机注入场景扰动 (提高泛化)
        if self.env_cfg.randomize_scenario:
            self._randomize_scenario()

        return self._get_obs(), self._get_info()

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """执行一个 RL 步进"""
        mode = self._action_map[action]
        self.orchestrator.mode = mode

        total_reward = 0.0
        vdc_dev_sum = 0.0
        freq_responded = False

        for _ in range(self.env_cfg.steps_per_action):
            # 1. 电网步进
            gs = self.grid.step()

            # 2. 逆变器 (VDC 稳压)
            self.inverter.step(
                Vdc=self.dc_bus.state.Vdc,
                grid_Va=gs.Va, grid_Vb=gs.Vb, grid_Vc=gs.Vc,
                mode=GridInvMode.VDC_CONTROL,
            )

            # 3. 功率调度
            po_s = self.orchestrator.step(
                Vdc=self.dc_bus.state.Vdc,
                f_grid=gs.f,
                SOC=self.flywheel.state.SOC,
                P_grid=self.inverter.state.P,
            )

            # 4. 直流母线
            self.dc_bus.step(
                P_rect=-self.inverter.state.P,
                P_flywheel=po_s.P_ref,
                P_load=getattr(self, '_active_load', 0.0),
            )

            # 5. 检测故障
            if self.flywheel.state.SOC < 0.03:
                self._fault_triggered = True
            if abs(self.dc_bus.state.Vdc - self.sys_cfg.constants.Vdc_nom) / self.sys_cfg.constants.Vdc_nom > 0.25:
                self._fault_triggered = True

            # 累计指标
            vdc_dev_sum += abs(self.dc_bus.state.Vdc - self.sys_cfg.constants.Vdc_nom)
            if abs(gs.f - self.sys_cfg.constants.f_nom) > 0.05:
                freq_responded = True

        # ── 奖励计算 ──
        ec = self.env_cfg
        Vdc_nom = self.sys_cfg.constants.Vdc_nom
        fw = self.flywheel

        avg_vdc_dev = vdc_dev_sum / ec.steps_per_action
        vdc_reward = ec.w_vdc * max(0.0, 100.0 - avg_vdc_dev * 2) / 100.0

        # 频率支撑: 事件发生时正确响应加分
        freq_reward = 0.0
        df = abs(self.grid.state.f - self.sys_cfg.constants.f_nom)
        if df > 0.05 and mode in (PowerMode.FREQ_REGULATION, PowerMode.VSG, PowerMode.INERTIA_SUPPORT):
            freq_reward = ec.w_freq * min(df / 0.5, 1.0)  # 频率偏差越大，正确响应分越高
        elif df < 0.02:
            freq_reward = ec.w_freq * 0.3  # 平稳时不做多余动作就加分

        # SOC 管理
        soc = fw.state.SOC
        soc_reward = ec.w_soc * (0.5 - abs(soc - 0.5))  # SOC 在 50% 左右最好

        # 稳定性: 小偏差加分
        stability = ec.w_stable * max(0.0, 1.0 - avg_vdc_dev / 5.0)

        reward = vdc_reward + freq_reward + soc_reward + stability

        # 惩罚
        if self._fault_triggered:
            reward += ec.fault_penalty
        if soc < 0.1:
            reward += ec.soc_low_penalty

        self._step_count += 1
        self._episode_reward += reward

        terminated = self._fault_triggered
        truncated = self._step_count >= self.env_cfg.max_episode_steps

        return self._get_obs(), float(reward), terminated, truncated, self._get_info()

    def _get_obs(self) -> np.ndarray:
        """构建归一化观测向量"""
        gs = self.grid.state
        fw = self.flywheel.state
        dc = self.dc_bus.state
        cfg = self.sys_cfg

        # 归一化到 [-1, 1]
        f_norm = max(-1.0, min(1.0, (gs.f - cfg.constants.f_nom) / 1.0))         # f
        df_norm = max(-1.0, min(1.0, (gs.f - 50.0) / 0.5))                       # df
        vdc_norm = max(-1.0, min(1.0, (dc.Vdc - cfg.constants.Vdc_nom) / 100.0)) # Vdc
        dvdc_norm = max(-1.0, min(1.0, (dc.Vdc - cfg.constants.Vdc_nom) / 50.0)) # dVdc
        soc_norm = max(-1.0, min(1.0, (fw.SOC - 0.5) * 2.0))                     # SOC → [-1,1]
        pload_norm = max(-1.0, min(1.0, getattr(self, '_active_load', 0.0) / 500e3))  # P_load
        omega_max_rad = cfg.flywheel.omega_max_rpm * 2 * math.pi / 60
        omega_norm = max(-1.0, min(1.0, (fw.omega / omega_max_rad) * 2.0 - 1.0))
        temp_norm = max(-1.0, min(1.0, (fw.temp - 40.0) / 80.0))                 # temp

        return np.array([
            f_norm, df_norm, vdc_norm, dvdc_norm,
            soc_norm, pload_norm, omega_norm, temp_norm,
        ], dtype=np.float32)

    def _get_info(self) -> dict:
        return {
            "step": self._step_count,
            "episode_reward": self._episode_reward,
            "soc": self.flywheel.state.SOC,
            "vdc": self.dc_bus.state.Vdc,
            "f_grid": self.grid.state.f,
            "fault": self._fault_triggered,
            "mode": self.orchestrator.mode.name if self.orchestrator else "N/A",
        }

    def _randomize_scenario(self):
        """随机注入场景扰动 (提高 RL 泛化能力)"""
        ec = self.env_cfg
        r = self.np_random

        if r.random() < ec.freq_dip_prob:
            df = r.uniform(-0.5, -0.1)  # 频率跌落 0.1-0.5Hz
            self.grid.set_frequency_deviation(df)

        if r.random() < ec.load_step_prob:
            self._active_load = r.uniform(100e3, 800e3)  # 100-800kW 负载

        if r.random() < ec.voltage_sag_prob:
            self.grid.set_voltage(r.uniform(0.6, 0.9))  # 60-90% 电压

    def render(self):
        """打印当前状态"""
        info = self._get_info()
        print(f"[{info['step']:4d}] mode={info['mode']:16s} "
              f"SOC={info['soc']:.2f} Vdc={info['vdc']:.0f}V "
              f"f={info['f_grid']:.2f}Hz reward={self._episode_reward:.1f}")

    def close(self):
        pass
