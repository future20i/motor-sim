"""
test_rl_env.py — RL 环境单元测试: 动作空间/观测空间/step/reset。

子系统: 应用层测试
依赖: rl_environment.py

RL 环境单元测试: 动作空间/观测空间/step/reset。
"""
import sys
import os
import pytest
import numpy as np

# 确保项目根目录在 sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rl_environment import FlywheelRLEnv, MODE_MAP, DC_MODE_MAP


class TestRLEnvironment:
    """FlywheelRLEnv 基础测试"""

    @pytest.fixture
    def env(self):
        """创建环境实例 (确定性初始化)"""
        env = FlywheelRLEnv(
            ts_rl=0.02,
            ts_physics=50e-6,
            max_episode_steps=100,
            random_init=False,
        )
        yield env
        env.close()

    @pytest.fixture
    def env_random(self):
        """创建环境实例 (随机初始化)"""
        env = FlywheelRLEnv(
            ts_rl=0.02,
            ts_physics=50e-6,
            max_episode_steps=100,
            random_init=True,
        )
        yield env
        env.close()

    # ═══════════════════════════════════════════════════════════
    # 测试 1: 环境创建
    # ═══════════════════════════════════════════════════════════

    def test_env_creation(self):
        """创建环境 → 验证 observation/action space"""
        env = FlywheelRLEnv(random_init=False)
        try:
            # 检查空间类型和形状
            assert env.observation_space.shape == (10,), \
                f"Expected obs shape (10,), got {env.observation_space.shape}"
            assert isinstance(env.observation_space.low, np.ndarray)
            assert isinstance(env.observation_space.high, np.ndarray)

            # 检查动作空间
            assert "mode" in env.action_space.spaces
            assert "droop" in env.action_space.spaces
            assert "deadband" in env.action_space.spaces
            assert "dc_mode" in env.action_space.spaces

            # mode: Discrete(6)
            assert env.action_space["mode"].n == 6
            # dc_mode: Discrete(4)
            assert env.action_space["dc_mode"].n == 4
            # droop: Box(0.01, 0.10)
            assert env.action_space["droop"].shape == (1,)
            # deadband: Box(0.01, 0.10)
            assert env.action_space["deadband"].shape == (1,)

            print("✓ 环境创建成功, 空间定义正确")
        finally:
            env.close()

    def test_action_space(self, env):
        """动作空间维度正确"""
        action = env.action_space.sample()

        assert isinstance(action, dict)
        assert "mode" in action
        assert "droop" in action
        assert "deadband" in action
        assert "dc_mode" in action
        assert 0 <= action["mode"] < 6
        assert 0 <= action["dc_mode"] < 4

        print("✓ 动作空间维度正确")

    # ═══════════════════════════════════════════════════════════
    # 测试 2: reset
    # ═══════════════════════════════════════════════════════════

    def test_reset(self, env):
        """reset() 返回有效 observation"""
        obs, info = env.reset()

        assert isinstance(obs, np.ndarray), f"Expected ndarray, got {type(obs)}"
        assert obs.shape == (10,), f"Expected shape (10,), got {obs.shape}"
        assert obs.dtype == np.float32, f"Expected float32, got {obs.dtype}"
        assert isinstance(info, dict)

        # 检查观察值在合理范围
        f_grid, df, dfdt, Vdc, dVdc, SOC, P_grid, P_load, f_pll, Vg = obs

        assert 48.0 <= f_grid <= 52.0, f"f_grid={f_grid} out of range"
        assert -2.0 <= df <= 2.0, f"df={df} out of range"
        assert 500.0 <= Vdc <= 900.0, f"Vdc={Vdc} out of range"
        assert 0.0 <= SOC <= 1.0, f"SOC={SOC} out of range"
        assert 48.0 <= f_pll <= 52.0, f"f_pll={f_pll} out of range"

        # 确定性初始化: SOC 应为 0.5
        assert abs(SOC - 0.5) < 0.01, f"Expected SOC≈0.5 (deterministic), got {SOC}"

        print(f"✓ reset 返回有效 observation: f={f_grid:.2f}, Vdc={Vdc:.0f}, SOC={SOC:.3f}")

    def test_reset_random(self, env_random):
        """reset() 随机初始化产生不同状态"""
        states = []
        for _ in range(5):
            obs, _ = env_random.reset(seed=None)
            states.append((float(obs[0]), float(obs[3]), float(obs[5])))
            # 重置内部状态，避免累积
            env_random.close()
            env_random = FlywheelRLEnv(random_init=True, max_episode_steps=100)

        # 检查至少有一组不同
        unique = set(states)
        assert len(unique) > 1, f"随机初始化应该产生不同状态, 但得到 {states}"

        print(f"✓ 随机初始化产生 {len(unique)}/{len(states)} 个不同状态")

    # ═══════════════════════════════════════════════════════════
    # 测试 3: step
    # ═══════════════════════════════════════════════════════════

    def test_step(self, env):
        """step() 返回 (obs, reward, terminated, truncated, info)"""
        obs, _ = env.reset()

        action = {
            "mode": np.int64(1),  # DC_BUS_CONTROL
            "droop": np.array([0.04], dtype=np.float32),
            "deadband": np.array([0.02], dtype=np.float32),
            "dc_mode": np.int64(1),  # ff_pi
        }

        obs2, reward, terminated, truncated, info = env.step(action)

        assert isinstance(obs2, np.ndarray)
        assert obs2.shape == (10,)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert isinstance(info, dict)

        # 正常运行不应立即终止
        assert not terminated, "不应在第一步就终止"

        print(f"✓ step 返回正确: reward={reward:.3f}, Vdc={obs2[3]:.0f}, SOC={obs2[5]:.3f}")

    def test_step_reward_range(self, env):
        """奖励在合理范围内"""
        env.reset()

        action = {
            "mode": np.int64(2),  # FREQ_REGULATION
            "droop": np.array([0.05], dtype=np.float32),
            "deadband": np.array([0.02], dtype=np.float32),
            "dc_mode": np.int64(1),
        }

        rewards = []
        for _ in range(20):
            obs, reward, terminated, truncated, _ = env.step(action)
            rewards.append(reward)
            if terminated or truncated:
                break

        assert len(rewards) > 0
        # 奖励应在合理范围 (正常大约 -0.5 到 2.5)
        for r in rewards:
            assert -25.0 <= r <= 3.0, f"奖励 {r} 超出预期范围"

        print(f"✓ 20 步奖励: min={min(rewards):.2f}, max={max(rewards):.2f}, avg={np.mean(rewards):.2f}")

    # ═══════════════════════════════════════════════════════════
    # 测试 4: 基本循环
    # ═══════════════════════════════════════════════════════════

    def test_basic_loop(self, env):
        """跑 100 步不崩溃"""
        obs, _ = env.reset()

        for i in range(100):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)

            assert obs is not None
            assert not np.any(np.isnan(obs)), f"Step {i}: NaN in observation"
            assert not np.any(np.isinf(obs)), f"Step {i}: Inf in observation"

            if terminated or truncated:
                break

        print(f"✓ 循环完成 {i + 1} 步, 最终 Vdc={obs[3]:.0f}, SOC={obs[5]:.3f}")

    def test_all_modes(self, env):
        """测试所有 6 种模式都能运行"""
        obs, _ = env.reset()

        for mode_idx in range(6):
            action = {
                "mode": np.int64(mode_idx),
                "droop": np.array([0.04], dtype=np.float32),
                "deadband": np.array([0.02], dtype=np.float32),
                "dc_mode": np.int64(1),
            }
            obs, reward, terminated, truncated, info = env.step(action)

            assert not np.any(np.isnan(obs)), f"模式 {mode_idx}: NaN in observation"
            assert not terminated, f"模式 {mode_idx}: 意外终止"

        print("✓ 所有 6 种模式正常运行")

    def test_all_dc_modes(self, env):
        """测试所有 4 种 DC 控制模式"""
        obs, _ = env.reset()

        for dc_mode_idx in range(4):
            action = {
                "mode": np.int64(1),  # DC_BUS_CONTROL
                "droop": np.array([0.04], dtype=np.float32),
                "deadband": np.array([0.02], dtype=np.float32),
                "dc_mode": np.int64(dc_mode_idx),
            }
            obs, reward, terminated, truncated, info = env.step(action)

            assert not np.any(np.isnan(obs)), f"DC 模式 {dc_mode_idx}: NaN in observation"
            mode_str = DC_MODE_MAP[dc_mode_idx]
            assert info["dc_mode_applied"] == mode_str, \
                f"Expected dc_mode_applied='{mode_str}', got '{info['dc_mode_applied']}'"

        print("✓ 所有 4 种 DC 控制模式正常运行")

    # ═══════════════════════════════════════════════════════════
    # 测试 5: 边界情况
    # ═══════════════════════════════════════════════════════════

    def test_low_soc_discharge(self, env):
        """低 SOC 下放电 → 应触发终止或越限惩罚"""
        env.reset()

        # 手动设置低 SOC
        env._flywheel.state.SOC = 0.02
        env._flywheel.state.E_stored = 0.02 * env._flywheel.cfg.E_max_joules
        env._flywheel.state.omega = env._flywheel.cfg.omega_min_rad * 1.02

        terminated_ever = False
        for _ in range(50):
            action = {
                "mode": np.int64(2),  # FREQ_REGULATION (放电)
                "droop": np.array([0.04], dtype=np.float32),
                "deadband": np.array([0.02], dtype=np.float32),
                "dc_mode": np.int64(1),
            }
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated:
                terminated_ever = True
                break

        # 低 SOC 下至少应该有越限惩罚
        assert terminated_ever or env._violation_occurred, \
            "低 SOC 下应该有越限惩罚或终止"

        print(f"✓ 低 SOC 场景: terminated={terminated_ever}, violation={env._violation_occurred}")

    def test_grid_frequency_disturbance(self, env):
        """电网频率扰动下环境正常运行"""
        env.reset()

        # 注入频率下降
        env._grid.set_frequency_deviation(-0.8)  # 49.2 Hz

        for i in range(30):
            action = {
                "mode": np.int64(2),  # FREQ_REGULATION
                "droop": np.array([0.04], dtype=np.float32),
                "deadband": np.array([0.02], dtype=np.float32),
                "dc_mode": np.int64(1),
            }
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated:
                break

        f_grid = obs[0]
        assert f_grid < 50.0, f"频率应低于 50Hz (频率下降场景), 但 f={f_grid}"

        print(f"✓ 频率扰动场景: f_grid={f_grid:.2f}Hz, Vdc={obs[3]:.0f}V")

    # ═══════════════════════════════════════════════════════════
    # 测试 6: 信息字段
    # ═══════════════════════════════════════════════════════════

    def test_info_fields(self, env):
        """info 字典包含必要字段"""
        obs, info = env.reset()
        assert "step" in info
        assert "Vdc" in info
        assert "SOC" in info
        assert "f_grid" in info
        assert "mode" in info
        assert "violation" in info

        action = {
            "mode": np.int64(1),
            "droop": np.array([0.04], dtype=np.float32),
            "deadband": np.array([0.02], dtype=np.float32),
            "dc_mode": np.int64(2),
        }
        obs, reward, terminated, truncated, info = env.step(action)
        assert "mode_applied" in info
        assert "dc_mode_applied" in info

        print(f"✓ info 字段完整: {list(info.keys())}")


# ═══════════════════════════════════════════════════════════
# 集成测试: 与环境交互的完整工作流
# ═══════════════════════════════════════════════════════════

class TestIntegration:
    """集成测试 — 验证环境与底层模型的正确交互"""

    def test_dc_bus_stability_with_dc_control(self):
        """DC_BUS_CONTROL 模式应能稳定母线电压"""
        env = FlywheelRLEnv(random_init=False, max_episode_steps=200)
        try:
            env.reset()
            # 设一个负载扰动
            env._dc_bus.set_load(200e3)

            Vdc_history = []
            for i in range(100):
                action = {
                    "mode": np.int64(1),  # DC_BUS_CONTROL
                    "droop": np.array([0.04], dtype=np.float32),
                    "deadband": np.array([0.02], dtype=np.float32),
                    "dc_mode": np.int64(1),  # ff_pi
                }
                obs, reward, terminated, truncated, _ = env.step(action)
                Vdc_history.append(float(obs[3]))
                if terminated:
                    break

            # DC_BUS_CONTROL 应能保持 Vdc 在合理范围
            for Vdc in Vdc_history[-20:]:  # 稳态
                assert 500.0 <= Vdc <= 900.0, f"Vdc={Vdc} 超出范围"

            print(f"✓ DC 母线控制: Vdc 范围 [{min(Vdc_history):.0f}, {max(Vdc_history):.0f}]V")
        finally:
            env.close()

    def test_freq_regulation_responds_to_deviation(self):
        """FREQ_REGULATION 模式应响应频率偏差"""
        env = FlywheelRLEnv(random_init=False, max_episode_steps=200)
        try:
            env.reset()
            # 注入频率下降 (应触发飞轮放电)
            env._grid.set_frequency_deviation(-0.5)  # 49.5 Hz

            soc_before = env._flywheel.state.SOC
            for _ in range(30):
                action = {
                    "mode": np.int64(2),  # FREQ_REGULATION
                    "droop": np.array([0.04], dtype=np.float32),
                    "deadband": np.array([0.02], dtype=np.float32),
                    "dc_mode": np.int64(1),
                }
                obs, reward, terminated, truncated, _ = env.step(action)
                if terminated:
                    break

            soc_after = env._flywheel.state.SOC
            # 频率下降 → 飞轮应放电 → SOC 应下降
            assert soc_after <= soc_before + 0.001, \
                f"频率下降应导致 SOC 下降, SOC: {soc_before:.4f} → {soc_after:.4f}"

            print(f"✓ 调频响应: SOC {soc_before:.4f} → {soc_after:.4f} (频率下降场景)")
        finally:
            env.close()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
