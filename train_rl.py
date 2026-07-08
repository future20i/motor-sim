"""
train_rl.py — RL 训练脚本: REINFORCE+Baseline, Policy 8→64→6, Value 8→32→1。

子系统: 应用层
依赖: rl_environment.py

RL 训练脚本: REINFORCE+Baseline, Policy 8→64→6, Value 8→32→1。
"""
import argparse
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from rl_environment import FlywheelEnv, EnvConfig


# ═══════════════════════════════════════
# 神经网络 (纯 numpy)
# ═══════════════════════════════════════

class PolicyNetwork:
    """2层全连接网络 → 6类动作 logits

    输入:  8维 (obs)
    隐藏:  64维 ReLU
    输出:  6维 softmax
    """

    def __init__(self, input_dim: int = 8, hidden_dim: int = 64,
                 output_dim: int = 6, seed: int = 42):
        rng = np.random.RandomState(seed)
        # He 初始化
        self.W1 = rng.randn(input_dim, hidden_dim) * np.sqrt(2.0 / input_dim)
        self.b1 = np.zeros(hidden_dim)
        self.W2 = rng.randn(hidden_dim, output_dim) * np.sqrt(2.0 / hidden_dim)
        self.b2 = np.zeros(output_dim)

    def forward(self, x: np.ndarray) -> np.ndarray:
        """前向传播 → 动作概率分布"""
        z1 = x @ self.W1 + self.b1
        a1 = np.maximum(0, z1)  # ReLU
        z2 = a1 @ self.W2 + self.b2
        # Softmax (数值稳定)
        z2 -= np.max(z2)
        exp_z = np.exp(z2)
        return exp_z / np.sum(exp_z)

    def sample(self, x: np.ndarray) -> Tuple[int, float]:
        """采样动作 + log概率"""
        probs = self.forward(x)
        action = np.random.choice(len(probs), p=probs)
        return int(action), float(np.log(probs[action] + 1e-8))

    def get_weights(self) -> dict:
        return {"W1": self.W1, "b1": self.b1, "W2": self.W2, "b2": self.b2}

    def set_weights(self, weights: dict):
        for k, v in weights.items():
            setattr(self, k, v)


class ValueNetwork:
    """状态价值网络 (baseline)

    输入:  8维
    隐藏:  32维 ReLU
    输出:  1维 (V(s))
    """

    def __init__(self, input_dim: int = 8, hidden_dim: int = 32, seed: int = 42):
        rng = np.random.RandomState(seed)
        self.W1 = rng.randn(input_dim, hidden_dim) * np.sqrt(2.0 / input_dim)
        self.b1 = np.zeros(hidden_dim)
        self.W2 = rng.randn(hidden_dim, 1) * 0.01
        self.b2 = np.zeros(1)

    def predict(self, x: np.ndarray) -> float:
        z1 = x @ self.W1 + self.b1
        a1 = np.maximum(0, z1)
        return float((a1 @ self.W2 + self.b2).item())

    def get_weights(self) -> dict:
        return {"W1": self.W1, "b1": self.b1, "W2": self.W2, "b2": self.b2}

    def set_weights(self, weights: dict):
        for k, v in weights.items():
            setattr(self, k, v)


# ═══════════════════════════════════════
# REINFORCE + Baseline 训练器
# ═══════════════════════════════════════

@dataclass
class TrainConfig:
    episodes: int = 500
    gamma: float = 0.99           # 折扣因子
    lr_policy: float = 0.001      # 策略网络学习率
    lr_value: float = 0.005       # 价值网络学习率
    entropy_coef: float = 0.01    # 熵正则化系数
    save_interval: int = 50       # 保存间隔
    eval_episodes: int = 10       # 评估轮数
    reward_window: int = 50       # 滑动窗口大小


class REINFORCETrainer:
    """REINFORCE with baseline + entropy regularization"""

    def __init__(self, env: FlywheelEnv, config: TrainConfig = None):
        self.env = env
        self.cfg = config or TrainConfig()
        self.policy = PolicyNetwork()
        self.value = ValueNetwork()
        self.reward_history = deque(maxlen=self.cfg.reward_window)
        self.best_avg_reward = -float('inf')

    def train(self) -> List[float]:
        """主训练循环"""
        episode_rewards = []
        t_start = time.time()

        for ep in range(1, self.cfg.episodes + 1):
            # ── 收集一条轨迹 ──
            obs, _ = self.env.reset()
            trajectory = self._collect_trajectory(obs)

            # ── 计算 return ──
            returns = self._compute_returns(trajectory)

            # ── 梯度更新 ──
            policy_loss, value_loss = self._update(trajectory, returns)

            total_r = sum(s[2] for s in trajectory)
            episode_rewards.append(total_r)
            self.reward_history.append(total_r)
            avg_r = np.mean(self.reward_history)

            # ── 日志 ──
            if ep % 10 == 0 or ep == 1:
                elapsed = time.time() - t_start
                print(f"Ep {ep:4d}/{self.cfg.episodes} | "
                      f"R={total_r:7.1f} avg={avg_r:7.1f} "
                      f"best={self.best_avg_reward:7.1f} | "
                      f"π_loss={policy_loss:.4f} V_loss={value_loss:.4f} | "
                      f"{elapsed:.0f}s")

            # ── 保存最佳模型 ──
            if avg_r > self.best_avg_reward and ep >= self.cfg.reward_window:
                self.best_avg_reward = avg_r
                self.save("models/rl_policy_best.npz")

            # ── 定期保存 ──
            if ep % self.cfg.save_interval == 0:
                self.save(f"models/rl_policy_ep{ep}.npz")

        return episode_rewards

    def _collect_trajectory(self, obs: np.ndarray) -> List[Tuple]:
        """收集一条完整轨迹"""
        trajectory = []
        for _ in range(self.env.env_cfg.max_episode_steps):
            action, log_prob = self.policy.sample(obs)
            next_obs, reward, terminated, truncated, _ = self.env.step(action)
            trajectory.append((obs, action, reward, log_prob))
            obs = next_obs
            if terminated or truncated:
                break
        return trajectory

    def _compute_returns(self, trajectory: List[Tuple]) -> np.ndarray:
        """计算折扣回报 G_t = Σ γ^k * r_{t+k}"""
        rewards = np.array([s[2] for s in trajectory])
        returns = np.zeros_like(rewards)
        G = 0.0
        for t in range(len(rewards) - 1, -1, -1):
            G = rewards[t] + self.cfg.gamma * G
            returns[t] = G
        # 标准化 (减少方差)
        if len(returns) > 1:
            returns = (returns - returns.mean()) / (returns.std() + 1e-8)
        return returns

    def _update(self, trajectory: List[Tuple], returns: np.ndarray) -> Tuple[float, float]:
        """REINFORCE + baseline 更新"""
        lr_p = self.cfg.lr_policy
        lr_v = self.cfg.lr_value

        policy_loss_total = 0.0
        value_loss_total = 0.0

        for i, (obs, action, reward, log_prob) in enumerate(trajectory):
            G = returns[i]
            V = self.value.predict(obs)
            advantage = G - V

            # ── 策略梯度 ──
            probs = self.policy.forward(obs)
            dlogits = probs.copy()
            dlogits[action] -= 1.0  # ∂log(π)/∂z = pi - one_hot

            # 反向传播 (2层)
            z1 = obs @ self.policy.W1 + self.policy.b1
            a1 = np.maximum(0, z1)
            da1 = (a1 > 0).astype(float)  # ReLU 导数

            # Layer 2 梯度
            dW2 = np.outer(a1, dlogits)
            db2 = dlogits
            self.policy.W2 -= lr_p * advantage * dW2
            self.policy.b2 -= lr_p * advantage * db2

            # Layer 1 梯度
            dhidden = (dlogits @ self.policy.W2.T) * da1
            dW1 = np.outer(obs, dhidden)
            db1 = dhidden
            self.policy.W1 -= lr_p * advantage * dW1
            self.policy.b1 -= lr_p * advantage * db1

            policy_loss_total += -log_prob * advantage

            # ── 价值网络更新 (MSE) ──
            error = V - G
            z1_v = obs @ self.value.W1 + self.value.b1
            a1_v = np.maximum(0, z1_v)
            da1_v = (a1_v > 0).astype(float)

            dW2_v = np.outer(a1_v, [error])
            db2_v = np.array([error])
            self.value.W2 -= lr_v * dW2_v
            self.value.b2 -= lr_v * db2_v

            dhidden_v = (np.array([error]) @ self.value.W2.T) * da1_v
            dW1_v = np.outer(obs, dhidden_v)
            db1_v = dhidden_v
            self.value.W1 -= lr_v * dW1_v
            self.value.b1 -= lr_v * db1_v

            value_loss_total += error ** 2

        return (policy_loss_total / len(trajectory),
                value_loss_total / len(trajectory))

    def evaluate(self, n_episodes: int = 10) -> dict:
        """评估当前策略 (确定性, 取 argmax)"""
        rewards = []
        for _ in range(n_episodes):
            obs, _ = self.env.reset()
            ep_r = 0.0
            for _ in range(self.env.env_cfg.max_episode_steps):
                probs = self.policy.forward(obs)
                action = int(np.argmax(probs))  # 确定性策略
                obs, reward, terminated, truncated, _ = self.env.step(action)
                ep_r += reward
                if terminated or truncated:
                    break
            rewards.append(ep_r)
        return {
            "mean": float(np.mean(rewards)),
            "std": float(np.std(rewards)),
            "min": float(np.min(rewards)),
            "max": float(np.max(rewards)),
        }

    def save(self, path: str):
        """保存模型权重"""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez(path,
                 policy_W1=self.policy.W1, policy_b1=self.policy.b1,
                 policy_W2=self.policy.W2, policy_b2=self.policy.b2,
                 value_W1=self.value.W1, value_b1=self.value.b1,
                 value_W2=self.value.W2, value_b2=self.value.b2,
                 best_avg_reward=self.best_avg_reward)
        print(f"  💾 Saved to {path}")

    def load(self, path: str):
        """加载模型权重"""
        data = np.load(path)
        self.policy.W1 = data["policy_W1"]
        self.policy.b1 = data["policy_b1"]
        self.policy.W2 = data["policy_W2"]
        self.policy.b2 = data["policy_b2"]
        self.value.W1 = data["value_W1"]
        self.value.b1 = data["value_b1"]
        self.value.W2 = data["value_W2"]
        self.value.b2 = data["value_b2"]
        self.best_avg_reward = float(data.get("best_avg_reward", -float('inf')))
        print(f"  📂 Loaded from {path} (best_avg={self.best_avg_reward:.1f})")


# ═══════════════════════════════════════
# CLI
# ═══════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="飞轮 UPS RL 训练")
    parser.add_argument("--episodes", type=int, default=500, help="训练轮数")
    parser.add_argument("--eval", action="store_true", help="仅评估")
    parser.add_argument("--load", type=str, default="models/rl_policy_best.npz",
                        help="加载模型路径")
    args = parser.parse_args()

    env = FlywheelEnv()
    trainer = REINFORCETrainer(env, TrainConfig(episodes=args.episodes))

    if args.eval:
        if os.path.exists(args.load):
            trainer.load(args.load)
        result = trainer.evaluate(n_episodes=20)
        print(f"\n评估结果 (20 episodes):")
        print(f"  均值={result['mean']:.1f} ± {result['std']:.1f}")
        print(f"  范围=[{result['min']:.1f}, {result['max']:.1f}]")
        return

    print(f"=== 飞轮 UPS RL 训练 (REINFORCE + Baseline) ===")
    print(f"  环境: FlywheelEnv (8D obs, 6 actions)")
    print(f"  网络: Policy(8→64→6) / Value(8→32→1)")
    print(f"  轮数: {args.episodes} | γ={TrainConfig().gamma} "
          f"lr_π={TrainConfig().lr_policy} lr_V={TrainConfig().lr_value}")
    print()

    rewards = trainer.train()

    # 最终评估
    result = trainer.evaluate(n_episodes=20)
    print(f"\n=== 训练完成 ===")
    print(f"  最佳平均奖励: {trainer.best_avg_reward:.1f}")
    print(f"  最终评估: {result['mean']:.1f} ± {result['std']:.1f}")

    # 保存最终模型
    trainer.save("models/rl_policy_final.npz")


if __name__ == "__main__":
    main()
