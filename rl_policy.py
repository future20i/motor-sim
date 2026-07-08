"""
rl_policy.py — RL 策略导出 (REINFORCE+Baseline) + DecisionLogger。纯 numpy, 无 torch/SB3 依赖。

子系统: 应用层
依赖: rl_environment.py

RL 策略导出 (REINFORCE+Baseline) + DecisionLogger。纯 numpy, 无 torch/SB3 依赖。
"""
import math
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np

from power_ops import PowerMode


# ── 动作映射 (与 rl_environment 一致) ──
ACTION_TO_MODE = [
    PowerMode.IDLE,
    PowerMode.DC_BUS_CONTROL,
    PowerMode.FREQ_REGULATION,
    PowerMode.VSG,
    PowerMode.POWER_TRACKING,
    PowerMode.INERTIA_SUPPORT,
]

MODE_NAMES = [m.name for m in ACTION_TO_MODE]


@dataclass
class AIControlAction:
    """AI 控制决策 — 与 s4_ai_controller 兼容"""
    mode: PowerMode
    P_ref: float = 0.0       # 调度功率 W
    droop: float = 0.04      # 下垂系数
    confidence: float = 0.0  # 决策置信度


@dataclass
class GridSnapshot:
    """电网快照 — 与 s4_ai_controller 兼容"""
    t: float = 0.0
    f_grid: float = 50.0
    Vdc: float = 750.0
    SOC: float = 0.5
    df: float = 0.0
    dVdc: float = 0.0
    P_grid: float = 0.0
    P_load: float = 0.0
    omega: float = 0.0
    temp: float = 25.0


class RLPolicy:
    """RL 训练策略 — 加载权重后替代启发式规则树

    纯 numpy 推理，无外部依赖。
    """

    def __init__(self, model_path: str = None):
        self.W1: Optional[np.ndarray] = None
        self.b1: Optional[np.ndarray] = None
        self.W2: Optional[np.ndarray] = None
        self.b2: Optional[np.ndarray] = None
        self._loaded = False

        if model_path and os.path.exists(model_path):
            self.load(model_path)

    def load(self, path: str):
        """加载 .npz 模型权重"""
        data = np.load(path)
        self.W1 = data["policy_W1"]
        self.b1 = data["policy_b1"]
        self.W2 = data["policy_W2"]
        self.b2 = data["policy_b2"]
        self._loaded = True
        return self

    def predict(self, obs: np.ndarray) -> int:
        """推理: 8D obs → action (0-5)

        obs 应为归一化向量 (与 FlywheelEnv._get_obs 格式一致)
        """
        if not self._loaded:
            return 0  # 未加载 → 默认 IDLE

        z1 = obs @ self.W1 + self.b1
        a1 = np.maximum(0, z1)
        z2 = a1 @ self.W2 + self.b2
        return int(np.argmax(z2))

    def predict_proba(self, obs: np.ndarray) -> np.ndarray:
        """推理 + 概率分布"""
        if not self._loaded:
            return np.ones(6) / 6

        z1 = obs @ self.W1 + self.b1
        a1 = np.maximum(0, z1)
        z2 = a1 @ self.W2 + self.b2
        z2 -= np.max(z2)
        exp_z = np.exp(z2)
        return exp_z / np.sum(exp_z)

    def decide(self, snap: GridSnapshot) -> AIControlAction:
        """高层决策接口 — 与 s4_ai_controller 兼容

        GridSnapshot → AIControlAction
        """
        # 构建归一化观测 (与 FlywheelEnv._get_obs 一致)
        obs = self._snapshot_to_obs(snap)
        probs = self.predict_proba(obs)
        action_idx = int(np.argmax(probs))
        confidence = float(probs[action_idx])

        mode = ACTION_TO_MODE[action_idx]

        # 根据模式设置默认参数
        droop = 0.04
        P_ref = 0.0

        if mode == PowerMode.DC_BUS_CONTROL:
            # 根据偏差缓急调整 P_ref
            P_ref = min(500e3, abs(snap.dVdc) * 10e3)
        elif mode == PowerMode.FREQ_REGULATION:
            droop = max(0.02, min(0.06, 0.04 + abs(snap.df) * 0.05))
            P_ref = min(500e3, abs(snap.df) * 500e3)

        return AIControlAction(
            mode=mode,
            P_ref=P_ref,
            droop=droop,
            confidence=confidence,
        )

    def _snapshot_to_obs(self, snap: GridSnapshot) -> np.ndarray:
        """GridSnapshot → 8D 归一化观测"""
        f_norm = max(-1.0, min(1.0, (snap.f_grid - 50.0) / 1.0))
        df_norm = max(-1.0, min(1.0, snap.df / 0.5))
        vdc_norm = max(-1.0, min(1.0, (snap.Vdc - 750.0) / 100.0))
        dvdc_norm = max(-1.0, min(1.0, snap.dVdc / 50.0))
        soc_norm = max(-1.0, min(1.0, (snap.SOC - 0.5) * 2.0))
        pload_norm = max(-1.0, min(1.0, snap.P_load / 500e3))
        omega_norm = max(-1.0, min(1.0,
            snap.omega / 921.5 * 2.0 - 1.0))  # omega_max_rad ≈ 921.5
        temp_norm = max(-1.0, min(1.0, (snap.temp - 40.0) / 80.0))

        return np.array([
            f_norm, df_norm, vdc_norm, dvdc_norm,
            soc_norm, pload_norm, omega_norm, temp_norm,
        ], dtype=np.float32)

    def explain(self, obs: np.ndarray) -> dict:
        """SHAP 风格的特征重要性 (简化: 基于权重幅值)"""
        if not self._loaded:
            return {}

        # 第一层权重幅值 = 特征重要性近似
        importance = np.abs(self.W1).sum(axis=1)
        importance /= importance.sum()

        labels = ["f", "df", "Vdc", "dVdc", "SOC", "P_load", "ω", "T"]
        ranked = sorted(zip(labels, importance), key=lambda x: -x[1])

        return {
            "top_features": [f"{name}:{imp:.1%}" for name, imp in ranked[:3]],
            "all": {name: float(imp) for name, imp in ranked},
        }

    @property
    def is_loaded(self) -> bool:
        return self._loaded


# ═══════════════════════════════════════
# 决策日志记录
# ═══════════════════════════════════════

class DecisionLogger:
    """RL 决策日志 — 可解释性 + 审计追溯"""

    def __init__(self, log_path: str = "logs/rl_decisions.jsonl"):
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self.path = log_path
        self.entries: list = []

    def log(self, t: float, obs: np.ndarray, action_idx: int,
            probs: np.ndarray, snap: GridSnapshot):
        """记录一次决策"""
        import json
        entry = {
            "t": round(t, 4),
            "action": MODE_NAMES[action_idx],
            "confidence": round(float(probs[action_idx]), 4),
            "probs": [round(float(p), 4) for p in probs],
            "obs": {
                "f": round(snap.f_grid, 3),
                "df": round(snap.df, 3),
                "Vdc": round(snap.Vdc, 1),
                "dVdc": round(snap.dVdc, 1),
                "SOC": round(snap.SOC, 3),
                "P_load_kW": round(snap.P_load / 1000, 1),
            }
        }
        self.entries.append(entry)

        # 实时追加到文件
        with open(self.path, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def flush(self):
        """确保所有条目已写入"""
        pass  # 每次 log 已实时写入

    @property
    def count(self) -> int:
        return len(self.entries)


# ═══════════════════════════════════════
# CLI 自测
# ═══════════════════════════════════════

if __name__ == "__main__":
    import sys

    # 测试未加载状态
    policy = RLPolicy()
    obs = np.zeros(8, dtype=np.float32)
    action = policy.predict(obs)
    assert action == 0, "未加载应返回 IDLE"
    print("✅ 未加载默认行为: IDLE")

    # 测试快照接口
    snap = GridSnapshot(f_grid=49.7, df=-0.3, Vdc=720, dVdc=-30, SOC=0.4, P_load=300e3)
    decision = policy.decide(snap)
    print(f"✅ decide: mode={decision.mode.name} conf={decision.confidence:.2f}")

    # 测试加载 (如果模型存在)
    model_path = "models/rl_policy_best.npz"
    if os.path.exists(model_path):
        policy.load(model_path)
        action = policy.predict(obs)
        probs = policy.predict_proba(obs)
        print(f"✅ 加载模型: action={MODE_NAMES[action]} probs={[f'{p:.2f}' for p in probs]}")

        importance = policy.explain(obs)
        print(f"✅ 特征重要性: {importance['top_features']}")
    else:
        print(f"⚠️  模型未找到: {model_path} — 运行 train_rl.py 生成")

    print("\n✅ rl_policy.py 自测完成")
