"""
grid_recorder.py — 真实录波回放: CSV/COMTRADE 格式电网录波数据导入。

子系统: 应用层 (v3.0)
依赖: grid_sim.py

真实录波回放: CSV/COMTRADE 格式电网录波数据导入。
"""
import math
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import io


# ═══════════════════════════════════════════════════════════════
# 数据类
# ═══════════════════════════════════════════════════════════════

@dataclass
class RecordedEvent:
    """录波事件标签"""
    t_start: float
    t_end: float
    event_type: str  # 'sag', 'swell', 'interruption', 'transient', 'freq_dip', 'freq_swell'
    description: str = ''


@dataclass
class GridRecorderState:
    """录波回放的当前状态 (与GridState兼容)"""
    Va: float = 0.0       # 瞬时相电压 V (峰值)
    Vb: float = 0.0
    Vc: float = 0.0
    f: float = 50.0       # 当前频率 Hz
    V_rms: float = 380.0  # 当前线电压 Vrms
    theta: float = 0.0    # 当前相位角 rad
    t: float = 0.0
    event_label: str = '' # 当前事件标签 (兼容GridSimulator的fault.name)


# ═══════════════════════════════════════════════════════════════
# GridRecorder
# ═══════════════════════════════════════════════════════════════

class GridRecorder:
    """从录波文件播放真实三相电压

    接口兼容 GridSimulator:
    - step(dt) → GridRecorderState (含 Va, Vb, Vc, f, V_rms, theta, t)
    - get_readable() → dict
    """

    def __init__(self, filepath: str = None, f_nom: float = 50.0, V_nom: float = 380.0):
        """
        Args:
            filepath: CSV/MAT 录波文件路径 (可选)
            f_nom: 标称频率 Hz (用于load_sample默认值)
            V_nom: 标称线电压 Vrms (用于load_sample默认值)
        """
        self.f_nom = f_nom
        self.V_nom = V_nom

        # 录波数据: NumPy arrays (lazy import to keep dependencies soft)
        self._t_arr = None       # 时间轴 [s]
        self._Va_arr = None      # 三相瞬时电压
        self._Vb_arr = None
        self._Vc_arr = None
        self._f_arr = None       # 频率 [Hz]
        self._Vrms_arr = None    # 线电压有效值 [V]
        self._theta_arr = None   # 相位角 [rad]
        self._label_arr = None   # 事件标签字符串 (可选)

        self._idx = 0            # 当前播放索引
        self._t = 0.0            # 当前仿真时间
        self._events: List[RecordedEvent] = []  # 事件列表
        self._sample_count = 0

        if filepath:
            self.load(filepath)

    # ── 加载 ────────────────────────────────────────────────

    def load(self, filepath: str):
        """加载录波文件 (CSV 或 MAT)

        CSV 格式 (至少需要 t, Va, Vb, Vc):
            t,Va,Vb,Vc,f         ← 最小列集
            t,Va,Vb,Vc,f,V_rms,event_label  ← 完整列集

        MAT 格式:
            struct with fields: time, Va, Vb, Vc, f (可选: V_rms, label)
        """
        ext = filepath.rsplit('.', 1)[-1].lower() if '.' in filepath else ''

        if ext == 'csv':
            self._load_csv(filepath)
        elif ext == 'mat':
            self._load_mat(filepath)
        else:
            raise ValueError(f"不支持的格式: {ext}, 请使用 .csv 或 .mat")

        self._sample_count = len(self._t_arr)
        self._idx = 0
        self._t = self._t_arr[0] if self._sample_count > 0 else 0.0

    def _load_csv(self, filepath: str):
        """从CSV加载"""
        import pandas as pd
        df = pd.read_csv(filepath)
        self._t_arr = df['t'].values.astype(float)

        # 三相电压
        self._Va_arr = df['Va'].values.astype(float)
        self._Vb_arr = df['Vb'].values.astype(float)
        self._Vc_arr = df['Vc'].values.astype(float)

        # 频率 (可选，如果没有则用标称值)
        if 'f' in df.columns:
            self._f_arr = df['f'].values.astype(float)
        else:
            self._f_arr = np.full_like(self._t_arr, self.f_nom)

        # 线电压有效值 (可选)
        if 'V_rms' in df.columns:
            self._Vrms_arr = df['V_rms'].values.astype(float)

        # 事件标签 (可选)
        if 'event_label' in df.columns:
            self._label_arr = df['event_label'].values
            self._build_events_from_labels()

    def _load_mat(self, filepath: str):
        """从MAT文件加载"""
        import numpy as np
        import scipy.io as sio
        data = sio.loadmat(filepath)

        # 支持 struct 或直接变量
        if 'rec' in data:
            rec = data['rec']
            # MATLAB struct 转 Python
            self._t_arr = rec['time'][0, 0].flatten()
            self._Va_arr = rec['Va'][0, 0].flatten()
            self._Vb_arr = rec['Vb'][0, 0].flatten()
            self._Vc_arr = rec['Vc'][0, 0].flatten()
            if 'f' in rec.dtype.names:
                self._f_arr = rec['f'][0, 0].flatten()
            else:
                self._f_arr = np.full_like(self._t_arr, self.f_nom)
            if 'V_rms' in rec.dtype.names:
                self._Vrms_arr = rec['V_rms'][0, 0].flatten()
            if 'label' in rec.dtype.names:
                self._label_arr = rec['label'][0, 0].flatten()
                self._build_events_from_labels()
        else:
            # 直接变量
            self._t_arr = data['time'].flatten()
            self._Va_arr = data['Va'].flatten()
            self._Vb_arr = data['Vb'].flatten()
            self._Vc_arr = data['Vc'].flatten()
            self._f_arr = data.get('f', np.full_like(self._t_arr, self.f_nom)).flatten() if 'f' in data else np.full_like(self._t_arr, self.f_nom)
            if isinstance(self._f_arr, np.ndarray) and self._f_arr.size == 1:
                self._f_arr = np.full_like(self._t_arr, float(self._f_arr))

    def _build_events_from_labels(self):
        """从label数组构建RecordedEvent列表"""
        if self._label_arr is None:
            return
        self._events = []
        prev_label = ''
        t_start = None
        for i, label in enumerate(self._label_arr):
            label_str = str(label).strip()
            if label_str and label_str != prev_label:
                if prev_label and t_start is not None:
                    self._events.append(RecordedEvent(
                        t_start=t_start,
                        t_end=self._t_arr[i - 1] if i > 0 else t_start,
                        event_type=prev_label,
                        description=f"录波事件: {prev_label}"
                    ))
                t_start = self._t_arr[i]
                prev_label = label_str
            elif not label_str and prev_label and t_start is not None:
                self._events.append(RecordedEvent(
                    t_start=t_start,
                    t_end=self._t_arr[i - 1] if i > 0 else t_start,
                    event_type=prev_label,
                    description=f"录波事件: {prev_label}"
                ))
                prev_label = ''
                t_start = None
        # 尾部事件
        if prev_label and t_start is not None:
            self._events.append(RecordedEvent(
                t_start=t_start,
                t_end=self._t_arr[-1],
                event_type=prev_label,
                description=f"录波事件: {prev_label}"
            ))

    # ── 样例数据生成 ───────────────────────────────────────

    def load_sample(self, event_type: str = 'all', duration: float = 2.0,
                    fs: float = 20000.0):
        """生成样例数据（用于开发测试）

        生成典型电网事件波形:
        - 'sag': 0.7pu 电压暂降, 持续200ms
        - 'swell': 1.2pu 电压暂升, 持续200ms
        - 'freq_dip': 频率降至49.5Hz, 持续500ms
        - 'freq_swell': 频率升至50.5Hz
        - 'interruption': 电压跌至0.05pu (断电)
        - 'transient': 振荡瞬态
        - 'all': 混合序列 (sag → swell → freq_dip → interruption)
        - 'intraday': 24小时日内模式 (负载曲线模拟)

        Args:
            event_type: 事件类型
            duration: 总时长 [s] (intraday模式忽略)
            fs: 采样率 [Hz]
        """
        import numpy as np

        n_samples = int(duration * fs)
        t = np.linspace(0, duration, n_samples, endpoint=False)
        dt = 1.0 / fs

        # 基础正弦波
        omega = 2 * math.pi * self.f_nom
        Va_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t)
        Vb_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t - 2 * math.pi / 3)
        Vc_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t + 2 * math.pi / 3)

        # 默认无事件
        scale = np.ones(n_samples)
        f_dev = np.zeros(n_samples)
        labels = np.array([''] * n_samples, dtype=object)

        if event_type == 'sag':
            # 0.7pu 暂降, 0.5s–0.7s
            mask = (t >= 0.5) & (t < 0.7)
            scale[mask] = 0.7
            labels[mask] = 'sag'

        elif event_type == 'swell':
            # 1.2pu 暂升
            mask = (t >= 0.5) & (t < 0.7)
            scale[mask] = 1.2
            labels[mask] = 'swell'

        elif event_type == 'freq_dip':
            # 频率降至 49.5Hz
            mask = (t >= 0.5) & (t < 1.0)
            f_dev[mask] = -0.5
            labels[mask] = 'freq_dip'

        elif event_type == 'freq_swell':
            # 频率升至 50.5Hz
            mask = (t >= 0.5) & (t < 1.0)
            f_dev[mask] = 0.5
            labels[mask] = 'freq_swell'

        elif event_type == 'interruption':
            # 断电 (电压 0.05pu)
            mask = (t >= 0.5) & (t < 0.8)
            scale[mask] = 0.05
            labels[mask] = 'interruption'

        elif event_type == 'transient':
            # 振荡瞬态: 0.5s时注入高频衰减振荡
            mask = (t >= 0.5) & (t < 0.55)
            transient = 0.3 * np.exp(-(t[mask] - 0.5) * 80) * np.sin(2 * math.pi * 500 * t[mask])
            n_transient = len(transient)
            scale_vals = np.ones(n_transient) + transient
            # 将 transient 叠加到 scale 上
            start_idx = np.searchsorted(t, 0.5)
            for j in range(n_transient):
                if start_idx + j < n_samples:
                    scale[start_idx + j] = scale_vals[j]
            labels[mask] = 'transient'

        elif event_type == 'all':
            # 混合序列: sag → swell → freq_dip → interruption
            # 0.3-0.5s sag
            m1 = (t >= 0.3) & (t < 0.5)
            scale[m1] = 0.7
            labels[m1] = 'sag'
            # 0.7-0.9s swell
            m2 = (t >= 0.7) & (t < 0.9)
            scale[m2] = 1.2
            labels[m2] = 'swell'
            # 1.1-1.5s freq_dip
            m3 = (t >= 1.1) & (t < 1.5)
            f_dev[m3] = -0.5
            labels[m3] = 'freq_dip'
            # 1.7-1.9s interruption
            m4 = (t >= 1.7) & (t < 1.9)
            scale[m4] = 0.05
            labels[m4] = 'interruption'

        elif event_type == 'intraday':
            # 24小时日内模式: 模拟负载波动引起的电压/频率变化
            duration = 24 * 3600  # 86400s
            # 降低采样率以控制内存
            fs = 100.0  # 100Hz 采样, 仅用于特征表示
            n_samples = int(duration * fs)
            t = np.linspace(0, duration, n_samples, endpoint=False)

            # 日负荷曲线: 夜间低, 上午升, 午间略降, 下午高, 傍晚峰值, 夜间回落
            # 用多段正弦近似
            hours = t / 3600.0
            base_load = 0.7 + 0.3 * np.sin(math.pi * (hours - 8) / 12)  # 0.7~1.0
            # 上午爬坡
            morning_ramp = np.clip((hours - 6) / 4, 0, 1)
            # 傍晚峰值
            evening_peak = np.exp(-((hours - 18) ** 2) / 8)
            load_factor = base_load * morning_ramp + 0.2 * evening_peak
            load_factor = np.clip(load_factor, 0.5, 1.1)

            # 电压随负载变化 (负载高 → 电压略降)
            scale = 1.0 - 0.05 * (load_factor - 0.7)
            scale = np.clip(scale, 0.95, 1.02)

            # 频率随负载微调
            f_dev = -0.1 * (load_factor - 0.7)

            # 标记时段
            labels = np.array([''] * n_samples, dtype=object)
            labels[(hours >= 18) & (hours < 20)] = 'peak_load'
            labels[(hours >= 0) & (hours < 4)] = 'light_load'

            # 重新生成基波 (由于采样率变了)
            omega = 2 * math.pi * self.f_nom
            Va_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t)
            Vb_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t - 2 * math.pi / 3)
            Vc_base = self.V_nom * math.sqrt(2.0 / 3.0) * np.cos(omega * t + 2 * math.pi / 3)

        else:
            raise ValueError(f"未知事件类型: {event_type}, "
                             f"支持: sag, swell, freq_dip, freq_swell, interruption, transient, all, intraday")

        # 频率积分生成相位: θ(t) = ∫ 2π(f_nom + Δf) dt
        f_instant = self.f_nom + f_dev
        theta = 2 * math.pi * np.cumsum(f_instant) * (1.0 / fs)
        theta = theta % (2 * math.pi)

        # 应用电压缩放和频率偏移重算三相
        # 注意: 这里我们用 phase 重算而非简单缩放 Va_base, 因为频率也会变
        Vpk = self.V_nom * math.sqrt(2.0 / 3.0)
        Va = Vpk * scale * np.cos(theta)
        Vb = Vpk * scale * np.cos(theta - 2 * math.pi / 3)
        Vc = Vpk * scale * np.cos(theta + 2 * math.pi / 3)

        # 存储
        self._t_arr = t
        self._Va_arr = Va
        self._Vb_arr = Vb
        self._Vc_arr = Vc
        self._f_arr = f_instant
        self._Vrms_arr = self.V_nom * scale
        self._label_arr = labels

        self._sample_count = n_samples
        self._idx = 0
        self._t = t[0]

        # 构建事件列表
        self._build_events_from_labels()

    # ── 步进 ────────────────────────────────────────────────

    def step(self, dt: float = 50e-6) -> GridRecorderState:
        """一个仿真步进，返回三相瞬时电压+频率

        接口兼容 GridSimulator.step() → GridState。
        当录波采样率与仿真步长不一致时，自动线性插值。

        Args:
            dt: 仿真步长 [s] (默认 50μs)

        Returns:
            GridRecorderState with Va, Vb, Vc, f, V_rms, theta, t, event_label
        """
        import numpy as np

        if self._sample_count == 0:
            # 无数据, 返回标称值
            self._t += dt
            omega = 2 * math.pi * self.f_nom
            Vpk = self.V_nom * math.sqrt(2.0 / 3.0)
            theta = (omega * self._t) % (2 * math.pi)
            return GridRecorderState(
                Va=Vpk * math.cos(theta),
                Vb=Vpk * math.cos(theta - 2 * math.pi / 3),
                Vc=Vpk * math.cos(theta + 2 * math.pi / 3),
                f=self.f_nom,
                V_rms=self.V_nom,
                theta=theta,
                t=self._t,
                event_label=''
            )

        self._t += dt

        # 如果超出录波范围, 循环或保持最后值
        if self._t > self._t_arr[-1]:
            # 循环回放 (wrap-around)
            self._idx = 0
            self._t = self._t_arr[0] + (self._t - self._t_arr[-1])

        # 线性插值找当前帧
        idx = np.searchsorted(self._t_arr, self._t, side='right') - 1
        idx = max(0, min(idx, self._sample_count - 2))

        t_lo = self._t_arr[idx]
        t_hi = self._t_arr[idx + 1]
        frac = (self._t - t_lo) / (t_hi - t_lo) if t_hi > t_lo else 0.0
        frac = max(0.0, min(1.0, frac))

        # 插值三相电压
        Va = self._Va_arr[idx] + frac * (self._Va_arr[idx + 1] - self._Va_arr[idx])
        Vb = self._Vb_arr[idx] + frac * (self._Vb_arr[idx + 1] - self._Vb_arr[idx])
        Vc = self._Vc_arr[idx] + frac * (self._Vc_arr[idx + 1] - self._Vc_arr[idx])

        # 插值频率
        f = self._f_arr[idx] + frac * (self._f_arr[idx + 1] - self._f_arr[idx])

        # 插值 V_rms
        if self._Vrms_arr is not None:
            V_rms = self._Vrms_arr[idx] + frac * (self._Vrms_arr[idx + 1] - self._Vrms_arr[idx])
        else:
            # 从瞬时电压估算
            V_rms = math.sqrt((Va**2 + Vb**2 + Vc**2) / 3) * math.sqrt(3.0 / 2.0)

        # 相位角 (从 Va 推算: Va = Vpk * cos(theta))
        Vpk_phase = V_rms * math.sqrt(2.0 / 3.0)
        if Vpk_phase > 1e-6:
            theta = math.acos(max(-1.0, min(1.0, Va / Vpk_phase)))
            # 判断象限: 如果 Vb > Vc 则在第二象限
            if Vb > Vc:
                theta = 2 * math.pi - theta
        else:
            theta = 0.0

        # 事件标签
        event_label = ''
        if self._label_arr is not None:
            label_at_idx = str(self._label_arr[idx]).strip()
            if label_at_idx:
                event_label = label_at_idx

        self._idx = idx

        return GridRecorderState(
            Va=float(Va),
            Vb=float(Vb),
            Vc=float(Vc),
            f=float(f),
            V_rms=float(V_rms),
            theta=float(theta),
            t=self._t,
            event_label=event_label
        )

    # ── 事件查询 ───────────────────────────────────────────

    def get_events(self) -> List[RecordedEvent]:
        """返回录波中的事件列表"""
        return list(self._events)

    # ── 兼容接口 ───────────────────────────────────────────

    def get_readable(self) -> dict:
        """返回可读状态 (兼容 GridSimulator.get_readable())"""
        state = self.step(0)  # 用 dt=0 获取当前帧, 不推进时间
        # 恢复时间
        self._t -= 1e-15  # 极小修正

        return {
            "f_Hz": state.f,
            "V_rms": state.V_rms,
            "V_abc": (state.Va, state.Vb, state.Vc),
            "theta_deg": math.degrees(state.theta) % 360,
            "df_Hz": state.f - self.f_nom,
            "fault": state.event_label or "NONE",
            "t_s": state.t,
        }

    # ── 属性 ───────────────────────────────────────────────

    @property
    def sample_count(self) -> int:
        return self._sample_count

    @property
    def duration(self) -> float:
        if self._sample_count > 0:
            return float(self._t_arr[-1] - self._t_arr[0])
        return 0.0

    @property
    def sample_rate(self) -> float:
        if self._sample_count > 1:
            return float(self._sample_count / (self._t_arr[-1] - self._t_arr[0]))
        return 0.0

    # ── 重置 ───────────────────────────────────────────────

    def reset(self):
        """重置播放位置"""
        self._idx = 0
        self._t = self._t_arr[0] if self._sample_count > 0 else 0.0
