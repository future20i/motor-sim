"""
test_grid_recorder.py — 电网录波回放测试。

子系统: 应用层测试
依赖: grid_recorder.py

电网录波回放测试。
"""
import sys
import os
import math
import pytest
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from grid_recorder import (
    GridRecorder, GridRecorderState, RecordedEvent
)


# ═══════════════════════════════════════════════════════════════
# 基本功能测试
# ═══════════════════════════════════════════════════════════════

class TestLoadSample:
    """测试 load_sample() 生成样例数据"""

    def test_load_sample_sag(self):
        """生成 sag 样例: 0.7pu 电压暂降"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        assert rec.sample_count > 0
        assert rec.duration == pytest.approx(2.0, rel=0.01)

        # 步进到暂降区间 (0.5s-0.7s)
        state_before = None
        state_during = None
        state_after = None

        for _ in range(6000):  # 步进 0.6s @ 50μs → ~12000步才到0.6s
            pass  # skip

        # 用较大的步长来快速跳到目标时间
        rec.reset()
        # 直接步进到 0.4s (正常)
        for _ in range(int(0.4 / 50e-6)):
            rec.step()
        state_before = rec.step()

        # 步进到 0.6s (暂降中)
        for _ in range(int(0.15 / 50e-6)):
            rec.step()
        state_during = rec.step()

        # 步进到 0.8s (恢复后)
        for _ in range(int(0.15 / 50e-6)):
            rec.step()
        state_after = rec.step()

        assert state_before.V_rms == pytest.approx(380.0, rel=0.1)
        assert state_during.V_rms == pytest.approx(380.0 * 0.7, rel=0.1)
        assert state_after.V_rms == pytest.approx(380.0, rel=0.1)

    def test_load_sample_swell(self):
        """生成 swell 样例: 1.2pu 电压暂升"""
        rec = GridRecorder()
        rec.load_sample(event_type='swell', duration=2.0, fs=10000.0)
        assert rec.sample_count == 20000

    def test_load_sample_freq_dip(self):
        """生成 freq_dip 样例: 频率降至49.5Hz"""
        rec = GridRecorder()
        rec.load_sample(event_type='freq_dip', duration=2.0, fs=10000.0)

        rec.reset()
        # 跳转到 0.7s (频率跌落中)
        for _ in range(int(0.7 / 50e-6)):
            rec.step()
        state = rec.step()

        assert state.f == pytest.approx(49.5, abs=0.1)

    def test_load_sample_interruption(self):
        """生成 interruption 样例"""
        rec = GridRecorder()
        rec.load_sample(event_type='interruption', duration=2.0, fs=10000.0)

        rec.reset()
        # 跳转到 0.6s (断电中)
        for _ in range(int(0.6 / 50e-6)):
            rec.step()
        state = rec.step()

        assert state.V_rms == pytest.approx(380.0 * 0.05, rel=0.1)
        assert state.event_label == 'interruption'

    def test_load_sample_all(self):
        """生成 all 混合序列"""
        rec = GridRecorder()
        rec.load_sample(event_type='all', duration=2.0, fs=10000.0)
        assert rec.sample_count > 0
        events = rec.get_events()
        assert len(events) >= 4  # sag, swell, freq_dip, interruption


class TestStep:
    """测试 step() 步进"""

    def test_step_returns_valid_state(self):
        """步进返回有效的 GridRecorderState"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        state = rec.step()
        assert isinstance(state, GridRecorderState)
        assert isinstance(state.Va, float)
        assert isinstance(state.Vb, float)
        assert isinstance(state.Vc, float)
        assert isinstance(state.f, float)
        assert isinstance(state.V_rms, float)
        assert isinstance(state.theta, float)
        assert isinstance(state.t, float)
        assert isinstance(state.event_label, str)

    def test_step_frequency_range(self):
        """步进中频率在合理范围"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        for _ in range(100):
            state = rec.step()
            assert 45.0 <= state.f <= 55.0

    def test_step_voltage_symmetry(self):
        """三相电压基本对称 (正常运行时)"""
        rec = GridRecorder()
        rec.load_sample(event_type='all', duration=2.0, fs=10000.0)

        # 正常区间 (0-0.3s)
        rec.reset()
        for _ in range(int(0.1 / 50e-6)):
            rec.step()
        state = rec.step()

        # 三相幅值应接近
        V_avg = (abs(state.Va) + abs(state.Vb) + abs(state.Vc)) / 3
        assert abs(state.Va) / V_avg == pytest.approx(1.0, abs=0.3) if V_avg > 0 else True

    def test_step_event_label_empty_outside_event(self):
        """非事件区间 event_label 为空"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        rec.reset()
        for _ in range(int(0.1 / 50e-6)):
            rec.step()
        state = rec.step()
        assert state.event_label == ''

    def test_step_time_monotonic(self):
        """时间单调递增"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        prev_t = -1.0
        for _ in range(100):
            state = rec.step()
            assert state.t >= prev_t
            prev_t = state.t


class TestIntradayPattern:
    """测试 24h 日内模式"""

    def test_intraday_pattern_generation(self):
        """生成 24h 日内模式"""
        rec = GridRecorder()
        rec.load_sample(event_type='intraday')
        assert rec.sample_count > 0
        # 24小时, 100Hz → 8,640,000 个采样点
        assert rec.duration == pytest.approx(86400.0, rel=0.01)
        events = rec.get_events()
        # 应有 peak_load 和 light_load 标签
        event_types = {e.event_type for e in events}
        assert 'peak_load' in event_types or 'light_load' in event_types

    def test_intraday_voltage_range(self):
        """日内电压在合理范围"""
        rec = GridRecorder()
        rec.load_sample(event_type='intraday')

        # 抽样检查几个时间点
        for target_t in [0.0, 21600.0, 43200.0, 64800.0, 86400.0]:
            if target_t >= rec.duration:
                continue
            rec.reset()
            rec._t = target_t
            state = rec.step(dt=1.0)  # 大步长
            assert 350.0 <= state.V_rms <= 400.0


class TestCompatibleInterface:
    """测试与 GridSimulator 接口兼容性"""

    def test_get_readable(self):
        """get_readable() 返回与 GridSimulator 兼容的 dict"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)
        rec.reset()
        rec.step()

        info = rec.get_readable()
        assert 'f_Hz' in info
        assert 'V_rms' in info
        assert 'V_abc' in info
        assert 'theta_deg' in info
        assert 'df_Hz' in info
        assert 'fault' in info
        assert 't_s' in info

        # V_abc 是三元组
        assert len(info['V_abc']) == 3

    def test_get_readable_df_Hz(self):
        """df_Hz 计算正确"""
        rec = GridRecorder(f_nom=50.0)
        rec.load_sample(event_type='freq_dip', duration=2.0, fs=10000.0)

        rec.reset()
        for _ in range(int(0.7 / 50e-6)):
            rec.step()

        info = rec.get_readable()
        assert info['df_Hz'] == pytest.approx(-0.5, abs=0.1)

    def test_reset(self):
        """reset() 重置播放位置"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)

        for _ in range(1000):
            rec.step()
        t_before = rec._t

        rec.reset()
        assert rec._t < t_before

    def test_empty_recorder(self):
        """空 Recorder (无数据加载) 不崩溃"""
        rec = GridRecorder()
        state = rec.step()
        assert state.f == 50.0
        assert state.V_rms == 380.0

    def test_sample_rate_property(self):
        """sample_rate 和 duration 属性正确"""
        rec = GridRecorder()
        rec.load_sample(event_type='sag', duration=2.0, fs=10000.0)
        assert rec.sample_rate == pytest.approx(10000.0, rel=0.01)
        assert rec.duration == pytest.approx(2.0, rel=0.01)


class TestCSVLoad:
    """测试从 CSV 文件加载"""

    def test_load_csv_basic(self):
        """从 CSV 文件加载基本录波数据"""
        import numpy as np

        # 生成一个临时 CSV
        t = np.linspace(0, 0.1, 1000)
        Va = 310.0 * np.cos(2 * math.pi * 50 * t)
        Vb = 310.0 * np.cos(2 * math.pi * 50 * t - 2 * math.pi / 3)
        Vc = 310.0 * np.cos(2 * math.pi * 50 * t + 2 * math.pi / 3)
        f = np.full_like(t, 50.0)

        csv_content = "t,Va,Vb,Vc,f\n"
        for i in range(len(t)):
            csv_content += f"{t[i]:.6f},{Va[i]:.6f},{Vb[i]:.6f},{Vc[i]:.6f},{f[i]:.3f}\n"

        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as fh:
            fh.write(csv_content)
            tmp_path = fh.name

        try:
            rec = GridRecorder(filepath=tmp_path)
            assert rec.sample_count == 1000

            state = rec.step()
            assert isinstance(state.Va, float)
            assert state.f == pytest.approx(50.0)
        finally:
            os.unlink(tmp_path)

    def test_load_csv_with_labels(self):
        """从带事件标签的 CSV 加载"""
        import numpy as np

        t = np.linspace(0, 0.1, 500)
        Va = 310.0 * np.cos(2 * math.pi * 50 * t)
        Vb = 310.0 * np.cos(2 * math.pi * 50 * t - 2 * math.pi / 3)
        Vc = 310.0 * np.cos(2 * math.pi * 50 * t + 2 * math.pi / 3)
        f = np.full_like(t, 50.0)
        labels = np.array([''] * len(t), dtype=object)
        labels[100:200] = 'sag'
        labels[300:400] = 'swell'

        csv_content = "t,Va,Vb,Vc,f,V_rms,event_label\n"
        for i in range(len(t)):
            csv_content += f"{t[i]:.6f},{Va[i]:.6f},{Vb[i]:.6f},{Vc[i]:.6f},{f[i]:.3f},380.0,{labels[i]}\n"

        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as fh:
            fh.write(csv_content)
            tmp_path = fh.name

        try:
            rec = GridRecorder(filepath=tmp_path)
            events = rec.get_events()
            event_types = {e.event_type for e in events}
            assert 'sag' in event_types
            assert 'swell' in event_types
        finally:
            os.unlink(tmp_path)


class TestTimeResampling:
    """测试时间重采样"""

    def test_resampling_low_to_high(self):
        """低采样率录波 → 高采样率仿真步长 (插值)"""
        rec = GridRecorder()
        # 10kHz 录波, 50μs 仿真步长 (20kHz 等效)
        rec.load_sample(event_type='sag', duration=0.1, fs=10000.0)

        # 步进1000次应对应 ~0.05s
        for _ in range(1000):
            state = rec.step(dt=50e-6)

        # 应该不崩溃且值合理
        assert state.t > 0.04
        assert 45.0 <= state.f <= 55.0

    def test_resampling_no_crash(self):
        """多次步进不崩溃"""
        rec = GridRecorder()
        rec.load_sample(event_type='all', duration=0.5, fs=5000.0)

        for _ in range(5000):
            state = rec.step(dt=50e-6)
        # 不应崩溃
        assert state.t > 0
