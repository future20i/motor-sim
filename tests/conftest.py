"""
conftest.py — pytest fixtures: SystemConfig 和 FlywheelEnergyStorage 实例。

子系统: 测试基础设施
依赖: 无

pytest fixtures: SystemConfig 和 FlywheelEnergyStorage 实例。
"""
import sys
import os

# 确保项目根目录在 sys.path 中
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
