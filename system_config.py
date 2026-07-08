"""
system_config.py — 所有子模块从这里取参数，不再各自定义默认值。

子系统: 基础层
依赖: 无 (单一事实来源)
手册对应章节: ARCHITECTURE.md §1.2 (系统配置表), CONTROL_SETPOINTS.md §8 (配置文件映射)

所有子模块从这里取参数，不再各自定义默认值。
"""
from dataclasses import dataclass, field


@dataclass
class PhysicalConstants:
    """系统级物理常量 — 所有模块共享"""
    Ts: float = 50e-6           # 全局仿真步长 s (20kHz 控制频率)
    Vdc_nom: float = 750.0      # 直流母线标称电压 V
    Vac_nom: float = 480.0      # 交流侧线电压 Vrms
    f_nom: float = 50.0         # 电网额定频率 Hz


@dataclass
class FlywheelMotorSpec:
    """飞轮电机电气参数 — 单极感应子(HIM), 6对极, 电励磁
    
    泓慧 500kW 飞轮电机真实参数:
      Rs≈5mΩ, Ld≈Lq≈2mH, Mdf≈10mH (互感)
      励磁: Rf=75Ω (DC母线直供≈750V/10A), Lf=5H, If_nom=10A
      
    电励磁等效 ψm ≈ Mdf·If_nom = 0.01×10 = 0.1 Wb (含 6 对极)
    """
    Rs: float = 0.005           # 定子电阻 Ω
    Ld: float = 0.002           # d 轴电感 H
    Lq: float = 0.002           # q 轴电感 H
    psi_m: float = 0.08         # 等效永磁磁链 Wb (非 HIM 电机使用, HIM 用 Mdf·If)
    P: int = 6                  # 极对数
    I_max: float = 1000.0       # 峰值电流 A (500kW@750Vdc)
    J_motor: float = 0.04       # 电机转子惯量 kg·m² (不含飞轮本体)
    B: float = 0.0005           # 阻尼系数 N·m·s
    
    # ── HIM 励磁专用参数 (感应子电机核心) ──
    Mdf: float = 0.01           # 定转子互感 H (励磁→d轴, 决定反电势)
    Rf: float = 75.0            # 励磁绕组电阻 Ω (750Vdc / 10A = 75Ω)
    Lf: float = 5.0             # 励磁绕组电感 H (τ=Lf/Rf≈67ms 建磁时间)
    If_nom: float = 10.0        # 额定励磁电流 A
    If_max: float = 10.0        # 最大励磁电流 A


@dataclass
class FlywheelMechanicalSpec:
    """飞轮机械/储能参数 — 泓慧 500kW/3kWh"""
    J: float = 33.9             # 总转动惯量 kg·m² (能量反推)
    mass_rotor: float = 855.0   # 转子质量 kg
    R_rotor: float = 0.380      # 转子外径 m (760mm)
    L_rotor: float = 0.260      # 转子厚度 m (260mm)
    material: str = "30Cr42MoV"
    sigma_y: float = 950e6      # 屈服强度 Pa

    omega_max_rpm: float = 8800.0    # 工作最高转速
    omega_min_rpm: float = 4400.0    # 放电截止转速
    omega_overspeed_rpm: float = 9900.0  # 超速保护 (112.5%)

    P_rated: float = 500e3      # 额定功率 W
    P_peak: float = 800e3       # 短时过载 W

    # 损耗系数 (需台架标定)
    k_windage: float = 0.0      # 风阻 (真空腔≈0)
    k_bearing: float = 0.5      # 轴承摩擦 Nm
    P_fixed: float = 200.0      # 固定损耗 W (控制/真空泵/传感器)

    # 热模型
    R_thermal: float = 0.5      # 热阻 °C/W
    C_thermal: float = 5000.0   # 热容 J/°C
    T_ambient: float = 25.0     # 环境温度
    T_max: float = 120.0        # 最高温度


@dataclass
class DCBusSpec:
    """直流母线电容参数"""
    C: float = 0.2              # 母线电容 F (200mF, 500kW 级)
    V_min: float = 550.0        # 最低工作电压 V
    V_max: float = 900.0        # 最高耐压 V
    R_esr: float = 0.01         # ESR Ω
    P_rect_max: float = 750e3   # 整流器最大功率 W
    P_inv_max: float = 750e3    # 逆变器最大功率 W


@dataclass
class GridInverterSpec:
    """网侧逆变器参数"""
    S_nom: float = 750e3        # 额定容量 VA
    L_f: float = 0.5e-3         # 滤波电感 H
    R_f: float = 0.01           # 滤波电阻 Ω
    C_f: float = 20e-6          # 滤波电容 F
    R_precharge: float = 50.0   # 预充电阻 Ω

    # 控制
    Kp_i: float = 10.0          # 电流环 Kp
    Ki_i: float = 100.0         # 电流环 Ki
    Kp_pll: float = 2.0         # PLL Kp
    Ki_pll: float = 50.0        # PLL Ki

    I_max: float = 2000.0       # 最大电流 A
    Vdc_max: float = 900.0      # 直流过压
    Vdc_min: float = 550.0      # 直流欠压


@dataclass
class SystemConfig:
    """系统级统一配置 — 聚合所有子模块规格"""
    constants: PhysicalConstants = field(default_factory=PhysicalConstants)
    motor: FlywheelMotorSpec = field(default_factory=FlywheelMotorSpec)
    flywheel: FlywheelMechanicalSpec = field(default_factory=FlywheelMechanicalSpec)
    dc_bus: DCBusSpec = field(default_factory=DCBusSpec)
    inverter: GridInverterSpec = field(default_factory=GridInverterSpec)

    # ── 工厂方法: 生成各子模块 Config ──
    def to_motor_config(self):
        from motor_base import PMSMConfig
        return PMSMConfig(
            P=self.motor.P,
            Rs=self.motor.Rs,
            Ld=self.motor.Ld,
            Lq=self.motor.Lq,
            psi_m=self.motor.psi_m,
            I_max=self.motor.I_max,
            Vdc_nom=self.constants.Vdc_nom,
            T_sample=self.constants.Ts,
            J=self.motor.J_motor,
            B=self.motor.B,
        )

    def to_flywheel_config(self):
        from flywheel_energy import FlywheelConfig
        return FlywheelConfig(
            J=self.flywheel.J,
            mass_rotor=self.flywheel.mass_rotor,
            R_rotor=self.flywheel.R_rotor,
            L_rotor=self.flywheel.L_rotor,
            material=self.flywheel.material,
            sigma_y=self.flywheel.sigma_y,
            omega_max=self.flywheel.omega_max_rpm,
            omega_min=self.flywheel.omega_min_rpm,
            omega_nom=self.flywheel.omega_max_rpm,
            omega_overspeed=self.flywheel.omega_overspeed_rpm,
            P_rated=self.flywheel.P_rated,
            P_peak=self.flywheel.P_peak,
            Vdc_nom=self.constants.Vdc_nom,
            Vac_nom=self.constants.Vac_nom,
            k_windage=self.flywheel.k_windage,
            k_bearing=self.flywheel.k_bearing,
            P_fixed=self.flywheel.P_fixed,
            R_thermal=self.flywheel.R_thermal,
            C_thermal=self.flywheel.C_thermal,
            T_ambient=self.flywheel.T_ambient,
            T_max=self.flywheel.T_max,
        )

    def to_dc_bus_config(self):
        from dc_bus import DCBusConfig
        return DCBusConfig(
            C=self.dc_bus.C,
            V_nom=self.constants.Vdc_nom,
            V_min=self.dc_bus.V_min,
            V_max=self.dc_bus.V_max,
            R_esr=self.dc_bus.R_esr,
            P_rect_max=self.dc_bus.P_rect_max,
            P_inv_max=self.dc_bus.P_inv_max,
        )

    def to_grid_inverter_config(self):
        from grid_inverter import GridInverterConfig
        return GridInverterConfig(
            S_nom=self.inverter.S_nom,
            Vdc_nom=self.constants.Vdc_nom,
            Vac_nom=self.constants.Vac_nom,
            f_nom=self.constants.f_nom,
            L_f=self.inverter.L_f,
            R_f=self.inverter.R_f,
            C_f=self.inverter.C_f,
            R_precharge=self.inverter.R_precharge,
            Ts=self.constants.Ts,
            Kp_i=self.inverter.Kp_i,
            Ki_i=self.inverter.Ki_i,
            Kp_pll=self.inverter.Kp_pll,
            Ki_pll=self.inverter.Ki_pll,
            I_max=self.inverter.I_max,
            Vdc_max=self.inverter.Vdc_max,
            Vdc_min=self.inverter.Vdc_min,
        )

    def to_controller_params(self):
        from control_algorithms import MotorModelParams
        return MotorModelParams(
            Rs=self.motor.Rs,
            Ld=self.motor.Ld,
            Lq=self.motor.Lq,
            psi_m=self.motor.psi_m,
            P=self.motor.P,
            Ts=self.constants.Ts,
        )

    def to_him_config(self):
        """生成 HIM 电机配置 (感应子电机专用, 使用真实飞轮参数)
        
        电机转子 + 飞轮本体同轴, J 合并为总惯量: J_motor + J_flywheel
        """
        from motor_base import HIMConfig
        J_total = self.motor.J_motor + self.flywheel.J  # 同轴总惯量
        return HIMConfig(
            P=self.motor.P, Rs=self.motor.Rs,
            Ld=self.motor.Ld, Lq=self.motor.Lq,
            Mdf=self.motor.Mdf,
            Rf=self.motor.Rf,
            Lf=self.motor.Lf,
            If_nom=self.motor.If_nom,
            If_max=self.motor.If_max,
            J=J_total, B=self.motor.B,
            I_max=self.motor.I_max,
            Vdc_nom=self.constants.Vdc_nom,
            T_sample=self.constants.Ts,
            N_max=self.flywheel.omega_max_rpm,
        )


# 全局默认配置 (测试可覆盖)
DEFAULT_CONFIG = SystemConfig()
