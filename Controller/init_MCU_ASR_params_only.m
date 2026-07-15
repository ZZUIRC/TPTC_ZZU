%% init_MCU_ASR_params_only.m
% 只初始化参数 + 自动注册 MATLAB Function Block 参数
% 运行这一句即可：
% run('init_MCU_ASR_params_only.m')
%
% 本文件不运行仿真、不画图。
% 作用是保证模型里用到的变量不会再出现“未定义函数或变量”。

%% ========== 原始车辆参数 ==========
m = 1412;                 % 整车质量 kg
mb = 1400;                % 车身质量 kg
mw = (m-mb)/4;            % 单轮非簧载/轮胎等效质量 kg

Bf = 1.6;                 % 前轴轮距 m
Br = 1.6;                 % 后轴轮距 m

Lf = 1.02;                % 质心到前轴距离 m
Lr = 1.89;                % 质心到后轴距离 m
L = Lf + Lr;              % 轴距 m

hg = 0.5;                 % 质心高度 m
Iz = 1536.7;              % 整车 z 轴转动惯量 kg*m^2
I_tire = 0.74063;             % 单个轮胎转动惯量 kg*m^2

f = 0.015;                % 滚动阻力系数
r = 0.30938;                % 车轮滚动半径 m
i_sw = 20;                % 转向系传动比
g = 9.8;                  % 重力加速度 m/s^2

Pn_motor = 80;            % 电机最大功率 kW
Tn_motor = 1000;          % 电机最大扭矩 Nm

vx0 = 22;                 % 初始纵向车速 m/s；做起步仿真可改成 0
w0 = vx0/r;               % 初始轮速 rad/s
v_obj = 22;               % 期望车速 m/s
u = 0.3;                  % 低附路面附着系数

k1 = -43214*2;            % 两前轮侧偏刚度之和 N/rad
k2 = -43214*2;            % 两后轮侧偏刚度之和 N/rad
c1 = k1*L/(m*Lr);         % 侧偏系数 1/rad
c2 = k2*L/(m*Lf);
K = -(1/c1 - 1/c2)/L;     % 稳定因数

%% ========== MCU_ASR_Modular_MIL 使用的参数 ==========
% 下面这些变量是模型里模块参数和 MATLAB Function Block 会直接调用的名字。
% 全部定义为 Simulink.Parameter。
Ts_asr = Simulink.Parameter(0.001);
Ts_asr.CoderInfo.StorageClass = 'Auto';

m_veh = Simulink.Parameter(m);
m_veh.CoderInfo.StorageClass = 'Auto';

g0 = Simulink.Parameter(g);
g0.CoderInfo.StorageClass = 'Auto';

R_tire = Simulink.Parameter(r);
R_tire.CoderInfo.StorageClass = 'Auto';

Crr = Simulink.Parameter(f);
Crr.CoderInfo.StorageClass = 'Auto';

u_road = Simulink.Parameter(u);
u_road.CoderInfo.StorageClass = 'Auto';

% 当前参数表没有真实减速比，先用 9.0 占位，后续替换真实值
gear_ratio_F = Simulink.Parameter(10.91);
gear_ratio_F.CoderInfo.StorageClass = 'Auto';

gear_ratio_R = Simulink.Parameter(11.62);
gear_ratio_R.CoderInfo.StorageClass = 'Auto';

eta_drv = Simulink.Parameter(0.95);
eta_drv.CoderInfo.StorageClass = 'Auto';

% 一根轴两个轮胎的一阶近似轮端等效转动惯量
J_axle = Simulink.Parameter(2*I_tire);
J_axle.CoderInfo.StorageClass = 'Auto';

% 当前默认 Tn_motor 是电机侧最大扭矩。
% 如果 Tn_motor 是轮端/轴端最大扭矩，应改成：
% Tmax_mot = Simulink.Parameter(Tn_motor/(gear_ratio.Value*eta_drv.Value));
Tmax_mot = Simulink.Parameter(Tn_motor);
Tmax_mot.CoderInfo.StorageClass = 'Auto';

tau_motor = Simulink.Parameter(0.018);
tau_motor.CoderInfo.StorageClass = 'Auto';

T_active = Simulink.Parameter(15);
T_active.CoderInfo.StorageClass = 'Auto';

alpha_thr = Simulink.Parameter(300);
alpha_thr.CoderInfo.StorageClass = 'Auto';

Ts_asr = Simulink.Parameter(0.001);
Ts_asr.CoderInfo.StorageClass = 'Auto';

Tlim_up_rate = Simulink.Parameter(120);
Tlim_up_rate.CoderInfo.StorageClass = 'Auto';

Tlim_down_rate = Simulink.Parameter(2000);
Tlim_down_rate.CoderInfo.StorageClass = 'Auto';

Tlim_warn_rate = Simulink.Parameter(600);
Tlim_warn_rate.CoderInfo.StorageClass = 'Auto';

% 后电机
% Tlim_up_rate_R = Simulink.Parameter(1133);
% Tlim_up_rate_R.CoderInfo.StorageClass = 'Auto';
% 
% Tlim_down_rate_R = Simulink.Parameter(3400);
% Tlim_down_rate_R.CoderInfo.StorageClass = 'Auto';
% 
% Tlim_warn_rate_R = Simulink.Parameter(170);
% Tlim_warn_rate_R.CoderInfo.StorageClass = 'Auto';





Kd_damp = Simulink.Parameter(0.025);
Kd_damp.CoderInfo.StorageClass = 'Auto';

T_damp_lim = Simulink.Parameter(18);
T_damp_lim.CoderInfo.StorageClass = 'Auto';

tau_lpf_damp = Simulink.Parameter(0.060);
tau_lpf_damp.CoderInfo.StorageClass = 'Auto';

C_slip = Simulink.Parameter(8.0);
C_slip.CoderInfo.StorageClass = 'Auto';

rho_air = Simulink.Parameter(1.225);
rho_air.CoderInfo.StorageClass = 'Auto';

CdA = Simulink.Parameter(0.65);
CdA.CoderInfo.StorageClass = 'Auto';

K_stab = Simulink.Parameter(K);
K_stab.CoderInfo.StorageClass = 'Auto';

%% ========== 自动注册 MATLAB Function Block 里的 Parameter ==========
% 这一段是为了彻底解决：
% 未定义函数或变量 Tmax_mot / T_active / Ts_asr 等问题。
% 如果当前目录存在 MCU_ASR_Modular_MIL.slx，就自动注册并保存模型。

model = 'MCU_ASR_Modular_MIL';

if exist([model '.slx'],'file') || bdIsLoaded(model)
    if ~bdIsLoaded(model)
        load_system(model);
    end

    registerParams([model '/01_Scenario/Road_Mu_Profile'], ...
        {'u_road'});

    registerParams([model '/03_MCU_ASR_Controller/Slip_Detector_Front/Slip_Risk_Logic'], ...
        {'T_active','alpha_thr'});

    registerParams([model '/03_MCU_ASR_Controller/Slip_Detector_Rear/Slip_Risk_Logic'], ...
        {'T_active','alpha_thr'});

    registerParams([model '/03_MCU_ASR_Controller/Torque_Limiter_Front/Fast_Down_Slow_Up_Limiter'], ...
        {'Tmax_mot','Tlim_up_rate','Tlim_down_rate','Tlim_warn_rate','Ts_asr'});

    registerParams([model '/03_MCU_ASR_Controller/Torque_Limiter_Rear/Fast_Down_Slow_Up_Limiter'], ...
        {'Tmax_mot','Tlim_up_rate','Tlim_down_rate','Tlim_warn_rate','Ts_asr'});

    registerParams([model '/05_Longitudinal_Vehicle_Plant/Vehicle_Plant_Dynamics'], ...
        {'Ts_asr','m_veh','g0','R_tire','Lf','Lr','gear_ratio_F','gear_ratio_R','eta_drv','J_axle', ...
         'Crr','rho_air','CdA','C_slip','vx0'});

    save_system(model);
end

disp('init_MCU_ASR_params_only.m 已完成：参数已初始化，MATLAB Function 参数已注册。');

%% ========== 本脚本内部函数 ==========
function registerParams(blockPath,names)
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',blockPath);

    if isempty(chart)
        warning('找不到 MATLAB Function Block: %s',blockPath);
        return;
    end

    for ii = 1:numel(names)
        nm = names{ii};
        data = chart.find('-isa','Stateflow.Data','Name',nm);
        if isempty(data)
            data = Stateflow.Data(chart);
            data.Name = nm;
        end
        data.Scope = 'Parameter';
    end
end
