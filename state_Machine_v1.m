function [risk,warn,state,load_ratio,follow_bad,...
    axis_diverge,startup_guard,limit_ratio] = f( ...
    Tcmd_fb,alpha,Tload,omega, ...
    Tcmd_fb_opp,alpha_opp,Tload_opp,omega_opp, ...
    T_active,alpha_thr,Ts,Treq)
%#codegen

% ============================================================
% MCU级 TPTC/ASR 滑移风险状态机
%
% 本版本不再需要实际电磁扭矩Te。
%
% 输入：
%   Tcmd_fb
%       本轴上一控制周期最终下发给电流环的目标扭矩，N*m
%       必须是经过TPTC/BMS/热限扭/电机能力限扭后的最终指令
%
%   alpha
%       本轴电机机械角加速度，rad/s^2
%
%   Tload
%       本轴电机侧负载转矩观测值，N*m
%
%   omega
%       本轴电机机械角速度，rad/s
%
%   Tcmd_fb_opp、alpha_opp、Tload_opp、omega_opp
%       对轴对应信号
%
%   Treq
%       限扭前VCU原始目标扭矩，N*m
%
% 注意：
%   Tcmd_fb不能接Treq。
%   推荐连接：
%
%   限扭器最终输出
%          |
%       Unit Delay
%          |
%       Tcmd_fb
%
% 状态：
%   0 Normal  ：limit_ratio = 1.00
%   1 Warning ：limit_ratio = 0.82
%   2 Risk    ：limit_ratio = 0.50~0.70
% ============================================================


%% ============================================================
% 1. 持久变量
% =============================================================

persistent initialized
persistent state_z

persistent active_cnt
persistent baseline_cnt
persistent baseline_valid
persistent cooldown_cnt

persistent q_fast_z
persistent q_ref_z
persistent q_rate_z

persistent omega_z
persistent omega_opp_z

persistent risk_enter_cnt
persistent warn_enter_cnt
persistent risk_reenter_cnt
persistent severe_load_cnt

persistent state_hold_cnt
persistent stable_cnt

persistent risk_ratio_z

% 最终目标扭矩的一阶动态代理
persistent Tproxy_z
persistent Tproxy_opp_z


%% ============================================================
% 2. 初始化
% =============================================================

if isempty(initialized)

    state_z = 0.0;

    active_cnt = 0.0;
    baseline_cnt = 0.0;
    baseline_valid = false;
    cooldown_cnt = 0.0;

    q_fast_z = 0.0;
    q_ref_z = 0.0;
    q_rate_z = 0.0;

    omega_z = 0.0;
    omega_opp_z = 0.0;

    risk_enter_cnt = 0.0;
    warn_enter_cnt = 0.0;
    risk_reenter_cnt = 0.0;
    severe_load_cnt = 0.0;

    state_hold_cnt = 0.0;
    stable_cnt = 0.0;

    risk_ratio_z = 0.60;

    Tproxy_z = 0.0;
    Tproxy_opp_z = 0.0;

    initialized = true;
end


%% ============================================================
% 3. 采样周期和输入保护
% =============================================================

if Ts > 0.0
    Ts_eff = Ts;
else
    Ts_eff = 1.0e-3;
end

T_active_eff = max(abs(T_active),1.0);
alpha_thr_eff = max(abs(alpha_thr),1.0);

Treq_abs = abs(Treq);
Tload_abs = abs(Tload);


%% ============================================================
% 4. 最终目标扭矩动态代理
%
% 用限扭后的最终目标扭矩，经一阶动态环节近似实际电磁扭矩。
%
% tau_torque_proxy后续应使用实车扭矩阶跃响应标定。
% 当前先采用18 ms。
% =============================================================

tau_torque_proxy = 0.018;

a_torque_proxy = ...
    Ts_eff/(tau_torque_proxy + Ts_eff);

Tproxy_z = ...
    Tproxy_z + ...
    a_torque_proxy*(Tcmd_fb - Tproxy_z);

Tproxy_opp_z = ...
    Tproxy_opp_z + ...
    a_torque_proxy*(Tcmd_fb_opp - Tproxy_opp_z);

Tproxy_abs = abs(Tproxy_z);
Tproxy_opp_abs = abs(Tproxy_opp_z);


%% ============================================================
% 5. 统一角加速度方向
%
% alpha_drv > 0表示电机转速幅值增加。
% =============================================================

drive_dir = 1.0;

if abs(omega) > 1.0

    if omega < 0.0
        drive_dir = -1.0;
    end

elseif Tproxy_abs > 1.0

    if Tproxy_z < 0.0
        drive_dir = -1.0;
    end

elseif Treq < 0.0

    drive_dir = -1.0;
end

alpha_drv = drive_dir*alpha;


drive_dir_opp = 1.0;

if abs(omega_opp) > 1.0

    if omega_opp < 0.0
        drive_dir_opp = -1.0;
    end

elseif Tproxy_opp_abs > 1.0

    if Tproxy_opp_z < 0.0
        drive_dir_opp = -1.0;
    end

elseif Tcmd_fb_opp < 0.0

    drive_dir_opp = -1.0;
end

alpha_opp_drv = drive_dir_opp*alpha_opp;


%% ============================================================
% 6. 标定参数
% =============================================================

% 起步保护
startup_guard_time = 0.050;
initial_blank_time = 0.050;

startup_guard_steps = ...
    max(1.0,ceil(startup_guard_time/Ts_eff));

initial_blank_steps = ...
    max(1.0,ceil(initial_blank_time/Ts_eff));


% 正常负载基线学习
baseline_learn_time = 0.400;

baseline_learn_steps = ...
    max(1.0,ceil(baseline_learn_time/Ts_eff));


% 回到Normal后的检测冷却
cooldown_time = 0.300;

cooldown_steps = ...
    max(1.0,ceil(cooldown_time/Ts_eff));


% Normal进入状态确认
risk_enter_time = 0.008;
warn_enter_time = 0.020;

risk_enter_steps = ...
    max(1.0,ceil(risk_enter_time/Ts_eff));

warn_enter_steps = ...
    max(1.0,ceil(warn_enter_time/Ts_eff));


% Warning重新升级Risk确认
risk_reenter_time = 0.020;

risk_reenter_steps = ...
    max(1.0,ceil(risk_reenter_time/Ts_eff));


% 持续严重负载不足确认
severe_load_time = 0.030;

severe_load_steps = ...
    max(1.0,ceil(severe_load_time/Ts_eff));


% Risk保持和恢复确认
risk_hold_time = 0.250;
risk_stable_time = 0.200;

risk_hold_steps = ...
    max(1.0,ceil(risk_hold_time/Ts_eff));

risk_stable_steps = ...
    max(1.0,ceil(risk_stable_time/Ts_eff));


% Warning保持和恢复确认
warn_hold_time = 0.300;
warn_stable_time = 0.300;

warn_hold_steps = ...
    max(1.0,ceil(warn_hold_time/Ts_eff));

warn_stable_steps = ...
    max(1.0,ceil(warn_stable_time/Ts_eff));


% 负载指标滤波
tau_q_fast = 0.030;
tau_q_ref = 1.500;
tau_q_rate = 0.030;

a_q_fast = ...
    Ts_eff/(tau_q_fast + Ts_eff);

a_q_ref = ...
    Ts_eff/(tau_q_ref + Ts_eff);

a_q_rate = ...
    Ts_eff/(tau_q_rate + Ts_eff);


% Normal状态负载阈值
q_warn_thr = 0.85;
q_risk_thr = 0.68;
q_dynamic_thr = 0.78;

% 严重负载支持能力阈值
severe_load_thr = 0.72;

% 相对负载下降速率阈值，单位1/s
q_rate_warn_thr = -0.30;
q_rate_risk_thr = -0.70;


% Risk恢复到Warning
support_risk_recover_thr = 0.80;

% Warning期间重新恶化
support_reenter_thr = 0.72;

% Warning退出Normal
support_normal_thr = 0.88;


% 角加速度阈值
alpha_warn_thr = 0.55*alpha_thr_eff;
alpha_risk_thr = 0.85*alpha_thr_eff;
alpha_strong_thr = 1.25*alpha_thr_eff;


% 限扭比例
warning_limit_ratio = 0.82;

risk_ratio_min = 0.50;
risk_ratio_max = 0.70;
risk_ratio_margin = 0.10;


% 前后轴动态分化，仅作诊断
alpha_diff_thr = 0.60*alpha_thr_eff;
omega_diff_thr = 10.0;
omega_diff_rate_thr = 80.0;


%% ============================================================
% 7. 有效驱动请求
%
% 必须使用限扭前Treq。
% =============================================================

active = (Treq_abs > T_active_eff);

if active

    active_cnt = active_cnt + 1.0;

else

    active_cnt = 0.0;

    state_z = 0.0;

    baseline_cnt = 0.0;
    baseline_valid = false;
    cooldown_cnt = 0.0;

    risk_enter_cnt = 0.0;
    warn_enter_cnt = 0.0;
    risk_reenter_cnt = 0.0;
    severe_load_cnt = 0.0;

    state_hold_cnt = 0.0;
    stable_cnt = 0.0;

    risk_ratio_z = 0.60;
end

startup_guard = ...
    active && ...
    (active_cnt <= startup_guard_steps);

initial_blank = ...
    active && ...
    (active_cnt <= initial_blank_steps);


%% ============================================================
% 8. 扭矩指令执行比例
%
% 不再使用Te/Treq，而使用：
%
% torque_ratio_applied = Tproxy/Treq
%
% Tproxy是最终目标扭矩经过一阶动态后的代理。
% =============================================================

if Treq_abs > 1.0

    torque_ratio_applied = ...
        Tproxy_abs/Treq_abs;

else

    torque_ratio_applied = 0.0;
end

if torque_ratio_applied < 0.0

    torque_ratio_applied = 0.0;

elseif torque_ratio_applied > 1.20

    torque_ratio_applied = 1.20;
end


%% ============================================================
% 9. 负载观测有效性
%
% 扭矩代理过小时，Tload观测值以及归一化指标不可靠。
% =============================================================

observer_valid = ...
    active && ...
    (Tproxy_abs > ...
     max(0.20*Treq_abs,...
         0.50*T_active_eff));


%% ============================================================
% 10. 原始负载指标
%
% q_raw = |Tload_hat|/|Treq|
%
% 分母仍然使用限扭前Treq。
% =============================================================

q_raw = ...
    Tload_abs/max(Treq_abs,1.0);

if q_raw < 0.0

    q_raw = 0.0;

elseif q_raw > 2.50

    q_raw = 2.50;
end


%% ============================================================
% 11. 当前负载快速滤波和变化率
% =============================================================

if ~active

    q_fast_z = q_raw;
    q_ref_z = q_raw;
    q_rate_z = 0.0;

elseif observer_valid

    if active_cnt <= 1.0

        q_fast_z = q_raw;
        q_ref_z = q_raw;
        q_rate_z = 0.0;

    else

        q_fast_old = q_fast_z;

        q_fast_z = ...
            q_fast_z + ...
            a_q_fast*(q_raw-q_fast_z);

        q_rate_raw = ...
            (q_fast_z-q_fast_old)/Ts_eff;

        q_rate_z = ...
            q_rate_z + ...
            a_q_rate*(q_rate_raw-q_rate_z);
    end

else

    q_rate_z = ...
        q_rate_z + ...
        a_q_rate*(0.0-q_rate_z);
end


%% ============================================================
% 12. 冷却计数
% =============================================================

if cooldown_cnt > 0.0
    cooldown_cnt = cooldown_cnt-1.0;
end


%% ============================================================
% 13. 正常负载基线学习
%
% 只有在：
%   1. Normal状态；
%   2. 扭矩代理接近原始请求；
%   3. 电机动态稳定；
%   4. 负载观测有效；
%
% 才学习正常负载基线。
% =============================================================

normal_baseline_condition = ...
    active && ...
    observer_valid && ...
    (state_z == 0.0) && ...
    (torque_ratio_applied > 0.92) && ...
    (abs(alpha_drv) < 0.35*alpha_thr_eff);


if normal_baseline_condition

    if ~baseline_valid

        baseline_cnt = baseline_cnt + 1.0;

        if baseline_cnt <= 1.0

            q_ref_z = q_fast_z;

        else

            q_ref_z = ...
                q_ref_z + ...
                a_q_ref*(q_fast_z-q_ref_z);
        end

        if baseline_cnt >= baseline_learn_steps
            baseline_valid = true;
        end

    else

        q_ref_pre = max(q_ref_z,0.02);

        q_full_pre = ...
            q_fast_z/q_ref_pre;

        q_rate_pre = ...
            q_rate_z/q_ref_pre;

        normal_for_update = ...
            (q_full_pre > 0.90) && ...
            (q_full_pre < 1.12) && ...
            (abs(q_rate_pre) < 0.12);

        if normal_for_update

            q_ref_z = ...
                q_ref_z + ...
                a_q_ref*(q_fast_z-q_ref_z);
        end
    end

elseif ~baseline_valid

    % 基线学习要求连续满足稳定条件
    baseline_cnt = 0.0;
end


%% ============================================================
% 14. 负载能力指标
%
% q_full：
%   当前负载相对于正常全扭矩基线的比例。
%
% q_support：
%   对当前限扭比例进行补偿后的负载支持能力。
% =============================================================

if baseline_valid

    q_ref_den = max(q_ref_z,0.02);

    q_full = ...
        q_fast_z/q_ref_den;

    q_rate_rel = ...
        q_rate_z/q_ref_den;

    applied_ratio_den = ...
        max(torque_ratio_applied,0.20);

    q_support = ...
        q_fast_z/...
        (q_ref_den*applied_ratio_den);

else

    q_full = 1.0;
    q_rate_rel = 0.0;
    q_support = 1.0;
end


if q_full < 0.0

    q_full = 0.0;

elseif q_full > 3.0

    q_full = 3.0;
end


if q_support < 0.0

    q_support = 0.0;

elseif q_support > 3.0

    q_support = 3.0;
end


load_ratio = q_support;


%% ============================================================
% 15. Normal状态检测器使能
% =============================================================

detector_armed = ...
    baseline_valid && ...
    active && ...
    (state_z == 0.0) && ...
    (cooldown_cnt <= 0.0) && ...
    (torque_ratio_applied > 0.90);


%% ============================================================
% 16. 一般负载异常
% =============================================================

load_low_warn = ...
    detector_armed && ...
    (q_full < q_warn_thr);

load_low_risk = ...
    detector_armed && ...
    (q_full < q_risk_thr);

load_drop_warn = ...
    detector_armed && ...
    (q_rate_rel < q_rate_warn_thr);

load_drop_risk = ...
    detector_armed && ...
    (q_rate_rel < q_rate_risk_thr);


%% ============================================================
% 17. 持续严重负载能力不足
%
% Normal使用q_full；
% Warning使用补偿限扭影响后的q_support。
% =============================================================

if state_z == 0.0

    severe_load_metric = q_full;

    severe_load_enable = ...
        detector_armed && ...
        (torque_ratio_applied > 0.90);

elseif state_z == 1.0

    severe_load_metric = q_support;

    severe_load_enable = ...
        active && ...
        observer_valid && ...
        (torque_ratio_applied > 0.50);

else

    severe_load_metric = q_support;
    severe_load_enable = false;
end


severe_load_raw = ...
    severe_load_enable && ...
    (severe_load_metric < severe_load_thr);


if severe_load_raw

    severe_load_cnt = severe_load_cnt + 1.0;

    if severe_load_cnt > severe_load_steps
        severe_load_cnt = severe_load_steps;
    end

else

    severe_load_cnt = 0.0;
end


severe_load_confirmed = ...
    (severe_load_cnt >= severe_load_steps);


follow_bad = ...
    load_low_warn || ...
    load_drop_warn || ...
    severe_load_confirmed || ...
    ( ...
      (state_z ~= 0.0) && ...
      (q_support < support_risk_recover_thr) ...
    );


%% ============================================================
% 18. 电机角加速度条件
% =============================================================

alpha_warn = ...
    (alpha_drv > alpha_warn_thr);

alpha_risk = ...
    (alpha_drv > alpha_risk_thr);

alpha_strong = ...
    (alpha_drv > alpha_strong_thr);


%% ============================================================
% 19. 前后轴动态分化诊断
%
% 当前仅作为诊断输出，不直接改变状态。
% =============================================================

omega_diff = ...
    abs(omega)-abs(omega_opp);

omega_diff_old = ...
    abs(omega_z)-abs(omega_opp_z);

d_omega_diff = ...
    (omega_diff-omega_diff_old)/Ts_eff;


alpha_diverge = ...
    (~startup_guard) && ...
    alpha_warn && ...
    (alpha_drv > ...
     alpha_opp_drv+alpha_diff_thr);


omega_diverge = ...
    (~startup_guard) && ...
    (omega_diff > omega_diff_thr) && ...
    (d_omega_diff > omega_diff_rate_thr);


axis_diverge = ...
    alpha_diverge || omega_diverge;


%% ============================================================
% 20. Normal状态进入条件
% =============================================================

risk_enter_raw = false;
warn_enter_raw = false;

if state_z == 0.0 && ...
        active && ...
        (~initial_blank)

    % 路径1：负载严重降低并伴随动态异常
    risk_path_main = ...
        load_low_risk && ...
        (alpha_risk || load_drop_risk);

    % 路径2：负载中度下降且快速恶化
    risk_path_dynamic = ...
        detector_armed && ...
        (q_full < q_dynamic_thr) && ...
        load_drop_risk && ...
        alpha_warn;

    % 路径3：极强角加速度且负载已经降低
    risk_path_strong = ...
        detector_armed && ...
        alpha_strong && ...
        (q_full < 0.90);

    % 路径4：持续严重负载不足
    % 用于解决电机平均角加速度不明显时的漏检
    risk_path_sustained_load = ...
        severe_load_confirmed;

    risk_enter_raw = ...
        risk_path_main || ...
        risk_path_dynamic || ...
        risk_path_strong || ...
        risk_path_sustained_load;


    % Warning：中度负载异常
    warn_path_main = ...
        load_low_warn && ...
        (alpha_warn || load_drop_warn);

    warn_enter_raw = ...
        warn_path_main;
end


% Risk优先
if risk_enter_raw
    warn_enter_raw = false;
end


%% ============================================================
% 21. Normal状态进入确认
% =============================================================

if risk_enter_raw

    risk_enter_cnt = risk_enter_cnt + 1.0;

    if risk_enter_cnt > risk_enter_steps
        risk_enter_cnt = risk_enter_steps;
    end

else

    risk_enter_cnt = 0.0;
end


if warn_enter_raw

    warn_enter_cnt = warn_enter_cnt + 1.0;

    if warn_enter_cnt > warn_enter_steps
        warn_enter_cnt = warn_enter_steps;
    end

else

    warn_enter_cnt = 0.0;
end


risk_enter = ...
    (risk_enter_cnt >= risk_enter_steps);

warn_enter = ...
    (warn_enter_cnt >= warn_enter_steps);


%% ============================================================
% 22. Warning重新升级Risk
% =============================================================

risk_reenter_raw = ...
    (state_z == 1.0) && ...
    active && ...
    ( ...
      alpha_strong || ...
      severe_load_confirmed || ...
      ( ...
        (q_support < support_reenter_thr) && ...
        alpha_warn ...
      ) ...
    );


if risk_reenter_raw

    risk_reenter_cnt = risk_reenter_cnt + 1.0;

    if risk_reenter_cnt > risk_reenter_steps
        risk_reenter_cnt = risk_reenter_steps;
    end

else

    risk_reenter_cnt = 0.0;
end


risk_reenter = ...
    (risk_reenter_cnt >= risk_reenter_steps);


%% ============================================================
% 23. 三状态状态机
% =============================================================

if ~active

    % 无驱动请求
    state_z = 0.0;

    state_hold_cnt = 0.0;
    stable_cnt = 0.0;


elseif state_z == 0.0

    % --------------------------------------------------------
    % Normal
    % --------------------------------------------------------

    state_hold_cnt = 0.0;
    stable_cnt = 0.0;

    if risk_enter

        % 根据发生风险时的负载能力计算初始限扭比例
        ratio_candidate = ...
            q_full+risk_ratio_margin;

        if ratio_candidate < risk_ratio_min

            risk_ratio_z = risk_ratio_min;

        elseif ratio_candidate > risk_ratio_max

            risk_ratio_z = risk_ratio_max;

        else

            risk_ratio_z = ratio_candidate;
        end

        state_z = 2.0;

        risk_enter_cnt = 0.0;
        warn_enter_cnt = 0.0;
        risk_reenter_cnt = 0.0;
        severe_load_cnt = 0.0;

        state_hold_cnt = 0.0;
        stable_cnt = 0.0;


    elseif warn_enter

        state_z = 1.0;

        warn_enter_cnt = 0.0;
        risk_reenter_cnt = 0.0;

        state_hold_cnt = 0.0;
        stable_cnt = 0.0;
    end


elseif state_z == 2.0

    % --------------------------------------------------------
    % Risk：快速限扭
    % --------------------------------------------------------

    state_hold_cnt = state_hold_cnt + 1.0;

    if state_hold_cnt >= risk_hold_steps

        risk_stable = ...
            (q_support > support_risk_recover_thr) && ...
            (alpha_drv < 0.55*alpha_thr_eff);

        if risk_stable

            stable_cnt = stable_cnt + 1.0;

            if stable_cnt >= risk_stable_steps

                % Risk先退到Warning
                state_z = 1.0;

                state_hold_cnt = 0.0;
                stable_cnt = 0.0;

                risk_reenter_cnt = 0.0;
                severe_load_cnt = 0.0;
            end

        else

            stable_cnt = 0.0;
        end
    end


else

    % --------------------------------------------------------
    % Warning：轻度限扭和恢复观察
    % --------------------------------------------------------

    state_hold_cnt = state_hold_cnt + 1.0;

    if risk_reenter

        % 恢复过程中再次恶化：
        % 按当前实际执行比例再降低约10%
        ratio_candidate = ...
            torque_ratio_applied-0.10;

        if ratio_candidate < risk_ratio_min

            risk_ratio_z = risk_ratio_min;

        elseif ratio_candidate > risk_ratio_max

            risk_ratio_z = risk_ratio_max;

        else

            risk_ratio_z = ratio_candidate;
        end

        state_z = 2.0;

        state_hold_cnt = 0.0;
        stable_cnt = 0.0;

        risk_reenter_cnt = 0.0;
        severe_load_cnt = 0.0;


    elseif state_hold_cnt >= warn_hold_steps

        normal_recovered = ...
            (q_support > support_normal_thr) && ...
            (abs(alpha_drv) < 0.40*alpha_thr_eff) && ...
            (~severe_load_confirmed);

        if normal_recovered

            stable_cnt = stable_cnt + 1.0;

            if stable_cnt >= warn_stable_steps

                state_z = 0.0;

                state_hold_cnt = 0.0;
                stable_cnt = 0.0;

                risk_reenter_cnt = 0.0;
                severe_load_cnt = 0.0;

                % 刚回Normal时暂时关闭检测，
                % 防止恢复扭矩造成再次误触发
                cooldown_cnt = cooldown_steps;
            end

        else

            stable_cnt = 0.0;
        end
    end
end


%% ============================================================
% 24. 输出
% =============================================================

state = state_z;

risk = ...
    (state_z == 2.0);

warn = ...
    (state_z == 1.0);


if state_z == 2.0

    % Risk：自适应强限扭
    limit_ratio = risk_ratio_z;

elseif state_z == 1.0

    % Warning：82%轻度限扭
    limit_ratio = warning_limit_ratio;

else

    % Normal：不限制
    limit_ratio = 1.0;
end


%% ============================================================
% 25. 更新历史量
% =============================================================

omega_z = omega;
omega_opp_z = omega_opp;

% 当前版本保留对轴负载接口
unused_Tload_opp = Tload_opp; %#ok<NASGU>

end