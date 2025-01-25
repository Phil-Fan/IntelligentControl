% Date: 2024-12-01
% Author: PhilFan 

% 倒立摆专家控制系统的S-function文件
% 输入参数:
%   t - 当前时间
%   x - 状态变量 [theta, dtheta/dt, theta_last, error_sum, error_last]
%   u - 输入变量 theta (摆杆角度)
%   flag - 仿真标志
%   K_b - 基本控制器增益
%   K_s - 切换控制器增益
%   theta_m - 最大角度阈值
%   theta_2 - 第二角度阈值
%   theta_1 - 第一角度阈值
%   Kp - PID控制器比例增益
%   Ki - PID控制器积分增益
%   Kd - PID控制器微分增益
%   angle - 初始角度
%   F_m - 最大控制力
% 输出参数:
%   sys - 返回值
%   x0 - 初始状态
%   str - 保留参数
%   ts - 采样时间
%   simStateCompliance - 仿真状态
% 功能说明:
%   该专家控制系统根据摆杆角度的不同区域,
%   自适应切换不同的控制策略(PID控制和能量控制),
%   实现倒立摆的平衡控制


function [sys,x0,str,ts,simStateCompliance] = expert_control(t,x,u,flag,K_b,K_s,theta_m,theta_2,theta_1,Kp,Ki,Kd,angle,F_m)
    % 主函数,包含四个输出:
    % sys - 包含某个子函数返回的值
    % x0 - 所有状态的初始化向量
    % str - 保留参数,总是一个空矩阵
    % ts - 返回系统采样时间
    % simStateCompliance - 仿真状态合规性
    
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(angle);
        case 1
            sys=mdlDerivatives(t,x,u);
        case 2
            sys=mdlUpdate(t,x,u,K_b,K_s,theta_m,theta_2,theta_1,Kp,Ki,Kd,F_m);
        case 3
            sys=mdlOutputs(t,x,u);
        case 4
            sys=mdlGetTimeOfNextVarHit(t,x,u);
        case 9
            sys=mdlTerminate(t,x,u);
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end

%% 子函数定义部分
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(angle)
    % 初始化回调子函数
    % 提供状态、输入输出、采样时间数目和初始状态的值
    % 初始化阶段,标志变量flag首先被置为0,S-function首次被调用时该子函数被调用
    
    sizes = simsizes;                  % 生成sizes数据结构
    sizes.NumContStates = 0;           % 连续状态数
    sizes.NumDiscStates = 5;           % 离散状态数
    sizes.NumOutputs = 1;              % 输出个数
    sizes.NumInputs = 1;               % 输入个数
    sizes.DirFeedthrough = 1;          % 是否存在直馈通道
    sizes.NumSampleTimes = 1;          % 采样时间个数
    
    sys = simsizes(sizes);             % 返回size数据结构
    x0 = [angle 0 angle 0 0];          % 设置初始状态
    str = [];                          % 保留变量置空
    ts = [0 0];                        % 设置采样时间
    simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
    sys = [];

function sys=mdlUpdate(t,x,u,K_b,K_s,theta_m,theta_2,theta_1,Kp,Ki,Kd,F_m)
    % 状态更新回调子函数
    % 给定t、x、u计算离散状态的更新
    
    theta_k = u;
    theta_k_1 = x(1);
    delta_theta_k = theta_k - theta_k_1;
    delta_theta_k_1 = x(1)-x(2);
    F = x(5);

    % PID控制参数
    K = 1;
    T = 0.0001;
    T_i = 0.001;
    T_d = 10;

    % 专家控制规则
    if abs(theta_k) >= theta_m
        % 规则1: 角度大于等于theta_m,施加最大外力
        F = sign(theta_k) * F_m;
    elseif abs(theta_k) >= theta_2
        % 规则2: theta_2到theta_m区间的控制
        if theta_k * delta_theta_k > 0
            K = K_b;
        elseif theta_k * delta_theta_k < 0
            if delta_theta_k * delta_theta_k_1 > 0
                K = 1;
            elseif delta_theta_k * delta_theta_k_1 < 0
                K = K_b;
            end
        end
        F = F + K * (Kp * delta_theta_k + (T / T_i) * theta_k + (T_d / T) * (delta_theta_k - delta_theta_k_1));
    elseif abs(theta_k) >= theta_1
        % 规则3: theta_1到theta_2区间的控制
        if theta_k * delta_theta_k > 0
            K = 1;
        else
            if delta_theta_k * delta_theta_k_1 > 0
                K = K_s;
            elseif delta_theta_k * delta_theta_k_1 < 0
                K = 1;
            end
        end
        F = F + K * (Kp * delta_theta_k + (T / T_i) * theta_k + (T_d / T) * (delta_theta_k - delta_theta_k_1));
    else
        K = 1;
        F = F + K * (Kp * delta_theta_k + (T / T_i) * theta_k + (T_d / T) * (delta_theta_k - delta_theta_k_1));
    end

    % 计算控制力
    %F = F + K * (Kp * delta_theta_k + (T / T_i) * theta_k + (T_d / T) * (delta_theta_k - delta_theta_k_1));

    % 更新系统状态
    sys = [theta_k, theta_k_1, delta_theta_k, delta_theta_k_1, F];

function sys=mdlOutputs(t,x,u)
    % 计算输出回调函数
    % 输出当前控制力F
    sys = [x(5)];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
    % 计算下一个采样时间
    % 仅在系统是变采样时间系统时调用
    sampleTime = 1;
    sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
    % 仿真结束时的回调函数
    sys = [];