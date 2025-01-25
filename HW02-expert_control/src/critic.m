% Date: 2024-12-03
% Author: PhilFan 

% 评价网络定义
% 输入参数:
%   t - 当前时间
%   x - 状态变量 [theta_min, cost_function]
%   u - 输入变量 theta (摆杆角度)
%   flag - 仿真标志
% 输出参数:
%   sys - 返回值
%   x0 - 初始状态
%   str - 保留参数
%   ts - 采样时间
%   simStateCompliance - 仿真状态
% 功能说明:
%   该评价网络用于评估倒立摆系统的性能
%   通过最小化超调来优化控制效果
%   cost_function基于MSE进行计算


function [sys, x0, str, ts, simStateCompliance] = critic(t, x, u, flag)
    % Critic S-Function
    % 输入：theta，目标值theta_target（无实际意义），
    % 输出：cost_function（基于最小theta的最大化）

    switch flag
        case 0
            [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes();
        case 1
            sys = mdlDerivatives(t, x, u);
        case 2
            sys = mdlUpdate(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case 4
            sys = mdlGetTimeOfNextVarHit(t, x, u);
        case 9
            sys = mdlTerminate(t, x, u);
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end

    %% 子函数定义
    function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes()
        % 初始化回调子函数
        
        sizes = simsizes;
        sizes.NumContStates = 0;   % 连续状态数
        sizes.NumDiscStates = 2;   % 离散状态数：[theta_min, cost_function]
        sizes.NumOutputs = 1;      % 输出个数（成本函数）
        sizes.NumInputs = 1;       % 输入个数（theta）
        sizes.DirFeedthrough = 1;  % 允许直馈通道
        sizes.NumSampleTimes = 1;  % 采样时间数
        sys = simsizes(sizes);     % 返回sizes数据结构
        x0 = [Inf, 0];             % 初始状态：[theta_min, cost_function]
        str = [];                  % 保留参数
        ts = [0 0];                % 采样时间
        simStateCompliance = 'UnknownSimState'; % 仿真状态合规性
    end

    function sys = mdlDerivatives(t, x, u)
        % 导数回调子函数（不使用，空实现）
        sys = [];
    end

    function sys = mdlUpdate(t, x, u)
        % 状态更新回调子函数
        % 更新最小值theta_min，保存当前theta_k与历史最小值比较
        theta_k = u;  % 当前的theta值
        theta_min = min(x(1), theta_k);  % 更新最小值
        cost_function = x(2) + u * u ;  % 使用MSE计算成本函数
        sys = [theta_min, cost_function];
    end

    function sys = mdlOutputs(t, x, u)
        % 输出回调子函数
        % 输出成本函数
        alpha = 100;
        beta = 0.01;
        cost = x(2);
        min = x(1);
        fi  = beta*cost + alpha * abs(min);
        sys = [fi];    % 输出成本函数
    end

    function sys = mdlGetTimeOfNextVarHit(t, x, u)
        % 计算下一个采样时间（定时采样）
        sampleTime = 1;  % 固定采样时间（1秒）
        sys = t + sampleTime;  % 下一次采样时间
    end

    function sys = mdlTerminate(t, x, u)
        % 仿真结束时的回调（输出成本函数）
        alpha = 100;
        beta = 0.01;
        %disp(['min: ', num2str(x(1))]);
        
        cost = x(2);
        min = x(1);
        fi  = beta*cost + alpha * abs(min);
        disp(['cost: ', num2str(fi)]);
        sys = [];
    end
end
