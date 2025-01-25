function [sys, x0, str, ts, simStateCompliance] = circuit(t, x, u, flag,m,g,K,R,L)
    % Critic S-Function
    % 输入：theta，目标值theta_target（无实际意义），
    % 输出：cost_function（基于最小theta的最大化）

    switch flag
        case 0
            [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes();
        case 1
            sys = mdlDerivatives(t, x, u,m,g,K,R,L);
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
        sizes.NumContStates = 3;   % 连续状态数
        sizes.NumDiscStates = 0;   % 离散状态数：[theta_min, cost_function]
        sizes.NumOutputs = 2;      % 输出个数
        sizes.NumInputs = 1;       % 输入个数
        sizes.DirFeedthrough = 0;  % 允许直馈通道
        sizes.NumSampleTimes = 1;  % 采样时间数
        sys = simsizes(sizes);     % 返回sizes数据结构
        x0 = [0.03 0 0];             % 初始状态：
        str = [];                  % 保留参数
        ts = [0 0];                % 采样时间
        simStateCompliance = 'UnknownSimState'; % 仿真状态合规性
    end

    function sys = mdlDerivatives(t, x, u,m,g,K,R,L)
        % 导数回调子函数（不使用，空实现）
        xx = x(1);
        dx = x(2);
        I = x(3);
        

        x_dot = dx;
        x_dot2 = (K * I^2 / xx^2 - m*g)/m;
        I_dot = (u - K*I/xx*dx - I*R)/L;
        
        %disp(u)
        %disp([x_dot;x_dot2;I_dot]);
        %disp(xx);
        sys = [x_dot;x_dot2;I_dot];
    end

    function sys = mdlUpdate(t, x, u)
        sys = [];
    end

    function sys = mdlOutputs(t, x, u)
        sys = [x(1);x(2)];
    end

    function sys = mdlGetTimeOfNextVarHit(t, x, u)
        % 计算下一个采样时间（定时采样）
        sampleTime = 1;  % 固定采样时间（1秒）
        sys = t + sampleTime;  % 下一次采样时间
    end

    function sys = mdlTerminate(t, x, u)
        % 仿真结束时的回调（输出成本函数）
        sys = [];
    end
end
