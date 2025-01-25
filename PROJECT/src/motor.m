function [sys, x0, str, ts, simStateCompliance] = motor(t, x, u, flag, R_s, L_d, L_q, psi_f, p, J, B)
    switch flag
        case 0
            [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes();
        case 1
            sys = mdlDerivatives(t, x, u, R_s, L_d, L_q, psi_f, p, J, B);
        case 2
            sys = mdlUpdate(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case 9
            sys = mdlTerminate(t, x, u);
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

% 初始化函数
function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes()
    % 定义状态、输入、输出的大小
    sizes = simsizes;

    sizes.NumContStates  = 3;  % 连续状态：I_d, I_q, omega
    sizes.NumDiscStates  = 0;  % 无离散状态
    sizes.NumOutputs     = 1;  % 输出：omega
    sizes.NumInputs      = 3;  % 输入：V_d, V_q, T_L
    sizes.DirFeedthrough = 0;  % 无直接传递
    sizes.NumSampleTimes = 1;  % 一个采样时间

    sys = simsizes(sizes);

    x0 = [0; 0; 0]; % 初始状态：I_d, I_q, omega
    str = [];
    ts = [0 0]; % 连续时间系统
    simStateCompliance = 'UnknownSimState'; % 状态兼容性
end

% 更新函数（此处为空，因为是连续系统）
function sys = mdlUpdate(~, ~, ~)
    sys = [];
end

% 输出函数
function sys = mdlOutputs(~, x, ~)
    sys = [x(3)];
end

% 终止函数
function sys = mdlTerminate(~, ~, ~)
    % 终止时执行清理操作（如果需要）
    sys = [];
end

% 动态方程求解
function sys = mdlDerivatives(~, x, u, R_s, L_d, L_q, psi_f, p, J, B)
    % 参数定义
    % R_s = 0.01;     % 定子电阻
    % L_d = 0.001;    % d 轴电感
    % L_q = 0.0015;   % q 轴电感
    % psi_f = 0.1;    % 永磁体磁链
    % p = 4;          % 极对数
    % J = 0.01;       % 转动惯量
    % B = 0.001;      % 阻尼系数

    % 输入提取
    V_d = u(1); % d 轴电压
    V_q = u(2); % q 轴电压
    T_L = u(3); % 负载转矩

    % 状态提取
    I_d = x(1); % d 轴电流
    I_q = x(2); % q 轴电流
    omega = x(3); % 转速

    % 电磁转矩计算
    T_e = (3/2) * p * (psi_f * I_q + (L_d - L_q) * I_d * I_q);

    % d 轴电流微分方程
    dId_dt = (1 / L_d) * (V_d - R_s * I_d + omega * L_q * I_q);

    % q 轴电流微分方程
    dIq_dt = (1 / L_q) * (V_q - R_s * I_q - omega * L_d * I_d - omega * psi_f);

    % 转速微分方程
    domega_dt = (1 / J) * (T_e - T_L - B * omega);

    % 返回微分方程结果
    sys = [dId_dt; dIq_dt; domega_dt];
end
