function [sys, x0, str, ts] = arm(t, x, u, flag)
    global input_data output_data time_data; % 全局变量存储数据
    switch flag
        case 0
            [sys, x0, str, ts] = mdlInitializeSizes();
        case 1
            sys = mdlDerivatives(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case 9
            mdlTerminate();
            sys = [];
        otherwise
            sys = [];
    end
end

function [sys, x0, str, ts] = mdlInitializeSizes()
    global input_data output_data time_data;
    % 初始化全局变量
    input_data = [];
    output_data = [];
    time_data = [];

    sizes = simsizes;
    sizes.NumContStates  = 4;  % 连续状态：q1, q2, dq1, dq2
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2;  % 输出：q1, q2
    sizes.NumInputs      = 2;  % 输入：tau1, tau2
    sizes.DirFeedthrough = 0;  % 无直接传递
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);

    x0 = [0; 0; 0; 0];  % 初始状态：[q1, q2, dq1, dq2]
    str = [];
    ts = [0 0];  % 连续时间系统
end

function sys = mdlDerivatives(t, x, u)
    % 提取状态变量和输入
    q1 = x(1); q2 = x(2);       % 关节角
    dq1 = x(3); dq2 = x(4);     % 关节角速度
    tau1 = u(1); tau2 = u(2);   % 输入力矩

    % 系统参数
    h1 = 0.0308; h2 = 0.0106; h3 = 0.0095;
    h4 = 0.2086; h5 = 0.0631; g = 9.8;

    % 惯性矩阵 M(q)
    m11 = h1 + h2 + 2 * h3 * cos(q2);
    m12 = h2 + h3 * cos(q2);
    m21 = m12; % 对称性
    m22 = h2;
    M = [m11, m12; m21, m22];

    % 科氏力和向心力矩阵 C(q, dq)
    c11 = -h3 * sin(q2) * dq2;
    c12 = -h3 * sin(q2) * (dq1 + dq2);
    c21 = h3 * sin(q2) * dq1;
    c22 = 0;
    C = [c11, c12; c21, c22];

    % 重力矩阵 G(q)
    g1 = h4 * g * cos(q1) + h5 * g * cos(q1 + q2);
    g2 = h5 * g * cos(q1 + q2);
    G = [g1; g2];

    % 力矩输入
    Tau = [tau1; tau2];

    % 动力学方程求解加速度
    ddq = M \ (Tau - C * [dq1; dq2] - G);

    % 状态微分
    dq = [dq1; dq2];  % 当前角速度
    sys = [dq; ddq];  % [角速度变化; 加速度]
end

function sys = mdlOutputs(t, x, u)
    global input_data output_data time_data;

    % 检查输入是否为空
    if isempty(u) || any(isnan(u))
        warning('Input u contains NaN or is empty at time t = %.2f', t);
        u = [0; 0]; % 设置默认值
    end

    % 输出关节角度 q1 和 q2
    q1 = x(1); 
    q2 = x(2);
    output = [q1; q2]; % 输出变量

    % 将数据追加到全局变量
    input_data = [input_data; u']; % 输入力矩 [tau1, tau2]
    output_data = [output_data; output']; % 输出关节角 [q1, q2]
    time_data = [time_data; t]; % 时间

    sys = output; % 返回输出
end

function mdlTerminate()
    global input_data output_data time_data;
    % 在仿真结束时保存数据到 data.mat
    save('data.mat', 'input_data', 'output_data', 'time_data');
end
