% 2-DOF 机械臂的物理模型 S-function实现

function [sys, x0, str, ts, simStateCompliance] = arm(t, x, u, flag)
    global input_data output_data time_data; % 全局变量存储数据
    switch flag
        case 0
            [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes();
        case 2
            sys = mdlUpdate(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case 9
            sys = mdlTerminate();
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes()
    global input_data output_data time_data;

    % 初始化全局变量
    input_data = [];
    output_data = [];
    time_data = [];

    sizes = simsizes;
    sizes.NumContStates  = 0;  % 无连续状态
    sizes.NumDiscStates  = 4; % 离散状态：[q1, q2, dq1, dq2]
    sizes.NumOutputs     = 2; % 输出：q1, q2
    sizes.NumInputs      = 2; % 输入：tau1, tau2
    sizes.DirFeedthrough = 0; % 无直接传递
    sizes.NumSampleTimes = 1; % 单一采样时间

    sys = simsizes(sizes);

    x0 = [0; 0; 0; 0]; % 初始状态：[q1, q2, dq1, dq2]
    str = [];
    ts = [0.01 0]; % 设置采样时间为 0.01 秒
    simStateCompliance = 'UnknownSimState'; % 仿真状态合规性
end

function sys = mdlUpdate(t, x, u)
    global input_data;
    input_data = [input_data; u(1), u(2)];     % 记录输入
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
    M = [m11, m12; m12, h2];

    % 科氏力和向心力矩阵 C(q, dq)
    c11 = -h3 * sin(q2) * dq2;
    c12 = -h3 * sin(q2) * (dq1 + dq2);
    c21 = h3 * sin(q2) * dq1;
    C = [c11, c12; c21, 0];

    % 重力矩阵 G(q)
    g1 = h4 * g * cos(q1) + h5 * g * cos(q1 + q2);
    g2 = h5 * g * cos(q1 + q2);
    G = [g1; g2];

    % 动力学方程求解加速度
    ddq = M \ (u - C * [dq1; dq2] - G);

    % 离散状态更新（积分）
    dt = 0.01; % 采样时间
    q1_next = q1 + dq1 * dt;
    q2_next = q2 + dq2 * dt;
    dq1_next = dq1 + ddq(1) * dt;
    dq2_next = dq2 + ddq(2) * dt;

    % 返回更新后的状态
    sys = [q1_next; q2_next; dq1_next; dq2_next];
end

function sys = mdlOutputs(t, x, u)
    global input_data output_data time_data;

    % 记录输入和输出数据
    
    output_data = [output_data; x(1:2)'];      % 记录输出
    time_data = [time_data; t];                % 记录时间

    sys = [x(1); x(2)]; % 返回输出
end

function sys = mdlTerminate()
    global input_data output_data time_data;

    % 仿真结束时保存数据到 data_level2.mat 和 data_level3.mat
    % 确保输入输出数据长度一致
    n = min(size(input_data, 1), size(output_data, 1));
    input_data = input_data(1:n, :);
    output_data = output_data(1:n, :);

    %% 二阶数据保存
    y_k_2 = output_data(1:end-2, :);  % y_k-2，从第1个样本开始到倒数第3个
    y_k_1 = output_data(2:end-1, :);  % y_k-1，从第2个样本开始到倒数第2个
    u_k_2 = input_data(1:end-2, :);   % u_k-2，从第1个样本开始到倒数第3个
    u_k_1 = input_data(2:end-1, :);   % u_k-1，从第2个样本开始到倒数第2个
    y_k = output_data(3:end, :);      % 输出向量，从第3个样本开始

    % 拼接特征矩阵和标签（二阶）
    features_level2 = [u_k_1, u_k_2, y_k_1, y_k_2];
    labels_level2 = y_k;

    save('data_level2.mat', 'features_level2', 'labels_level2', 'time_data');

    %% 三阶数据保存
    y_k_3 = output_data(1:end-3, :);  % y_k-3，从第1个样本开始到倒数第4个
    y_k_2 = output_data(2:end-2, :);  % y_k-2，从第2个样本开始到倒数第3个
    y_k_1 = output_data(3:end-1, :);  % y_k-1，从第3个样本开始到倒数第2个
    u_k_3 = input_data(1:end-3, :);   % u_k-3，从第1个样本开始到倒数第4个
    u_k_2 = input_data(2:end-2, :);   % u_k-2，从第2个样本开始到倒数第3个
    u_k_1 = input_data(3:end-1, :);   % u_k-1，从第3个样本开始到倒数第2个
    y_k = output_data(4:end, :);      % 输出向量，从第4个样本开始

    % 拼接特征矩阵和标签（三阶）
    features_level3 = [u_k_1, u_k_2, u_k_3, y_k_1, y_k_2, y_k_3];
    labels_level3 = y_k;

    save('data_level3.mat', 'features_level3', 'labels_level3', 'time_data');


    sys = []; % 终止函数无返回值
end
