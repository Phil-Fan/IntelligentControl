function [sys,x0,str,ts,simStateCompliance] = rbfpid(t,x,u,flag)
    % RBF神经网络PID控制器
    % 网络结构: 3-6-1
    % 输入: 误差e、误差积分、误差微分
    % 输出: 控制量u
    
    % 采样时间
    Ts = 0.001;
    
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Ts);
        case 2
            sys = mdlUpdate(x,u,Ts);
        case 3
            sys = mdlOutputs(t,x,u);
        case {1,4,9}
            sys = [];
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
    
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Ts)
    sizes = simsizes;
    
    sizes.NumContStates  = 0;      % 连续状态个数
    sizes.NumDiscStates  = 3;      % 离散状态个数
    sizes.NumOutputs     = 5;      % 输出个数
    sizes.NumInputs      = 4;      % 输入个数
    sizes.DirFeedthrough = 1;      % 直通标志
    sizes.NumSampleTimes = 1;      % 采样时间个数
    
    sys = simsizes(sizes);
    x0  = [0;0;0];                 % 初始状态
    str = [];
    ts  = [Ts 0];                  % 采样时间
    simStateCompliance = 'UnknownSimState';
    
function sys = mdlUpdate(x,u,Ts)
    % 更新离散状态
    sys = [u(1);                   % 误差e(k)
           x(2) + u(1)*Ts;         % 误差积分
           (u(1) - u(2))];         % 误差微分e(k)-e(k-1)
    
function sys = mdlOutputs(t,x,u)
    persistent weights weights_prev weights_prev2 hidden_output centers pid_params
    
    % 学习参数设置
    learning_rate_pid = [0.01 0.01 1];    % PID参数学习率
    learning_rate_rbf = 0.06;             % RBF网络学习率
    momentum = 0.3;                       % 动量因子
    gaussian_width = 5;                   % 高斯函数宽度
    
    if t == 0
        % 初始化参数
        pid_params = [0.1 0.2 0.1];       % PID参数初值[Kp Ki Kd]
        % 初始化高斯基函数中心
        centers = [linspace(-1,1,6);      % 误差范围
                  linspace(0,1,6);        % 积分范围
                  linspace(0,1,6)];       % 微分范围
        hidden_output = zeros(6,1);       % 隐层输出
        weights = zeros(6,1);             % 网络权值
        weights_prev = weights;           % 上一次权值
        weights_prev2 = weights_prev;     % 上上次权值
    end

    % 计算PID控制器输出
    control_output = pid_params*x;
    
    % RBF网络输入
    rbf_input = [control_output u(3) u(4)]';

    % 计算隐层输出
    for j = 1:6
        hidden_output(j) = exp(-norm(rbf_input - centers(:,j))^2/(2*gaussian_width^2));
    end
    
    % 计算网络输出
    network_output = weights'*hidden_output;
    
    % 更新网络权值
    delta_weights = learning_rate_rbf*(u(3) - network_output)*hidden_output;
    weights = weights_prev + delta_weights + momentum*(weights_prev - weights_prev2);

    % 计算雅可比矩阵
    jacobian = weights.*hidden_output.*(-rbf_input(1) + centers(1,:))'/gaussian_width^2;   
    delta_output = sum(jacobian);
    
    % 更新PID参数(限制为非负)
    pid_params = max(pid_params + u(1) * delta_output * x' .* learning_rate_pid, 0);
    
    % 保存历史权值
    weights_prev2 = weights_prev;
    weights_prev = weights;
    
    % 输出结果
    sys = [control_output;network_output;pid_params(:)];