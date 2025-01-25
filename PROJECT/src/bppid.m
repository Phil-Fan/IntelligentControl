function [sys,x0,str,ts,simStateCompliance] = bppid(t,x,u,flag,T,nh,xite,alfa)
    % 输入参数:
    % nh: 隐含层神经元数量 (hidden_neurons)
    % xite: 学习率 (learning_rate)
    % alfa: 动量因子 (momentumfactor)
    % T: 采样时间 (sampling_time)
    % 这些参数已经使用mask方法在simulink中设置
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(T,nh); % 初始化函数
        case 1
            sys = mdlDerivatives(t, x, u);
        case 2
            sys = mdlUpdate(t, x, u);
        case 3
            sys = mdlOutputs(t,x,u,nh,xite,alfa); % 输出函数
        case 9
            sys = mdlTerminate(t, x, u);
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(T,nh)
    % 调用初始化函数
    % 输入参数:
    %   T  - 采样时间
    %   nh - 隐含层神经元个数
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    % 输出维数:
    % - 控制量u(1个)
    % - PID参数Kp,Ki,Kd(3个) 
    % - 隐含层权值(nh×3=3nh个)
    % - 输出层权值(3×nh=3nh个)
    % 总共:1+3+3nh+3nh=4+6nh个
    sizes.NumOutputs     = 4+6*nh;  
    % 输入维数:
    % - 系统状态量[e(k),e(k-1),e(k-2),y(k),y(k-1),r(k),u(k-1)](7个)
    % - 前两次隐含层权值(2×nh×3=6nh个)
    % - 前两次输出层权值(2×3×nh=6nh个)
    % 总共:7+6nh+6nh=7+12nh个
    sizes.NumInputs      = 7+12*nh;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0  = [];
    str = [];
    ts  = [T 0];
    simStateCompliance = 'UnknownSimState';
end

function sys = mdlOutputs(~,~,u,nh,xite,alfa)
    % 如果权值未初始化,则随机初始化
    if any(isnan(u(8:7+12*nh)))
        hiddenWeights_prev2 = 0.5 * randn(nh,3) - 0.25; % 隐含层前两次权值(k-2)
        outputWeights_prev2 = 0.5 * randn(3,nh) - 0.25; % 输出层前两次权值(k-2) 
        hiddenWeights_prev1 = 0.5 * randn(nh,3) - 0.25; % 隐含层前一次权值(k-1)
        outputWeights_prev1 = 0.5 * randn(3,nh) - 0.25; % 输出层前一次权值(k-1)
    else
        hiddenWeights_prev2 = reshape(u(8:7+3*nh),nh,3);      % 隐含层前两次权值矩阵(nh×3)
        outputWeights_prev2 = reshape(u(8+3*nh:7+6*nh),3,nh); % 输出层前两次权值矩阵(3×nh)
        hiddenWeights_prev1 = reshape(u(8+6*nh:7+9*nh),nh,3); % 隐含层前一次权值矩阵(nh×3)
        outputWeights_prev1 = reshape(u(8+9*nh:7+12*nh),3,nh);% 输出层前一次权值矩阵(3×nh)
    end
    
    % 神经网络输入
    networkInput = [u(6),u(4),u(1)];  % [r(k),y(k),e(k)]
    pidInput = [u(1)-u(2);u(1);u(1)+u(3)-2*u(2)];  % [e(k)-e(k-1);e(k);e(k)+e(k-2)-2*e(k-1)]
    
    % 计算隐含层输入
    hiddenInput = networkInput * hiddenWeights_prev1';  % 维数:1×nh
    
    % 限制隐含层输入范围,防止溢出
    for i = 1:length(hiddenInput)
        hiddenInput(i) = min(max(hiddenInput(i), -500), 500);
    end
    
    % 隐含层输出(使用Sigmoid激活函数)
    hiddenOutput = exp(hiddenInput)./(exp(hiddenInput)+exp(-hiddenInput));
    % hiddenOutput = max(0, hiddenInput);  % 可选用ReLU激活函数
    
    % 输出层计算
    outputLayerInput = outputWeights_prev1 * hiddenOutput';  % 维数:3×1
    
    % PID参数计算(使用Sigmoid导数作为激活函数)
    pidParams = 2./(exp(outputLayerInput)+exp(-outputLayerInput)).^2;
    pidParams = pidParams/100;  % 缩放PID参数
    % 计算控制量
    controlOutput = u(7) + pidParams' * pidInput;
    
    % 计算梯度
    gradientSign = sign((u(4)-u(5))/(controlOutput-u(7)+0.0000001));
    pidParamsGradient = 2./(exp(pidParams)+exp(-pidParams)).^2;
    
    % 下面是BP神经网络的权值更新部分
    % 计算delta项
    delta = u(1) * gradientSign * pidInput .* pidParamsGradient;
    
    % 更新权值
    outputWeights = outputWeights_prev1 + xite*delta*hiddenOutput + alfa*(outputWeights_prev1-outputWeights_prev2);
    
    % 计算隐含层梯度
    hiddenGradient = 2./(exp(hiddenOutput)+exp(-hiddenOutput)).^2;
    % hiddenGradient = double(hiddenInput > 0);  % ReLU导数
    
    % 更新隐含层权值
    hiddenWeights = hiddenWeights_prev1 + xite*(hiddenGradient.*(delta'*outputWeights))'*networkInput + alfa*(hiddenWeights_prev1-hiddenWeights_prev2);
    
    % 整合输出
    sys = [controlOutput;pidParams(:);hiddenWeights(:);outputWeights(:)];
end

function sys = mdlDerivatives(~,~,~)
    sys = [];
end

function sys = mdlUpdate(~, ~, ~)
    sys = [];
end

function sys = mdlTerminate(~, ~, ~)
    sys = [];
end