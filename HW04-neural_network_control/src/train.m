% 使用matlab训练模型
function train(level)
    % 输入：
    % level - 指定数据级别，1表示level2，2表示level3等

    % 根据level加载相应的数据文件
    if level == 2
        dataFile = 'data_level2.mat';
    elseif level == 3
        dataFile = 'data_level3.mat';
    else
        error('Unsupported level: %d. Only level 2 and level 3 are supported.', level);
    end
    
    % 加载数据
    load(dataFile);  % 导入数据

    % 根据 level 动态选择相应的数据变量
    if level == 2
        x = features_level2';  % 转置为符合网络输入格式
        t = labels_level2';  % 转置为符合网络目标格式
    elseif level == 3
        x = features_level3';  % 转置为符合网络输入格式
        t = labels_level3';  % 转置为符合网络目标格式
    end

    % 选择训练函数
    trainFcn = 'trainlm';  % Levenberg-Marquardt 反向传播

    % 创建拟合网络
    hiddenLayerSize = 30;
    net = fitnet(hiddenLayerSize, trainFcn);

    % 输入输出的预处理函数
    net.input.processFcns = {'removeconstantrows', 'mapminmax'};
    net.output.processFcns = {'removeconstantrows', 'mapminmax'};

    % 设置数据的划分方式
    net.divideFcn = 'dividerand';  % 随机划分数据
    net.divideMode = 'sample';  % 划分所有样本
    net.divideParam.trainRatio = 70/100;
    net.divideParam.valRatio = 15/100;
    net.divideParam.testRatio = 15/100;

    % 选择性能函数
    net.performFcn = 'mse';  % 均方误差

    % 选择绘图函数
    net.plotFcns = {'plotperform', 'plottrainstate', 'ploterrhist', ...
        'plotregression', 'plotfit'};

    % 训练网络
    [net, tr] = train(net, x, t);

    % 测试网络
    y = net(x);
    e = gsubtract(t, y);
    performance = perform(net, t, y);

    % 重新计算训练、验证和测试性能
    trainTargets = t .* tr.trainMask{1};
    valTargets = t .* tr.valMask{1};
    testTargets = t .* tr.testMask{1};
    trainPerformance = perform(net, trainTargets, y);
    valPerformance = perform(net, valTargets, y);
    testPerformance = perform(net, testTargets, y);

    % 确保 figures 文件夹存在
%     if ~exist('figures', 'dir')
%         mkdir('figures');
%     end

    % 绘制并保存训练性能图
%     figure, plotperform(tr);
%     saveas(gcf, ['figures/plotperform_level' num2str(level) '.fig']);  % 保存为 .fig 文件

    % 绘制并保存训练状态图
%     figure, plottrainstate(tr);
%     saveas(gcf, ['figures/plottrainstate_level' num2str(level) '.fig']);  % 保存为 .fig 文件

    % 绘制并保存误差直方图
%     figure, ploterrhist(e);
%     saveas(gcf, ['figures/ploterrhist_level' num2str(level) '.fig']);  % 保存为 .fig 文件

    % 绘制并保存回归图
%     figure, plotregression(t, y);
%     saveas(gcf, ['figures/plotregression_level' num2str(level) '.fig']);  % 保存为 .fig 文件

    % 绘制并保存拟合图
%     figure, plotfit(net, x, t);
%     saveas(gcf, ['figures/plotfit_level' num2str(level) '.fig']);  % 保存为 .fig 文件

    % 保存模型
%     modelFile = ['model_level' num2str(level) '.mat'];
%     save(modelFile, 'net');  % 保存神经网络模型
    gensim(net);
end
