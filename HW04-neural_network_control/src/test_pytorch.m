% 验证加载pytorch训练模型的可行性

% 加载 ONNX 模型
% 添加 InputDataFormats 参数来指定输入格式
model = importONNXNetwork('model_level2.onnx', ...
    'OutputLayerType', 'regression', ...
    'InputDataFormats', 'BC');  % B=batch size, C=channels

% 示例输入
u = [1.1; 1.1; 1.1; 1.1; 2.2; 2.2; 2.2; 2.2];
u = reshape(u, [1, 8]); % 将输入调整为 [1, 8]

% 预测
y = predict(model, u);

% 显示预测结果
disp('预测输出:');
disp(y);
