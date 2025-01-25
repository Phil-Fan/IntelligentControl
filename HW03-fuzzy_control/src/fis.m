clear;
close all;

% % 创建模糊推理系统，使用 newfis
% fuzzyController = mamfis( ...
%     'NumInputs',1,'NumInputMFs',2,...
%     'NumOutputs',1,'NumOutputMFS',2,...
%     'AddRule','none');
% 
% % 定义输入变量x（位置误差）及其隶属度函数，范围 [-0.04, 0.04]
% fuzzyController.Inputs(1).Name = 'x';
% fuzzyController.Inputs(1).Range = [-0.04 0.04];
% 
% fuzzyController = addMF(fuzzyController, 'input', 1, 'EN', 'gaussmf', [0.01 -0.04]); % EN: Extremely Negative
% fuzzyController = addMF(fuzzyController, 'input', 1, 'VN', 'gaussmf', [0.01 -0.03]); % VN: Very Negative
% fuzzyController = addMF(fuzzyController, 'input', 1, 'N', 'gaussmf', [0.01 -0.02]);  % N: Negative
% fuzzyController = addMF(fuzzyController, 'input', 1, 'NZ', 'gaussmf', [0.01 -0.01]); % Z: Zero
% fuzzyController = addMF(fuzzyController, 'input', 1, 'PZ', 'gaussmf', [0.01 0.01]);  % Z: Zero
% fuzzyController = addMF(fuzzyController, 'input', 1, 'P', 'gaussmf', [0.01 0.02]);   % P: Positive
% fuzzyController = addMF(fuzzyController, 'input', 1, 'VP', 'gaussmf', [0.01 0.03]);  % VP: Very Positive
% fuzzyController = addMF(fuzzyController, 'input', 1, 'EP', 'gaussmf', [0.01 0.04]);  % EP: Extremely Positive
% 
% % 可视化x的隶属度函数
% figure;
% subplot(311);
% plotmf(fuzzyController, 'input', 1);
% title('位置误差 (x) 的隶属度函数');
% 
% % 定义输入变量dx/dt（位置误差变化率）及其隶属度函数，范围 [-0.5, 0.5]
% fuzzyController.Inputs(2).Name = 'dx';
% fuzzyController.Inputs(2).Range = [-0.5 0.5];
% fuzzyController = addMF(fuzzyController, 'input', 2, 'EN', 'gaussmf', [0.1 -0.5]);  % EN: Extremely Negative
% fuzzyController = addMF(fuzzyController, 'input', 2, 'VN', 'gaussmf', [0.1 -0.4]);  % VN: Very Negative
% fuzzyController = addMF(fuzzyController, 'input', 2, 'N', 'gaussmf', [0.1 -0.3]);  % N: Negative
% fuzzyController = addMF(fuzzyController, 'input', 2, 'NZ', 'gaussmf', [0.1 -0.1]);     % Z: Zero
% fuzzyController = addMF(fuzzyController, 'input', 2, 'PZ', 'gaussmf', [0.1 0.1]);     % Z: Zero
% fuzzyController = addMF(fuzzyController, 'input', 2, 'P', 'gaussmf', [0.1 0.3]);   % P: Positive
% fuzzyController = addMF(fuzzyController, 'input', 2, 'VP', 'gaussmf', [0.1 0.4]);   % VP: Very Positive
% fuzzyController = addMF(fuzzyController, 'input', 2, 'EP', 'gaussmf', [0.1 0.5]);   % EP: Extremely Positive
% 
% % 可视化dx/dt的隶属度函数
% 
% subplot(312);
% plotmf(fuzzyController, 'input', 2);
% title('位置误差变化率 (dx/dt) 的隶属度函数');
% 
% % 定义输出变量U（控制电压）的隶属度函数，范围 [-10, 10]
% fuzzyController = addvar(fuzzyController, 'output', 'U', [-10 10]);
% fuzzyController = addMF(fuzzyController, 'output', 1, 'EL', 'trimf', [-10 -10 -7]);  % EL: Extremely Low
% fuzzyController = addMF(fuzzyController, 'output', 1, 'VL', 'trimf', [-10 -7 -4]);  % VL: Very Low
% fuzzyController = addMF(fuzzyController, 'output', 1, 'L', 'trimf', [-7 -4 -1]);   % L: Low
% fuzzyController = addMF(fuzzyController, 'output', 1, 'NZ', 'trimf', [-3 -1 1]);  % NZ:negetive ZERO
% fuzzyController = addMF(fuzzyController, 'output', 1, 'PZ', 'trimf', [-1 1 3]);  % M: positive ZERO
% fuzzyController = addMF(fuzzyController, 'output', 1, 'H', 'trimf', [1 4 7]);  % H: High
% fuzzyController = addMF(fuzzyController, 'output', 1, 'VH', 'trimf', [4 7 10]); % VH: Very High
% fuzzyController = addMF(fuzzyController, 'output', 1, 'EH', 'trimf', [7 10 10]); % EH: Extremely High
% 
% % 可视化U的隶属度函数
% 
% subplot(313);
% plotmf(fuzzyController, 'output', 1);
% title('控制电压 (U) 的隶属度函数');

fuzzyController = readfis('Controller4.fis');

table = [
 [1, 1, 1, 1, 6, 5, 5, 5],
 [1, 2, 2, 1, 7, 5, 5, 5],
 [2, 2, 2, 1, 2, 6 ,6, 6],
 [2, 3, 3, 2, 7, 6, 6, 6],
 [3, 3, 3, 2, 8, 7, 6, 7],
 [3, 3, 4, 2,8, 7, 7, 7],
 [3, 4, 4, 2, 8, 8, 7, 8],
 [4, 4, 4, 4, 8, 8, 8, 8]];
% 生成规则矩阵
rules = [];
for i = 1:8
    for j = 1:8
        disp(table(i,j));
        output = table(i,j);
        rules = [rules; i j  output 1 1]; % 1 1表示使用 'min' 合成和 'centroid' 解模糊
    end
end

% disp(rules)

% 添加规则到模糊系统
fuzzyController = addrule(fuzzyController, rules);
%showrule(fuzzyController,'Format','symbolic');
% 可视化规则
ruleview(fuzzyController);

%figure;
%plotfis(fuzzyController);

% 输出 surface
figure;
gensurf(fuzzyController,[1,2],1);
saveas(gcf, 'surf.jpg');



% 保存为.fis文件
writefis(fuzzyController, 'Controller41.fis');

testInputs = [0.01, -0.25];  % 例如，x=0.01, dx/dt=-0.25
output = evalfis(fuzzyController,testInputs);
disp(['控制电压U的输出：', num2str(output)]);
