# 使用pytorch训练模型
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import scipy.io as sio

# Step 1: Load Data
data = sio.loadmat('data_level2.mat')
inputs = torch.tensor(data['features_level2'], dtype=torch.float32)
targets = torch.tensor(data['labels_level2'], dtype=torch.float32)

# Step 2: Prepare Dataset and DataLoader
dataset = TensorDataset(inputs, targets)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# Step 3: Define the Model
class RoboticArmNet(nn.Module):
    def __init__(self, input_size, output_size):
        super(RoboticArmNet, self).__init__()
        # 简单的三层网络结构
        self.net = nn.Sequential(
            nn.Linear(input_size, 32),
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, output_size)
        )
        
        # 初始化权重
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)
                nn.init.zeros_(m.bias)

    def forward(self, x):
        return self.net(x)

# Initialize the model
input_size = inputs.shape[1]  # 8
output_size = targets.shape[1]  # 2
print(input_size, output_size)
model = RoboticArmNet(input_size, output_size)

# Step 4: Define Loss and Optimizer
criterion = nn.MSELoss()
# 使用Adam优化器，通常收敛更快且效果更好
optimizer = optim.Adam(model.parameters(), lr=0.001)
# 学习率调度器
scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)

# Step 5: 划分训练集和测试集
total_size = len(dataset)
train_size = int(0.8 * total_size)
test_size = total_size - train_size
train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size, test_size])

train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# Step 6: 训练模型
epochs = 200  # 增加训练轮次
best_loss = float('inf')
for epoch in range(epochs):
    model.train()
    epoch_loss = 0
    for batch_inputs, batch_targets in train_loader:
        # 前向传播
        outputs = model(batch_inputs)
        loss = criterion(outputs, batch_targets)
        epoch_loss += loss.item()

        # 反向传播和优化
        optimizer.zero_grad()
        loss.backward()
        # 梯度裁剪，防止梯度爆炸
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
    
    avg_loss = epoch_loss / len(train_loader)
    scheduler.step(avg_loss)  # 更新学习率
    
    if avg_loss < best_loss:
        best_loss = avg_loss
        torch.save(model.state_dict(), 'best_model_level2.pth')

    if (epoch + 1) % 10 == 0:
        print(f'训练轮次 [{epoch + 1}/{epochs}], 平均损失: {avg_loss:.4f}')

# Step 7: 测试模型
model.eval()
test_loss = 0
with torch.no_grad():
    for inputs, targets in test_loader:
        outputs = model(inputs)
        test_loss += criterion(outputs, targets).item()
        
avg_test_loss = test_loss / len(test_loader)
print(f'测试集平均损失: {avg_test_loss:.4f}')

# 计算关节角度误差（以度为单位）
model.eval()
total_angle_error = 0
with torch.no_grad():
    for inputs, targets in test_loader:
        outputs = model(inputs)
        # 假设输出是弧度，转换为角度
        angle_error = torch.abs(outputs - targets) * 180 / 3.14159
        total_angle_error += torch.mean(angle_error).item()

avg_angle_error = total_angle_error / len(test_loader)
print(f'平均关节角度误差: {avg_angle_error:.2f}度')

# Step 8: 保存模型
torch.save(model.state_dict(), 'model_level2.pth')
torch.onnx.export(model, torch.randn(1, input_size), 'model_level2.onnx', input_names=['input'], output_names=['output'])
