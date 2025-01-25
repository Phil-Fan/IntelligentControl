# 神经网络辨识作业

## 环境配置

需要先安装 Deep Learning Toolbox 和 Simulink 工具箱

完成我的额外实验还需要安装 pytorch 库 和 Deep Learning Toolbox Converter for ONNX Model Format库


## 文件介绍


```
│  README.md
│
├─report						// LaTeX报告文件夹
│  │  ref.bib
│  │  thesis.tex
│  │
│  └─figures
│
└─src
    │  arm.m                    // 2-DOF机械臂的simulink模型
    │  data_level2.mat			// 生成的训练所需的数据（运行s-function自动生成）
    │  data_level3.mat
    │  model.slx                //仿真文件
    │  train.m                  // 使用matlab训练模型
    |
    │  train_pytorch.py         // 使用pytorch训练模型
    │  test_pytorch.m           // 使用pytorch训练的模型进行测试
    │  model_level2.onnx        // 使用pytorch训练的模型文件
    │
    └─learn_toolbox             // 神经网络工具箱的学习过程以及数据文件
            test_multi.m
            test_train.m
            test_train.mat
            untitled19.m
            神经网络案例——辛烷值和光谱分析.xlsx
```