# 倒立摆控制系统

>Author: PhilFan
>Date: 2024-11-30 to 2024-12-03

## 项目文件说明

```shell
.
├── expert_control.prj     # 项目文件
|   ├── exp.slx                # 专家控制系统模型
|   ├── expert_control.m       # 专家控制器主程序
|   ├── critic.m              # 系统评价器
|   └── balance_car.m         # 倒立摆小车模型
└── README.md             # 项目说明文档
```

## 题目描述

![](https://philfan-pic.oss-cn-beijing.aliyuncs.com/img/20241115215028.png)

如图所示为车载倒立摆系统，一辆小车在水平轨道上移动，小车上有一个可绕固定点转动的倒立摆。控制小车在水平方向的移动可使摆杆维持直立不倒，这和手掌移动可使直立木棒不倒的现象类似。

忽略车轮与地面的摩擦力等阻力，可推导出车载倒立摆的动力学方程如下：
$$
\begin{cases}
(M + m) \ddot{x} + m l (\ddot{\theta} \cos \theta + ml \dot{\theta}^2 \sin \theta) = F \\
ml^2 \ddot{\theta} + ml \ddot{x} \cos \theta - mgl \sin \theta = 0
\end{cases}
$$

其中的参数如表所示：

| 参数 | 大小 |
| --- | --- |
| 摆杆质量 $m$ | 0.5kg |
| 小车质量 $M$ | 1kg |
| 摆杆转动轴心到摆杆质心的长度 $l$ | 0.5m |
| 摆杆与垂直向上方向的夹角 $\theta$ | $[0, \pi]$ rad |
| 重力加速度 $g$ | 9.8m/s² |
| 施加在小车上的水平外力 $F$ | $[-F_m, F_m]$ N |
| 小车在水平方向的位移 $x$ | 不限制 |

增量型离散PID控制算法如下：

$$
F(k) = F(k-1) + K \left[ K_p \Delta \theta (k) + \frac{T}{T_i} \theta (k) + \frac{T}{T_d} (\Delta \theta (k) - \Delta \theta (k-1)) \right]
$$

其中 $T$ 为采样时间，$\Delta \theta (k) = \theta (k) - \theta (k-1)$

若 $F_m = 25$，取 $T = 0.0001s$，$K_p = 200$，$K_i = 3$，$K_d = 10$

设计 $0 < \theta_1 < \theta_2 < \theta_m$，$0 < K_s < 1 < K_b$


在离散PID控制基础上，采用专家PID控制方案，规则如下：

1. 若 $|\theta (k)| \geq \theta_m$ 时，则 $F(k) = \text{sgn}(\theta) F_m$
2. 若 $\theta_2 \leq |\theta (k)| < \theta_m$ 时，
    1. 若 $\theta (k) \Delta \theta (k) > 0$ 时，则 $K = K_b$
    2. 若 $\theta (k) \Delta \theta (k) < 0$ 时，
        a. 若 $\Delta \theta (k) \Delta \theta (k-1) > 0$ 时，则 $K = 1$
        b. 若 $\Delta \theta (k) \Delta \theta (k-1) < 0$ 时，则 $K = K_b$
3. 若 $\theta_1 \leq |\theta (k)| < \theta_2$ 时，
    1. 若 $\theta (k) \Delta \theta (k) > 0$ 时，则 $K = 1$
    2. 若 $\theta (k) \Delta \theta (k) < 0$ 时，
        a. 若 $\Delta \theta (k) \Delta \theta (k-1) > 0$ 时，则 $K = K_s$
        b. 若 $\Delta \theta (k) \Delta \theta (k-1) < 0$ 时，则 $K = 1$
4. 若 $|\theta (k)| < \theta_1$ 时，则 $K = 1$

若小车和摆杆静止，摆杆与垂直向上方向的初始夹角 $\theta(0) = \frac{\pi}{4} \text{ rad}$，请：

1. 给出上述专家PID控制方案的合适参数 $\theta_1, \theta_2, \theta_m$ 和 $K_s, K_b$，通过调节 $F$ 使倒立摆的摆杆夹角 $\theta$ 恢复并维持在期望值（$\theta_d = 0$），在 matlab 中进行仿真，给出位移 $x$、夹角 $\theta$ 和水平力 $F$ 的变化曲线，并比较专家PID控制与常规PID控制的结果（可尝试参数 $\theta_1 = 0.1, \theta_2 = 0.3, \theta_m = 0.5$ 和 $K_s = 1, K_b = 1.3$）。

2. 针对不同的初始夹角 $\theta(0)$，给出专家PID控制的结果。（可能需要调整相关参数 $\theta_1, \theta_2, \theta_m$ 和 $K_s, K_b$）



## 思路整理

经过搜索和阅读题目，我认为本题可以分三个步骤来完成

- 建立小车倒立摆的物理模型
- 建立普通PID和专家PID的控制器
- 优化参数

对于第一个步骤，由于普通的状态空间模型不能表达非线性模型，所以这里我们采用了S-function进行表达

普通PID使用了Matlab中自带的PID模块。

专家PID则使用题干中的控制策略，使用S-function构造一个输入是$\theta$，状态变量是$[\theta, \frac{d\theta}{dt}, \theta_{last}, error_{sum}, error_{last}]$,输出变量是$F$的控制器。

对于调参来讲，我们先使用题目中推荐的参数进行测试，开始使用了观察法，比较普通PID和专家PID的控制效果；

之后我参考机器学习中的损失函数，设计了基于MSE和最小超调$theta$值的$cost\ funtion$，并设计了critic评分器，用来比较PID和专家PID的实验结果.

同时，代码增加了详细的注释和说明，并添加了版本管理系统。


## 版本更新记录

| 版本号 | 更新日期 | 更新内容 | 备注 |
|--------|----------|----------|------|
| v1.0.0 | 2024-11 | • 实现基础倒立摆控制系统<br>• 包含专家控制器 (expert_control.m)<br>| 初始版本 |
| v1.1.0 | 2024-11 | 增加评价器功能 (critic.m)，用于判断模型的好坏<br> 改进控制器性能参数 | 性能优化 |
| v1.2.0 | 2024-12-01 | 优化评价器评估方法<br>提升系统稳定性，增加对比 | 参数调节 |
| v1.2.1 | 2024-12-03 | 优化代码逻辑，增加注释和说明文档| Docs优化 |


## 使用说明

1. 运行 `expert_control.prj ` 启动控制项目
2. 运行 `exp.slx` 启动控制系统

## 注意事项

- 运行前请确保MATLAB已安装Simulink工具箱
- 建议使用MATLAB R2022b或更高版本
- 参数调整时请注意保存原始配置

## 可能的优化方案

- [ ] 增加与传统PID的对比
- [ ] 增加根据critic函数自动优化求解的方法
- [ ] 增加3D animation界面
- [ ] 自动化地调节PID参数，或者通过可视化地调节；目前调节参数只能调完一次重新运行，效率比较低
- [ ] 完善系统文档
