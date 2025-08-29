# 视觉伺服系统使用指南

## 概述

本项目实现了两种主要的视觉伺服方法：
- **PBVS (Position-Based Visual Servoing)**: 基于位置的视觉伺服
- **IBVS (Image-Based Visual Servoing)**: 基于图像的视觉伺服

系统集成了YOLO目标检测和RealSense深度相机，可以实现对目标物体的精确视觉伺服控制。

## 文件结构

```
vision/
├── visual_servoing.py          # 基础视觉伺服实现
├── advanced_visual_servoing.py # 高级视觉伺服系统
├── camera_calibration.py      # 相机标定工具
├── yolo_detect.py             # YOLO检测（原有）
├── yolo_train.py              # YOLO训练（原有）
└── visual_servoing_guide.md   # 使用指南
```

## 环境要求

### 硬件要求
- Intel RealSense 深度相机 (D435/D455等)
- 标准棋盘格标定板 (9x6内角点，方格尺寸2.5cm)

### 软件依赖
```bash
pip install opencv-python
pip install pyrealsense2
pip install ultralytics
pip install scipy
pip install numpy
```

## 使用步骤

### 1. 相机标定

首先需要对相机进行标定以获得准确的内参矩阵：

```bash
python camera_calibration.py
```

标定流程：
1. 选择 "1. 运行完整标定流程"
2. 准备9x6内角点的标准棋盘格
3. 在不同角度和距离拍摄20张棋盘格图像
4. 系统自动计算相机内参并保存到 `camera_calibration.json`

### 2. 训练YOLO模型

使用您的目标物体数据集训练YOLO模型：

```bash
python yolo_train.py
```

训练完成后，模型权重保存在 `yolo_train/weights/best.pt`

### 3. 运行视觉伺服

#### 基础版本
```bash
python visual_servoing.py
```

#### 高级版本（推荐）
```bash
python advanced_visual_servoing.py
```

选择运行模式：
- 1: PBVS (基于位置的视觉伺服)
- 2: IBVS (基于图像的视觉伺服)
- 3: 相机标定

## 视觉伺服方法对比

### PBVS (基于位置的视觉伺服)

**原理:**
- 使用相机获取目标的3D位置和姿态
- 在笛卡尔空间中定义误差和控制律
- 控制机器人末端执行器到达期望的3D位置

**优点:**
- 直观易懂，误差定义在3D空间
- 轨迹规划简单
- 对相机标定精度要求相对较低

**缺点:**
- 需要目标的3D模型
- 对深度测量精度要求高
- 可能出现局部最小值问题

**控制律:**
```
v = -λ * (s - s*)
```
其中 s = [x, y, z, rx, ry, rz] 为当前位姿，s* 为期望位姿

### IBVS (基于图像的视觉伺服)

**原理:**
- 直接使用图像特征作为反馈
- 在图像空间中定义误差和控制律
- 通过图像雅可比矩阵将图像误差转换为机器人运动

**优点:**
- 不需要精确的3D模型
- 对相机标定误差不敏感
- 收敛性好

**缺点:**
- 轨迹可能不是直线
- 图像雅可比矩阵计算复杂
- 可能出现奇异性问题

**控制律:**
```
v = -λ * L^+ * (s - s*)
```
其中 L 为图像雅可比矩阵，s 为当前图像特征，s* 为期望图像特征

## 参数配置

### 控制参数
- `lambda_pos`: 位置控制增益 (推荐值: 0.5)
- `lambda_rot`: 旋转控制增益 (推荐值: 0.3)
- `max_velocity`: 最大线速度 (推荐值: 0.1 m/s)
- `max_angular_velocity`: 最大角速度 (推荐值: 0.5 rad/s)

### 期望状态设置
- PBVS: 修改 `desired_position` 和 `desired_orientation`
- IBVS: 修改 `desired_features` (图像坐标和尺寸)

## 实际应用示例

### 机器人抓取任务
1. 使用YOLO检测目标物体
2. 使用PBVS将机器人移动到目标上方
3. 执行抓取动作

### 目标跟踪任务
1. 使用IBVS保持目标在图像中心
2. 维持固定的观察距离和角度

## 故障排除

### 常见问题

1. **目标检测不稳定**
   - 检查光照条件
   - 重新训练YOLO模型
   - 调整检测阈值

2. **相机标定精度低**
   - 使用更多标定图像
   - 确保棋盘格尺寸准确
   - 在不同角度和距离拍摄

3. **控制震荡**
   - 降低控制增益
   - 增加死区范围
   - 检查相机帧率

4. **深度信息不准确**
   - 检查目标表面反射特性
   - 调整RealSense曝光参数
   - 使用多点深度平均

### 调试技巧

1. **可视化调试**
   - 观察检测框和期望位置
   - 监控误差和控制量变化
   - 记录数据进行离线分析

2. **参数调优**
   - 从小增益开始逐步增大
   - 测试不同的死区设置
   - 验证速度限制是否合理

## 扩展功能

### 多目标伺服
可以扩展系统支持多个目标的同时伺服控制

### 动态目标跟踪
添加卡尔曼滤波器预测目标运动

### 机器人接口
连接实际机器人控制器（如ROS、UR机器人等）

## 参考文献

1. Chaumette, F., & Hutchinson, S. (2006). Visual servo control. IEEE Robotics & Automation Magazine.
2. Corke, P. (2017). Robotics, vision and control: fundamental algorithms In MATLAB.
3. Marchand, E., Spindler, F., & Chaumette, F. (2005). ViSP for visual servoing: a generic software platform.

## 联系方式

如有问题或建议，请通过GitHub Issues或邮件联系。
