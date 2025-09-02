# 机器人视觉伺服系统

基于robotcontrol.py的PBVS和IBVS视觉伺服系统，整合了YOLO目标检测和RealSense深度相机。

## 目录

1. [系统概述](#系统概述)
2. [环境设置](#环境设置)
3. [代码文件说明](#代码文件说明)
4. [功能特性](#功能特性)
5. [快速开始](#快速开始)
6. [视觉伺服方法](#视觉伺服方法)
7. [系统架构](#系统架构)
8. [参数配置](#参数配置)
9. [使用指南](#使用指南)
10. [故障排除](#故障排除)
11. [API参考](#api参考)
12. [扩展开发](#扩展开发)

## 系统概述

本系统实现了两种主要的视觉伺服方法：

- **PBVS (Position-Based Visual Servoing)**: 基于位置的视觉伺服 - 在3D空间中进行精确位置控制
- **IBVS (Image-Based Visual Servoing)**: 基于图像的视觉伺服 - 直接在图像空间中进行伺服控制

系统集成了Auboi5机器人控制、YOLO目标检测和RealSense深度相机，可以实现对目标物体的精确视觉伺服控制。

## 环境设置

### 已完成的设置

✅ **Miniconda已安装**: `/home/wang-yb/miniconda3/`  
✅ **conda环境已创建**: `vision` (Python 3.8)  
✅ **依赖包已安装**: 所有requirements.txt中的包  
✅ **环境测试通过**: 所有关键依赖包可正常导入  

### 快速启动环境

#### 方法1：使用设置脚本（推荐）

```bash
# 在项目目录中运行
./setup_environment.sh
```

#### 方法2：手动激活环境

```bash
# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vision

# 验证环境
python --version  # 应该显示Python 3.8.x
which python      # 应该显示conda环境中的Python路径
```

### 环境信息

- **环境名称**: `vision`
- **Python版本**: 3.8.20
- **conda位置**: `/home/wang-yb/miniconda3/`
- **环境位置**: `/home/wang-yb/miniconda3/envs/vision/`

### 已安装的核心依赖

- `opencv-python>=4.5.0` - 计算机视觉库
- `numpy>=1.21.0` - 数值计算库
- `pyrealsense2>=2.50.0` - RealSense相机SDK
- `ultralytics>=8.0.0` - YOLO目标检测
- `torch>=1.9.0` - PyTorch深度学习框架
- `torchvision>=0.10.0` - PyTorch视觉工具
- `scipy>=1.7.0` - 科学计算库
- `matplotlib>=3.3.0` - 绘图库
- `Pillow>=8.3.0` - 图像处理库

### 避免ROS2冲突

- 使用独立的conda环境 `vision`
- 与系统Python和ROS2 Python完全隔离
- 每个环境有独立的包管理

## 代码文件说明

### 核心驱动程序

#### 1. `robot_visual_servoing_integrated.py`
**主要功能**: 整合的视觉伺服驱动系统
- **作用**: 系统的主入口，集成了PBVS和IBVS两种视觉伺服方法
- **核心类**: `RobotVisualServoing` - 完整的视觉伺服控制系统
- **主要功能**:
  - 机器人连接和控制
  - RealSense相机管理
  - YOLO目标检测
  - PBVS和IBVS控制算法
  - 实时可视化和用户交互
- **运行方式**: `python robot_visual_servoing_integrated.py`
- **支持模式**: 
  - 模式1: PBVS（基于位置的视觉伺服）
  - 模式2: IBVS（基于图像的视觉伺服）

#### 2. `robotcontrol.py`
**主要功能**: Auboi5机器人控制接口
- **作用**: 提供与Auboi5机械臂的底层通信接口
- **核心类**: `Auboi5Robot` - 机器人控制器
- **主要功能**:
  - 机器人连接和初始化
  - 运动控制（关节空间和笛卡尔空间）
  - 安全监控和错误处理
  - 日志记录和状态监控
- **依赖库**: `libpyauboi5` - Auboi5机器人SDK
- **使用场景**: 被其他模块调用，不直接运行

### 仿真系统模块

#### 3. `robot_visual_servoing_simulation.py`
**主要功能**: 集成视觉伺服仿真系统
- **作用**: 支持仿真和真实机械臂模式切换的视觉伺服系统
- **核心类**: `RobotVisualServoing` - 统一的视觉伺服控制系统
- **主要功能**:
  - 仿真/真实模式无缝切换
  - PBVS和IBVS控制算法
  - PyBullet仿真环境集成
  - 与现有代码完全兼容
- **运行方式**: `python robot_visual_servoing_simulation.py`

#### 4. `pybullet_robot_adapter.py`
**主要功能**: PyBullet机械臂仿真适配器
- **作用**: 模拟真实Auboi5机械臂的接口，提供仿真环境
- **核心类**: `PyBulletAuboi5Robot` - 仿真机械臂控制器
- **主要功能**:
  - Auboi5六轴机械臂仿真模型
  - 底座固定到地面
  - 末端RGB-D相机仿真
  - 完全兼容robotcontrol.py接口
- **运行方式**: 作为库被其他模块导入

#### 5. `auboi5_demo.py`
**主要功能**: Auboi5机械臂运动演示
- **作用**: 展示机械臂的运动能力和工作空间
- **主要功能**:
  - 关节运动演示
  - 工作空间分析
  - 相机视图展示
- **运行方式**: `python auboi5_demo.py`

#### 7. `camera_calibration.py`
**主要功能**: 相机标定工具
- **作用**: 标定RealSense相机的内参矩阵和畸变系数
- **核心类**: `CameraCalibration` - 相机标定工具
- **主要功能**:
  - 棋盘格图像采集
  - 相机内参标定
  - 畸变系数计算
  - 标定结果保存和加载
- **输出文件**: `camera_calibration.json` - 标定参数文件
- **运行方式**: `python camera_calibration.py`
- **使用流程**:
  1. 准备9x6内角点的标准棋盘格
  2. 在不同角度拍摄20张图像
  3. 自动计算并保存标定参数

### 目标检测模块

#### 8. `yolo_detect.py`
**主要功能**: YOLO目标检测演示
- **作用**: 使用训练好的YOLO模型进行实时目标检测
- **主要功能**:
  - 加载YOLO模型
  - RealSense相机图像获取
  - 实时目标检测和显示
  - 深度信息获取
- **模型文件**: 使用 `yolo_train/weights/best.pt`
- **运行方式**: `python yolo_detect.py`
- **使用场景**: 测试YOLO模型性能，验证检测效果

#### 9. `yolo_train.py`
**主要功能**: YOLO模型训练脚本
- **作用**: 训练自定义的YOLO目标检测模型
- **主要功能**:
  - 加载预训练YOLOv5模型
  - 使用自定义数据集训练
  - 模型权重保存
- **配置文件**: 需要配置数据集路径（当前指向番茄检测数据集）
- **输出目录**: `yolo_train/` - 训练结果和权重文件
- **运行方式**: `python yolo_train.py`
- **训练参数**: 100轮训练，640像素图像尺寸

### 测试和验证模块

#### 10. `test_system.py`
**主要功能**: 系统功能测试
- **作用**: 快速测试系统各个组件的工作状态
- **主要功能**:
  - 测试相机连接
  - 测试YOLO模型加载
  - 测试机器人连接
  - 系统完整性验证
- **运行方式**: `python test_system.py --all`
- **测试项目**:
  - RealSense相机连接和图像获取
  - YOLO模型加载和推理
  - 依赖库完整性检查

#### 11. `test_simulation.py`
**主要功能**: PyBullet仿真系统测试
- **作用**: 快速测试PyBullet仿真环境和机械臂功能
- **主要功能**:
  - 检查PyBullet依赖库
  - 测试Auboi5机械臂模型
  - 测试仿真相机功能
  - 验证YOLO模型集成
- **运行方式**: `python test_simulation.py`
- **使用场景**: 验证仿真环境是否正常工作

### 配置和权重文件

#### 12. `requirements.txt`
**主要功能**: Python依赖包列表
- **作用**: 定义项目所需的所有Python包及版本
- **包含的主要包**:
  - opencv-python, numpy, scipy - 计算机视觉和数值计算
  - pyrealsense2 - RealSense相机支持
  - ultralytics - YOLO目标检测
  - torch, torchvision - 深度学习框架
  - matplotlib, Pillow - 图像处理和可视化
  - pybullet - 物理仿真引擎
- **使用方式**: `pip install -r requirements.txt`

#### 13. `setup_environment.sh`
**主要功能**: 环境设置脚本
- **作用**: 自动激活conda环境并进行环境检查
- **主要功能**:
  - 激活vision conda环境
  - 检查Python版本
  - 验证关键依赖包
- **运行方式**: `./setup_environment.sh`

#### 14. `yolov5nu.pt`
**主要功能**: 预训练YOLO模型
- **作用**: YOLOv5的预训练权重文件
- **使用场景**: 作为训练的起始点或直接用于检测

#### 15. `yolo_train/` 目录
**主要功能**: YOLO训练输出目录
- **作用**: 存储YOLO模型训练的所有结果
- **主要文件**:
  - `weights/best.pt` - 训练得到的最佳模型权重
  - `weights/last.pt` - 最后一轮的模型权重
  - `results.csv` - 训练过程数据
  - `*.png` - 训练过程可视化图表
  - `args.yaml` - 训练参数配置

## 功能特性

### ✅ 核心功能
- 基于Auboi5机器人的完整控制接口
- RealSense D435i相机支持
- YOLO v8目标检测
- 实时视觉伺服控制
- RGB-D图像对齐和深度感知

### ✅ 安全功能
- 速度限制和紧急停止
- 目标丢失检测和处理
- 碰撞检测和工作空间限制
- 实时状态监控

### ✅ 用户友好
- 简化的一体化设计
- 实时可视化显示
- 交互式参数调整
- 详细的状态反馈

## 快速开始

### 1. 环境准备
```bash
# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vision

# 或使用设置脚本
./setup_environment.sh
```

### 2. 系统测试
```bash
# 测试相机连接
python -c "import pyrealsense2 as rs; print('RealSense相机可用')"

# 测试YOLO模型
python -c "from ultralytics import YOLO; print('YOLO模型可用')"

# 运行完整系统测试
python test_system.py --all
```

### 3. 相机标定（首次使用）
```bash
python camera_calibration.py
```

### 4. 运行视觉伺服

**真实机械臂模式**:
```bash
# 运行主程序
python robot_visual_servoing_integrated.py
```

**仿真模式（推荐先测试）**:
```bash
# 运行仿真系统
python robot_visual_servoing_simulation.py

# 或使用启动脚本
./run_simulation.sh
```

### 5. 选择模式
- **模式1**: PBVS - 适用于精确位置控制
- **模式2**: IBVS - 适用于图像空间控制
- **仿真模式**: 支持所有功能，无需真实硬件

### 6. 操作控制
- **'q'**: 退出程序
- **'e'**: 紧急停止
- **'r'**: (IBVS模式) 重新设置期望特征

## 视觉伺服方法

### PBVS (基于位置的视觉伺服)

#### 工作原理
1. 使用相机和深度信息获取目标的3D位置和姿态
2. 在笛卡尔空间中定义位姿误差
3. 通过控制律将误差转换为机器人运动命令
4. 控制机器人末端执行器到达期望的3D位置

#### 控制律
```
位置误差: e_p = P_desired - P_current
旋转误差: e_r = log(R_desired × R_current^T)
控制速度: v = λ_pos × e_p, ω = λ_rot × e_r
```

#### 优点
- 直观易懂，误差定义在3D空间
- 轨迹规划简单直接
- 适合精确位置控制任务

#### 缺点
- 需要准确的深度信息
- 对相机标定精度要求较高
- 需要目标的3D几何模型

#### 适用场景
- 机器人抓取任务
- 精确装配作业
- 工件定位操作

### IBVS (基于图像的视觉伺服)

#### 工作原理
1. 直接使用图像特征作为反馈信号
2. 在图像空间中定义特征误差
3. 通过图像雅可比矩阵将图像误差转换为机器人运动
4. 保持目标在图像中的期望位置和尺寸

#### 控制律
```
特征误差: e_s = s_desired - s_current
图像雅可比: L = ∂s/∂v (特征对机器人速度的偏导)
控制速度: v = -λ × L^+ × e_s
```

#### 优点
- 不需要精确的3D模型
- 对相机标定误差不敏感
- 收敛性好，稳定性强

#### 缺点
- 轨迹可能不是直线
- 图像雅可比矩阵计算复杂
- 可能出现局部最小值

#### 适用场景
- 目标跟踪任务
- 视觉导航
- 图像稳定控制

## 系统架构

```
robot_visual_servoing_integrated.py (主驱动程序)
│
├── RobotVisualServoing (主类)
│   ├── 机器人控制接口 (robotcontrol.py)
│   │   ├── connect_robot()
│   │   ├── send_velocity_command()
│   │   └── emergency_stop_robot()
│   │
│   ├── 视觉系统
│   │   ├── setup_camera()
│   │   ├── get_frames()
│   │   └── detect_target() (YOLO)
│   │
│   ├── PBVS模块
│   │   ├── estimate_target_pose_pbvs()
│   │   ├── compute_pose_error_pbvs()
│   │   ├── compute_control_law_pbvs()
│   │   └── visualize_pbvs()
│   │
│   └── IBVS模块
│       ├── extract_image_features_ibvs()
│       ├── compute_control_law_ibvs()
│       └── visualize_ibvs()
│
├── 辅助工具
│   ├── camera_calibration.py (相机标定)
│   ├── yolo_train.py (模型训练)
│   ├── yolo_detect.py (检测测试)
│   └── test_system.py (系统测试)
│
└── 基础模块
    ├── visual_servoing.py (基础实现)
    └── advanced_visual_servoing.py (高级功能)
```

## 参数配置

### 控制参数
```python
# 基本控制参数
lambda_pos = 0.3        # 位置/特征控制增益 (0.1-0.5)
lambda_rot = 0.2        # 旋转控制增益 (0.1-0.3)

# 速度限制
max_linear_velocity = 0.05      # 最大线速度 m/s
max_angular_velocity = 0.3      # 最大角速度 rad/s

# 安全参数
max_target_lost = 20    # 最大目标丢失次数
```

### PBVS专用参数
```python
# 期望位置 (相机坐标系)
desired_position = [0.0, 0.0, 0.4]  # [x, y, z] 米

# 目标3D模型点 (假设10cm×10cm正方形)
object_3d_points = [
    [-0.05, -0.05, 0], [0.05, -0.05, 0],
    [0.05, 0.05, 0], [-0.05, 0.05, 0]
]

# 死区设置
position_deadzone = 0.01        # 位置死区 1cm
orientation_deadzone = 0.05     # 姿态死区 ~3度
```

### IBVS专用参数
```python
# 期望图像特征 [x, y, width, height]
desired_features = [320, 240, 120, 120]  # 图像中心，120×120像素

# 死区设置
feature_deadzone = 8            # 特征死区 8像素
```

### 相机参数 (RealSense D435i默认)
```python
camera_matrix = [
    [615.0, 0, 320.0],
    [0, 615.0, 240.0],
    [0, 0, 1]
]
dist_coeffs = [0.1, -0.2, 0, 0, 0]
```

## 使用指南

### 基本操作流程

#### 1. 系统准备
```bash
# 检查硬件连接
- 确保机器人电源开启
- 连接RealSense相机
- 检查网络连接

# 激活环境并启动系统
conda activate vision
python robot_visual_servoing_integrated.py
```

#### 2. 参数设置
```python
# 根据具体任务调整参数
vs = RobotVisualServoing(
    robot_ip='192.168.1.100',  # 机器人IP
    robot_port=8899,           # 机器人端口
    model_path='yolo_train/weights/best.pt'  # YOLO模型
)

# 调整控制参数
vs.lambda_pos = 0.2  # 降低增益获得更稳定的控制
vs.max_linear_velocity = 0.03  # 降低速度提高安全性
```

#### 3. 运行控制
```python
# PBVS模式
vs.run_pbvs()

# IBVS模式  
vs.run_ibvs()
```

### 高级使用技巧

#### 1. 参数调优策略
```python
# 保守参数 - 稳定但较慢
conservative_params = {
    'lambda_pos': 0.1,
    'lambda_rot': 0.05,
    'max_linear_velocity': 0.02
}

# 标准参数 - 平衡性能
standard_params = {
    'lambda_pos': 0.3,
    'lambda_rot': 0.2,
    'max_linear_velocity': 0.05
}

# 激进参数 - 快速但可能不稳定
aggressive_params = {
    'lambda_pos': 0.5,
    'lambda_rot': 0.3,
    'max_linear_velocity': 0.08
}
```

#### 2. 目标检测优化
```python
# 提高检测稳定性
- 确保良好的光照条件
- 目标与背景有足够对比度
- 避免目标部分遮挡
- 保持相机稳定

# YOLO模型优化
- 使用针对特定目标训练的模型
- 调整置信度阈值
- 考虑使用更大的模型获得更高精度
```

#### 3. 安全操作建议
```python
# 工作空间设置
- 设置合理的工作空间边界
- 确保紧急停止按钮可及
- 监控机器人运动状态
- 准备手动接管

# 测试流程
1. 先在仿真环境中测试
2. 使用较低的速度参数开始
3. 逐步增加控制增益
4. 监控系统稳定性
```

## 故障排除

### 常见问题及解决方案

#### 1. 机器人连接问题
**问题**: 无法连接到机器人
```bash
解决方案:
- 检查机器人IP地址和端口
- 确认机器人服务器运行状态
- 检查网络连接和防火墙设置
- 验证机器人控制权限
```

#### 2. 相机初始化失败
**问题**: RealSense相机无法初始化
```bash
解决方案:
- 检查相机USB连接
- 更新RealSense驱动程序
- 确认相机设备权限
- 尝试重新插拔相机
```

#### 3. 目标检测不稳定
**问题**: YOLO检测结果不稳定
```bash
解决方案:
- 改善光照条件
- 调整相机位置和角度
- 重新训练YOLO模型
- 调整检测置信度阈值
- 增加目标与背景的对比度
```

#### 4. 控制系统震荡
**问题**: 机器人运动不稳定，出现震荡
```bash
解决方案:
- 降低控制增益 (lambda_pos, lambda_rot)
- 增加死区范围
- 检查相机帧率和延迟
- 优化控制循环频率
- 检查机器人动力学参数
```

#### 5. 深度信息不准确
**问题**: 深度测量存在误差
```bash
解决方案:
- 检查目标表面材质 (避免反光、透明)
- 调整RealSense曝光参数
- 使用多点深度值平均
- 考虑环境光照影响
- 校准深度相机
```

#### 6. conda环境问题
**问题**: conda命令未找到或环境冲突
```bash
解决方案:
# 重新初始化conda
source ~/miniconda3/etc/profile.d/conda.sh
conda init bash
source ~/.bashrc

# 检查环境是否正确激活
conda info --envs
which python

# 与ROS2冲突时的处理
- 确保在conda环境中运行项目
- 检查PYTHONPATH环境变量
- 使用 conda deactivate 退出环境后再使用ROS2
```

### 调试技巧

#### 1. 可视化调试
```python
# 观察关键信息
- 检测框位置和大小
- 期望位置显示
- 误差数值变化
- 控制命令大小

# 数据记录
- 保存误差历史
- 记录控制命令
- 分析收敛性能
```

#### 2. 分步测试
```python
# 测试步骤
1. 仅测试目标检测
2. 验证位姿估计精度
3. 检查控制律计算
4. 测试机器人响应
5. 整体系统集成
```

## API参考

### 主类: RobotVisualServoing

#### 构造函数
```python
def __init__(self, robot_ip='localhost', robot_port=8899, 
             model_path='yolo_train/weights/best.pt'):
    """
    初始化视觉伺服系统
    
    Args:
        robot_ip (str): 机器人IP地址
        robot_port (int): 机器人端口
        model_path (str): YOLO模型路径
    """
```

#### 核心方法
```python
def connect_robot(self) -> bool:
    """连接机器人，返回连接状态"""

def get_frames(self) -> Tuple[np.ndarray, np.ndarray]:
    """获取RGB-D图像帧"""

def detect_target(self, color_image) -> Optional[Dict]:
    """检测目标，返回检测结果"""

def send_velocity_command(self, velocity) -> bool:
    """发送速度命令到机器人"""

def emergency_stop_robot(self):
    """紧急停止机器人"""

def run_pbvs(self):
    """运行PBVS控制循环"""

def run_ibvs(self):
    """运行IBVS控制循环"""

def cleanup(self):
    """清理系统资源"""
```

## 扩展开发

### 1. 添加新的视觉伺服方法

```python
class CustomVisualServoing(RobotVisualServoing):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 自定义初始化
    
    def compute_custom_control_law(self, features):
        """实现自定义控制律"""
        # 自定义算法实现
        return control_velocity
    
    def run_custom(self):
        """运行自定义控制循环"""
        # 实现控制循环
        pass
```

### 2. 集成其他机器人平台

```python
class ROSVisualServoing(RobotVisualServoing):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # ROS初始化
        import rospy
        from geometry_msgs.msg import Twist
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def send_velocity_command(self, velocity):
        """发送ROS速度命令"""
        twist = Twist()
        twist.linear.x = velocity[0]
        twist.linear.y = velocity[1]
        twist.linear.z = velocity[2]
        twist.angular.x = velocity[3]
        twist.angular.y = velocity[4]
        twist.angular.z = velocity[5]
        self.cmd_pub.publish(twist)
        return True
```

## 实际应用示例

### 机器人抓取任务
1. 使用YOLO检测目标物体
2. 使用PBVS将机器人移动到目标上方
3. 执行抓取动作

### 目标跟踪任务
1. 使用IBVS保持目标在图像中心
2. 维持固定的观察距离和角度

## 参考文献

1. Chaumette, F., & Hutchinson, S. (2006). Visual servo control. IEEE Robotics & Automation Magazine.
2. Corke, P. (2017). Robotics, vision and control: fundamental algorithms In MATLAB.
3. Marchand, E., Spindler, F., & Chaumette, F. (2005). ViSP for visual servoing: a generic software platform.

## 许可证和贡献

### 许可证
本项目基于现有的robotcontrol.py接口开发，请遵循相应的许可证要求。

### 贡献指南
欢迎提交问题报告和功能请求。在贡献代码前，请确保：

1. 代码符合PEP 8规范
2. 添加适当的注释和文档
3. 通过所有测试用例
4. 提供使用示例

---

## PyBullet仿真环境

### 仿真系统特性

本项目现已集成基于PyBullet的Auboi5六轴机械臂仿真环境：

- ✅ **环境隔离**: 完全在conda vision环境中运行，避免与ROS2冲突
- ✅ **物理仿真**: 基于PyBullet的真实物理仿真
- ✅ **Auboi5模型**: 精确的六轴机械臂模型，底座固定到地面
- ✅ **末端相机**: 集成RGB-D相机传感器
- ✅ **无缝切换**: 支持仿真和真实机械臂模式切换
- ✅ **完全兼容**: 与现有视觉伺服代码完全兼容

### 仿真架构

```
robot_visual_servoing_simulation.py (集成视觉伺服系统)
├── RobotVisualServoing (视觉伺服主类)
│   ├── 仿真/真实模式切换
│   ├── PBVS和IBVS算法
│   └── 与现有代码完全兼容
│
├── pybullet_robot_adapter.py (PyBullet适配器)
│   ├── PyBulletAuboi5Robot (仿真机械臂类)
│   │   ├── Auboi5六轴机械臂模型
│   │   ├── 底座固定到地面
│   │   ├── 末端RGB-D相机
│   │   ├── 真实机械臂接口模拟
│   │   └── 物理仿真引擎
│   │
│   └── 完全兼容robotcontrol.py接口
```

### 仿真快速开始

**1. 启动仿真系统**
```bash
# 使用启动脚本（推荐）
./run_simulation.sh

# 或直接运行
python robot_visual_servoing_simulation.py
```

**2. 选择模式**
- S: PyBullet仿真模式
- R: 真实机械臂模式

**3. 选择控制方法**
- 1: PBVS（基于位置的视觉伺服）
- 2: IBVS（基于图像的视觉伺服）

**4. 机械臂演示**
```bash
# Auboi5机械臂运动演示
python auboi5_demo.py

# 系统测试
python test_simulation.py
```

### Auboi5机械臂规格

```
- 基座高度: 0.165m
- 肩部高度: 0.158m  
- 大臂长度: 0.425m
- 小臂长度: 0.392m
- 手腕长度: 0.109m
- 法兰长度: 0.082m
- 总工作高度: ~1.3m
- 工作半径: ~0.85m
- 关节限制: 基于实际Auboi5限制
```

### 仿真与实际系统对比

| 特性 | 仿真系统 | 实际系统 |
|------|----------|----------|
| 机械臂控制 | PyBullet物理仿真 | Auboi5实际机械臂 |
| 相机输入 | 仿真RGB-D相机 | RealSense D435i |
| 目标检测 | 相同YOLO模型 | 相同YOLO模型 |
| 视觉伺服算法 | 相同算法实现 | 相同算法实现 |
| 安全机制 | 仿真碰撞检测 | 实际安全监控 |
| 接口兼容性 | 100%兼容 | 原生支持 |

---

**安全提醒**: 使用本系统前请确保充分了解机器人安全操作规程，并在安全的环境中进行测试。建议先在仿真环境中验证算法，再部署到实际机器人系统。

**环境提醒**: 每次运行项目前，请确保已激活conda环境 `vision`。