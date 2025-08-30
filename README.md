# 机器人视觉伺服系统

基于robotcontrol.py的PBVS和IBVS视觉伺服系统，整合了YOLO目标检测和RealSense深度相机。

## 目录

1. [系统概述](#系统概述)
2. [功能特性](#功能特性)
3. [环境要求](#环境要求)
4. [快速开始](#快速开始)
5. [视觉伺服方法](#视觉伺服方法)
6. [系统架构](#系统架构)
7. [参数配置](#参数配置)
8. [使用指南](#使用指南)
9. [故障排除](#故障排除)
10. [API参考](#api参考)
11. [扩展开发](#扩展开发)

## 系统概述

本系统实现了两种主要的视觉伺服方法：

- **PBVS (Position-Based Visual Servoing)**: 基于位置的视觉伺服 - 在3D空间中进行精确位置控制
- **IBVS (Image-Based Visual Servoing)**: 基于图像的视觉伺服 - 直接在图像空间中进行伺服控制

系统集成了Auboi5机器人控制、YOLO目标检测和RealSense深度相机，可以实现对目标物体的精确视觉伺服控制。

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

## 环境要求

### 硬件要求
- Auboi5机械臂或兼容机器人
- Intel RealSense 深度相机 (D435/D455等)
- 标准棋盘格标定板 (可选，用于精确标定)

### 软件依赖
```bash
pip install opencv-python
pip install numpy
pip install scipy
pip install pyrealsense2
pip install ultralytics
```

### 系统要求
- Python 3.7+
- Windows/Linux
- 至少4GB内存
- 支持CUDA的GPU (推荐，用于YOLO加速)

## 快速开始

### 1. 系统测试
```bash
# 测试相机连接
python -c "import pyrealsense2 as rs; print('RealSense相机可用')"

# 测试YOLO模型
python -c "from ultralytics import YOLO; print('YOLO模型可用')"

# 运行系统测试
python test_system.py --all
```

### 2. 运行视觉伺服
```bash
# 运行主程序
python robot_visual_servoing_integrated.py
```

### 3. 选择模式
- **模式1**: PBVS - 适用于精确位置控制
- **模式2**: IBVS - 适用于图像空间控制

### 4. 操作控制
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
robot_visual_servoing_integrated.py
│
├── RobotVisualServoing (主类)
│   ├── 机器人控制接口
│   │   ├── connect_robot()
│   │   ├── send_velocity_command()
│   │   └── emergency_stop_robot()
│   │
│   ├── 视觉系统
│   │   ├── setup_camera()
│   │   ├── get_frames()
│   │   └── detect_target()
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

# 启动系统
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

### 相机标定

首先需要对相机进行标定以获得准确的内参矩阵：

```bash
python camera_calibration.py
```

标定流程：
1. 选择 "1. 运行完整标定流程"
2. 准备9x6内角点的标准棋盘格
3. 在不同角度和距离拍摄20张棋盘格图像
4. 系统自动计算相机内参并保存到 `camera_calibration.json`

### YOLO模型训练

使用您的目标物体数据集训练YOLO模型：

```bash
python yolo_train.py
```

训练完成后，模型权重保存在 `yolo_train/weights/best.pt`

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

#### 3. 参数敏感性分析
```python
# 测试不同参数组合
for lambda_pos in [0.1, 0.2, 0.3, 0.4, 0.5]:
    for lambda_rot in [0.1, 0.15, 0.2, 0.25, 0.3]:
        # 运行短时间测试
        # 记录性能指标
        # 分析稳定性和收敛速度
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

#### PBVS专用方法
```python
def estimate_target_pose_pbvs(self, target) -> Optional[Dict]:
    """估计目标3D位姿"""

def compute_pose_error_pbvs(self, current_pose) -> Optional[np.ndarray]:
    """计算位姿误差"""

def compute_control_law_pbvs(self, pose_error) -> np.ndarray:
    """计算PBVS控制律"""

def visualize_pbvs(self, image, target):
    """PBVS可视化"""
```

#### IBVS专用方法
```python
def extract_image_features_ibvs(self, target, depth_image) -> Optional[np.ndarray]:
    """提取图像特征"""

def compute_control_law_ibvs(self, current_features) -> np.ndarray:
    """计算IBVS控制律"""

def visualize_ibvs(self, image, target):
    """IBVS可视化"""
```

### 数据结构

#### 目标检测结果
```python
target = {
    'center': (x, y),           # 目标中心像素坐标
    'size': (width, height),    # 目标尺寸
    'confidence': 0.85          # 检测置信度
}
```

#### 目标位姿 (PBVS)
```python
target_pose = {
    'position': np.array([x, y, z]),        # 3D位置 (米)
    'orientation': np.array([[...]])         # 3×3旋转矩阵
}
```

#### 图像特征 (IBVS)
```python
features = np.array([x, y, width, height, depth])  # 图像特征向量
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

### 3. 添加多目标支持

```python
def detect_multiple_targets(self, color_image):
    """检测多个目标"""
    results = self.model(color_image)
    targets = []
    
    for box in results[0].boxes:
        if box.conf[0].cpu().numpy() > 0.5:
            x_center, y_center, width, height = box.xywh[0].cpu().numpy()
            targets.append({
                'center': (int(x_center), int(y_center)),
                'size': (int(width), int(height)),
                'confidence': float(box.conf[0].cpu().numpy()),
                'class': int(box.cls[0].cpu().numpy())
            })
    
    return targets
```

### 4. 添加学习能力

```python
class AdaptiveVisualServoing(RobotVisualServoing):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.performance_history = []
        self.adaptive_gains = True
    
    def adapt_parameters(self, error_history):
        """根据历史性能自适应调整参数"""
        recent_errors = error_history[-10:]  # 最近10次误差
        avg_error = np.mean([np.linalg.norm(e) for e in recent_errors])
        
        if avg_error > 0.1:  # 误差较大，降低增益
            self.lambda_pos *= 0.9
        elif avg_error < 0.02:  # 误差较小，可以提高增益
            self.lambda_pos *= 1.1
        
        # 限制增益范围
        self.lambda_pos = np.clip(self.lambda_pos, 0.1, 0.8)
```

### 5. 性能监控和分析

```python
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'convergence_time': [],
            'steady_state_error': [],
            'control_effort': [],
            'success_rate': 0
        }
    
    def update_metrics(self, error, control_cmd, timestamp):
        """更新性能指标"""
        self.metrics['steady_state_error'].append(np.linalg.norm(error))
        self.metrics['control_effort'].append(np.linalg.norm(control_cmd))
    
    def generate_report(self):
        """生成性能报告"""
        report = {
            'avg_error': np.mean(self.metrics['steady_state_error']),
            'max_error': np.max(self.metrics['steady_state_error']),
            'avg_control': np.mean(self.metrics['control_effort']),
            'stability_score': self.calculate_stability()
        }
        return report
```

## 文件说明

### 核心文件
- `robot_visual_servoing_integrated.py` - 整合的主要驱动代码
- `test_system.py` - 系统测试脚本
- `robotcontrol.py` - 机器人控制接口

### 基础组件
- `visual_servoing.py` - 基础视觉伺服实现
- `advanced_visual_servoing.py` - 高级视觉伺服系统
- `camera_calibration.py` - 相机标定工具
- `yolo_detect.py` - YOLO目标检测
- `yolo_train.py` - YOLO模型训练

### 实际应用示例

#### 机器人抓取任务
1. 使用YOLO检测目标物体
2. 使用PBVS将机器人移动到目标上方
3. 执行抓取动作

#### 目标跟踪任务
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

### 联系信息
- GitHub Issues: 提交问题和建议
- 邮件: 联系项目维护者
- 文档: 查看最新文档和更新

---

**安全提醒**: 使用本系统前请确保充分了解机器人安全操作规程，并在安全的环境中进行测试。建议先在仿真环境中验证算法，再部署到实际机器人系统。