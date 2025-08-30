# 机器人视觉伺服系统

基于 robotcontrol.py 的 PBVS 和 IBVS 视觉伺服实现

## 系统概述

本系统整合了机器人控制、计算机视觉和深度学习技术，实现了两种主要的视觉伺服方法：

1. **PBVS (Position-Based Visual Servoing)** - 基于位置的视觉伺服
2. **IBVS (Image-Based Visual Servoing)** - 基于图像的视觉伺服

## 主要特性

- ✅ 基于 Auboi5 机器人的完整控制接口
- ✅ RealSense D435i 相机支持
- ✅ YOLO v8 目标检测
- ✅ 实时视觉伺服控制
- ✅ 安全功能（速度限制、紧急停止、碰撞检测）
- ✅ 数据记录和可视化
- ✅ 模块化设计，易于扩展

## 系统架构

```
robot_visual_servoing.py
├── RobotVisualServoingBase (基类)
│   ├── 机器人连接和控制
│   ├── 相机设置和图像获取
│   ├── YOLO目标检测
│   └── 安全功能
├── RobotPBVS (PBVS实现)
│   ├── 3D位姿估计
│   ├── 位姿误差计算
│   └── PBVS控制律
└── RobotIBVS (IBVS实现)
    ├── 图像特征提取
    ├── 图像雅可比计算
    └── IBVS控制律
```

## 安装依赖

```bash
pip install opencv-python
pip install numpy
pip install scipy
pip install pyrealsense2
pip install ultralytics
pip install json
```

## 快速开始

### 1. 系统测试

```bash
# 运行交互式测试
python test_robot_visual_servoing.py

# 运行所有测试
python test_robot_visual_servoing.py --all

# 测试特定组件
python test_robot_visual_servoing.py --camera
python test_robot_visual_servoing.py --yolo
python test_robot_visual_servoing.py --robot
```

### 2. 运行视觉伺服

```bash
# 启动主程序
python robot_visual_servoing.py

# 选择模式：
# 1 - PBVS模式
# 2 - IBVS模式
# 3 - 系统配置
```

### 3. 操作说明

#### PBVS 模式
- 目标：将机器人末端移动到目标物体的期望3D位置
- 控制：按 'q' 退出，'s' 保存数据，'e' 紧急停止
- 显示：绿色框显示当前检测，红色圆圈显示期望位置

#### IBVS 模式
- 目标：将目标物体移动到图像中的期望位置和尺寸
- 控制：按 'q' 退出，'s' 保存数据，'e' 紧急停止，'r' 重设期望特征
- 显示：绿色框显示当前目标，红色框显示期望位置

## 配置文件

系统配置存储在 `robot_visual_servoing_config.json` 中：

```json
{
  "robot_config": {
    "ip": "localhost",
    "port": 8899,
    "collision_level": 6
  },
  "pbvs_config": {
    "desired_position": [0.0, 0.0, 0.4],
    "lambda_pos": 0.3,
    "lambda_rot": 0.2
  },
  "ibvs_config": {
    "desired_features": [320, 240, 120, 120],
    "lambda_pos": 0.3
  },
  "safety_config": {
    "max_linear_velocity": 0.05,
    "max_angular_velocity": 0.3
  }
}
```

## API 参考

### RobotVisualServoingBase

基础类，提供通用功能：

```python
class RobotVisualServoingBase:
    def __init__(self, robot_ip, robot_port, model_path, calibration_file)
    def connect_robot(self) -> bool
    def get_aligned_frames(self) -> Tuple[np.ndarray, np.ndarray]
    def detect_target(self, image) -> Dict
    def send_velocity_command(self, velocity) -> bool
    def emergency_stop_robot(self)
```

### RobotPBVS

基于位置的视觉伺服：

```python
class RobotPBVS(RobotVisualServoingBase):
    def estimate_target_pose(self, target) -> Dict
    def compute_pose_error(self, current_pose) -> np.ndarray
    def compute_control_law(self, pose_error) -> np.ndarray
    def run(self)
```

### RobotIBVS

基于图像的视觉伺服：

```python
class RobotIBVS(RobotVisualServoingBase):
    def extract_image_features(self, target, depth_image) -> np.ndarray
    def compute_image_jacobian(self, features) -> np.ndarray
    def compute_control_law(self, current_features) -> np.ndarray
    def run(self)
```

## 安全功能

### 1. 速度限制
- 线速度限制：默认 0.05 m/s
- 角速度限制：默认 0.3 rad/s
- 实时速度检查和限制

### 2. 目标丢失检测
- 连续丢失计数：默认 20 次
- 自动紧急停止
- 状态显示和报警

### 3. 紧急停止
- 键盘快捷键：'e'
- 立即停止机器人运动
- 安全状态锁定

### 4. 工作空间限制
- 可配置的工作空间边界
- 超出边界自动停止
- 碰撞等级设置

## 数据记录

系统自动记录以下数据：

```json
{
  "error_history": [...],      // 误差历史
  "control_history": [...],    // 控制命令历史
  "pose_history": [...],       // 位姿历史
  "parameters": {...},         // 控制参数
  "timestamp": 1234567890      // 时间戳
}
```

保存文件格式：
- PBVS: `pbvs_data_timestamp.json`
- IBVS: `ibvs_data_timestamp.json`

## 故障排除

### 1. 机器人连接失败
- 检查机器人IP地址和端口
- 确认机器人服务器运行状态
- 检查网络连接

### 2. 相机初始化失败
- 确认RealSense相机连接
- 检查USB接口和驱动
- 验证相机权限

### 3. YOLO检测失败
- 检查模型文件路径
- 确认模型文件完整性
- 验证目标物体在视野中

### 4. 控制不稳定
- 调整控制增益参数
- 检查相机标定质量
- 优化光照条件

## 参数调优

### PBVS 参数
- `lambda_pos`: 位置控制增益 (0.1-0.5)
- `lambda_rot`: 旋转控制增益 (0.1-0.3)
- `position_deadzone`: 位置死区 (0.005-0.02)
- `orientation_deadzone`: 姿态死区 (0.02-0.1)

### IBVS 参数
- `lambda_pos`: 特征控制增益 (0.1-0.5)
- `feature_deadzone`: 特征死区 (5-15 像素)
- `desired_features`: 期望特征位置和尺寸

### 安全参数
- `max_linear_velocity`: 最大线速度 (0.01-0.1)
- `max_angular_velocity`: 最大角速度 (0.1-0.5)
- `max_target_lost`: 最大目标丢失次数 (10-30)

## 扩展开发

### 添加新的视觉伺服方法

1. 继承 `RobotVisualServoingBase`
2. 实现特定的控制律
3. 添加可视化功能
4. 集成到主程序

```python
class CustomVisualServoing(RobotVisualServoingBase):
    def compute_control_law(self, features):
        # 实现自定义控制律
        pass
    
    def run(self):
        # 实现控制循环
        pass
```

### 添加新的传感器

1. 扩展 `setup_camera` 方法
2. 修改 `get_aligned_frames` 方法
3. 更新配置文件

## 许可证

本项目基于现有的 robotcontrol.py 接口开发，请遵循相应的许可证要求。

## 贡献

欢迎提交问题报告和功能请求。在贡献代码前，请确保：

1. 代码符合PEP 8规范
2. 添加适当的测试
3. 更新相关文档
4. 通过所有测试用例

## 联系信息

如有问题或建议，请通过以下方式联系：
- 提交 GitHub Issue
- 发送邮件至项目维护者

---

**注意：使用本系统前请确保充分了解机器人安全操作规程，并在安全的环境中进行测试。**
