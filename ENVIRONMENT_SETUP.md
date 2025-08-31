# 环境设置指南

## 概述

本指南帮助您在Ubuntu系统中设置conda环境来运行视觉伺服项目，同时避免与ROS2的冲突。

## 已完成的设置

✅ **Miniconda已安装**: `/home/wang-yb/miniconda3/`  
✅ **conda环境已创建**: `vision` (Python 3.8)  
✅ **依赖包已安装**: 所有requirements.txt中的包  
✅ **环境测试通过**: 所有关键依赖包可正常导入  

## 快速开始

### 方法1：使用设置脚本（推荐）

```bash
# 在项目目录中运行
./setup_environment.sh
```

### 方法2：手动激活环境

```bash
# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vision

# 验证环境
python --version  # 应该显示Python 3.8.x
which python      # 应该显示conda环境中的Python路径
```

## 环境信息

- **环境名称**: `vision`
- **Python版本**: 3.8.20
- **conda位置**: `/home/wang-yb/miniconda3/`
- **环境位置**: `/home/wang-yb/miniconda3/envs/vision/`

## 已安装的包

### 核心依赖
- `opencv-python>=4.5.0` - 计算机视觉库
- `numpy>=1.21.0` - 数值计算库
- `pyrealsense2>=2.50.0` - RealSense相机SDK
- `ultralytics>=8.0.0` - YOLO目标检测
- `torch>=1.9.0` - PyTorch深度学习框架
- `torchvision>=0.10.0` - PyTorch视觉工具
- `scipy>=1.7.0` - 科学计算库
- `matplotlib>=3.3.0` - 绘图库
- `Pillow>=8.3.0` - 图像处理库

### 自动安装的依赖
- CUDA相关包（用于GPU加速）
- 各种科学计算和机器学习库

## 避免ROS2冲突的策略

### 1. 环境隔离
- 使用独立的conda环境 `vision`
- 与系统Python和ROS2 Python完全隔离
- 每个环境有独立的包管理

### 2. 路径管理
- conda环境优先于系统Python
- 避免PYTHONPATH冲突
- 使用环境特定的包版本

### 3. 依赖版本控制
- 固定关键包的版本
- 避免与ROS2包的版本冲突
- 使用requirements.txt管理依赖

## 常用命令

### 环境管理
```bash
# 激活环境
conda activate vision

# 退出环境
conda deactivate

# 查看当前环境
conda info --envs

# 查看已安装的包
conda list
```

### 项目运行
```bash
# 测试系统
python test_system.py --all

# 运行主程序
python robot_visual_servoing_integrated.py

# 相机标定
python camera_calibration.py

# YOLO训练
python yolo_train.py
```

### 包管理
```bash
# 安装新包
pip install package_name

# 更新包
pip install --upgrade package_name

# 导出环境
conda env export > environment.yml
```

## 故障排除

### 1. conda命令未找到
```bash
# 重新初始化conda
source ~/miniconda3/etc/profile.d/conda.sh
conda init bash
source ~/.bashrc
```

### 2. 包导入错误
```bash
# 检查环境是否正确激活
conda info --envs
which python

# 重新安装包
pip install --force-reinstall package_name
```

### 3. 与ROS2的冲突
- 确保在conda环境中运行项目
- 检查PYTHONPATH环境变量
- 使用 `conda deactivate` 退出环境后再使用ROS2

### 4. 权限问题
```bash
# 修复权限
chmod +x setup_environment.sh
chmod +x *.py
```

## 性能优化

### GPU加速
- PyTorch已安装CUDA支持
- 检查GPU可用性：
```python
import torch
print(f"CUDA可用: {torch.cuda.is_available()}")
print(f"GPU数量: {torch.cuda.device_count()}")
```

### 内存优化
- 使用较小的YOLO模型
- 调整图像分辨率
- 监控内存使用情况

## 安全注意事项

1. **机器人安全**: 在安全环境中测试
2. **相机权限**: 确保有USB设备访问权限
3. **网络安全**: 机器人IP地址配置
4. **数据备份**: 定期备份训练好的模型

## 更新和维护

### 更新conda
```bash
conda update conda
```

### 更新环境
```bash
conda update --all
```

### 清理缓存
```bash
conda clean --all
```

## 联系支持

如果遇到问题，请检查：
1. 环境是否正确激活
2. 依赖包是否正确安装
3. 硬件连接是否正常
4. 权限设置是否正确

---

**注意**: 每次运行项目前，请确保已激活conda环境 `vision`。

