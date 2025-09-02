#!/bin/bash

# PyBullet机械臂视觉伺服仿真系统启动脚本
# 确保在conda vision环境中运行，避免与ROS2冲突

echo "=== PyBullet机械臂视觉伺服仿真系统 ==="
echo "正在激活conda vision环境..."

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vision

# 检查环境
echo "当前Python路径: $(which python)"
echo "当前环境: $CONDA_DEFAULT_ENV"

# 检查必要的包
echo ""
echo "检查依赖包..."
python -c "import pybullet; print('✓ PyBullet 可用')" 2>/dev/null || echo "✗ PyBullet 不可用，请运行: pip install pybullet"
python -c "import cv2; print('✓ OpenCV 可用')" 2>/dev/null || echo "✗ OpenCV 不可用"
python -c "import numpy; print('✓ NumPy 可用')" 2>/dev/null || echo "✗ NumPy 不可用"
python -c "from ultralytics import YOLO; print('✓ YOLO 可用')" 2>/dev/null || echo "✗ YOLO 不可用"
python -c "from scipy.spatial.transform import Rotation; print('✓ SciPy 可用')" 2>/dev/null || echo "✗ SciPy 不可用"

echo ""
echo "=== 启动选项 ==="
echo "1. 视觉伺服仿真系统 (支持PBVS/IBVS)"
echo "2. Auboi5机械臂运动演示"
echo "3. 系统测试"

read -p "请选择运行模式 (1-3): " choice

case $choice in
    1)
        echo "启动视觉伺服仿真系统..."
        python robot_visual_servoing_simulation.py
        ;;
    2)
        echo "启动Auboi5机械臂运动演示..."
        python auboi5_demo.py
        ;;
    3)
        echo "启动系统测试..."
        python test_simulation.py
        ;;
    *)
        echo "无效选择，启动视觉伺服仿真系统..."
        python robot_visual_servoing_simulation.py
        ;;
esac

echo ""
echo "仿真结束"
