#!/bin/bash

# 视觉伺服项目环境设置脚本
# 用于在conda环境中运行项目，避免与ROS2冲突

echo "=== 视觉伺服项目环境设置 ==="
echo "正在激活conda环境..."

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vision

if [ $? -eq 0 ]; then
    echo "✅ conda环境激活成功"
    echo "当前Python环境: $(which python)"
    echo "Python版本: $(python --version)"
    
    # 检查关键依赖
    echo ""
    echo "检查关键依赖包..."
    python -c "
import sys
packages = ['cv2', 'numpy', 'pyrealsense2', 'ultralytics', 'torch']
for pkg in packages:
    try:
        __import__(pkg)
        print(f'✅ {pkg} 导入成功')
    except ImportError as e:
        print(f'❌ {pkg} 导入失败: {e}')
        sys.exit(1)
print('所有依赖包检查完成！')
"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 环境设置完成！现在可以运行您的视觉伺服项目了。"
        echo ""
        echo "常用命令："
        echo "  python test_system.py --all          # 测试系统"
        echo "  python robot_visual_servoing_integrated.py  # 运行主程序"
        echo "  python camera_calibration.py        # 相机标定"
        echo ""
        echo "要退出环境，请运行: conda deactivate"
    else
        echo "❌ 依赖包检查失败，请检查安装"
        exit 1
    fi
else
    echo "❌ conda环境激活失败"
    exit 1
fi
