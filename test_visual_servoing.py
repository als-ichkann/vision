#!/usr/bin/env python3
"""
视觉伺服系统测试脚本
"""

import sys
import os

def check_dependencies():
    """检查依赖库是否安装"""
    required_packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'pyrealsense2': 'pyrealsense2',
        'ultralytics': 'ultralytics',
        'scipy': 'scipy'
    }
    
    missing_packages = []
    
    for package, pip_name in required_packages.items():
        try:
            __import__(package)
            print(f"✓ {package} 已安装")
        except ImportError:
            print(f"✗ {package} 未安装")
            missing_packages.append(pip_name)
    
    if missing_packages:
        print(f"\n请安装缺失的包:")
        print(f"pip install {' '.join(missing_packages)}")
        return False
    
    return True

def check_files():
    """检查必要文件是否存在"""
    required_files = [
        'visual_servoing.py',
        'advanced_visual_servoing.py',
        'camera_calibration.py',
        'yolo_train/weights/best.pt'
    ]
    
    missing_files = []
    
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✓ {file_path} 存在")
        else:
            print(f"✗ {file_path} 不存在")
            missing_files.append(file_path)
    
    if missing_files:
        print(f"\n缺失文件:")
        for file_path in missing_files:
            if 'best.pt' in file_path:
                print(f"  {file_path} - 请先运行 yolo_train.py 训练模型")
            else:
                print(f"  {file_path}")
        return False
    
    return True

def test_camera_connection():
    """测试相机连接"""
    try:
        import pyrealsense2 as rs
        
        # 尝试连接相机
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        try:
            pipeline.start(config)
            print("✓ RealSense相机连接成功")
            pipeline.stop()
            return True
        except Exception as e:
            print(f"✗ RealSense相机连接失败: {e}")
            return False
            
    except ImportError:
        print("✗ pyrealsense2 未安装，无法测试相机")
        return False

def test_yolo_model():
    """测试YOLO模型加载"""
    try:
        from ultralytics import YOLO
        
        model_path = 'yolo_train/weights/best.pt'
        if not os.path.exists(model_path):
            print(f"✗ YOLO模型文件不存在: {model_path}")
            return False
        
        try:
            model = YOLO(model_path)
            print("✓ YOLO模型加载成功")
            return True
        except Exception as e:
            print(f"✗ YOLO模型加载失败: {e}")
            return False
            
    except ImportError:
        print("✗ ultralytics 未安装，无法测试YOLO模型")
        return False

def main():
    """主测试函数"""
    print("=== 视觉伺服系统测试 ===\n")
    
    print("1. 检查依赖库...")
    deps_ok = check_dependencies()
    print()
    
    print("2. 检查必要文件...")
    files_ok = check_files()
    print()
    
    if not deps_ok or not files_ok:
        print("❌ 系统检查失败，请解决上述问题后重试")
        return False
    
    print("3. 测试相机连接...")
    camera_ok = test_camera_connection()
    print()
    
    print("4. 测试YOLO模型...")
    model_ok = test_yolo_model()
    print()
    
    if camera_ok and model_ok:
        print("✅ 所有测试通过！系统准备就绪")
        print("\n可以运行以下命令开始使用:")
        print("  python camera_calibration.py    # 相机标定")
        print("  python visual_servoing.py       # 基础视觉伺服")
        print("  python advanced_visual_servoing.py  # 高级视觉伺服")
        return True
    else:
        print("⚠️  部分测试失败，但基本功能可能仍可使用")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
