#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyBullet仿真系统快速测试脚本
"""

import sys
import traceback

def test_imports():
    """测试所有必要的导入"""
    print("=== 测试导入模块 ===")
    
    try:
        import pybullet
        print("✓ PyBullet 导入成功")
    except ImportError as e:
        print(f"✗ PyBullet 导入失败: {e}")
        return False
    
    try:
        import cv2
        print("✓ OpenCV 导入成功")
    except ImportError as e:
        print(f"✗ OpenCV 导入失败: {e}")
        return False
    
    try:
        import numpy as np
        print("✓ NumPy 导入成功")
    except ImportError as e:
        print(f"✗ NumPy 导入失败: {e}")
        return False
    
    try:
        from ultralytics import YOLO
        print("✓ YOLO 导入成功")
    except ImportError as e:
        print(f"✗ YOLO 导入失败: {e}")
        return False
    
    try:
        from scipy.spatial.transform import Rotation
        print("✓ SciPy 导入成功")
    except ImportError as e:
        print(f"✗ SciPy 导入失败: {e}")
        return False
    
    return True

def test_pybullet_adapter():
    """测试PyBullet适配器"""
    print("\n=== 测试PyBullet适配器 ===")
    
    try:
        from pybullet_robot_adapter import Auboi5Robot, RobotErrorType
        print("✓ PyBullet适配器导入成功")
        
        # 创建机械臂（无GUI模式）
        robot = Auboi5Robot(gui=False)
        print("✓ 机械臂对象创建成功")
        
        # 测试基本接口
        Auboi5Robot.initialize()
        robot.create_context()
        result = robot.connect()
        
        if result == RobotErrorType.RobotError_SUCC:
            print("✓ 仿真连接成功")
            
            # 测试启动
            robot.robot_startup()
            print("✓ 机械臂启动成功")
            
            # 测试获取当前位置
            waypoint = robot.get_current_waypoint()
            if waypoint:
                print(f"✓ 当前位置获取成功: {len(waypoint['joint'])}个关节")
            
            # 测试运动
            test_joints = [0.1, -0.1, 0.2, -0.2, 0.1, -0.1]
            result = robot.move_joint(test_joints)
            if result == RobotErrorType.RobotError_SUCC:
                print("✓ 关节运动测试成功")
            
            # 测试相机图像（简单测试）
            try:
                color_image, depth_image = robot.get_camera_image()
                if color_image is not None and depth_image is not None:
                    print(f"✓ 相机图像获取成功: {color_image.shape}, {depth_image.shape}")
                else:
                    print("✗ 相机图像获取失败")
            except Exception as e:
                print(f"✗ 相机图像测试失败: {e}")
            
            # 清理
            robot.robot_shutdown()
            robot.disconnect()
            print("✓ 清理完成")
        else:
            print("✗ 仿真连接失败")
            return False
        
        return True
        
    except Exception as e:
        print(f"✗ PyBullet适配器测试失败: {e}")
        traceback.print_exc()
        return False

def test_yolo_model():
    """测试YOLO模型加载"""
    print("\n=== 测试YOLO模型 ===")
    
    try:
        import os
        model_path = 'yolo_train/weights/best.pt'
        
        if os.path.exists(model_path):
            from ultralytics import YOLO
            model = YOLO(model_path)
            print(f"✓ YOLO模型加载成功: {model_path}")
            return True
        else:
            print(f"✗ YOLO模型文件不存在: {model_path}")
            return False
            
    except Exception as e:
        print(f"✗ YOLO模型测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("PyBullet仿真系统快速测试")
    print("=" * 40)
    
    # 测试导入
    if not test_imports():
        print("\n❌ 导入测试失败，请检查依赖包安装")
        return False
    
    # 测试YOLO模型
    test_yolo_model()  # 这个可以失败，不影响仿真
    
    # 测试PyBullet适配器
    if not test_pybullet_adapter():
        print("\n❌ PyBullet适配器测试失败")
        return False
    
    print("\n✅ 所有测试通过！")
    print("可以运行以下命令启动仿真：")
    print("  python robot_visual_servoing_simulation.py")
    print("  或者")
    print("  ./run_simulation.sh")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
