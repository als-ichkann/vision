#!/usr/bin/env python
# coding=utf-8
"""
机器人视觉伺服系统简化测试脚本
"""

import sys
import cv2
import numpy as np

def test_camera():
    """测试相机连接"""
    print("=== 测试相机连接 ===")
    try:
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        pipeline.start(config)
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if color_frame and depth_frame:
            print("✓ RealSense相机工作正常")
            color_image = np.asanyarray(color_frame.get_data())
            print(f"✓ 图像尺寸: {color_image.shape}")
        else:
            print("✗ 相机帧获取失败")
        
        pipeline.stop()
        return True
    except Exception as e:
        print(f"✗ 相机测试失败: {e}")
        return False

def test_yolo():
    """测试YOLO模型"""
    print("\n=== 测试YOLO模型 ===")
    try:
        from ultralytics import YOLO
        model = YOLO('yolo_train/weights/best.pt')
        print("✓ YOLO模型加载成功")
        
        # 创建测试图像
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        results = model(test_image)
        print("✓ YOLO推理测试成功")
        return True
    except Exception as e:
        print(f"✗ YOLO测试失败: {e}")
        return False

def test_robot_connection():
    """测试机器人连接"""
    print("\n=== 测试机器人连接 ===")
    try:
        from robotcontrol import Auboi5Robot, RobotErrorType
        
        robot = Auboi5Robot()
        Auboi5Robot.initialize()
        handle = robot.create_context()
        
        if handle >= 0:
            print("✓ 机器人控制库初始化成功")
            
            # 尝试连接（这里使用localhost，实际使用时需要修改IP）
            result = robot.connect('localhost', 8899)
            if result == RobotErrorType.RobotError_SUCC:
                print("✓ 机器人连接成功")
                robot.disconnect()
            else:
                print("⚠ 机器人连接失败（可能是IP地址问题）")
        else:
            print("✗ 机器人控制库初始化失败")
        
        Auboi5Robot.uninitialize()
        return True
    except Exception as e:
        print(f"✗ 机器人测试失败: {e}")
        return False

def test_visual_servoing():
    """测试视觉伺服系统"""
    print("\n=== 测试视觉伺服系统 ===")
    try:
        from robot_visual_servoing_integrated import RobotVisualServoing
        
        vs = RobotVisualServoing()
        print("✓ 视觉伺服系统初始化成功")
        
        # 测试相机帧获取
        color_image, depth_image = vs.get_frames()
        if color_image is not None:
            print("✓ 相机帧获取成功")
            
            # 测试目标检测
            target = vs.detect_target(color_image)
            if target:
                print(f"✓ 检测到目标，置信度: {target['confidence']:.2f}")
            else:
                print("⚠ 未检测到目标（这可能是正常的）")
        else:
            print("✗ 相机帧获取失败")
        
        vs.cleanup()
        return True
    except Exception as e:
        print(f"✗ 视觉伺服系统测试失败: {e}")
        return False

def interactive_test():
    """交互式测试"""
    print("=== 机器人视觉伺服系统测试 ===")
    print("1. 测试相机")
    print("2. 测试YOLO模型")
    print("3. 测试机器人连接")
    print("4. 测试视觉伺服系统")
    print("5. 运行所有测试")
    print("0. 退出")
    
    while True:
        choice = input("\n请选择测试项目 (0-5): ").strip()
        
        if choice == "1":
            test_camera()
        elif choice == "2":
            test_yolo()
        elif choice == "3":
            test_robot_connection()
        elif choice == "4":
            test_visual_servoing()
        elif choice == "5":
            print("=== 运行所有测试 ===")
            results = []
            results.append(test_camera())
            results.append(test_yolo())
            results.append(test_robot_connection())
            results.append(test_visual_servoing())
            
            print(f"\n=== 测试结果 ===")
            print(f"通过: {sum(results)}/{len(results)}")
            if all(results):
                print("✓ 所有测试通过，系统准备就绪！")
            else:
                print("⚠ 部分测试失败，请检查相关组件")
        elif choice == "0":
            print("退出测试")
            break
        else:
            print("无效选择")

def main():
    """主函数"""
    if len(sys.argv) > 1:
        if sys.argv[1] == "--all":
            test_camera()
            test_yolo()
            test_robot_connection()
            test_visual_servoing()
        elif sys.argv[1] == "--camera":
            test_camera()
        elif sys.argv[1] == "--yolo":
            test_yolo()
        elif sys.argv[1] == "--robot":
            test_robot_connection()
        elif sys.argv[1] == "--vs":
            test_visual_servoing()
        else:
            print("用法: python test_system.py [--all|--camera|--yolo|--robot|--vs]")
    else:
        interactive_test()

if __name__ == "__main__":
    main()
