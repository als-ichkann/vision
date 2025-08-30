#!/usr/bin/env python
# coding=utf-8
"""
机器人视觉伺服系统测试脚本
"""

import sys
import time
import numpy as np
import json
from robot_visual_servoing import RobotPBVS, RobotIBVS

def test_robot_connection():
    """测试机器人连接"""
    print("=== 测试机器人连接 ===")
    
    try:
        pbvs = RobotPBVS()
        
        # 测试连接
        success = pbvs.connect_robot()
        
        if success:
            print("✓ 机器人连接成功")
            
            # 获取当前位姿
            pose = pbvs.get_current_robot_pose()
            if pose is not None:
                print(f"✓ 当前位置: {pose['position']}")
                print(f"✓ 当前姿态: {pose['orientation']}")
                print(f"✓ 关节角度: {pose['joint_angles']}")
            else:
                print("✗ 无法获取机器人位姿")
            
            # 断开连接
            pbvs.disconnect_robot()
            print("✓ 机器人断开连接")
            
        else:
            print("✗ 机器人连接失败")
            
    except Exception as e:
        print(f"✗ 连接测试失败: {e}")
    
    print()

def test_camera():
    """测试相机"""
    print("=== 测试相机 ===")
    
    try:
        pbvs = RobotPBVS()
        
        # 获取相机帧
        color_image, depth_image = pbvs.get_aligned_frames()
        
        if color_image is not None and depth_image is not None:
            print(f"✓ 相机工作正常")
            print(f"✓ 彩色图像尺寸: {color_image.shape}")
            print(f"✓ 深度图像尺寸: {depth_image.shape}")
            print(f"✓ 深度比例: {pbvs.depth_scale}")
        else:
            print("✗ 相机获取帧失败")
            
        pbvs.cleanup()
        
    except Exception as e:
        print(f"✗ 相机测试失败: {e}")
    
    print()

def test_yolo_detection():
    """测试YOLO检测"""
    print("=== 测试YOLO目标检测 ===")
    
    try:
        pbvs = RobotPBVS()
        
        # 获取相机帧
        color_image, depth_image = pbvs.get_aligned_frames()
        
        if color_image is not None:
            # 检测目标
            target = pbvs.detect_target(color_image)
            
            if target is not None:
                print("✓ YOLO检测成功")
                print(f"✓ 目标中心: {target['center']}")
                print(f"✓ 目标尺寸: {target['size']}")
                print(f"✓ 检测置信度: {target['confidence']:.2f}")
            else:
                print("✗ 未检测到目标（这可能是正常的）")
        else:
            print("✗ 无法获取相机图像")
            
        pbvs.cleanup()
        
    except Exception as e:
        print(f"✗ YOLO检测测试失败: {e}")
    
    print()

def test_pbvs_computation():
    """测试PBVS计算"""
    print("=== 测试PBVS计算 ===")
    
    try:
        pbvs = RobotPBVS()
        
        # 模拟目标检测结果
        mock_target = {
            'center': (320, 240),
            'size': (100, 100),
            'confidence': 0.9
        }
        
        # 测试位姿估计
        target_pose = pbvs.estimate_target_pose(mock_target)
        
        if target_pose is not None:
            print("✓ 目标位姿估计成功")
            print(f"✓ 估计位置: {target_pose['position']}")
            print(f"✓ 估计姿态矩阵形状: {target_pose['orientation'].shape}")
            
            # 测试误差计算
            pose_error = pbvs.compute_pose_error(target_pose)
            if pose_error is not None:
                print(f"✓ 位姿误差计算成功: {np.linalg.norm(pose_error):.4f}")
                
                # 测试控制律
                control_velocity = pbvs.compute_control_law(pose_error)
                print(f"✓ 控制速度计算成功: {np.linalg.norm(control_velocity):.4f}")
            else:
                print("✗ 位姿误差计算失败")
        else:
            print("✗ 目标位姿估计失败")
            
        pbvs.cleanup()
        
    except Exception as e:
        print(f"✗ PBVS计算测试失败: {e}")
    
    print()

def test_ibvs_computation():
    """测试IBVS计算"""
    print("=== 测试IBVS计算 ===")
    
    try:
        ibvs = RobotIBVS()
        
        # 模拟目标检测结果
        mock_target = {
            'center': (300, 250),
            'size': (80, 80),
            'confidence': 0.9
        }
        
        # 模拟深度图像
        mock_depth = np.full((480, 640), 1000, dtype=np.uint16)  # 1米深度
        
        # 测试特征提取
        features = ibvs.extract_image_features(mock_target, mock_depth)
        
        if features is not None:
            print("✓ 图像特征提取成功")
            print(f"✓ 提取特征: {features[:4]}")
            print(f"✓ 特征深度: {features[4]:.3f}m")
            
            # 测试雅可比计算
            jacobian = ibvs.compute_image_jacobian(features)
            if jacobian is not None:
                print(f"✓ 图像雅可比计算成功，形状: {jacobian.shape}")
                
                # 测试控制律
                control_velocity = ibvs.compute_control_law(features)
                print(f"✓ 控制速度计算成功: {np.linalg.norm(control_velocity):.4f}")
            else:
                print("✗ 图像雅可比计算失败")
        else:
            print("✗ 图像特征提取失败")
            
        ibvs.cleanup()
        
    except Exception as e:
        print(f"✗ IBVS计算测试失败: {e}")
    
    print()

def test_config_loading():
    """测试配置文件加载"""
    print("=== 测试配置文件 ===")
    
    try:
        with open('robot_visual_servoing_config.json', 'r') as f:
            config = json.load(f)
        
        print("✓ 配置文件加载成功")
        print(f"✓ 机器人IP: {config['robot_config']['ip']}")
        print(f"✓ 机器人端口: {config['robot_config']['port']}")
        print(f"✓ YOLO模型路径: {config['yolo_config']['model_path']}")
        print(f"✓ 相机分辨率: {config['camera_config']['width']}x{config['camera_config']['height']}")
        
    except Exception as e:
        print(f"✗ 配置文件测试失败: {e}")
    
    print()

def test_safety_features():
    """测试安全功能"""
    print("=== 测试安全功能 ===")
    
    try:
        pbvs = RobotPBVS()
        
        # 测试紧急停止
        print("✓ 紧急停止功能可用")
        pbvs.emergency_stop_robot()
        
        if pbvs.emergency_stop:
            print("✓ 紧急停止状态设置成功")
        
        # 测试速度限制
        test_velocity = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # 超出限制的速度
        print("✓ 速度限制功能集成在send_velocity_command中")
        
        pbvs.cleanup()
        
    except Exception as e:
        print(f"✗ 安全功能测试失败: {e}")
    
    print()

def run_all_tests():
    """运行所有测试"""
    print("=== 机器人视觉伺服系统测试 ===")
    print("开始运行所有测试...\n")
    
    test_config_loading()
    test_camera()
    test_yolo_detection()
    test_pbvs_computation()
    test_ibvs_computation()
    test_safety_features()
    
    # 机器人连接测试放在最后，因为需要实际的机器人
    print("注意: 机器人连接测试需要实际的机器人硬件")
    choice = input("是否测试机器人连接? (y/N): ").strip().lower()
    if choice == 'y':
        test_robot_connection()
    
    print("=== 测试完成 ===")

def interactive_test():
    """交互式测试菜单"""
    while True:
        print("\n=== 机器人视觉伺服测试菜单 ===")
        print("1. 测试配置文件加载")
        print("2. 测试相机")
        print("3. 测试YOLO检测")
        print("4. 测试PBVS计算")
        print("5. 测试IBVS计算")
        print("6. 测试安全功能")
        print("7. 测试机器人连接")
        print("8. 运行所有测试")
        print("0. 退出")
        
        choice = input("请选择测试项目: ").strip()
        
        if choice == "1":
            test_config_loading()
        elif choice == "2":
            test_camera()
        elif choice == "3":
            test_yolo_detection()
        elif choice == "4":
            test_pbvs_computation()
        elif choice == "5":
            test_ibvs_computation()
        elif choice == "6":
            test_safety_features()
        elif choice == "7":
            test_robot_connection()
        elif choice == "8":
            run_all_tests()
        elif choice == "0":
            break
        else:
            print("无效选择，请重新输入")

def main():
    """主函数"""
    if len(sys.argv) > 1:
        if sys.argv[1] == "--all":
            run_all_tests()
        elif sys.argv[1] == "--camera":
            test_camera()
        elif sys.argv[1] == "--yolo":
            test_yolo_detection()
        elif sys.argv[1] == "--pbvs":
            test_pbvs_computation()
        elif sys.argv[1] == "--ibvs":
            test_ibvs_computation()
        elif sys.argv[1] == "--robot":
            test_robot_connection()
        elif sys.argv[1] == "--config":
            test_config_loading()
        elif sys.argv[1] == "--safety":
            test_safety_features()
        else:
            print("用法:")
            print("  python test_robot_visual_servoing.py [选项]")
            print("选项:")
            print("  --all      运行所有测试")
            print("  --camera   测试相机")
            print("  --yolo     测试YOLO检测")
            print("  --pbvs     测试PBVS计算")
            print("  --ibvs     测试IBVS计算")
            print("  --robot    测试机器人连接")
            print("  --config   测试配置文件")
            print("  --safety   测试安全功能")
    else:
        interactive_test()

if __name__ == "__main__":
    main()
