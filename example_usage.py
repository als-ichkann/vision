#!/usr/bin/env python
# coding=utf-8
"""
机器人视觉伺服系统使用示例
"""

import time
import numpy as np
from robot_visual_servoing import RobotPBVS, RobotIBVS

def pbvs_example():
    """PBVS使用示例"""
    print("=== PBVS 使用示例 ===")
    
    # 创建PBVS实例
    pbvs = RobotPBVS(
        robot_ip='localhost',  # 机器人IP
        robot_port=8899,       # 机器人端口
        model_path='yolo_train/weights/best.pt',  # YOLO模型路径
        calibration_file='camera_calibration.json'  # 相机标定文件
    )
    
    # 设置期望位置（相机坐标系）
    pbvs.desired_position = np.array([0.0, 0.0, 0.4])  # 前方40cm
    
    # 设置控制参数
    pbvs.lambda_pos = 0.3  # 位置控制增益
    pbvs.lambda_rot = 0.2  # 旋转控制增益
    pbvs.max_linear_velocity = 0.05  # 最大线速度
    
    # 运行PBVS
    try:
        pbvs.run()
    except KeyboardInterrupt:
        print("用户中断PBVS")
    finally:
        pbvs.cleanup()

def ibvs_example():
    """IBVS使用示例"""
    print("=== IBVS 使用示例 ===")
    
    # 创建IBVS实例
    ibvs = RobotIBVS(
        robot_ip='localhost',
        robot_port=8899,
        model_path='yolo_train/weights/best.pt',
        calibration_file='camera_calibration.json'
    )
    
    # 设置期望图像特征 [x, y, width, height]
    ibvs.desired_features = np.array([320, 240, 120, 120])
    
    # 设置控制参数
    ibvs.lambda_pos = 0.3  # 特征控制增益
    ibvs.feature_deadzone = 8  # 特征死区（像素）
    ibvs.max_linear_velocity = 0.05
    
    # 运行IBVS
    try:
        ibvs.run()
    except KeyboardInterrupt:
        print("用户中断IBVS")
    finally:
        ibvs.cleanup()

def custom_pbvs_example():
    """自定义PBVS参数示例"""
    print("=== 自定义PBVS参数示例 ===")
    
    pbvs = RobotPBVS()
    
    # 自定义期望位置和姿态
    pbvs.desired_position = np.array([0.1, 0.0, 0.5])  # 右侧10cm，前方50cm
    pbvs.desired_orientation = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])  # 向下看的姿态
    
    # 自定义目标3D模型（假设目标是矩形，20cm x 10cm）
    pbvs.object_3d_points = np.array([
        [-0.1, -0.05, 0],
        [0.1, -0.05, 0],
        [0.1, 0.05, 0],
        [-0.1, 0.05, 0]
    ], dtype=np.float32)
    
    # 调整控制参数
    pbvs.lambda_pos = 0.2  # 降低位置增益
    pbvs.lambda_rot = 0.1  # 降低旋转增益
    pbvs.position_deadzone = 0.005  # 更小的位置死区
    pbvs.max_linear_velocity = 0.03  # 更慢的速度
    
    try:
        pbvs.run()
    except KeyboardInterrupt:
        print("用户中断自定义PBVS")
    finally:
        pbvs.cleanup()

def monitoring_example():
    """监控模式示例（仅检测，不控制机器人）"""
    print("=== 监控模式示例 ===")
    
    pbvs = RobotPBVS()
    
    # 不连接机器人，仅用于视觉检测
    print("监控模式：仅进行目标检测和位姿估计")
    print("按 'q' 退出")
    
    try:
        while True:
            # 获取相机帧
            color_image, depth_image = pbvs.get_aligned_frames()
            if color_image is None:
                continue
            
            # 检测目标
            target = pbvs.detect_target(color_image)
            
            if target is not None:
                # 估计目标位姿
                target_pose = pbvs.estimate_target_pose(target)
                
                if target_pose is not None:
                    pos = target_pose['position']
                    print(f"检测到目标 - 位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), "
                          f"置信度: {target['confidence']:.2f}")
                
                # 可视化
                pbvs.visualize_pbvs(color_image, target, target_pose)
            else:
                print("未检测到目标")
            
            # 显示图像
            import cv2
            cv2.imshow('Monitoring', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("用户中断监控")
    finally:
        pbvs.cleanup()

def parameter_tuning_example():
    """参数调优示例"""
    print("=== 参数调优示例 ===")
    
    pbvs = RobotPBVS()
    
    # 测试不同的控制参数
    test_parameters = [
        {'lambda_pos': 0.1, 'lambda_rot': 0.05, 'name': '保守参数'},
        {'lambda_pos': 0.3, 'lambda_rot': 0.2, 'name': '标准参数'},
        {'lambda_pos': 0.5, 'lambda_rot': 0.3, 'name': '激进参数'}
    ]
    
    for params in test_parameters:
        print(f"\n测试 {params['name']}:")
        print(f"  lambda_pos: {params['lambda_pos']}")
        print(f"  lambda_rot: {params['lambda_rot']}")
        
        pbvs.lambda_pos = params['lambda_pos']
        pbvs.lambda_rot = params['lambda_rot']
        
        # 这里可以运行一个短时间的测试
        print(f"  参数设置完成，可以运行测试")
        
        # 实际使用时，可以在这里调用 pbvs.run() 并记录性能数据
        
        choice = input("  是否继续下一组参数? (y/N): ")
        if choice.lower() != 'y':
            break
    
    pbvs.cleanup()

def main():
    """主函数"""
    print("=== 机器人视觉伺服系统使用示例 ===")
    print("1. PBVS 基础示例")
    print("2. IBVS 基础示例")
    print("3. 自定义PBVS参数示例")
    print("4. 监控模式示例")
    print("5. 参数调优示例")
    print("0. 退出")
    
    while True:
        choice = input("\n请选择示例 (0-5): ").strip()
        
        if choice == "1":
            pbvs_example()
        elif choice == "2":
            ibvs_example()
        elif choice == "3":
            custom_pbvs_example()
        elif choice == "4":
            monitoring_example()
        elif choice == "5":
            parameter_tuning_example()
        elif choice == "0":
            print("退出示例程序")
            break
        else:
            print("无效选择，请重新输入")

if __name__ == "__main__":
    main()
