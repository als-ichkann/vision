#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auboi5机械臂PyBullet仿真演示
展示机械臂的运动能力和相机视图
"""

import numpy as np
import cv2
import time
from pybullet_robot_adapter import Auboi5Robot, RobotErrorType

def demo_joint_movements():
    """演示关节运动"""
    print("=== Auboi5机械臂关节运动演示 ===")
    
    # 创建机械臂
    robot = Auboi5Robot(gui=True)
    
    try:
        # 初始化
        Auboi5Robot.initialize()
        robot.create_context()
        
        # 连接
        result = robot.connect()
        if result != RobotErrorType.RobotError_SUCC:
            print("连接失败")
            return
        
        # 启动
        robot.robot_startup()
        robot.init_profile()
        
        print("机械臂初始化完成，开始演示...")
        
        # 定义几个关键姿态
        poses = [
            [0, 0, 0, 0, 0, 0],                    # 零位
            [np.pi/4, -np.pi/6, np.pi/3, 0, np.pi/6, 0],  # 姿态1
            [-np.pi/4, -np.pi/4, np.pi/2, np.pi/2, -np.pi/4, np.pi/2],  # 姿态2
            [0, -np.pi/3, np.pi/2, 0, np.pi/3, 0],        # 姿态3
            [np.pi/2, 0, 0, 0, 0, np.pi],                 # 姿态4
            [0, 0, 0, 0, 0, 0],                    # 回到零位
        ]
        
        pose_names = [
            "零位姿态",
            "前伸姿态", 
            "侧向姿态",
            "工作姿态",
            "展示姿态",
            "回零位置"
        ]
        
        for i, (pose, name) in enumerate(zip(poses, pose_names)):
            print(f"\n步骤 {i+1}: {name}")
            print(f"目标关节角度: {[f'{angle:.2f}' for angle in pose]}")
            
            # 移动到目标位置
            result = robot.move_joint(pose)
            if result == RobotErrorType.RobotError_SUCC:
                print("✓ 运动完成")
                
                # 获取当前位置
                waypoint = robot.get_current_waypoint()
                if waypoint:
                    print(f"当前位置: {[f'{joint:.2f}' for joint in waypoint['joint']]}")
                    print(f"末端位置: {[f'{pos:.3f}' for pos in waypoint['pos']]}")
                
                # 获取相机图像
                color_image, depth_image = robot.get_camera_image()
                if color_image is not None:
                    cv2.putText(color_image, f"Pose {i+1}: {name}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(color_image, "Press 'q' to continue", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow('Auboi5 Camera View', color_image)
                    
                    # 等待用户按键或自动继续
                    key = cv2.waitKey(3000) & 0xFF
                    if key == ord('q'):
                        print("用户跳过当前姿态")
                    elif key == 27:  # ESC键
                        print("用户退出演示")
                        break
                else:
                    time.sleep(2)
            else:
                print("✗ 运动失败")
            
            # 仿真步进
            for _ in range(50):
                robot.step_simulation()
                time.sleep(0.02)
        
        print("\n演示完成！")
        
    except KeyboardInterrupt:
        print("演示中断")
    except Exception as e:
        print(f"演示错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        cv2.destroyAllWindows()
        robot.robot_shutdown()
        robot.disconnect()
        Auboi5Robot.uninitialize()

def demo_workspace():
    """演示工作空间"""
    print("=== Auboi5机械臂工作空间演示 ===")
    
    robot = Auboi5Robot(gui=True)
    
    try:
        # 初始化
        Auboi5Robot.initialize()
        robot.create_context()
        robot.connect()
        robot.robot_startup()
        robot.init_profile()
        
        print("绘制工作空间边界...")
        
        # 在不同高度平面上测试可达性
        heights = [0.3, 0.5, 0.7, 0.9]
        
        for height in heights:
            print(f"\n测试高度: {height}m")
            
            # 在该高度的圆周上测试点
            radius_range = np.linspace(0.2, 0.8, 5)
            angle_range = np.linspace(0, 2*np.pi, 8)
            
            reachable_points = []
            
            for radius in radius_range:
                for angle in angle_range:
                    target_x = radius * np.cos(angle)
                    target_y = radius * np.sin(angle)
                    target_z = height
                    
                    # 测试该点是否可达
                    current_waypoint = robot.get_current_waypoint()
                    if current_waypoint:
                        ik_result = robot.inverse_kin(
                            current_waypoint['joint'],
                            [target_x, target_y, target_z],
                            [1, 0, 0, 0]  # 简单的姿态
                        )
                        
                        if ik_result:
                            reachable_points.append([target_x, target_y, target_z])
            
            print(f"在高度{height}m处找到{len(reachable_points)}个可达点")
        
        print("工作空间分析完成")
        
    except Exception as e:
        print(f"工作空间演示错误: {e}")
    finally:
        robot.robot_shutdown()
        robot.disconnect()
        Auboi5Robot.uninitialize()

def main():
    """主函数"""
    print("Auboi5机械臂PyBullet仿真演示")
    print("1. 关节运动演示")
    print("2. 工作空间演示")
    
    choice = input("请选择演示类型 (1-2): ").strip()
    
    if choice == "1":
        demo_joint_movements()
    elif choice == "2":
        demo_workspace()
    else:
        print("无效选择，运行关节运动演示")
        demo_joint_movements()

if __name__ == "__main__":
    main()
