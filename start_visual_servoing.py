#!/usr/bin/env python
# coding=utf-8
"""
机器人视觉伺服系统快速启动脚本
"""

import sys
import argparse
from robot_visual_servoing import RobotPBVS, RobotIBVS

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='机器人视觉伺服系统')
    parser.add_argument('mode', choices=['pbvs', 'ibvs'], help='视觉伺服模式')
    parser.add_argument('--robot-ip', default='localhost', help='机器人IP地址')
    parser.add_argument('--robot-port', type=int, default=8899, help='机器人端口')
    parser.add_argument('--model', default='yolo_train/weights/best.pt', help='YOLO模型路径')
    parser.add_argument('--calibration', default='camera_calibration.json', help='相机标定文件')
    parser.add_argument('--lambda-pos', type=float, default=0.3, help='位置控制增益')
    parser.add_argument('--lambda-rot', type=float, default=0.2, help='旋转控制增益')
    parser.add_argument('--max-vel', type=float, default=0.05, help='最大线速度')
    parser.add_argument('--max-ang-vel', type=float, default=0.3, help='最大角速度')
    
    args = parser.parse_args()
    
    print(f"=== 启动 {args.mode.upper()} 视觉伺服系统 ===")
    print(f"机器人地址: {args.robot_ip}:{args.robot_port}")
    print(f"YOLO模型: {args.model}")
    print(f"相机标定: {args.calibration}")
    print()
    
    try:
        if args.mode == 'pbvs':
            # 启动PBVS
            pbvs = RobotPBVS(
                robot_ip=args.robot_ip,
                robot_port=args.robot_port,
                model_path=args.model,
                calibration_file=args.calibration
            )
            
            # 设置参数
            pbvs.lambda_pos = args.lambda_pos
            pbvs.lambda_rot = args.lambda_rot
            pbvs.max_linear_velocity = args.max_vel
            pbvs.max_angular_velocity = args.max_ang_vel
            
            print("PBVS参数:")
            print(f"  位置增益: {pbvs.lambda_pos}")
            print(f"  旋转增益: {pbvs.lambda_rot}")
            print(f"  最大线速度: {pbvs.max_linear_velocity} m/s")
            print(f"  最大角速度: {pbvs.max_angular_velocity} rad/s")
            print()
            
            pbvs.run()
            
        elif args.mode == 'ibvs':
            # 启动IBVS
            ibvs = RobotIBVS(
                robot_ip=args.robot_ip,
                robot_port=args.robot_port,
                model_path=args.model,
                calibration_file=args.calibration
            )
            
            # 设置参数
            ibvs.lambda_pos = args.lambda_pos
            ibvs.max_linear_velocity = args.max_vel
            ibvs.max_angular_velocity = args.max_ang_vel
            
            print("IBVS参数:")
            print(f"  特征增益: {ibvs.lambda_pos}")
            print(f"  最大线速度: {ibvs.max_linear_velocity} m/s")
            print(f"  最大角速度: {ibvs.max_angular_velocity} rad/s")
            print(f"  期望特征: {ibvs.desired_features}")
            print()
            
            ibvs.run()
            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行错误: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
