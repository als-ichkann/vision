#!/usr/bin/env python
# coding=utf-8
"""
基于robotcontrol.py的视觉伺服驱动系统
整合PBVS和IBVS功能的简化版本
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import json
import time
from scipy.spatial.transform import Rotation as R
from typing import Dict, Optional
import logging

# 导入机器人控制模块
from robotcontrol import Auboi5Robot, RobotErrorType, logger_init

class RobotVisualServoing:
    """机器人视觉伺服系统"""
    
    def __init__(self, robot_ip='localhost', robot_port=8899, 
                 model_path='yolo_train/weights/best.pt'):
        """初始化视觉伺服系统"""
        
        # 初始化日志
        logger_init()
        
        # 机器人控制器
        self.robot = Auboi5Robot()
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_connected = False
        
        # 加载YOLO模型
        self.model = YOLO(model_path)
        print(f"YOLO模型加载成功: {model_path}")
        
        # 相机参数（RealSense D435i默认参数）
        self.camera_matrix = np.array([
            [615.0, 0, 320.0],
            [0, 615.0, 240.0],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0])
        
        # 设置相机
        self.setup_camera()
        
        # 控制参数
        self.lambda_pos = 0.3
        self.lambda_rot = 0.2
        self.max_linear_velocity = 0.05
        self.max_angular_velocity = 0.3
        
        # PBVS参数
        self.desired_position = np.array([0.0, 0.0, 0.4])  # 期望位置
        self.object_3d_points = np.array([  # 目标3D模型点
            [-0.05, -0.05, 0], [0.05, -0.05, 0],
            [0.05, 0.05, 0], [-0.05, 0.05, 0]
        ], dtype=np.float32)
        
        # IBVS参数
        self.desired_features = np.array([320, 240, 120, 120])  # 期望特征
        
        # 安全参数
        self.emergency_stop = False
        self.target_lost_count = 0
        self.max_target_lost = 20
        
    def setup_camera(self):
        """设置RealSense相机"""
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.align = rs.align(rs.stream.color)
            print("相机初始化成功")
        except Exception as e:
            print(f"相机初始化失败: {e}")
            raise
    
    def connect_robot(self):
        """连接机器人"""
        try:
            Auboi5Robot.initialize()
            handle = self.robot.create_context()
            result = self.robot.connect(self.robot_ip, self.robot_port)
            
            if result != RobotErrorType.RobotError_SUCC:
                return False
            
            self.robot.robot_startup()
            self.robot.init_profile()
            self.robot.set_joint_maxacc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
            self.robot.set_joint_maxvelc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
            self.robot.set_end_max_line_acc(0.1)
            self.robot.set_end_max_line_velc(0.05)
            self.robot.set_collision_class(6)
            
            self.robot_connected = True
            print("机器人连接成功")
            return True
        except Exception as e:
            print(f"机器人连接失败: {e}")
            return False
    
    def get_frames(self):
        """获取RGB-D帧"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None
                
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            return color_image, depth_image
        except:
            return None, None
    
    def detect_target(self, color_image):
        """检测目标"""
        try:
            results = self.model(color_image)
            if len(results[0].boxes) == 0:
                self.target_lost_count += 1
                return None
            
            self.target_lost_count = 0
            best_box = max(results[0].boxes, key=lambda x: x.conf[0].cpu().numpy())
            
            if best_box.conf[0].cpu().numpy() > 0.5:
                x_center, y_center, width, height = best_box.xywh[0].cpu().numpy()
                return {
                    'center': (int(x_center), int(y_center)),
                    'size': (int(width), int(height)),
                    'confidence': float(best_box.conf[0].cpu().numpy())
                }
            return None
        except:
            return None
    
    def send_velocity_command(self, velocity):
        """发送速度命令"""
        if not self.robot_connected or self.emergency_stop:
            return False
        
        try:
            # 限制速度
            linear_vel = velocity[:3]
            linear_speed = np.linalg.norm(linear_vel)
            if linear_speed > self.max_linear_velocity:
                linear_vel = linear_vel / linear_speed * self.max_linear_velocity
            
            # 获取当前位姿并计算目标位置
            waypoint = self.robot.get_current_waypoint()
            if waypoint is None:
                return False
            
            dt = 0.1
            target_pos = np.array(waypoint['pos']) + linear_vel * dt
            
            # 使用逆运动学
            ik_result = self.robot.inverse_kin(waypoint['joint'], target_pos, waypoint['ori'])
            if ik_result is not None:
                result = self.robot.move_joint(ik_result['joint'], False)
                return result == RobotErrorType.RobotError_SUCC
            return False
        except:
            return False
    
    def emergency_stop_robot(self):
        """紧急停止"""
        self.emergency_stop = True
        if self.robot_connected:
            try:
                self.robot.move_stop()
                print("机器人紧急停止")
            except:
                pass
    
    def run_pbvs(self):
        """运行PBVS"""
        print("=== 启动PBVS视觉伺服 ===")
        print("按 'q' 退出，按 'e' 紧急停止")
        
        if not self.connect_robot():
            print("机器人连接失败")
            return
        
        try:
            while True:
                color_image, depth_image = self.get_frames()
                if color_image is None:
                    continue
                
                target = self.detect_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        self.emergency_stop_robot()
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    # PBVS控制
                    target_pose = self.estimate_target_pose_pbvs(target)
                    if target_pose and not self.emergency_stop:
                        pose_error = self.compute_pose_error_pbvs(target_pose)
                        if pose_error is not None:
                            control_velocity = self.compute_control_law_pbvs(pose_error)
                            self.send_velocity_command(control_velocity)
                            
                            error_norm = np.linalg.norm(pose_error)
                            print(f"PBVS - 位姿误差: {error_norm:.4f}")
                    
                    # 可视化
                    self.visualize_pbvs(color_image, target)
                
                cv2.imshow('Robot PBVS', color_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('e'):
                    self.emergency_stop_robot()
                    break
                    
        except KeyboardInterrupt:
            print("用户中断")
        finally:
            self.cleanup()
    
    def run_ibvs(self):
        """运行IBVS"""
        print("=== 启动IBVS视觉伺服 ===")
        print("按 'q' 退出，按 'e' 紧急停止，按 'r' 重设期望特征")
        
        if not self.connect_robot():
            print("机器人连接失败")
            return
        
        try:
            while True:
                color_image, depth_image = self.get_frames()
                if color_image is None:
                    continue
                
                target = self.detect_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        self.emergency_stop_robot()
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    # IBVS控制
                    current_features = self.extract_image_features_ibvs(target, depth_image)
                    if current_features is not None and not self.emergency_stop:
                        control_velocity = self.compute_control_law_ibvs(current_features)
                        self.send_velocity_command(control_velocity)
                        
                        feature_error = self.desired_features - current_features[:4]
                        error_norm = np.linalg.norm(feature_error)
                        print(f"IBVS - 特征误差: {error_norm:.2f}")
                    
                    # 可视化
                    self.visualize_ibvs(color_image, target)
                
                cv2.imshow('Robot IBVS', color_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('e'):
                    self.emergency_stop_robot()
                    break
                elif key == ord('r') and target:
                    # 重设期望特征
                    self.desired_features = np.array([
                        target['center'][0], target['center'][1],
                        target['size'][0], target['size'][1]
                    ])
                    print(f"期望特征已更新: {self.desired_features}")
                    
        except KeyboardInterrupt:
            print("用户中断")
        finally:
            self.cleanup()
    
    # PBVS相关方法
    def estimate_target_pose_pbvs(self, target):
        """估计目标位姿（PBVS）"""
        try:
            x_center, y_center = target['center']
            width, height = target['size']
            
            image_points = np.array([
                [x_center - width/2, y_center - height/2],
                [x_center + width/2, y_center - height/2],
                [x_center + width/2, y_center + height/2],
                [x_center - width/2, y_center + height/2]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                self.object_3d_points, image_points,
                self.camera_matrix, self.dist_coeffs
            )
            
            if success:
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                return {
                    'position': tvec.flatten(),
                    'orientation': rotation_matrix
                }
            return None
        except:
            return None
    
    def compute_pose_error_pbvs(self, current_pose):
        """计算位姿误差（PBVS）"""
        try:
            position_error = self.desired_position - current_pose['position']
            current_rot = R.from_matrix(current_pose['orientation'])
            desired_rot = R.from_matrix(np.eye(3))
            rotation_error = (desired_rot * current_rot.inv()).as_rotvec()
            return np.concatenate([position_error, rotation_error])
        except:
            return None
    
    def compute_control_law_pbvs(self, pose_error):
        """计算PBVS控制律"""
        position_error = pose_error[:3]
        rotation_error = pose_error[3:]
        
        # 应用死区
        if np.linalg.norm(position_error) < 0.01:
            position_error = np.zeros(3)
        if np.linalg.norm(rotation_error) < 0.05:
            rotation_error = np.zeros(3)
        
        linear_velocity = self.lambda_pos * position_error
        angular_velocity = self.lambda_rot * rotation_error
        
        return np.concatenate([linear_velocity, angular_velocity])
    
    def visualize_pbvs(self, image, target):
        """可视化PBVS"""
        if target:
            x, y = target['center']
            w, h = target['size']
            cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(image, f"Conf: {target['confidence']:.2f}", 
                       (x-w//2, y-h//2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 显示期望位置
        if self.desired_position[2] > 0:
            desired_u = int(self.desired_position[0] * self.camera_matrix[0, 0] / 
                           self.desired_position[2] + self.camera_matrix[0, 2])
            desired_v = int(self.desired_position[1] * self.camera_matrix[1, 1] / 
                           self.desired_position[2] + self.camera_matrix[1, 2])
            cv2.circle(image, (desired_u, desired_v), 8, (0, 0, 255), 2)
        
        # 状态信息
        status = "PBVS Mode" + (" - EMERGENCY STOP" if self.emergency_stop else "")
        cv2.putText(image, status, (10, image.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if self.emergency_stop else (255, 255, 255), 2)
    
    # IBVS相关方法
    def extract_image_features_ibvs(self, target, depth_image):
        """提取图像特征（IBVS）"""
        try:
            x, y = target['center']
            width, height = target['size']
            
            depth_value = depth_image[y, x] * self.depth_scale
            if depth_value == 0:
                depth_value = 0.4  # 默认深度
            
            return np.array([x, y, width, height, depth_value])
        except:
            return None
    
    def compute_control_law_ibvs(self, current_features):
        """计算IBVS控制律"""
        try:
            feature_error = self.desired_features - current_features[:4]
            
            # 应用死区
            if np.linalg.norm(feature_error) < 8:
                return np.zeros(6)
            
            # 计算图像雅可比矩阵
            x, y, width, height, z = current_features
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            
            x_norm = (x - cx) / fx
            y_norm = (y - cy) / fy
            
            L = np.zeros((4, 6))
            L[0, :] = [-fx/z, 0, x_norm/z, x_norm*y_norm/fx, -(fx + x_norm**2*fx)/fx, y_norm]
            L[1, :] = [0, -fy/z, y_norm/z, (fy + y_norm**2*fy)/fy, -x_norm*y_norm/fy, -x_norm]
            L[2, :] = [-fx/z, 0, width/z, 0, 0, 0]
            L[3, :] = [0, -fy/z, height/z, 0, 0, 0]
            
            L_pinv = np.linalg.pinv(L)
            control_velocity = -self.lambda_pos * L_pinv @ feature_error
            return control_velocity
        except:
            return np.zeros(6)
    
    def visualize_ibvs(self, image, target):
        """可视化IBVS"""
        # 当前特征
        if target:
            x, y = target['center']
            w, h = target['size']
            cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        
        # 期望特征
        desired_x, desired_y, desired_w, desired_h = self.desired_features
        cv2.rectangle(image, 
                     (int(desired_x-desired_w/2), int(desired_y-desired_h/2)),
                     (int(desired_x+desired_w/2), int(desired_y+desired_h/2)),
                     (0, 0, 255), 2)
        cv2.circle(image, (int(desired_x), int(desired_y)), 8, (0, 0, 255), 2)
        
        # 状态信息
        status = "IBVS Mode" + (" - EMERGENCY STOP" if self.emergency_stop else "")
        cv2.putText(image, status, (10, image.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if self.emergency_stop else (255, 255, 255), 2)
    
    def cleanup(self):
        """清理资源"""
        try:
            self.pipeline.stop()
        except:
            pass
        
        if self.robot_connected:
            try:
                self.robot.move_stop()
                self.robot.robot_shutdown()
                self.robot.disconnect()
                Auboi5Robot.uninitialize()
            except:
                pass
        
        cv2.destroyAllWindows()

def main():
    """主函数"""
    print("=== 机器人视觉伺服系统 ===")
    print("1. PBVS - 基于位置的视觉伺服")
    print("2. IBVS - 基于图像的视觉伺服")
    
    choice = input("请选择模式 (1-2): ").strip()
    
    # 获取机器人连接参数
    robot_ip = input("机器人IP地址 (默认: localhost): ").strip() or "localhost"
    robot_port = int(input("机器人端口 (默认: 8899): ").strip() or "8899")
    
    # 创建视觉伺服系统
    vs = RobotVisualServoing(robot_ip=robot_ip, robot_port=robot_port)
    
    if choice == "1":
        vs.run_pbvs()
    elif choice == "2":
        vs.run_ibvs()
    else:
        print("无效选择")

if __name__ == "__main__":
    main()
