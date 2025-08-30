#!/usr/bin/env python
# coding=utf-8
"""
基于robotcontrol.py的视觉伺服驱动代码
整合PBVS和IBVS视觉伺服功能
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import json
import time
import threading
import queue
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Optional, Dict, List
import logging

# 导入机器人控制模块
from robotcontrol import Auboi5Robot, RobotErrorType, RobotDefaultParameters, logger_init

class RobotVisualServoingBase:
    """机器人视觉伺服基类"""
    
    def __init__(self, robot_ip='localhost', robot_port=8899, 
                 model_path='yolo_train/weights/best.pt', 
                 calibration_file='camera_calibration.json'):
        """
        初始化视觉伺服系统
        
        Args:
            robot_ip: 机器人IP地址
            robot_port: 机器人端口
            model_path: YOLO模型路径
            calibration_file: 相机标定文件路径
        """
        # 初始化日志
        logger_init()
        
        # 机器人控制器
        self.robot = Auboi5Robot()
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_connected = False
        
        # 加载YOLO模型
        try:
            self.model = YOLO(model_path)
            print(f"YOLO模型加载成功: {model_path}")
        except Exception as e:
            print(f"YOLO模型加载失败: {e}")
            raise
        
        # 加载相机标定参数
        self.load_camera_calibration(calibration_file)
        
        # 设置相机
        self.setup_camera()
        
        # 控制参数
        self.lambda_pos = 0.3  # 位置控制增益
        self.lambda_rot = 0.2  # 旋转控制增益
        self.max_linear_velocity = 0.05  # 最大线速度 m/s
        self.max_angular_velocity = 0.3  # 最大角速度 rad/s
        self.convergence_threshold = 0.01  # 收敛阈值
        
        # 安全参数
        self.emergency_stop = False
        self.target_lost_count = 0
        self.max_target_lost = 20
        self.collision_detection = True
        
        # 数据记录
        self.error_history = []
        self.control_history = []
        self.pose_history = []
        
        # 控制线程
        self.control_thread = None
        self.control_active = False
        self.control_queue = queue.Queue()
        
    def load_camera_calibration(self, filename):
        """加载相机标定参数"""
        try:
            with open(filename, 'r') as f:
                calib_data = json.load(f)
            
            self.camera_matrix = np.array(calib_data['camera_matrix'])
            self.dist_coeffs = np.array(calib_data['dist_coeffs'])
            print("相机标定参数加载成功")
            
        except FileNotFoundError:
            print(f"警告: 未找到标定文件 {filename}，使用默认参数")
            # RealSense D435i 默认参数
            self.camera_matrix = np.array([
                [615.0, 0, 320.0],
                [0, 615.0, 240.0],
                [0, 0, 1]
            ])
            self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0])
    
    def setup_camera(self):
        """设置RealSense相机"""
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置流
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # 启动相机
            profile = self.pipeline.start(self.config)
            
            # 获取深度比例
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # 对齐深度到彩色图像
            self.align = rs.align(rs.stream.color)
            
            print("相机初始化成功")
            
        except Exception as e:
            print(f"相机初始化失败: {e}")
            raise
    
    def connect_robot(self):
        """连接机器人"""
        try:
            # 系统初始化
            Auboi5Robot.initialize()
            
            # 创建上下文
            handle = self.robot.create_context()
            print(f"机器人控制句柄: {handle}")
            
            # 连接机器人
            result = self.robot.connect(self.robot_ip, self.robot_port)
            
            if result != RobotErrorType.RobotError_SUCC:
                print(f"连接机器人失败: {self.robot_ip}:{self.robot_port}")
                return False
            
            # 启动机器人
            result = self.robot.robot_startup()
            if result != RobotErrorType.RobotError_SUCC:
                print("机器人启动失败")
                return False
            
            # 初始化配置
            self.robot.init_profile()
            
            # 设置关节限制
            self.robot.set_joint_maxacc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
            self.robot.set_joint_maxvelc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
            
            # 设置末端限制
            self.robot.set_end_max_line_acc(0.1)
            self.robot.set_end_max_line_velc(0.05)
            self.robot.set_end_max_angle_acc(0.5)
            self.robot.set_end_max_angle_velc(0.3)
            
            # 设置碰撞等级
            self.robot.set_collision_class(6)
            
            self.robot_connected = True
            print("机器人连接成功")
            return True
            
        except Exception as e:
            print(f"连接机器人时发生错误: {e}")
            return False
    
    def disconnect_robot(self):
        """断开机器人连接"""
        if self.robot_connected:
            try:
                self.robot.move_stop()
                self.robot.robot_shutdown()
                self.robot.disconnect()
                Auboi5Robot.uninitialize()
                self.robot_connected = False
                print("机器人断开连接")
            except Exception as e:
                print(f"断开机器人连接时发生错误: {e}")
    
    def get_aligned_frames(self):
        """获取对齐的RGB-D帧"""
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
            
        except Exception as e:
            print(f"获取相机帧失败: {e}")
            return None, None
    
    def detect_target(self, color_image):
        """检测目标对象"""
        try:
            results = self.model(color_image)
            
            if len(results[0].boxes) == 0:
                self.target_lost_count += 1
                return None
            
            # 重置丢失计数
            self.target_lost_count = 0
            
            # 获取置信度最高的检测结果
            best_box = None
            best_confidence = 0
            
            for box in results[0].boxes:
                confidence = box.conf[0].cpu().numpy()
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_box = box
            
            if best_box is not None and best_confidence > 0.5:
                x_center, y_center, width, height = best_box.xywh[0].cpu().numpy()
                x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
                
                return {
                    'center': (int(x_center), int(y_center)),
                    'size': (int(width), int(height)),
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'confidence': best_confidence
                }
            
            return None
            
        except Exception as e:
            print(f"目标检测失败: {e}")
            return None
    
    def get_current_robot_pose(self):
        """获取当前机器人位姿"""
        if not self.robot_connected:
            return None
            
        try:
            waypoint = self.robot.get_current_waypoint()
            if waypoint is None:
                return None
            
            # 返回位置和姿态
            return {
                'position': np.array(waypoint['pos']),
                'orientation': np.array(waypoint['ori']),  # 四元数 [w, x, y, z]
                'joint_angles': np.array(waypoint['joint'])
            }
            
        except Exception as e:
            print(f"获取机器人位姿失败: {e}")
            return None
    
    def send_velocity_command(self, velocity):
        """发送速度命令到机器人"""
        if not self.robot_connected or self.emergency_stop:
            return False
        
        try:
            # 限制速度
            linear_vel = velocity[:3]
            angular_vel = velocity[3:]
            
            # 限制线速度
            linear_speed = np.linalg.norm(linear_vel)
            if linear_speed > self.max_linear_velocity:
                linear_vel = linear_vel / linear_speed * self.max_linear_velocity
            
            # 限制角速度
            angular_speed = np.linalg.norm(angular_vel)
            if angular_speed > self.max_angular_velocity:
                angular_vel = angular_vel / angular_speed * self.max_angular_velocity
            
            # 获取当前位姿
            current_pose = self.get_current_robot_pose()
            if current_pose is None:
                return False
            
            # 计算目标位置（基于速度积分）
            dt = 0.1  # 时间步长
            target_pos = current_pose['position'] + linear_vel * dt
            
            # 计算目标姿态（简化处理）
            current_ori = current_pose['orientation']
            
            # 使用逆运动学计算关节角
            try:
                ik_result = self.robot.inverse_kin(
                    current_pose['joint_angles'], 
                    target_pos, 
                    current_ori
                )
                
                if ik_result is not None:
                    # 发送关节运动命令
                    result = self.robot.move_joint(ik_result['joint'], False)  # 异步运动
                    return result == RobotErrorType.RobotError_SUCC
                    
            except Exception as e:
                print(f"逆运动学计算失败: {e}")
                return False
            
            return False
            
        except Exception as e:
            print(f"发送速度命令失败: {e}")
            return False
    
    def emergency_stop_robot(self):
        """紧急停止机器人"""
        self.emergency_stop = True
        if self.robot_connected:
            try:
                self.robot.move_stop()
                print("机器人紧急停止")
            except Exception as e:
                print(f"紧急停止失败: {e}")
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """将像素坐标转换为3D坐标"""
        if depth_value == 0:
            return None
            
        # 转换为相机坐标系
        z = depth_value * self.depth_scale
        x = (pixel_x - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
        y = (pixel_y - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
        
        return np.array([x, y, z])
    
    def save_experiment_data(self, filename_prefix):
        """保存实验数据"""
        if len(self.error_history) > 0:
            data = {
                'error_history': [err.tolist() if isinstance(err, np.ndarray) else err 
                                 for err in self.error_history],
                'control_history': [ctrl.tolist() if isinstance(ctrl, np.ndarray) else ctrl 
                                   for ctrl in self.control_history],
                'pose_history': self.pose_history,
                'parameters': {
                    'lambda_pos': self.lambda_pos,
                    'lambda_rot': self.lambda_rot,
                    'max_linear_velocity': self.max_linear_velocity,
                    'max_angular_velocity': self.max_angular_velocity
                },
                'timestamp': time.time()
            }
            
            filename = f"{filename_prefix}_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"实验数据已保存到 {filename}")
    
    def cleanup(self):
        """清理资源"""
        self.control_active = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        
        try:
            self.pipeline.stop()
        except:
            pass
        
        self.disconnect_robot()
        cv2.destroyAllWindows()


class RobotPBVS(RobotVisualServoingBase):
    """基于位置的视觉伺服 (Position-Based Visual Servoing)"""
    
    def __init__(self, robot_ip='localhost', robot_port=8899, 
                 model_path='yolo_train/weights/best.pt', 
                 calibration_file='camera_calibration.json'):
        super().__init__(robot_ip, robot_port, model_path, calibration_file)
        
        # 期望的目标位置和姿态（相机坐标系）
        self.desired_position = np.array([0.0, 0.0, 0.4])  # 相机前方40cm
        self.desired_orientation = np.eye(3)  # 面向相机
        
        # 目标物体的3D模型点（假设为正方形，边长10cm）
        self.object_3d_points = np.array([
            [-0.05, -0.05, 0],
            [0.05, -0.05, 0],
            [0.05, 0.05, 0],
            [-0.05, 0.05, 0]
        ], dtype=np.float32)
        
        # 控制死区
        self.position_deadzone = 0.01  # 1cm
        self.orientation_deadzone = 0.05  # ~3度
    
    def get_image_points_from_bbox(self, target):
        """从边界框获取图像特征点"""
        x_center, y_center = target['center']
        width, height = target['size']
        
        # 使用边界框的四个角点作为特征点
        points = np.array([
            [x_center - width/2, y_center - height/2],
            [x_center + width/2, y_center - height/2],
            [x_center + width/2, y_center + height/2],
            [x_center - width/2, y_center + height/2]
        ], dtype=np.float32)
        
        return points
    
    def estimate_target_pose(self, target):
        """使用PnP算法估计目标位姿"""
        try:
            image_points = self.get_image_points_from_bbox(target)
            
            success, rvec, tvec = cv2.solvePnP(
                self.object_3d_points, 
                image_points,
                self.camera_matrix, 
                self.dist_coeffs
            )
            
            if success:
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                return {
                    'position': tvec.flatten(),
                    'orientation': rotation_matrix
                }
            
            return None
            
        except Exception as e:
            print(f"位姿估计失败: {e}")
            return None
    
    def compute_pose_error(self, current_pose):
        """计算位姿误差"""
        if current_pose is None:
            return None
        
        # 位置误差
        position_error = self.desired_position - current_pose['position']
        
        # 旋转误差（使用轴角表示）
        current_rot = R.from_matrix(current_pose['orientation'])
        desired_rot = R.from_matrix(self.desired_orientation)
        rotation_error = (desired_rot * current_rot.inv()).as_rotvec()
        
        return np.concatenate([position_error, rotation_error])
    
    def compute_control_law(self, pose_error):
        """计算PBVS控制律"""
        if pose_error is None:
            return np.zeros(6)
        
        # 分离位置和旋转误差
        position_error = pose_error[:3]
        rotation_error = pose_error[3:]
        
        # 应用死区
        if np.linalg.norm(position_error) < self.position_deadzone:
            position_error = np.zeros(3)
        if np.linalg.norm(rotation_error) < self.orientation_deadzone:
            rotation_error = np.zeros(3)
        
        # 计算控制速度
        linear_velocity = self.lambda_pos * position_error
        angular_velocity = self.lambda_rot * rotation_error
        
        return np.concatenate([linear_velocity, angular_velocity])
    
    def run(self):
        """运行PBVS控制循环"""
        print("=== 启动基于位置的视觉伺服 (PBVS) ===")
        print("按 'q' 退出，按 's' 保存数据，按 'e' 紧急停止")
        
        # 连接机器人
        if not self.connect_robot():
            print("机器人连接失败，退出")
            return
        
        try:
            while True:
                # 获取相机帧
                color_image, depth_image = self.get_aligned_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        print("目标长时间丢失，停止机器人")
                        self.emergency_stop_robot()
                    
                    # 显示目标丢失状态
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(color_image, f"Lost Count: {self.target_lost_count}", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 估计目标位姿
                    target_pose = self.estimate_target_pose(target)
                    
                    if target_pose is not None and not self.emergency_stop:
                        # 计算位姿误差
                        pose_error = self.compute_pose_error(target_pose)
                        
                        if pose_error is not None:
                            # 计算控制命令
                            control_velocity = self.compute_control_law(pose_error)
                            
                            # 发送控制命令
                            success = self.send_velocity_command(control_velocity)
                            
                            # 记录数据
                            self.error_history.append(pose_error.copy())
                            self.control_history.append(control_velocity.copy())
                            
                            # 显示信息
                            error_norm = np.linalg.norm(pose_error)
                            control_norm = np.linalg.norm(control_velocity)
                            
                            print(f"位姿误差: {error_norm:.4f}, 控制速度: {control_norm:.4f}, "
                                  f"命令发送: {'成功' if success else '失败'}")
                            
                            # 检查收敛
                            if error_norm < self.convergence_threshold:
                                print("已收敛到期望位置")
                    
                    # 可视化结果
                    self.visualize_pbvs(color_image, target, target_pose)
                
                # 显示图像
                cv2.imshow('Robot PBVS', color_image)
                
                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.save_experiment_data('pbvs_data')
                elif key == ord('e'):
                    self.emergency_stop_robot()
                    break
                
        except KeyboardInterrupt:
            print("用户中断")
        except Exception as e:
            print(f"运行时错误: {e}")
        finally:
            self.cleanup()
    
    def visualize_pbvs(self, image, target, target_pose):
        """可视化PBVS结果"""
        if target:
            # 绘制检测框
            x, y = target['center']
            w, h = target['size']
            cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            
            # 显示置信度
            cv2.putText(image, f"Conf: {target['confidence']:.2f}", 
                       (x-w//2, y-h//2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 绘制期望位置（投影到图像平面）
        if self.desired_position[2] > 0:
            desired_u = int(self.desired_position[0] * self.camera_matrix[0, 0] / 
                           self.desired_position[2] + self.camera_matrix[0, 2])
            desired_v = int(self.desired_position[1] * self.camera_matrix[1, 1] / 
                           self.desired_position[2] + self.camera_matrix[1, 2])
            cv2.circle(image, (desired_u, desired_v), 8, (0, 0, 255), 2)
            cv2.putText(image, "Desired", (desired_u+10, desired_v), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 显示状态信息
        status_text = [
            f"Mode: PBVS",
            f"Robot: {'Connected' if self.robot_connected else 'Disconnected'}",
            f"Emergency: {'STOP' if self.emergency_stop else 'Normal'}",
            f"Errors: {len(self.error_history)}"
        ]
        
        if target_pose:
            pos = target_pose['position']
            status_text.append(f"Target: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        
        for i, text in enumerate(status_text):
            color = (0, 0, 255) if self.emergency_stop else (255, 255, 255)
            cv2.putText(image, text, (10, image.shape[0] - 120 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


class RobotIBVS(RobotVisualServoingBase):
    """基于图像的视觉伺服 (Image-Based Visual Servoing)"""
    
    def __init__(self, robot_ip='localhost', robot_port=8899, 
                 model_path='yolo_train/weights/best.pt', 
                 calibration_file='camera_calibration.json'):
        super().__init__(robot_ip, robot_port, model_path, calibration_file)
        
        # 期望的图像特征
        self.desired_features = np.array([320, 240, 120, 120])  # [x, y, width, height]
        self.feature_deadzone = 8  # 像素死区
        
        # 特征深度（用于雅可比计算）
        self.feature_depth = 0.4  # 默认深度40cm
    
    def extract_image_features(self, target, depth_image):
        """提取图像特征"""
        if target is None:
            return None
        
        x, y = target['center']
        width, height = target['size']
        
        # 获取目标中心的深度
        try:
            depth_value = depth_image[y, x] * self.depth_scale
            if depth_value == 0 or depth_value > 2.0:  # 深度无效或过远
                depth_value = self.feature_depth
        except:
            depth_value = self.feature_depth
        
        return np.array([x, y, width, height, depth_value])
    
    def compute_image_jacobian(self, features):
        """计算图像雅可比矩阵"""
        if features is None:
            return None
        
        x, y, width, height, z = features
        
        # 相机内参
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # 归一化坐标
        x_norm = (x - cx) / fx
        y_norm = (y - cy) / fy
        
        # 图像雅可比矩阵 (4x6)
        L = np.zeros((4, 6))
        
        # 对于点特征 (x, y)
        L[0, :] = [-fx/z, 0, x_norm/z, 
                   x_norm*y_norm/fx, -(fx + x_norm**2*fx)/fx, y_norm]
        L[1, :] = [0, -fy/z, y_norm/z, 
                   (fy + y_norm**2*fy)/fy, -x_norm*y_norm/fy, -x_norm]
        
        # 对于尺寸特征（简化处理）
        L[2, :] = [-fx/z, 0, width/z, 0, 0, 0]  # width
        L[3, :] = [0, -fy/z, height/z, 0, 0, 0]  # height
        
        return L
    
    def compute_control_law(self, current_features):
        """计算IBVS控制律"""
        if current_features is None:
            return np.zeros(6)
        
        # 特征误差
        feature_error = self.desired_features - current_features[:4]
        
        # 应用死区
        if np.linalg.norm(feature_error) < self.feature_deadzone:
            return np.zeros(6)
        
        # 计算图像雅可比矩阵
        L = self.compute_image_jacobian(current_features)
        if L is None:
            return np.zeros(6)
        
        # 使用伪逆计算控制速度
        try:
            L_pinv = np.linalg.pinv(L)
            control_velocity = -self.lambda_pos * L_pinv @ feature_error
            return control_velocity
        except np.linalg.LinAlgError:
            return np.zeros(6)
    
    def run(self):
        """运行IBVS控制循环"""
        print("=== 启动基于图像的视觉伺服 (IBVS) ===")
        print("按 'q' 退出，按 's' 保存数据，按 'e' 紧急停止")
        print("按 'r' 重新设置期望特征")
        
        # 连接机器人
        if not self.connect_robot():
            print("机器人连接失败，退出")
            return
        
        try:
            while True:
                # 获取相机帧
                color_image, depth_image = self.get_aligned_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        print("目标长时间丢失，停止机器人")
                        self.emergency_stop_robot()
                    
                    # 显示目标丢失状态
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(color_image, f"Lost Count: {self.target_lost_count}", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 提取图像特征
                    current_features = self.extract_image_features(target, depth_image)
                    
                    if current_features is not None and not self.emergency_stop:
                        # 计算控制命令
                        control_velocity = self.compute_control_law(current_features)
                        
                        # 发送控制命令
                        success = self.send_velocity_command(control_velocity)
                        
                        # 计算特征误差
                        feature_error = self.desired_features - current_features[:4]
                        
                        # 记录数据
                        self.error_history.append(feature_error.copy())
                        self.control_history.append(control_velocity.copy())
                        
                        # 显示信息
                        error_norm = np.linalg.norm(feature_error)
                        control_norm = np.linalg.norm(control_velocity)
                        
                        print(f"特征误差: {error_norm:.2f}, 控制速度: {control_norm:.4f}, "
                              f"命令发送: {'成功' if success else '失败'}")
                        
                        # 检查收敛
                        if error_norm < self.feature_deadzone:
                            print("已收敛到期望特征")
                    
                    # 可视化结果
                    self.visualize_ibvs(color_image, target, current_features)
                
                # 显示图像
                cv2.imshow('Robot IBVS', color_image)
                
                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.save_experiment_data('ibvs_data')
                elif key == ord('e'):
                    self.emergency_stop_robot()
                    break
                elif key == ord('r') and target is not None:
                    # 重新设置期望特征为当前目标
                    self.desired_features = np.array([
                        target['center'][0], target['center'][1],
                        target['size'][0], target['size'][1]
                    ])
                    print(f"期望特征已更新为: {self.desired_features}")
                
        except KeyboardInterrupt:
            print("用户中断")
        except Exception as e:
            print(f"运行时错误: {e}")
        finally:
            self.cleanup()
    
    def visualize_ibvs(self, image, target, current_features):
        """可视化IBVS结果"""
        # 绘制当前特征
        if target:
            x, y = target['center']
            w, h = target['size']
            cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            
            # 显示置信度
            cv2.putText(image, f"Conf: {target['confidence']:.2f}", 
                       (x-w//2, y-h//2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 绘制期望特征
        desired_x, desired_y, desired_w, desired_h = self.desired_features
        cv2.rectangle(image, 
                     (int(desired_x-desired_w/2), int(desired_y-desired_h/2)),
                     (int(desired_x+desired_w/2), int(desired_y+desired_h/2)),
                     (0, 0, 255), 2)
        cv2.circle(image, (int(desired_x), int(desired_y)), 8, (0, 0, 255), 2)
        cv2.putText(image, "Desired", (int(desired_x)+10, int(desired_y)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 显示状态信息
        status_text = [
            f"Mode: IBVS",
            f"Robot: {'Connected' if self.robot_connected else 'Disconnected'}",
            f"Emergency: {'STOP' if self.emergency_stop else 'Normal'}",
            f"Errors: {len(self.error_history)}"
        ]
        
        if current_features is not None:
            curr = current_features[:4]
            status_text.extend([
                f"Current: ({curr[0]:.0f}, {curr[1]:.0f}, {curr[2]:.0f}, {curr[3]:.0f})",
                f"Desired: ({desired_x:.0f}, {desired_y:.0f}, {desired_w:.0f}, {desired_h:.0f})",
                f"Depth: {current_features[4]:.3f}m"
            ])
        
        for i, text in enumerate(status_text):
            color = (0, 0, 255) if self.emergency_stop else (255, 255, 255)
            cv2.putText(image, text, (10, image.shape[0] - 140 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    """主程序入口"""
    print("=== 机器人视觉伺服系统 ===")
    print("基于 robotcontrol.py 的 PBVS 和 IBVS 实现")
    print()
    print("1. PBVS - 基于位置的视觉伺服")
    print("2. IBVS - 基于图像的视觉伺服")
    print("3. 系统配置")
    print()
    
    choice = input("请选择运行模式 (1-3): ").strip()
    
    if choice == "1":
        # PBVS模式
        robot_ip = input("机器人IP地址 (默认: localhost): ").strip() or "localhost"
        robot_port = int(input("机器人端口 (默认: 8899): ").strip() or "8899")
        
        pbvs = RobotPBVS(robot_ip=robot_ip, robot_port=robot_port)
        pbvs.run()
        
    elif choice == "2":
        # IBVS模式
        robot_ip = input("机器人IP地址 (默认: localhost): ").strip() or "localhost"
        robot_port = int(input("机器人端口 (默认: 8899): ").strip() or "8899")
        
        ibvs = RobotIBVS(robot_ip=robot_ip, robot_port=robot_port)
        ibvs.run()
        
    elif choice == "3":
        # 系统配置
        print("\n=== 系统配置 ===")
        print("1. 相机标定")
        print("2. YOLO模型训练")
        print("3. 参数调整")
        
        config_choice = input("请选择配置选项: ").strip()
        
        if config_choice == "1":
            try:
                from camera_calibration import CameraCalibration
                calibrator = CameraCalibration()
                calibrator.run_calibration_process()
            except ImportError:
                print("相机标定模块未找到，请确保 camera_calibration.py 存在")
        elif config_choice == "2":
            print("请运行 yolo_train.py 进行模型训练")
        elif config_choice == "3":
            print("参数调整功能待实现")
        else:
            print("无效选择")
    else:
        print("无效选择")


if __name__ == "__main__":
    main()
