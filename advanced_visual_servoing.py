import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import json
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Optional, Dict
import time
import threading
import queue

class RobotController:
    """机器人控制器接口（模拟）"""
    
    def __init__(self):
        self.current_pose = np.array([0, 0, 0, 0, 0, 0])  # [x, y, z, rx, ry, rz]
        self.is_moving = False
        
    def send_velocity_command(self, velocity):
        """发送速度命令到机器人"""
        # 这里应该连接到实际的机器人控制器
        print(f"发送速度命令: 线速度={velocity[:3]:.3f}, 角速度={velocity[3:]:.3f}")
        
        # 模拟机器人运动
        dt = 0.1  # 时间步长
        self.current_pose += velocity * dt
        
    def get_current_pose(self):
        """获取当前机器人姿态"""
        return self.current_pose.copy()
    
    def stop(self):
        """停止机器人运动"""
        self.send_velocity_command(np.zeros(6))


class AdvancedVisualServoing:
    """高级视觉伺服系统"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt', calibration_file='camera_calibration.json'):
        # 加载YOLO模型
        self.model = YOLO(model_path)
        
        # 加载相机标定参数
        self.load_camera_calibration(calibration_file)
        
        # RealSense相机配置
        self.setup_camera()
        
        # 机器人控制器
        self.robot = RobotController()
        
        # 控制参数
        self.lambda_pos = 0.5
        self.lambda_rot = 0.3
        self.convergence_threshold = 0.01
        self.max_velocity = 0.1  # 最大线速度 m/s
        self.max_angular_velocity = 0.5  # 最大角速度 rad/s
        
        # 目标跟踪
        self.target_lost_count = 0
        self.max_target_lost = 10
        
        # 数据记录
        self.error_history = []
        self.control_history = []
        
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
            # 使用默认相机参数
            self.camera_matrix = np.array([
                [615.0, 0, 320.0],
                [0, 615.0, 240.0],
                [0, 0, 1]
            ])
            self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0])
    
    def setup_camera(self):
        """设置相机"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # 启动相机
        profile = self.pipeline.start(self.config)
        
        # 获取深度比例
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # 对齐深度到彩色图像
        self.align = rs.align(rs.stream.color)
    
    def get_aligned_frames(self):
        """获取对齐的RGB-D帧"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None
            
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image
    
    def detect_and_track_target(self, color_image):
        """检测和跟踪目标"""
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
        
        if best_box is not None:
            x_center, y_center, width, height = best_box.xywh[0].cpu().numpy()
            return {
                'center': (int(x_center), int(y_center)),
                'size': (int(width), int(height)),
                'confidence': best_confidence,
                'bbox': best_box.xyxy[0].cpu().numpy()
            }
        
        return None
    
    def estimate_target_pose_3d(self, target, depth_image):
        """估计目标的3D位置"""
        if target is None:
            return None
            
        x, y = target['center']
        width, height = target['size']
        
        # 获取目标区域的深度值
        x1, y1, x2, y2 = target['bbox'].astype(int)
        depth_roi = depth_image[y1:y2, x1:x2]
        
        # 过滤无效深度值
        valid_depths = depth_roi[depth_roi > 0]
        if len(valid_depths) == 0:
            return None
        
        # 使用中位数深度
        depth_value = np.median(valid_depths) * self.depth_scale
        
        # 转换为3D坐标
        z = depth_value
        x_3d = (x - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
        y_3d = (y - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
        
        # 估计姿态（简化为面向相机）
        rotation_matrix = np.eye(3)
        
        return {
            'position': np.array([x_3d, y_3d, z]),
            'orientation': rotation_matrix,
            'size_3d': np.array([width * z / self.camera_matrix[0, 0], 
                               height * z / self.camera_matrix[1, 1]])
        }


class PBVS_Advanced(AdvancedVisualServoing):
    """高级PBVS系统"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt', calibration_file='camera_calibration.json'):
        super().__init__(model_path, calibration_file)
        
        # 期望的目标位置和姿态
        self.desired_position = np.array([0, 0, 0.5])  # 相机坐标系下
        self.desired_orientation = np.eye(3)
        
        # 控制死区
        self.position_deadzone = 0.005  # 5mm
        self.orientation_deadzone = 0.05  # ~3度
    
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
        
        # 限制最大速度
        linear_speed = np.linalg.norm(linear_velocity)
        if linear_speed > self.max_velocity:
            linear_velocity = linear_velocity / linear_speed * self.max_velocity
            
        angular_speed = np.linalg.norm(angular_velocity)
        if angular_speed > self.max_angular_velocity:
            angular_velocity = angular_velocity / angular_speed * self.max_angular_velocity
        
        return np.concatenate([linear_velocity, angular_velocity])
    
    def run(self):
        """运行PBVS控制循环"""
        print("启动高级PBVS视觉伺服系统...")
        print("按'q'退出，按's'保存数据")
        
        try:
            while True:
                color_image, depth_image = self.get_aligned_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_and_track_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        print("目标丢失，停止机器人")
                        self.robot.stop()
                    
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow('Advanced PBVS', color_image)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                
                # 估计目标3D位姿
                target_pose = self.estimate_target_pose_3d(target, depth_image)
                
                if target_pose is not None:
                    # 计算位姿误差
                    pose_error = self.compute_pose_error(target_pose)
                    
                    # 计算控制命令
                    control_velocity = self.compute_control_law(pose_error)
                    
                    # 发送控制命令
                    self.robot.send_velocity_command(control_velocity)
                    
                    # 记录数据
                    self.error_history.append(pose_error)
                    self.control_history.append(control_velocity)
                    
                    # 显示信息
                    error_norm = np.linalg.norm(pose_error)
                    print(f"位姿误差: {error_norm:.4f}, 控制速度: {np.linalg.norm(control_velocity):.4f}")
                    
                    # 可视化
                    self.visualize_results(color_image, target, target_pose, pose_error, control_velocity)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                elif cv2.waitKey(1) & 0xFF == ord('s'):
                    self.save_data()
                    
        except KeyboardInterrupt:
            print("用户中断")
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.robot.stop()
            self.pipeline.stop()
            cv2.destroyAllWindows()
    
    def visualize_results(self, image, target, target_pose, pose_error, control_velocity):
        """可视化结果"""
        # 绘制检测框
        x, y = target['center']
        w, h = target['size']
        cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        
        # 绘制期望位置（投影到图像平面）
        desired_u = int(self.desired_position[0] * self.camera_matrix[0, 0] / self.desired_position[2] + self.camera_matrix[0, 2])
        desired_v = int(self.desired_position[1] * self.camera_matrix[1, 1] / self.desired_position[2] + self.camera_matrix[1, 2])
        cv2.circle(image, (desired_u, desired_v), 8, (0, 0, 255), 2)
        
        # 显示文本信息
        info_text = [
            f"Position: ({target_pose['position'][0]:.3f}, {target_pose['position'][1]:.3f}, {target_pose['position'][2]:.3f})",
            f"Error: {np.linalg.norm(pose_error):.4f}",
            f"Control: {np.linalg.norm(control_velocity):.4f}",
            f"Confidence: {target['confidence']:.2f}"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(image, text, (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow('Advanced PBVS', image)
    
    def save_data(self):
        """保存实验数据"""
        if len(self.error_history) > 0:
            data = {
                'error_history': [err.tolist() for err in self.error_history],
                'control_history': [ctrl.tolist() for ctrl in self.control_history],
                'timestamp': time.time()
            }
            
            filename = f"pbvs_data_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"数据已保存到 {filename}")


class IBVS_Advanced(AdvancedVisualServoing):
    """高级IBVS系统"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt', calibration_file='camera_calibration.json'):
        super().__init__(model_path, calibration_file)
        
        # 期望的图像特征
        self.desired_features = np.array([320, 240, 150, 150])  # [x, y, width, height]
        self.feature_deadzone = 5  # 像素
    
    def extract_image_features(self, target, depth_image):
        """提取图像特征"""
        if target is None:
            return None
            
        x, y = target['center']
        width, height = target['size']
        
        # 获取深度
        depth_value = depth_image[y, x] * self.depth_scale
        if depth_value == 0:
            depth_value = 0.5  # 默认深度
        
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
        
        # 图像雅可比矩阵
        L = np.zeros((4, 6))
        
        # 点特征的雅可比
        L[0, :] = [-fx/z, 0, x_norm/z, x_norm*y_norm/fx, -(fx + x_norm**2*fx)/fx, y_norm]
        L[1, :] = [0, -fy/z, y_norm/z, (fy + y_norm**2*fy)/fy, -x_norm*y_norm/fy, -x_norm]
        
        # 尺寸特征的雅可比（简化）
        L[2, :] = [-fx/z, 0, width/z, 0, 0, 0]
        L[3, :] = [0, -fy/z, height/z, 0, 0, 0]
        
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
            
            # 限制速度
            linear_velocity = control_velocity[:3]
            angular_velocity = control_velocity[3:]
            
            linear_speed = np.linalg.norm(linear_velocity)
            if linear_speed > self.max_velocity:
                linear_velocity = linear_velocity / linear_speed * self.max_velocity
                
            angular_speed = np.linalg.norm(angular_velocity)
            if angular_speed > self.max_angular_velocity:
                angular_velocity = angular_velocity / angular_speed * self.max_angular_velocity
            
            return np.concatenate([linear_velocity, angular_velocity])
            
        except np.linalg.LinAlgError:
            return np.zeros(6)
    
    def run(self):
        """运行IBVS控制循环"""
        print("启动高级IBVS视觉伺服系统...")
        print("按'q'退出，按's'保存数据")
        
        try:
            while True:
                color_image, depth_image = self.get_aligned_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_and_track_target(color_image)
                
                if target is None:
                    if self.target_lost_count > self.max_target_lost:
                        print("目标丢失，停止机器人")
                        self.robot.stop()
                    
                    cv2.putText(color_image, "Target Lost", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow('Advanced IBVS', color_image)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                
                # 提取图像特征
                current_features = self.extract_image_features(target, depth_image)
                
                if current_features is not None:
                    # 计算控制命令
                    control_velocity = self.compute_control_law(current_features)
                    
                    # 发送控制命令
                    self.robot.send_velocity_command(control_velocity)
                    
                    # 计算特征误差
                    feature_error = self.desired_features - current_features[:4]
                    
                    # 记录数据
                    self.error_history.append(feature_error)
                    self.control_history.append(control_velocity)
                    
                    # 显示信息
                    error_norm = np.linalg.norm(feature_error)
                    print(f"特征误差: {error_norm:.2f}, 控制速度: {np.linalg.norm(control_velocity):.4f}")
                    
                    # 可视化
                    self.visualize_results(color_image, target, current_features, feature_error, control_velocity)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                elif cv2.waitKey(1) & 0xFF == ord('s'):
                    self.save_data()
                    
        except KeyboardInterrupt:
            print("用户中断")
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.robot.stop()
            self.pipeline.stop()
            cv2.destroyAllWindows()
    
    def visualize_results(self, image, target, current_features, feature_error, control_velocity):
        """可视化结果"""
        # 绘制当前特征
        x, y = target['center']
        w, h = target['size']
        cv2.rectangle(image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        
        # 绘制期望特征
        desired_x, desired_y, desired_w, desired_h = self.desired_features
        cv2.rectangle(image, 
                     (int(desired_x-desired_w/2), int(desired_y-desired_h/2)),
                     (int(desired_x+desired_w/2), int(desired_y+desired_h/2)),
                     (0, 0, 255), 2)
        cv2.circle(image, (int(desired_x), int(desired_y)), 8, (0, 0, 255), 2)
        
        # 显示文本信息
        info_text = [
            f"Current: ({x}, {y}, {w}, {h})",
            f"Desired: ({desired_x}, {desired_y}, {desired_w}, {desired_h})",
            f"Error: {np.linalg.norm(feature_error):.2f}",
            f"Control: {np.linalg.norm(control_velocity):.4f}",
            f"Depth: {current_features[4]:.3f}m"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(image, text, (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow('Advanced IBVS', image)
    
    def save_data(self):
        """保存实验数据"""
        if len(self.error_history) > 0:
            data = {
                'error_history': [err.tolist() for err in self.error_history],
                'control_history': [ctrl.tolist() for ctrl in self.control_history],
                'desired_features': self.desired_features.tolist(),
                'timestamp': time.time()
            }
            
            filename = f"ibvs_data_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"数据已保存到 {filename}")


def main():
    print("=== 高级视觉伺服系统 ===")
    print("1. 高级PBVS (基于位置的视觉伺服)")
    print("2. 高级IBVS (基于图像的视觉伺服)")
    print("3. 相机标定")
    
    choice = input("请选择运行模式: ")
    
    if choice == "1":
        pbvs = PBVS_Advanced()
        pbvs.run()
    elif choice == "2":
        ibvs = IBVS_Advanced()
        ibvs.run()
    elif choice == "3":
        from camera_calibration import CameraCalibration
        calibrator = CameraCalibration()
        calibrator.run_calibration_process()
    else:
        print("无效选择")


if __name__ == "__main__":
    main()
