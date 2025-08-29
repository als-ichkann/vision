import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import math
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Optional
import time

class VisualServoing:
    """视觉伺服基类"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt'):
        # 加载YOLO模型
        self.model = YOLO(model_path)
        
        # 相机内参矩阵（需要根据实际相机标定结果修改）
        self.camera_matrix = np.array([
            [615.0, 0, 320.0],
            [0, 615.0, 240.0],
            [0, 0, 1]
        ])
        
        # 畸变系数
        self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0])
        
        # RealSense相机配置
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # 获取深度传感器的深度比例
        self.pipeline.start(self.config)
        profile = self.pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # 目标期望状态
        self.desired_position = np.array([0, 0, 0.5])  # 期望位置 (x, y, z)
        self.desired_orientation = np.eye(3)  # 期望姿态
        
        # 控制增益
        self.lambda_pos = 0.5  # 位置控制增益
        self.lambda_rot = 0.3  # 旋转控制增益
        
    def get_frames(self):
        """获取RGB-D帧"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None
            
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image
    
    def detect_target(self, color_image):
        """使用YOLO检测目标"""
        results = self.model(color_image)
        
        if len(results[0].boxes) == 0:
            return None
            
        # 获取第一个检测到的目标
        box = results[0].boxes[0]
        x_center, y_center, width, height = box.xywh[0].cpu().numpy()
        confidence = box.conf[0].cpu().numpy()
        
        return {
            'center': (int(x_center), int(y_center)),
            'size': (int(width), int(height)),
            'confidence': confidence
        }
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """将像素坐标转换为3D坐标"""
        if depth_value == 0:
            return None
            
        # 转换为相机坐标系
        z = depth_value * self.depth_scale
        x = (pixel_x - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
        y = (pixel_y - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
        
        return np.array([x, y, z])
    
    def estimate_pose_pnp(self, image_points, object_points):
        """使用PnP算法估计目标姿态"""
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, 
            self.camera_matrix, self.dist_coeffs
        )
        
        if success:
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            return rotation_matrix, tvec.flatten()
        return None, None


class PBVS(VisualServoing):
    """基于位置的视觉伺服 (Position-Based Visual Servoing)"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt'):
        super().__init__(model_path)
        
        # 目标物体的3D模型点（以目标中心为原点）
        # 这里假设目标是一个边长为0.1m的立方体
        self.object_3d_points = np.array([
            [-0.05, -0.05, 0],
            [0.05, -0.05, 0],
            [0.05, 0.05, 0],
            [-0.05, 0.05, 0]
        ], dtype=np.float32)
    
    def get_image_points_from_bbox(self, bbox):
        """从边界框获取图像特征点"""
        x_center, y_center = bbox['center']
        width, height = bbox['size']
        
        # 使用边界框的四个角点作为特征点
        points = np.array([
            [x_center - width/2, y_center - height/2],
            [x_center + width/2, y_center - height/2],
            [x_center + width/2, y_center + height/2],
            [x_center - width/2, y_center + height/2]
        ], dtype=np.float32)
        
        return points
    
    def compute_control_law(self, current_pose, desired_pose):
        """计算PBVS控制律"""
        # 位置误差
        position_error = desired_pose[:3] - current_pose[:3]
        
        # 旋转误差（使用轴角表示）
        current_rotation = R.from_matrix(current_pose[3:].reshape(3, 3))
        desired_rotation = R.from_matrix(desired_pose[3:].reshape(3, 3))
        rotation_error = (desired_rotation * current_rotation.inv()).as_rotvec()
        
        # 计算控制速度
        linear_velocity = self.lambda_pos * position_error
        angular_velocity = self.lambda_rot * rotation_error
        
        return np.concatenate([linear_velocity, angular_velocity])
    
    def run(self):
        """运行PBVS控制循环"""
        print("开始PBVS视觉伺服...")
        
        try:
            while True:
                color_image, depth_image = self.get_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_target(color_image)
                if target is None:
                    print("未检测到目标")
                    cv2.imshow('PBVS', color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                
                # 获取图像特征点
                image_points = self.get_image_points_from_bbox(target)
                
                # 估计目标姿态
                rotation_matrix, translation = self.estimate_pose_pnp(
                    image_points, self.object_3d_points
                )
                
                if rotation_matrix is not None:
                    # 构造当前姿态向量
                    current_pose = np.concatenate([translation, rotation_matrix.flatten()])
                    desired_pose = np.concatenate([self.desired_position, self.desired_orientation.flatten()])
                    
                    # 计算控制命令
                    control_velocity = self.compute_control_law(current_pose, desired_pose)
                    
                    # 显示信息
                    print(f"当前位置: {translation}")
                    print(f"控制速度: 线速度={control_velocity[:3]}, 角速度={control_velocity[3:]}")
                    
                    # 在图像上绘制检测结果
                    cv2.circle(color_image, target['center'], 5, (0, 255, 0), -1)
                    cv2.rectangle(color_image, 
                                (target['center'][0] - target['size'][0]//2, 
                                 target['center'][1] - target['size'][1]//2),
                                (target['center'][0] + target['size'][0]//2, 
                                 target['center'][1] + target['size'][1]//2),
                                (255, 0, 0), 2)
                
                cv2.imshow('PBVS', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


class IBVS(VisualServoing):
    """基于图像的视觉伺服 (Image-Based Visual Servoing)"""
    
    def __init__(self, model_path='yolo_train/weights/best.pt'):
        super().__init__(model_path)
        
        # 期望的图像特征（目标在期望位置时的图像坐标）
        self.desired_features = np.array([320, 240, 100, 100])  # [x, y, width, height]
        
        # 特征深度（假设值或从深度图获取）
        self.feature_depth = 0.5  # 米
    
    def extract_image_features(self, bbox, depth_image):
        """提取图像特征"""
        if bbox is None:
            return None
            
        x, y = bbox['center']
        width, height = bbox['size']
        
        # 获取目标中心的深度
        depth_value = depth_image[y, x]
        if depth_value == 0:
            depth_value = int(self.feature_depth / self.depth_scale)
        
        z = depth_value * self.depth_scale
        
        return np.array([x, y, width, height, z])
    
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
        L[0, :] = [-fx/z, 0, x_norm/z, x_norm*y_norm/fx, -(fx + x_norm**2*fx)/fx, y_norm]
        L[1, :] = [0, -fy/z, y_norm/z, (fy + y_norm**2*fy)/fy, -x_norm*y_norm/fy, -x_norm]
        
        # 对于尺寸特征（简化处理）
        L[2, :] = [-fx/z, 0, width/z, 0, 0, 0]  # width
        L[3, :] = [0, -fy/z, height/z, 0, 0, 0]  # height
        
        return L
    
    def compute_control_law(self, current_features, desired_features):
        """计算IBVS控制律"""
        if current_features is None:
            return np.zeros(6)
            
        # 特征误差
        feature_error = desired_features - current_features[:4]
        
        # 计算图像雅可比矩阵
        L = self.compute_image_jacobian(current_features)
        if L is None:
            return np.zeros(6)
        
        # 使用伪逆计算控制速度
        try:
            L_pinv = np.linalg.pinv(L)
            control_velocity = -self.lambda_pos * L_pinv @ feature_error
        except:
            control_velocity = np.zeros(6)
        
        return control_velocity
    
    def run(self):
        """运行IBVS控制循环"""
        print("开始IBVS视觉伺服...")
        
        try:
            while True:
                color_image, depth_image = self.get_frames()
                if color_image is None:
                    continue
                
                # 检测目标
                target = self.detect_target(color_image)
                
                # 提取图像特征
                current_features = self.extract_image_features(target, depth_image)
                
                if current_features is not None:
                    # 计算控制命令
                    control_velocity = self.compute_control_law(current_features, self.desired_features)
                    
                    # 显示信息
                    print(f"当前特征: {current_features[:4]}")
                    print(f"期望特征: {self.desired_features}")
                    print(f"特征误差: {self.desired_features - current_features[:4]}")
                    print(f"控制速度: {control_velocity}")
                    
                    # 在图像上绘制当前特征和期望特征
                    if target:
                        # 当前目标
                        cv2.circle(color_image, target['center'], 5, (0, 255, 0), -1)
                        cv2.rectangle(color_image, 
                                    (target['center'][0] - target['size'][0]//2, 
                                     target['center'][1] - target['size'][1]//2),
                                    (target['center'][0] + target['size'][0]//2, 
                                     target['center'][1] + target['size'][1]//2),
                                    (255, 0, 0), 2)
                    
                    # 期望位置
                    desired_x, desired_y, desired_w, desired_h = self.desired_features
                    cv2.circle(color_image, (int(desired_x), int(desired_y)), 5, (0, 0, 255), -1)
                    cv2.rectangle(color_image,
                                (int(desired_x - desired_w/2), int(desired_y - desired_h/2)),
                                (int(desired_x + desired_w/2), int(desired_y + desired_h/2)),
                                (0, 0, 255), 2)
                else:
                    print("未检测到目标或无法提取特征")
                
                cv2.imshow('IBVS', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    # 选择运行模式
    mode = input("选择运行模式 (1: PBVS, 2: IBVS): ")
    
    if mode == "1":
        pbvs = PBVS()
        pbvs.run()
    elif mode == "2":
        ibvs = IBVS()
        ibvs.run()
    else:
        print("无效选择")
