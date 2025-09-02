#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyBullet机械臂仿真适配器
模拟robotcontrol.py中Auboi5Robot的接口，使现有的robot_visual_servoing_integrated.py
可以无缝切换到仿真模式
"""

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import math
from typing import Dict, Optional, List, Tuple

# 模拟robotcontrol.py中的错误类型
class RobotErrorType:
    RobotError_SUCC = 0
    RobotError_Base = 2000
    RobotError_NoLink = RobotError_Base + 3
    RobotError_Move = RobotError_Base + 4
    RobotError_LOGIN_FAILED = RobotError_Base + 5
    RobotError_NotLogin = RobotError_Base + 6

def logger_init():
    """模拟日志初始化"""
    print("PyBullet仿真日志系统初始化")

class PyBulletAuboi5Robot:
    """
    PyBullet版本的Auboi5Robot
    模拟真实机械臂的接口，用于仿真环境
    """
    
    def __init__(self, gui=True):
        """初始化PyBullet机械臂仿真"""
        self.gui = gui
        self.physics_client = None
        self.robot_id = None
        self.camera_link_index = None
        self.joint_indices = []
        self.connected = False
        self.startup_done = False
        
        # 机械臂参数
        self.joint_limits = [
            (-np.pi, np.pi),    # joint_1
            (-1.57, 1.57),      # joint_2
            (-1.57, 1.57),      # joint_3
            (-np.pi, np.pi),    # joint_4
            (-1.57, 1.57),      # joint_5
            (-np.pi, np.pi),    # joint_6
        ]
        
        # 相机内参（与RealSense D435i匹配）
        self.camera_matrix = np.array([
            [615.0, 0, 320.0],
            [0, 615.0, 240.0],
            [0, 0, 1]
        ])
        
        # 仿真参数
        self.image_width = 640
        self.image_height = 480
        self.near_plane = 0.01
        self.far_plane = 10.0
        
        # 深度比例（模拟RealSense）
        self.depth_scale = 0.001  # 1mm = 0.001m
        
        print("PyBullet Auboi5机械臂适配器初始化完成")
    
    @staticmethod
    def initialize():
        """静态方法：初始化机械臂控制库"""
        return RobotErrorType.RobotError_SUCC
    
    @staticmethod
    def uninitialize():
        """静态方法：反初始化机械臂控制库"""
        return RobotErrorType.RobotError_SUCC
    
    def create_context(self):
        """创建机械臂控制上下文句柄"""
        # 启动PyBullet物理仿真
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        return self.physics_client
    
    def connect(self, ip='localhost', port=8899):
        """连接机械臂服务器（仿真中直接创建机械臂）"""
        try:
            # 加载地面
            p.loadURDF("plane.urdf")
            
            # 创建机械臂
            self._create_robot()
            
            # 创建目标物体
            self._create_target_objects()
            
            self.connected = True
            print(f"PyBullet仿真机械臂连接成功 (模拟连接到 {ip}:{port})")
            return RobotErrorType.RobotError_SUCC
        except Exception as e:
            print(f"PyBullet仿真连接失败: {e}")
            return RobotErrorType.RobotError_LOGIN_FAILED
    
    def disconnect(self):
        """断开机械臂连接"""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            self.physics_client = None
        self.connected = False
        return RobotErrorType.RobotError_SUCC
    
    def robot_startup(self, collision=6, tool_dynamics=None):
        """启动机械臂"""
        if not self.connected:
            return RobotErrorType.RobotError_NoLink
        
        # 设置Auboi5机械臂的初始位置（更安全的姿态）
        initial_joints = [0, -np.pi/6, np.pi/3, 0, np.pi/6, 0]  # 轻微弯曲的安全姿态
        for i, joint_pos in enumerate(initial_joints):
            if i < len(self.joint_indices):
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_pos)
        
        # 等待仿真稳定
        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01)
        
        self.startup_done = True
        print("PyBullet机械臂启动完成")
        return RobotErrorType.RobotError_SUCC
    
    def robot_shutdown(self):
        """关闭机械臂"""
        self.startup_done = False
        print("PyBullet机械臂关闭")
        return RobotErrorType.RobotError_SUCC
    
    def init_profile(self):
        """初始化机械臂控制全局属性"""
        print("PyBullet机械臂配置初始化")
        return RobotErrorType.RobotError_SUCC
    
    def set_joint_maxacc(self, joint_maxacc):
        """设置六个关节的最大加速度"""
        print(f"设置关节最大加速度: {joint_maxacc}")
        return RobotErrorType.RobotError_SUCC
    
    def set_joint_maxvelc(self, joint_maxvelc):
        """设置六个关节的最大速度"""
        print(f"设置关节最大速度: {joint_maxvelc}")
        return RobotErrorType.RobotError_SUCC
    
    def set_end_max_line_acc(self, end_maxacc):
        """设置机械臂末端最大线加速度"""
        print(f"设置末端最大线加速度: {end_maxacc}")
        return RobotErrorType.RobotError_SUCC
    
    def set_end_max_line_velc(self, end_maxvelc):
        """设置机械臂末端最大线速度"""
        print(f"设置末端最大线速度: {end_maxvelc}")
        return RobotErrorType.RobotError_SUCC
    
    def set_collision_class(self, grade):
        """设置机械臂碰撞等级"""
        print(f"设置碰撞等级: {grade}")
        return RobotErrorType.RobotError_SUCC
    
    def get_current_waypoint(self):
        """获取机械臂当前位置信息"""
        if not self.connected or self.robot_id is None:
            return None
        
        try:
            # 获取关节位置
            joint_positions = []
            for joint_idx in self.joint_indices:
                joint_state = p.getJointState(self.robot_id, joint_idx)
                joint_positions.append(joint_state[0])
            
            # 获取末端执行器位姿
            if self.camera_link_index is not None:
                link_state = p.getLinkState(self.robot_id, self.camera_link_index)
                position = list(link_state[4])  # 世界坐标位置
                orientation = list(link_state[5])  # 四元数姿态
            else:
                # 如果没有相机链节，使用最后一个关节
                link_state = p.getLinkState(self.robot_id, len(self.joint_indices) - 1)
                position = list(link_state[4])
                orientation = list(link_state[5])
            
            return {
                'joint': joint_positions,
                'pos': position,
                'ori': orientation
            }
        except Exception as e:
            print(f"获取当前位置失败: {e}")
            return None
    
    def move_joint(self, joint_radian, issync=True):
        """机械臂轴动"""
        if not self.connected or not self.startup_done:
            return RobotErrorType.RobotError_NoLink
        
        try:
            if len(joint_radian) != len(self.joint_indices):
                print(f"关节数量不匹配: 期望{len(self.joint_indices)}, 得到{len(joint_radian)}")
                return RobotErrorType.RobotError_Move
            
            # 检查关节限制
            for i, (joint_pos, (min_limit, max_limit)) in enumerate(zip(joint_radian, self.joint_limits)):
                if joint_pos < min_limit or joint_pos > max_limit:
                    print(f"关节{i+1}超出限制: {joint_pos} not in [{min_limit}, {max_limit}]")
                    return RobotErrorType.RobotError_Move
            
            # 设置关节目标位置
            for i, joint_pos in enumerate(joint_radian):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=self.joint_indices[i],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_pos,
                    maxVelocity=1.0,
                    force=100
                )
            
            # 如果是同步模式，等待运动完成
            if issync:
                self._wait_for_motion_complete(joint_radian)
            
            return RobotErrorType.RobotError_SUCC
        except Exception as e:
            print(f"关节运动失败: {e}")
            return RobotErrorType.RobotError_Move
    
    def move_stop(self):
        """停止机械臂运动"""
        if not self.connected:
            return RobotErrorType.RobotError_NoLink
        
        try:
            # 获取当前关节位置并设为目标位置（停止运动）
            current_joints = []
            for joint_idx in self.joint_indices:
                joint_state = p.getJointState(self.robot_id, joint_idx)
                current_joints.append(joint_state[0])
            
            # 设置当前位置为目标位置
            for i, joint_pos in enumerate(current_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=self.joint_indices[i],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_pos,
                    maxVelocity=0.1,
                    force=100
                )
            
            print("PyBullet机械臂停止运动")
            return RobotErrorType.RobotError_SUCC
        except Exception as e:
            print(f"停止运动失败: {e}")
            return RobotErrorType.RobotError_Move
    
    def inverse_kin(self, joint_radian, pos, ori):
        """逆运动学计算"""
        if not self.connected or self.robot_id is None:
            return None
        
        try:
            # 使用PyBullet的逆运动学求解器
            # 获取末端执行器链节索引
            end_effector_index = self.camera_link_index if self.camera_link_index is not None else len(self.joint_indices) - 1
            
            # PyBullet逆运动学求解
            joint_poses = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=end_effector_index,
                targetPosition=pos,
                targetOrientation=ori,
                maxNumIterations=100,
                residualThreshold=1e-5
            )
            
            # 只返回前6个关节的解
            joint_solution = list(joint_poses[:len(self.joint_indices)])
            
            return {
                'joint': joint_solution,
                'pos': pos,
                'ori': ori
            }
        except Exception as e:
            print(f"逆运动学计算失败: {e}")
            return None
    
    def forward_kin(self, joint_radian):
        """正运动学计算"""
        if not self.connected or self.robot_id is None:
            return None
        
        try:
            # 临时设置关节位置
            original_positions = []
            for i, joint_idx in enumerate(self.joint_indices):
                joint_state = p.getJointState(self.robot_id, joint_idx)
                original_positions.append(joint_state[0])
                if i < len(joint_radian):
                    p.resetJointState(self.robot_id, joint_idx, joint_radian[i])
            
            # 获取末端执行器位姿
            end_effector_index = self.camera_link_index if self.camera_link_index is not None else len(self.joint_indices) - 1
            link_state = p.getLinkState(self.robot_id, end_effector_index)
            position = list(link_state[4])
            orientation = list(link_state[5])
            
            # 恢复原始关节位置
            for i, joint_pos in enumerate(original_positions):
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_pos)
            
            return {
                'joint': joint_radian,
                'pos': position,
                'ori': orientation
            }
        except Exception as e:
            print(f"正运动学计算失败: {e}")
            return None
    
    def get_camera_image(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """获取仿真相机图像（用于替代RealSense）"""
        if not self.connected or self.robot_id is None:
            return None, None
        
        try:
            # 获取相机链节的世界坐标
            if self.camera_link_index is not None:
                link_state = p.getLinkState(self.robot_id, self.camera_link_index)
                camera_position = link_state[4]
                camera_orientation = link_state[5]
            else:
                # 如果没有相机链节，使用最后一个关节
                link_state = p.getLinkState(self.robot_id, len(self.joint_indices) - 1)
                camera_position = link_state[4]
                camera_orientation = link_state[5]
            
            # 计算相机的观察方向
            rotation_matrix = p.getMatrixFromQuaternion(camera_orientation)
            rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
            
            # 相机朝向（相机安装时朝+X方向，即机械臂末端前方）
            camera_forward = rotation_matrix @ np.array([1, 0, 0])
            camera_up = rotation_matrix @ np.array([0, 0, 1])
            
            target_position = np.array(camera_position) + camera_forward * 1.0
            
            # 计算视图矩阵
            view_matrix = p.computeViewMatrix(
                cameraEyePosition=camera_position,
                cameraTargetPosition=target_position,
                cameraUpVector=camera_up
            )
            
            # 计算投影矩阵
            aspect_ratio = self.image_width / self.image_height
            fov = 60  # 视场角度
            projection_matrix = p.computeProjectionMatrixFOV(
                fov=fov,
                aspect=aspect_ratio,
                nearVal=self.near_plane,
                farVal=self.far_plane
            )
            
            # 渲染图像
            width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
                width=self.image_width,
                height=self.image_height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )
            
            # 转换图像格式
            rgb_array = np.array(rgb_img).reshape(height, width, 4)
            color_image = rgb_array[:, :, :3]  # 去掉alpha通道，转换为BGR
            color_image = cv2.cvtColor(color_image.astype(np.uint8), cv2.COLOR_RGB2BGR)
            
            # 转换深度图（模拟RealSense格式）
            depth_buffer = np.array(depth_img).reshape(height, width)
            # 将深度缓冲区转换为实际距离（米），然后转换为毫米（模拟RealSense）
            depth_real = self.far_plane * self.near_plane / (
                self.far_plane - (self.far_plane - self.near_plane) * depth_buffer)
            depth_image = (depth_real / self.depth_scale).astype(np.uint16)  # 转换为毫米单位
            
            return color_image, depth_image
        except Exception as e:
            print(f"获取相机图像失败: {e}")
            return None, None
    
    def step_simulation(self):
        """单步仿真（需要在控制循环中调用）"""
        if self.physics_client is not None:
            p.stepSimulation()
    
    def _create_robot(self):
        """创建Auboi5六轴机械臂模型"""
        print("创建Auboi5六轴机械臂模型...")
        
        try:
            # 尝试加载URDF文件
            import os
            urdf_path = "auboi5_robot.urdf"
            if os.path.exists(urdf_path):
                print(f"加载Auboi5 URDF文件: {urdf_path}")
                self.robot_id = p.loadURDF(urdf_path, [0, 0, 0])
                
                # 固定底座到地面
                p.createConstraint(
                    parentBodyUniqueId=self.robot_id,
                    parentLinkIndex=-1,  # 底座
                    childBodyUniqueId=-1,  # 世界坐标系
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0]
                )
                
            else:
                raise FileNotFoundError("URDF文件不存在")
                
        except Exception as e:
            print(f"URDF加载失败: {e}")
            print("使用简化机械臂模型")
            
            # 创建简化的六轴机械臂
            base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.08])
            base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.08], rgbaColor=[0.3, 0.3, 0.3, 1])
            
            # 链节参数
            link_masses = [2, 3, 2, 1, 1, 0.5]
            link_collision_shapes = []
            link_visual_shapes = []
            link_positions = []
            link_orientations = []
            link_inertial_positions = []
            link_inertial_orientations = []
            parent_indices = []
            joint_types = []
            joint_axis = []
            
            # 创建6个链节
            for i in range(6):
                # 根据链节位置调整大小
                if i < 2:  # 前两个链节较大
                    half_extents = [0.04, 0.04, 0.1]
                    color = [0.8, 0.2, 0.2, 1] if i == 0 else [0.2, 0.8, 0.2, 1]
                elif i < 4:  # 中间两个链节中等
                    half_extents = [0.03, 0.03, 0.08]
                    color = [0.2, 0.2, 0.8, 1] if i == 2 else [0.8, 0.8, 0.2, 1]
                else:  # 后两个链节较小
                    half_extents = [0.02, 0.02, 0.05]
                    color = [0.8, 0.2, 0.8, 1] if i == 4 else [0.2, 0.8, 0.8, 1]
                
                collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
                visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
                
                link_collision_shapes.append(collision_shape)
                link_visual_shapes.append(visual_shape)
                link_positions.append([0, 0, half_extents[2] * 1.5])
                link_orientations.append([0, 0, 0, 1])
                link_inertial_positions.append([0, 0, 0])
                link_inertial_orientations.append([0, 0, 0, 1])
                parent_indices.append(i - 1 if i > 0 else -1)
                joint_types.append(p.JOINT_REVOLUTE)
                
                # 交替设置关节轴
                if i % 2 == 0:
                    joint_axis.append([0, 0, 1])  # Z轴旋转
                else:
                    joint_axis.append([0, 1, 0])  # Y轴旋转
            
            # 创建机械臂
            self.robot_id = p.createMultiBody(
                baseMass=5.0,
                baseCollisionShapeIndex=base_collision,
                baseVisualShapeIndex=base_visual,
                basePosition=[0, 0, 0.08],
                baseOrientation=[0, 0, 0, 1],
                linkMasses=link_masses,
                linkCollisionShapeIndices=link_collision_shapes,
                linkVisualShapeIndices=link_visual_shapes,
                linkPositions=link_positions,
                linkOrientations=link_orientations,
                linkInertialFramePositions=link_inertial_positions,
                linkInertialFrameOrientations=link_inertial_orientations,
                linkParentIndices=parent_indices,
                linkJointTypes=joint_types,
                linkJointAxis=joint_axis
            )
            
            # 固定底座到地面
            p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=-1,
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0.08]
            )
        
        # 获取关节信息
        num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] != p.JOINT_FIXED:  # 不是固定关节
                self.joint_indices.append(i)
        
        # 相机链节索引（使用最后一个关节作为相机位置）
        self.camera_link_index = len(self.joint_indices) - 1 if self.joint_indices else 0
        
        # 设置关节限制（基于Auboi5实际限制）
        joint_limits = [
            (-np.pi, np.pi),        # Joint 1: ±180°
            (-2.059, 2.059),        # Joint 2: ±118°
            (-np.pi, np.pi),        # Joint 3: ±180°
            (-np.pi, np.pi),        # Joint 4: ±180°
            (-2.059, 2.059),        # Joint 5: ±118°
            (-np.pi, np.pi),        # Joint 6: ±180°
        ]
        
        for i, (lower, upper) in enumerate(joint_limits):
            if i < len(self.joint_indices):
                p.changeDynamics(self.robot_id, self.joint_indices[i], 
                               jointLowerLimit=lower, jointUpperLimit=upper)
        
        # 更新关节限制列表
        self.joint_limits = joint_limits[:len(self.joint_indices)]
        
        print(f"Auboi5机械臂创建成功！")
        print(f"- 机械臂ID: {self.robot_id}")
        print(f"- 关节数量: {len(self.joint_indices)}")
        print(f"- 相机链节索引: {self.camera_link_index}")
        print(f"- 底座已固定到地面")
    
    def _create_target_objects(self):
        """创建目标物体（在Auboi5工作范围内）"""
        # Auboi5机械臂的工作半径约为0.85m，考虑安全距离
        working_radius = 0.7
        table_height = 0.4
        
        # 创建工作台面
        table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.4, 0.02])
        table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.4, 0.02], 
                                         rgbaColor=[0.8, 0.6, 0.4, 1])  # 木色
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision, 
                         baseVisualShapeIndex=table_visual, 
                         basePosition=[working_radius*0.8, 0, table_height])
        
        try:
            # 红色立方体 - 在机械臂前方
            red_cube = p.loadURDF("cube_small.urdf", [working_radius*0.7, 0.1, table_height + 0.1], 
                                globalScaling=2.0)
            p.changeVisualShape(red_cube, -1, rgbaColor=[1, 0, 0, 1])
        except:
            # 如果URDF加载失败，创建简单的立方体
            red_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
            red_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
            p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=red_collision, 
                             baseVisualShapeIndex=red_visual, 
                             basePosition=[working_radius*0.7, 0.1, table_height + 0.1])
        
        # 蓝色长方体 - 机械臂右前方
        blue_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.04, 0.04, 0.08])
        blue_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.04, 0.04, 0.08], rgbaColor=[0, 0, 1, 1])
        p.createMultiBody(baseMass=0.3, baseCollisionShapeIndex=blue_collision, 
                         baseVisualShapeIndex=blue_visual, 
                         basePosition=[working_radius*0.8, -0.15, table_height + 0.12])
        
        # 绿色球体 - 机械臂左前方
        green_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.06)
        green_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.06, rgbaColor=[0, 1, 0, 1])
        p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=green_collision, 
                         baseVisualShapeIndex=green_visual, 
                         basePosition=[working_radius*0.6, 0.2, table_height + 0.1])
        
        # 黄色圆柱体 - 机械臂正前方远一点
        yellow_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.03, height=0.1)
        yellow_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.03, length=0.1, rgbaColor=[1, 1, 0, 1])
        p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=yellow_collision, 
                         baseVisualShapeIndex=yellow_visual, 
                         basePosition=[working_radius*0.9, 0, table_height + 0.15])
        
        print(f"目标物体创建完成 - 工作半径: {working_radius}m, 台面高度: {table_height}m")
    
    def _wait_for_motion_complete(self, target_joints, tolerance=0.01, timeout=10.0):
        """等待运动完成"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_joints = []
            for joint_idx in self.joint_indices:
                joint_state = p.getJointState(self.robot_id, joint_idx)
                current_joints.append(joint_state[0])
            
            # 检查是否到达目标位置
            errors = [abs(current - target) for current, target in zip(current_joints, target_joints)]
            if all(error < tolerance for error in errors):
                break
            
            p.stepSimulation()
            time.sleep(0.01)

# 为了保持兼容性，创建一个别名
Auboi5Robot = PyBulletAuboi5Robot

if __name__ == "__main__":
    # 测试PyBullet机械臂适配器
    import cv2
    import numpy as np
    
    print("测试PyBullet机械臂适配器...")
    
    # 创建机械臂
    robot = Auboi5Robot(gui=True)
    
    try:
        # 初始化
        Auboi5Robot.initialize()
        robot.create_context()
        
        # 连接
        result = robot.connect()
        if result == RobotErrorType.RobotError_SUCC:
            print("连接成功")
            
            # 启动
            robot.robot_startup()
            robot.init_profile()
            
            # 测试运动
            print("测试关节运动...")
            test_joints = [0.5, -0.3, 0.6, 0.2, -0.4, 0.1]
            robot.move_joint(test_joints)
            
            # 测试相机
            print("测试相机图像...")
            for i in range(100):
                color_image, depth_image = robot.get_camera_image()
                if color_image is not None:
                    cv2.imshow('PyBullet Camera', color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                robot.step_simulation()
                time.sleep(0.033)
            
            # 清理
            robot.robot_shutdown()
            robot.disconnect()
        
    except KeyboardInterrupt:
        print("测试中断")
    finally:
        cv2.destroyAllWindows()
        Auboi5Robot.uninitialize()
        print("测试完成")
