import cv2
import numpy as np
import pyrealsense2 as rs
import os
import json
from typing import Tuple, List

class CameraCalibration:
    """相机标定工具"""
    
    def __init__(self):
        # 棋盘格参数
        self.checkerboard_size = (9, 6)  # 内角点数量 (宽, 高)
        self.square_size = 0.025  # 每个方格的实际尺寸 (米)
        
        # 标定数据存储
        self.object_points = []  # 3D点
        self.image_points = []   # 2D点
        
        # RealSense相机配置
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 创建保存标定图片的文件夹
        self.calibration_dir = "calibration_images"
        if not os.path.exists(self.calibration_dir):
            os.makedirs(self.calibration_dir)
    
    def generate_object_points(self):
        """生成棋盘格的3D坐标点"""
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 
                              0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp
    
    def capture_calibration_images(self, num_images=20):
        """采集标定图像"""
        print(f"开始采集{num_images}张标定图像...")
        print("按空格键拍照，按'q'退出")
        
        self.pipeline.start(self.config)
        captured_count = 0
        objp = self.generate_object_points()
        
        try:
            while captured_count < num_images:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                # 寻找棋盘格角点
                ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
                
                # 绘制实时预览
                display_image = color_image.copy()
                if ret:
                    cv2.drawChessboardCorners(display_image, self.checkerboard_size, corners, ret)
                    cv2.putText(display_image, "Found checkerboard! Press SPACE to capture", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display_image, "Move checkerboard into view", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                cv2.putText(display_image, f"Captured: {captured_count}/{num_images}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                cv2.imshow('Camera Calibration', display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' ') and ret:
                    # 保存图像和角点
                    img_name = f"{self.calibration_dir}/calib_{captured_count:02d}.jpg"
                    cv2.imwrite(img_name, color_image)
                    
                    # 提高角点精度
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    
                    self.object_points.append(objp)
                    self.image_points.append(corners2)
                    
                    captured_count += 1
                    print(f"已采集 {captured_count}/{num_images} 张图像")
                
                elif key == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
        
        return captured_count >= 10  # 至少需要10张图像进行标定
    
    def calibrate_camera(self, image_size=(640, 480)):
        """执行相机标定"""
        if len(self.object_points) < 10:
            print("标定图像数量不足，至少需要10张")
            return None
        
        print("开始相机标定...")
        
        # 执行标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, image_size, None, None
        )
        
        if ret:
            print("标定成功！")
            print(f"重投影误差: {ret:.4f}")
            print("相机内参矩阵:")
            print(camera_matrix)
            print("畸变系数:")
            print(dist_coeffs.flatten())
            
            # 保存标定结果
            calibration_data = {
                'camera_matrix': camera_matrix.tolist(),
                'dist_coeffs': dist_coeffs.flatten().tolist(),
                'image_size': image_size,
                'reprojection_error': ret,
                'num_images': len(self.object_points)
            }
            
            with open('camera_calibration.json', 'w') as f:
                json.dump(calibration_data, f, indent=4)
            
            print("标定结果已保存到 camera_calibration.json")
            return calibration_data
        else:
            print("标定失败")
            return None
    
    def load_calibration(self, filename='camera_calibration.json'):
        """加载已保存的标定结果"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            camera_matrix = np.array(data['camera_matrix'])
            dist_coeffs = np.array(data['dist_coeffs'])
            
            return camera_matrix, dist_coeffs
        except FileNotFoundError:
            print(f"标定文件 {filename} 不存在")
            return None, None
    
    def test_calibration(self):
        """测试标定结果"""
        camera_matrix, dist_coeffs = self.load_calibration()
        if camera_matrix is None:
            print("请先进行相机标定")
            return
        
        print("测试相机标定结果...")
        print("按'q'退出测试")
        
        self.pipeline.start(self.config)
        
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                
                # 去畸变
                undistorted = cv2.undistort(color_image, camera_matrix, dist_coeffs)
                
                # 并排显示原图和去畸变图像
                combined = np.hstack((color_image, undistorted))
                cv2.putText(combined, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(combined, "Undistorted", (650, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Calibration Test', combined)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
    
    def run_calibration_process(self):
        """完整的标定流程"""
        print("=== 相机标定流程 ===")
        print("1. 准备一个标准棋盘格（9x6内角点）")
        print("2. 确保棋盘格方格尺寸为2.5cm")
        print("3. 在不同角度和距离拍摄棋盘格")
        
        # 采集标定图像
        if self.capture_calibration_images():
            # 执行标定
            result = self.calibrate_camera()
            if result:
                # 测试标定结果
                test = input("是否测试标定结果? (y/n): ")
                if test.lower() == 'y':
                    self.test_calibration()
        else:
            print("标定图像采集失败")


def main():
    calibrator = CameraCalibration()
    
    while True:
        print("\n=== 相机标定工具 ===")
        print("1. 运行完整标定流程")
        print("2. 测试已有标定结果")
        print("3. 加载标定参数")
        print("4. 退出")
        
        choice = input("请选择操作: ")
        
        if choice == "1":
            calibrator.run_calibration_process()
        elif choice == "2":
            calibrator.test_calibration()
        elif choice == "3":
            camera_matrix, dist_coeffs = calibrator.load_calibration()
            if camera_matrix is not None:
                print("相机内参矩阵:")
                print(camera_matrix)
                print("畸变系数:")
                print(dist_coeffs)
        elif choice == "4":
            break
        else:
            print("无效选择")


if __name__ == "__main__":
    main()
