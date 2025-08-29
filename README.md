# 视觉伺服系统 (Visual Servoing System)

基于YOLO目标检测和RealSense深度相机的PBVS/IBVS视觉伺服系统。

## 功能特性

- **PBVS (基于位置的视觉伺服)**: 在3D空间中进行精确位置控制
- **IBVS (基于图像的视觉伺服)**: 直接在图像空间中进行伺服控制  
- **YOLO目标检测**: 自动识别和跟踪目标物体
- **深度感知**: 利用RealSense相机获取3D信息
- **相机标定**: 内置相机标定工具
- **实时可视化**: 显示检测结果和控制状态

## 快速开始

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 系统测试
```bash
python test_visual_servoing.py
```

### 3. 相机标定
```bash
python camera_calibration.py
```

### 4. 运行视觉伺服
```bash
# 高级版本（推荐）
python advanced_visual_servoing.py

# 基础版本
python visual_servoing.py
```

## 文件说明

- `visual_servoing.py` - 基础视觉伺服实现
- `advanced_visual_servoing.py` - 高级视觉伺服系统（推荐）
- `camera_calibration.py` - 相机标定工具
- `test_visual_servoing.py` - 系统测试脚本
- `visual_servoing_guide.md` - 详细使用指南
- `yolo_detect.py` - YOLO目标检测
- `yolo_train.py` - YOLO模型训练

## 使用指南

详细使用说明请参考 [visual_servoing_guide.md](visual_servoing_guide.md)

## 硬件要求

- Intel RealSense 深度相机 (D435/D455等)
- 标准棋盘格标定板 (9x6内角点，方格尺寸2.5cm)
