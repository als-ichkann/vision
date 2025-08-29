from ultralytics import YOLO

# Load a COCO-pretrained YOLOv5n model
model = YOLO("yolov5n.pt")

# Display model information (optional)
model.info()
if __name__ == '__main__':
    # Train the model on the COCO8 example dataset for 100 epochs
    results = model.train(data="C:\\Users\\Wang\\Documents\\PyCharm\\UNet-Robot\\UNet-Robot\\tomato_pick.v2i.yolov5pytorch\\data.yaml", epochs=100, imgsz=640, workers=4)
