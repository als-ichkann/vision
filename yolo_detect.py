import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np

# Load the trained YOLO model
model = YOLO('yolo_train/weights/best.pt')

# Configure the RealSense camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth stream

# Start the pipeline
pipeline.start(config)

# Loop to process frames
try:
    while True:
        # Wait for a set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get the color frame and depth frame
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # If no frame, skip the iteration
        if not color_frame or not depth_frame:
            continue

        # Convert RealSense frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Use YOLOv5 for inference
        results = model(color_image)

        # Process detection results and extract bounding boxes
        for result in results[0].boxes:
            # Get the coordinates (x, y, width, height) of the bounding box
            x_center, y_center, width, height = result.xywh[0]
            confidence = result.conf[0]
            class_id = result.cls[0]

            # Get class name from results
            class_name = results[0].names[int(class_id)]

            # Print the detected object's class and coordinates
            print(f"Object: {class_name}, Coordinates: ({x_center}, {y_center}, {width}, {height}), Confidence: {confidence:.2f}")

            # Get the depth value at the center of the bounding box
            # Note: depth is in millimeters
            depth_value = depth_image[int(y_center), int(x_center)]  # Depth at the center pixel

            # Print the depth value
            print(f"Depth: {depth_value} mm")

            # If you want to get the distance to the object (in meters):
            distance = depth_value / 1000.0  # Convert depth from mm to meters
            print(f"Distance to object: {distance:.2f} meters")

        # Draw bounding boxes and class labels on the frame
        frame = results[0].plot()

        # Display the result
        cv2.imshow('YOLOv5 Object Detection with Depth', frame)

        # Exit loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the RealSense camera and close OpenCV window
    pipeline.stop()
    cv2.destroyAllWindows()
