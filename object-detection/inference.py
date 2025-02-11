import cv2
from ultralytics import YOLO
import time
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"


def main():
    # Try different backend for video capture
    camera_index = 0
    cap = cv2.VideoCapture(camera_index)  # Use DirectShow backend
    
     
    if not cap.isOpened():
        # Try DirectShow with different indices
        for index in range(2):
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            if cap.isOpened():
                break
    
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    model = YOLO('yolo11m.pt')

    selected_classes = [0, 32, 39, 40, 41, 56, 57, 58, 59, 60, 62, 65, 67, 73, 74, 75, 77]
    
    # Set properties after confirming camera is opened
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Add a small delay to allow camera initialization
    time.sleep(1)
    
    prev_time = time.time()
    fps = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame")
            time.sleep(0.1)  # Add small delay before retrying
            continue
            
        results = model(frame, conf=0.5, classes=selected_classes)  # confidence threshold of 0.5
        
        for result in results:
            boxes = result.boxes  # Boxes object for bbox outputs
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                class_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = model.names[class_id]
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                label = f'{class_name} {conf:.2f}'
                cv2.putText(frame, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time
        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('YOLO Webcam Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
