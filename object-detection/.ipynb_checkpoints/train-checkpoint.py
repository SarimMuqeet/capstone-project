from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO('yolov8n.pt')
    model.train(
        data='data.yaml',
        epochs=5,
        batch=16,
        imgsz=640,
        device='0'  # Specify GPU device
    )
