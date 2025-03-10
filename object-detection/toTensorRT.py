from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("yolo11m.pt")

model.export(format="engine", int8=True, data="coco.yaml")
