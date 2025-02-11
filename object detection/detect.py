from ultralytics import YOLO
import cv2
import torch
import torchvision.transforms as transforms
from PIL import Image
import os
import urllib.request
import random

# Download imagenet_classes.txt
def download_imagenet_classes(filepath="imagenet_classes.txt"):
    if not os.path.exists(filepath):
        print(f"Downloading {filepath}...")
        url = "https://raw.githubusercontent.com/pytorch/hub/master/imagenet_classes.txt"
        urllib.request.urlretrieve(url, filepath)
        print("Download complete.")
    else:
        print(f"{filepath} already exists.")

# Generating class colours for bounding boxes
def get_class_colors(num_classes):
    random.seed(42)
    colors = []
    for _ in range(num_classes):
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        colors.append(color)
    return colors

#initialize video capture with webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()


#load YOLO model for object detection
yolo_model = YOLO("yolo11n.pt")

#load ResNet model for object classification
resnet_model = torch.hub.load('pytorch/vision:v0.10.0', 'resnet50', pretrained=True)
resnet_model.eval()  # Set to evaluation mode

#define image transformations for ResNet
preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
])

#download and load imagenet_classes.txt
download_imagenet_classes("imagenet_classes.txt")
with open("imagenet_classes.txt") as f:
    imagenet_classes = [line.strip() for line in f.readlines()]


#define predefined mappings
placement_mapping = {
    'cup': 'kitchen_cabinet',
    'clothing': 'hamper',
    'lamp': 'table',
    'houseplant': 'table',
    'toy': 'box',
    'footwear': 'shoe_rack',
    'book': 'bookshelf',
    'box': 'storage_bin',
    'pillow': 'bed',
    'bottle': 'kitchen_cabinet',
}

# decide placement
def decide_placement(object_class, context_data=None):
    # use predefined mapping
    placement = placement_mapping.get(object_class.lower(), 'miscellaneous')
    return placement

NUM_CLASSES = len(yolo_model.names)
class_colors = get_class_colors(NUM_CLASSES)


while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # object detection
    results = yolo_model(frame)

    for result in results:
        boxes = result.boxes
        class_ids = result.boxes.cls.cpu().numpy().astype(int) 
        confidences = result.boxes.conf.cpu().numpy()

        for box, class_id, confidence in zip(boxes, class_ids, confidences):
            yolo_class = yolo_model.names[class_id]
            confidence_percentage = confidence * 100

            color = class_colors[class_id % NUM_CLASSES]

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(frame.shape[1] - 1, x2)
            y2 = min(frame.shape[0] - 1, y2)

            cropped_img = frame[y1:y2, x1:x2]
            if cropped_img.size == 0:
                continue 

            #convert to PIL Image for ResNet
            pil_img = Image.fromarray(cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB))
            input_tensor = preprocess(pil_img)
            input_batch = input_tensor.unsqueeze(0)

            #use GPU if available
            if torch.cuda.is_available():
                input_batch = input_batch.to('cuda')
                resnet_model.to('cuda')

            with torch.no_grad():
                output = resnet_model(input_batch)

            #predict using ResNet
            _, predicted_idx = torch.max(output, 1)
            resnet_class = imagenet_classes[predicted_idx.item()]
            
            #decide where it goes!
            placement = decide_placement(yolo_class)

            #labelling
            label = f"{yolo_class} {confidence_percentage:.2f}% | placement: {placement}"

            #console
            print(f"Detected Object: {yolo_class}, ResNet Classification: {resnet_class}, Placement: {placement}")

            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

            y_label = max(y1 - text_height - baseline - 10, 0)

            cv2.rectangle(frame, (x1, y_label), (x1 + text_width, y_label + text_height + baseline), color, -1)

            cv2.putText(frame, label, (x1, y_label + text_height + baseline - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    cv2.imshow('Home Organizer Robot', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
