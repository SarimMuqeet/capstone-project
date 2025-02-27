import cv2
from ultralytics import YOLO
import time
import os
import requests
import numpy as np
import socket
import urllib.request
from PIL import Image
from io import BytesIO
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

def test_connection(ip, port=80, timeout=2):
    """Test if the device is reachable on the network"""
    try:
        socket.create_connection((ip, port), timeout=timeout)
        return True
    except (socket.timeout, socket.error):
        return False

def main():
    # MaixSense A075 camera IP address
    maixsense_ip = "192.168.233.1"
    
    # Check if device is reachable
    print(f"Testing connection to MaixSense at {maixsense_ip}...")
    if not test_connection(maixsense_ip):
        print(f"Error: Cannot connect to MaixSense at {maixsense_ip}")
        print("Please check that the device is connected and powered on.")
        return
    
    print("Connection successful! Attempting to get video feed...")
    
    # Try different possible endpoints for the RGB image
    endpoints = [
        "/rgb",           # Standard RGB endpoint
        "/image",         # Alternative endpoint
        "/camera/rgb",    # Another possible endpoint
        "/stream",        # Streaming endpoint
        "/video"          # Video endpoint
    ]
    
    # Load YOLO model
    model = YOLO('yolo11m.pt')
    selected_classes = [0, 32, 39, 40, 41, 56, 57, 58, 59, 60, 62, 65, 67, 73, 74, 75, 77]
    
    prev_time = time.time()
    fps = 0
    
    # Try to find a working endpoint
    working_endpoint = None
    for endpoint in endpoints:
        try:
            url = f"http://{maixsense_ip}{endpoint}"
            print(f"Trying endpoint: {url}")
            response = requests.get(url, timeout=3)
            if response.status_code == 200 and len(response.content) > 0:
                # Try to decode the image to verify it's valid
                img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                test_frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if test_frame is not None:
                    working_endpoint = endpoint
                    print(f"Found working endpoint: {endpoint}")
                    break
        except Exception as e:
            print(f"Endpoint {endpoint} error: {e}")
    
    if working_endpoint is None:
        print("Error: Could not find a working image endpoint")
        print("You may need to check the MaixSense documentation for the correct endpoint")
        return
    
    rgb_url = f"http://{maixsense_ip}{working_endpoint}"
    print(f"Using endpoint: {rgb_url}")
    
    # Main processing loop
    while True:
        try:
            # Get RGB image from MaixSense
            response = requests.get(rgb_url, stream=True, timeout=5)
            
            if response.status_code == 200:
                content_length = len(response.content)
                print(f"Response content length: {content_length} bytes")
                
                if content_length > 0:
                    # Try PIL first as an alternative method
                    try:
                        image = Image.open(BytesIO(response.content))
                        frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
                    except Exception as pil_error:
                        print(f"PIL decode failed: {pil_error}")
                        # Fall back to OpenCV
                        img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is None:
                        print("Error: Could not decode image")
                        time.sleep(0.5)
                        continue
                    
                    # Run object detection
                    results = model(frame, conf=0.5, classes=selected_classes)
                    
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
                    
                    # Calculate FPS
                    current_time = time.time()
                    fps = 1 / (current_time - prev_time)
                    prev_time = current_time
                    cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Display the frame
                    cv2.imshow('MaixSense YOLO Detection', frame)
                else:
                    print("Error: Empty response content")
                    time.sleep(0.5)
            else:
                print(f"Error: HTTP status code {response.status_code}")
                time.sleep(1)
                
        except Exception as e:
            print(f"Error connecting to MaixSense: {e}")
            time.sleep(1)  # Wait before retrying
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
