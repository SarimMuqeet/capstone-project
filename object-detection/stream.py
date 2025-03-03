import cv2
from ultralytics import YOLO
import time
import os
import requests
import numpy as np
import socket
import struct
from PIL import Image
from io import BytesIO

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

# Global variables for object tracking
MAX_DISAPPEARED = 30
objects = {}
next_object_id = 0

def test_connection(ip, port=80, timeout=2):
    """Test if the device is reachable on the network"""
    try:
        socket.create_connection((ip, port), timeout=timeout)
        return True
    except (socket.timeout, socket.error):
        return False

def frame_config_decode(frame_config):
    '''
        @frame_config bytes
        @return fields, tuple (trigger_mode, deep_mode, deep_shift, ir_mode, status_mode, status_mask, rgb_mode, rgb_res, expose_time)
    '''
    return struct.unpack("<BBBBBBBBi", frame_config)

def frame_config_encode(trigger_mode=1, deep_mode=1, deep_shift=255, ir_mode=1, status_mode=2, status_mask=7, rgb_mode=1, rgb_res=0, expose_time=0):
    return struct.pack("<BBBBBBBBi",
                       trigger_mode, deep_mode, deep_shift, ir_mode, status_mode, status_mask, rgb_mode, rgb_res, expose_time)

def frame_payload_decode(frame_data: bytes, with_config: tuple):
    deep_data_size, rgb_data_size = struct.unpack("<ii", frame_data[:8])
    frame_payload = frame_data[8:]
    # 0:16bit 1:8bit, resolution: 320*240
    deepth_size = (320*240*2) >> with_config[1]
    deepth_img = struct.unpack("<%us" % deepth_size, frame_payload[:deepth_size])[
        0] if 0 != deepth_size else None
    frame_payload = frame_payload[deepth_size:]

    # 0:16bit 1:8bit, resolution: 320*240
    ir_size = (320*240*2) >> with_config[3]
    ir_img = struct.unpack("<%us" % ir_size, frame_payload[:ir_size])[
        0] if 0 != ir_size else None
    frame_payload = frame_payload[ir_size:]

    status_size = (320*240//8) * (16 if 0 == with_config[4] else
                                  2 if 1 == with_config[4] else 8 if 2 == with_config[4] else 1)
    status_img = struct.unpack("<%us" % status_size, frame_payload[:status_size])[
        0] if 0 != status_size else None
    frame_payload = frame_payload[status_size:]

    assert(deep_data_size == deepth_size+ir_size+status_size)

    rgb_size = len(frame_payload)
    assert(rgb_data_size == rgb_size)
    rgb_img = struct.unpack("<%us" % rgb_size, frame_payload[:rgb_size])[
        0] if 0 != rgb_size else None

    if (not rgb_img is None) and (1 == with_config[6]):
        jpeg = cv2.imdecode(np.frombuffer(
            rgb_img, 'uint8', rgb_size), cv2.IMREAD_COLOR)
        if not jpeg is None:
            rgb = cv2.cvtColor(jpeg, cv2.COLOR_BGR2RGB)
            rgb_img = rgb.tobytes()
        else:
            rgb_img = None

    return (deepth_img, ir_img, status_img, rgb_img)

def get_frame_from_http(host, port=80):
    r = requests.get(f'http://{host}:{port}/getdeep')
    if(r.status_code == requests.codes.ok):
        return r.content
    return None

def get_center_position(x1, y1, x2, y2):
    """Calculate the center position of a bounding box"""
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    return center_x, center_y

def get_distance_from_depth(depth_frame, center_x, center_y, rgb_width=640, rgb_height=480, depth_width=320, depth_height=240, deep_mode=1):
    """
    Calculate the distance from the camera using the depth frame
    Converts RGB coordinates to depth coordinates and gets the depth value
    """
    # Convert RGB coordinates to depth coordinates (scale down)
    depth_x = int(center_x * depth_width / rgb_width)
    depth_y = int(center_y * depth_height / rgb_height)
    
    # Ensure coordinates are within bounds
    depth_x = max(0, min(depth_x, depth_width - 1))
    depth_y = max(0, min(depth_y, depth_height - 1))
    
    # Get depth value at the center point
    depth_value = depth_frame[depth_y, depth_x]
    
    # Depth value in mm
    distance_mm = depth_value
    
    return distance_mm

def calculate_iou(bbox1, bbox2):
    x1, y1, x2, y2 = bbox1
    x1_, y1_, x2_, y2_ = bbox2
    
    xi1 = max(x1, x1_)
    yi1 = max(y1, y1_)
    xi2 = min(x2, x2_)
    yi2 = min(y2, y2_)
    
    inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    
    box1_area = (x2 - x1) * (y2 - y1)
    box2_area = (x2_ - x1_) * (y2_ - y1_)
    
    union_area = box1_area + box2_area - inter_area
    
    iou = inter_area / union_area if union_area > 0 else 0
    return iou

def update_object_tracking(detected_objects):
    global objects

    # Increment disappeared count for all existing objects
    for object_id in list(objects.keys()):
        objects[object_id]['disappeared'] += 1

    # Update or add new objects
    for detected_object in detected_objects:
        matched = False
        for object_id, obj in objects.items():
            if calculate_iou(detected_object['bbox'], obj['bbox']) > 0.5:
                objects[object_id].update({
                    'bbox': detected_object['bbox'],
                    'class_name': detected_object['class_name'],
                    'confidence': detected_object['confidence'],
                    'center': detected_object['center'],
                    'distance': detected_object['distance'],
                    'disappeared': 0
                })
                matched = True
                break

        if not matched:
            new_id = max(objects.keys()) + 1 if objects else 1
            objects[new_id] = {
                'bbox': detected_object['bbox'],
                'class_name': detected_object['class_name'],
                'confidence': detected_object['confidence'],
                'center': detected_object['center'],
                'distance': detected_object['distance'],
                'disappeared': 0
            }

    # Remove objects that have been disappeared for too long
    for object_id in list(objects.keys()):
        if objects[object_id]['disappeared'] > MAX_DISAPPEARED:
            del objects[object_id]

    return objects

def image_coord_to_world(x, y, distance, fx, fy, cx, cy):
    """Convert image coordinates and distance to world coordinates"""
    # Calculate normalized image coordinates
    x_norm = (x - cx) / fx
    y_norm = (cy - y) / fy
    
    # Calculate scaling factor
    scale = distance / np.sqrt(x_norm**2 + y_norm**2 + 1)
    
    # Calculate world coordinates
    x_world = x_norm * scale
    y_world = y_norm * scale
    z_world = scale  # This is the depth component
    
    return x_world, y_world, z_world


def main():
    # MaixSense A075V camera IP address
    maixsense_ip = "192.168.233.1"
    port = 80

    fx, fy = 320, 320  # Focal length
    cx, cy = 320, 240  # Principal point
    
    # Check if device is reachable
    print(f"Testing connection to MaixSense at {maixsense_ip}...")
    if not test_connection(maixsense_ip):
        print(f"Error: Cannot connect to MaixSense at {maixsense_ip}")
        print("Please check that the device is connected and powered on.")
        return
    
    print("Connection successful! Setting up configuration...")
    
    # Configure the camera - using deep_mode=0 for 16-bit depth data for better accuracy
    camera_config = frame_config_encode(1, 0, 255, 0, 2, 7, 1, 0, 0)
    if not post_encode_config(camera_config, maixsense_ip, port):
        print("Failed to set camera configuration")
        return
    
    # Extract deep_mode from config for distance calculation
    config_values = frame_config_decode(camera_config)
    deep_mode = config_values[1]
    
    # Load YOLO model
    model = YOLO('yolo11m.pt')
    selected_classes = [0, 32, 39, 40, 41, 56, 57, 58, 59, 60, 62, 65, 67, 73, 74, 75, 77]
    
    prev_time = time.time()
    fps = 0
    
    # Create OpenCV window
    cv2.namedWindow('MaixSense YOLO Detection', cv2.WINDOW_NORMAL)
    
    # Main processing loop
    while True:
        try:
            # Get frame from MaixSense
            frame_data = get_frame_from_http(maixsense_ip, port)
            
            if frame_data:
                # Decode the frame
                config = frame_config_decode(frame_data[16:16+12])
                frame_bytes = frame_payload_decode(frame_data[16+12:], config)
                
                # Get RGB data
                rgb_data = frame_bytes[3]
                
                # Get depth data
                depth_data = frame_bytes[0]
                depth_frame = None
                if depth_data:
                    depth_frame = np.frombuffer(depth_data, 'uint16' if 0 == config[1] else 'uint8').reshape(240, 320)
                
                # Store detected objects with positions
                detected_objects = []

                if rgb_data and depth_frame is not None:
                    # Convert RGB data to numpy array
                    rgb_frame = np.frombuffer(rgb_data, 'uint8').reshape((480, 640, 3))
                    
                    # Convert to BGR for OpenCV
                    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                    
                    # Run object detection
                    results = model(frame, conf=0.5, classes=selected_classes)
                    
                    for result in results:
                        boxes = result.boxes  # Boxes object for bbox outputs
                        for box in boxes:
                            # Get box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            
                            # Calculate center position
                            center_x, center_y = get_center_position(x1, y1, x2, y2)
                            
                            # Get distance from depth frame
                            distance_mm = get_distance_from_depth(depth_frame, center_x, center_y, 
                                                                 rgb_width=640, rgb_height=480, 
                                                                 depth_width=320, depth_height=240,
                                                                 deep_mode=config[1])
                            
                            class_id = int(box.cls[0])
                            conf = float(box.conf[0])
                            class_name = model.names[class_id]

                            x_world, y_world, z_world = image_coord_to_world(center_x, center_y, distance_mm, fx, fy, cx, cy)
                            
                            # Add to detected objects list
                            detected_objects.append({
                                'class_name': class_name,
                                'confidence': conf,
                                'bbox': (x1, y1, x2, y2),
                                'center': (center_x, center_y),
                                'distance': distance_mm
                            })

                tracked_objects = update_object_tracking(detected_objects)
                
                print("\n--- Frame Information ---")
                print(f"FPS: {fps:.1f}")
                print("Tracked Objects:")

                for obj_id, obj in tracked_objects.items():
                    x1, y1, x2, y2 = obj['bbox']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, obj['center'], 5, (0, 0, 255), -1)
                    
                    label = f"ID:{obj_id} {obj['class_name']} {obj['confidence']:.2f} {obj['distance']:.0f}mm"
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    print(f"  Object ID: {obj_id}")
                    print(f"    Class: {obj['class_name']}")
                    print(f"    Confidence: {obj['confidence']:.2f}")
                    print(f"    Bounding Box: ({x1}, {y1}, {x2}, {y2})")
                    print(f"    Center (image coordinates): ({center_x}, {center_y})")
                    print(f"    World Coordinates (mm): X={x_world:.2f}, Y={y_world:.2f}, Z={z_world:.2f}")
                    print(f"    Distance: {obj['distance']:.2f} mm")
                    print(f"    Disappeared count: {obj['disappeared']}")
                    print()

                    if obj['disappeared'] == 0:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        label = f"ID:{obj_id} {obj['class_name']} {obj['confidence']:.2f} {obj['distance']:.0f}mm"
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
                    if rgb_data is None:
                        print("Error: No RGB data in frame")
                    if depth_frame is None:
                        print("Error: No depth data in frame")
            else:
                print("Error: Failed to get frame from MaixSense")
                time.sleep(0.5)
                
        except Exception as e:
            print(f"Error processing frame: {e}")
            time.sleep(1)  # Wait before retrying
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

def post_encode_config(config, host, port=80):
    try:
        r = requests.post(f'http://{host}:{port}/set_cfg', config)
        return r.status_code == requests.codes.ok
    except Exception as e:
        print(f"Error setting config: {e}")
        return False

if __name__ == '__main__':
    main()
