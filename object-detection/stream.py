import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import requests
import struct
import math
import time

# Camera and model configuration
HOST = '192.168.233.1'
PORT = 80
FRAME_WIDTH = 640  # Changed from 320
FRAME_HEIGHT = 480 # Changed from 240
FX, FY = 320, 320  # Updated focal lengths (was 366.5)
CX, CY = 320, 240  # Updated principal points (was 160,120)

# Depth configuration aligned with RGB
DEPTH_WIDTH = 320
DEPTH_HEIGHT = 240
DEPTH_FX = 320     # Matches RGB focal length
DEPTH_FY = 320
DEPTH_CX = 160      # Half of RGB CX (320/2)
DEPTH_CY = 120      # Half of RGB CY (240/2)

DEPTH_SAMPLE_SIZE = 5  # Sample 5x5 area around center point
DEPTH_SCALE = 1.0   # Raw depth is in mm
MIN_VALID_DEPTH_MM = 100   # 0.1m
MAX_VALID_DEPTH_MM = 3000  # 3m
SMOOTHING_FACTOR = 0.3 

# Field of View declarations (unused in coordinate conversion)
RGB_H_FOV = 120
RGB_V_FOV = 120
DEPTH_H_FOV = 55
DEPTH_V_FOV = 72

ENGINE_PATH = "./tensorrt_demos/yolo/yolov4-tiny-fp16.trt"
CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# Tracking configuration
MATCH_IOU_THRESH = 0.4  # Increased from 0.3
MAX_DISAPPEARED = 15

objects = {}
next_object_id = 0

def calculate_iou(bbox1, bbox2):
    xi1 = max(bbox1[0], bbox2[0])
    yi1 = max(bbox1[1], bbox2[1])
    xi2 = min(bbox1[2], bbox2[2])
    yi2 = min(bbox1[3], bbox2[3])
    inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    box1_area = (bbox1[2]-bbox1[0])*(bbox1[3]-bbox1[1])
    box2_area = (bbox2[2]-bbox2[0])*(bbox2[3]-bbox2[1])
    union_area = box1_area + box2_area - inter_area
    return inter_area / union_area if union_area > 0 else 0

def update_object_tracking(detections):
    global objects, next_object_id

    current_centers = {
        obj_id: (obj['center'], obj['bbox']) 
        for obj_id, obj in objects.items()
    }

    # Create detected objects with centers
    detected_objects = [{
        'bbox': (x1, y1, x2, y2),
        'center': ((x1+x2)//2, (y1+y2)//2),
        'class_id': cid,
        'confidence': conf
    } for (x1, y1, x2, y2, cid, conf) in detections]

    # Match using IoU and center distance
    matches = []
    unmatched_detections = []
    unmatched_trackers = []

    # Check for matches
    for d_idx, det in enumerate(detected_objects):
        best_match = None
        best_iou = MATCH_IOU_THRESH
        best_dist = float('inf')
        
        for t_idx, (obj_id, tracker) in enumerate(current_centers.items()):
            iou = calculate_iou(det['bbox'], tracker[1])
            dist = math.dist(det['center'], tracker[0])
            
            if iou > best_iou or (iou == best_iou and dist < best_dist):
                best_iou = iou
                best_dist = dist
                best_match = obj_id

        if best_match is not None:
            matches.append((best_match, d_idx))
        else:
            unmatched_detections.append(d_idx)

    # Find unmatched trackers
    matched_trackers = set(m[0] for m in matches)
    for obj_id in current_centers:
        if obj_id not in matched_trackers:
            unmatched_trackers.append(obj_id)

    # Update matched objects
    for obj_id, d_idx in matches:
        det = detected_objects[d_idx]
        obj = objects[obj_id]
        
        # Update position directly from detection
        obj['center'] = det['center']
        obj['bbox'] = det['bbox']
        obj['confidence'] = 0.9 * obj['confidence'] + 0.1 * det['confidence']
        obj['disappeared'] = 0

    # Handle unmatched trackers
    for obj_id in unmatched_trackers:
        objects[obj_id]['disappeared'] += 1
        if objects[obj_id]['disappeared'] > MAX_DISAPPEARED:
            del objects[obj_id]

    # Create new objects for unmatched detections
    for d_idx in unmatched_detections:
        det = detected_objects[d_idx]
        objects[next_object_id] = {
            'bbox': det['bbox'],
            'center': det['center'],
            'class_id': det['class_id'],
            'confidence': det['confidence'],
            'disappeared': 0
        }
        next_object_id += 1

    return objects

def image_coord_to_world(x, y, depth_mm):
    """Improved perspective-aware coordinate conversion"""
    if depth_mm < MIN_VALID_DEPTH_MM or depth_mm > MAX_VALID_DEPTH_MM:
        return (0, 0, 0)
    
    # Convert to meters for calculation
    depth = depth_mm / 1000.0
    
    # Normalized image coordinates (RGB camera)
    x_norm = (x - CX) / FX
    y_norm = (CY - y) / FY  # Y axis points up
    
    # Perspective projection
    scale = depth / math.sqrt(x_norm**2 + y_norm**2 + 1)
    
    # Convert to millimeters
    x_mm = x_norm * scale * 1000
    y_mm = y_norm * scale * 1000
    z_mm = depth * 1000
    
    return x_mm, y_mm, z_mm

def frame_config_encode(trigger_mode=1, deep_mode=1, deep_shift=255, ir_mode=1, status_mode=2, status_mask=7, rgb_mode=1, rgb_res=0, expose_time=0):
    return struct.pack("<BBBBBBBBi",
                       trigger_mode, deep_mode, deep_shift, ir_mode, status_mode, status_mask, rgb_mode, rgb_res, expose_time)

def post_encode_config(config=frame_config_encode(), host=HOST, port=PORT):
    r = requests.post(f'http://{host}:{port}/set_cfg', config)
    return r.status_code == requests.codes.ok

def get_frame_from_http(host=HOST, port=PORT):
    r = requests.get(f'http://{host}:{port}/getdeep')
    if r.status_code == requests.codes.ok:
        return r.content
    return None

def get_engine(engine_path):
    with open(engine_path, "rb") as f, trt.Runtime(trt.Logger(trt.Logger.WARNING)) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def preprocess_image(image):
    input_image = cv2.resize(image, (416, 416))
    input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
    input_image = input_image.transpose((2, 0, 1)).astype(np.float32)
    input_image /= 255.0
    return input_image

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

def postprocess(outputs, image_shape, conf_thresh=0.5, nms_thresh=0.4):
    """Decodes YOLOv4-tiny raw outputs into bounding boxes"""
    orig_h, orig_w = image_shape[:2]
    input_dim = 416  # YOLOv4-tiny default input size

    anchors = {
        32: [(81,82), (135,169), (344,319)],  # for 13×13 grid
        16: [(10,14), (23,27), (37,58)]       # for 26×26 grid
    }

    strides = [32, 16]
    detections = []

    for output, stride in zip(outputs, strides):
        grid_size = input_dim // stride
        output = output.reshape(3, 85, grid_size, grid_size).transpose(0, 2, 3, 1)

        for anchor_idx in range(3):
            anchor_w, anchor_h = anchors[stride][anchor_idx]
            for i in range(grid_size):
                for j in range(grid_size):
                    tx, ty, tw, th = output[anchor_idx, i, j, :4]
                    objectness_raw = output[anchor_idx, i, j, 4]
                    class_raw = output[anchor_idx, i, j, 5:]

                    # Apply sigmoid
                    bx = (sigmoid(tx) + j) * stride
                    by = (sigmoid(ty) + i) * stride
                    bw = anchor_w * math.exp(tw)
                    bh = anchor_h * math.exp(th)
                    objectness = sigmoid(objectness_raw)
                    class_scores = sigmoid(class_raw)

                    # Get class with max score
                    class_id = np.argmax(class_scores)
                    class_conf = class_scores[class_id]
                    score = objectness * class_conf

                    if score > conf_thresh:
                        # Convert to image coordinates
                        x1 = bx - bw / 2.0
                        y1 = by - bh / 2.0
                        x2 = bx + bw / 2.0
                        y2 = by + bh / 2.0

                        scale_x = orig_w / input_dim
                        scale_y = orig_h / input_dim
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)

                        x1 = max(0, x1)
                        y1 = max(0, y1)
                        x2 = min(orig_w - 1, x2)
                        y2 = min(orig_h - 1, y2)

                        detections.append([x1, y1, x2, y2, class_id, score])

    # Apply NMS
    if len(detections) == 0:
        return []

    boxes = np.array([det[:4] for det in detections])
    confidences = np.array([det[5] for det in detections])
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), confidences.tolist(), conf_thresh, nms_thresh)
    
    return [detections[i] for i in indices] if len(indices) > 0 else []

def get_depth_coordinates(u_rgb, v_rgb):
    """Convert RGB coordinates to depth coordinates"""
    # From Code 1's get_distance_from_depth
    depth_width = 320
    depth_height = 240
    rgb_width = 320  # Match Code 2's FRAME_WIDTH
    rgb_height = 240  # Match Code 2's FRAME_HEIGHT
    
    u_depth = int(u_rgb * depth_width / rgb_width)
    v_depth = int(v_rgb * depth_height / rgb_height)
    
    # Clamp to valid range
    u_depth = max(0, min(depth_width-1, u_depth))
    v_depth = max(0, min(depth_height-1, v_depth))
    
    return u_depth, v_depth

def get_depth_value(depth_frame, u_rgb, v_rgb):
    """Accurate depth sampling with coordinate scaling"""
    # Convert RGB to Depth coordinates
    u_depth = int(u_rgb * DEPTH_WIDTH / FRAME_WIDTH)
    v_depth = int(v_rgb * DEPTH_HEIGHT / FRAME_HEIGHT)
    
    # Sample 3x3 area for stability
    depth_samples = []
    for du in [-1, 0, 1]:
        for dv in [-1, 0, 1]:
            x = np.clip(u_depth + du, 0, DEPTH_WIDTH-1)
            y = np.clip(v_depth + dv, 0, DEPTH_HEIGHT-1)
            val = depth_frame[y, x]
            if MIN_VALID_DEPTH_MM <= val <= MAX_VALID_DEPTH_MM:
                depth_samples.append(val)
    
    return np.median(depth_samples) if depth_samples else 0
    
def main():
    # Configure camera (Enable IR projector with ir_mode=1)
    # In Code 2's main() function, replace camera config with:
    if not post_encode_config(frame_config_encode(
        deep_mode=0,        # Keep 16-bit depth
        ir_mode=1,          # Enable IR projector
        expose_time=5000,   # Increased exposure time for better depth
        deep_shift=0,
        status_mask=0
    )):
        print("Failed to configure camera")
        return

    # Set up TensorRT
    engine = get_engine(ENGINE_PATH)
    context = engine.create_execution_context()

    # Tensor names
    input_name = "000_net"
    output_names = ["030_convolutional", "037_convolutional"]

    # Allocate memory
    input_shape = engine.get_tensor_shape(input_name)
    d_input = cuda.mem_alloc(trt.volume(input_shape) * np.dtype(np.float32).itemsize)

    # Prepare outputs
    h_outputs = {}
    d_outputs = {}
    for name in output_names:
        shape = context.get_tensor_shape(name)
        h_outputs[name] = cuda.pagelocked_empty(trt.volume(shape), dtype=np.float32)
        d_outputs[name] = cuda.mem_alloc(h_outputs[name].nbytes)

    prev_time = time.time()
    fps = 0

    stream = cuda.Stream()

    while True:
        frame_data = get_frame_from_http()
        if frame_data is None:
            print("Failed to get frame from camera")
            continue

        # Parse frame data
        config = struct.unpack("<BBBBBBBBi", frame_data[16:16+12])
        (trigger_mode, deep_mode, deep_shift, ir_mode, status_mode, status_mask, rgb_mode, rgb_res, expose_time) = config

        frame_payload = frame_data[16+12:]  # Skip frame_id, frame_stamp, config
        deep_data_size, rgb_data_size = struct.unpack("<ii", frame_payload[:8])

        # Calculate sizes based on config
        deepth_size = (320 * 240 * 2) >> deep_mode  # deep_mode=0: 16-bit (153600 bytes)
        ir_size = (320 * 240 * 2) >> ir_mode        # ir_mode=1: 8-bit (76800 bytes)
        status_size = (320 * 240 // 8) * (16 if status_mode == 0 else 2 if status_mode == 1 else 8 if status_mode == 2 else 1)

        # Check if deep_data_size matches the sum of individual parts
        expected_deep_data_size = deepth_size + ir_size + status_size
        if deep_data_size != expected_deep_data_size:
            print(f"Deep data size mismatch: expected {expected_deep_data_size}, got {deep_data_size}")
            continue

        # Extract each part from the payload
        frame_payload_rest = frame_payload[8:]  # Skip deep_data_size and rgb_data_size
        deepth_data = frame_payload_rest[:deepth_size]
        ir_data = frame_payload_rest[deepth_size:deepth_size + ir_size]
        status_data = frame_payload_rest[deepth_size + ir_size:deepth_size + ir_size + status_size]
        rgb_data = frame_payload_rest[deepth_size + ir_size + status_size:deepth_size + ir_size + status_size + rgb_data_size]

        # Decode RGB image
        frame = cv2.imdecode(np.frombuffer(rgb_data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            print("Failed to decode RGB image")
            continue

        # Process depth data
        try:
            # Keep depth in mm (don't divide by 1000)
            depth_data = np.frombuffer(deepth_data, dtype=np.uint16)
            depth_array = depth_data.reshape((DEPTH_HEIGHT, DEPTH_WIDTH))
            
            # Filter invalid values
            valid_mask = (depth_array >= MIN_VALID_DEPTH_MM) & (depth_array <= MAX_VALID_DEPTH_MM)
            depth_array = np.where(valid_mask, depth_array, 0)
        except Exception as e:
            print(f"Depth error: {str(e)}")
            depth_array = np.zeros((DEPTH_HEIGHT, DEPTH_WIDTH), dtype=np.uint16)
        
        # Preprocess and infer
        input_image = preprocess_image(frame)
        cuda.memcpy_htod_async(d_input, input_image.ravel(), stream)
        
        context.set_tensor_address(input_name, int(d_input))
        for name in output_names:
            context.set_tensor_address(name, int(d_outputs[name]))
        
        context.execute_async_v3(stream.handle)
        for name in output_names:
            cuda.memcpy_dtoh_async(h_outputs[name], d_outputs[name], stream)
        
        stream.synchronize()

        # Postprocess
        detections = postprocess(
            [h_outputs["030_convolutional"], h_outputs["037_convolutional"]], 
            frame.shape
        )

        tracked_objects = update_object_tracking(detections)
        
        # FPS calculation
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time

        # Process tracked objects with stabilized depth
                # Process tracked objects with stabilized depth
        for obj_id, obj in tracked_objects.items():
            u_rgb, v_rgb = obj['center']
            
            # Convert to depth coordinates
            u_depth = int(u_rgb * (DEPTH_WIDTH / FRAME_WIDTH))
            v_depth = int(v_rgb * (DEPTH_HEIGHT / FRAME_HEIGHT))
            
            half_size = DEPTH_SAMPLE_SIZE // 2
            depth_samples = []
            
            # Sample with boundary checks
            for du in range(-half_size, half_size+1):
                for dv in range(-half_size, half_size+1):
                    sample_u = np.clip(u_depth + du, 0, DEPTH_WIDTH-1)
                    sample_v = np.clip(v_depth + dv, 0, DEPTH_HEIGHT-1)
                    depth_val = depth_array[sample_v, sample_u]
                    
                    # Check if value is valid and not NaN
                    if not np.isnan(depth_val) and MIN_VALID_DEPTH_MM <= depth_val <= MAX_VALID_DEPTH_MM:
                        depth_samples.append(depth_val)
            
            # Only update if we get enough samples
            min_valid_samples = 3  # Require at least 3 valid samples
            if len(depth_samples) >= min_valid_samples:
                median_depth = np.median(depth_samples)
                
                # Initialize smoothed depth if needed
                if 'smoothed_depth' not in obj:
                    obj['smoothed_depth'] = median_depth
                else:
                    # EMA smoothing only when we have new data
                    obj['smoothed_depth'] = (SMOOTHING_FACTOR * median_depth + 
                                           (1 - SMOOTHING_FACTOR) * obj['smoothed_depth'])
                
                # Store valid depth
                d = obj['smoothed_depth']
                obj['valid_frames'] = obj.get('valid_frames', 0) + 1
            else:
                # Use last valid depth if available
                d = obj.get('smoothed_depth', 0)
                if 'valid_frames' in obj:
                    obj['valid_frames'] -= 1

            # Handle persistent invalid measurements
            if obj.get('valid_frames', 0) < -5:  # 5 consecutive bad frames
                d = 0
            else:
                obj['world_coords'] = image_coord_to_world(u_rgb, v_rgb, d)
            
            # Debug output
            status = "VALID" if d > MIN_VALID_DEPTH_MM else "INVALID"
            print(f"Obj {obj_id} {status} | Samples: {len(depth_samples)} | Depth: {d:.2f}m")
        
        # Draw tracked objects with IDs
        for obj_id, obj in tracked_objects.items():
            if 'world_coords' not in obj:
                continue
                
            x1, y1, x2, y2 = obj['bbox']
            u_rgb, v_rgb = obj['center']
            
            # Get accurate depth measurement
            depth_mm = get_depth_value(depth_array, u_rgb, v_rgb)
            
            # Store smoothed depth
            if 'smoothed_depth' not in obj:
                obj['smoothed_depth'] = depth_mm
            else:
                obj['smoothed_depth'] = SMOOTHING_FACTOR * depth_mm + (1 - SMOOTHING_FACTOR) * obj['smoothed_depth']
            
            # Convert to world coordinates
            x_mm, y_mm, z_mm = image_coord_to_world(u_rgb, v_rgb, obj['smoothed_depth'])
            
            # Store for display
            obj['world_coords'] = (x_mm, y_mm, z_mm)
            obj['distance'] = z_mm  # Use Z directly for distance

            # Update the display label:
            label = (f"{CLASS_NAMES[obj['class_id']]} ID:{obj_id} "
                     f"Dist: {obj['distance']/1000:.2f}m")  # Convert mm to meters
            
            # Draw elements
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            text_y = y1 - 10 if y1 > 20 else y1 + 20
            cv2.putText(frame, label, (x1, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.circle(frame, (u_rgb, v_rgb), 3, (0,0,255), -1)

        # Display
        cv2.imshow("Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()