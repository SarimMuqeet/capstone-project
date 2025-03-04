import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import requests
import struct
import matplotlib.pyplot as plt
import math

# MaixSense A075V camera settings
HOST = '192.168.233.1'
PORT = 80
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# YOLOv4-tiny TensorRT settings
ENGINE_PATH = "./tensorrt_demos/yolo/yolov4-tiny-fp16.trt"
INPUT_SHAPE = (1, 3, 416, 416)
NUM_CLASSES = 80  # Adjust based on your model
SELECTED_CLASSES = [0, 32, 39, 40, 41, 56, 57, 58, 59, 60, 62, 65, 67, 73, 74, 75, 77]

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
        # Correct reshaping with transpose
        output = output.reshape(3, 85, grid_size, grid_size).transpose(0, 2, 3, 1)

        for anchor_idx in range(3):
            anchor_w, anchor_h = anchors[stride][anchor_idx]
            for i in range(grid_size):
                for j in range(grid_size):
                    tx, ty, tw, th = output[anchor_idx, i, j, :4]
                    objectness_raw = output[anchor_idx, i, j, 4]
                    class_raw = output[anchor_idx, i, j, 5:]  # 80 classes

                    # --- 1) Apply sigmoid to center offsets, objectness, class scores ---
                    bx = (sigmoid(tx) + j) * stride
                    by = (sigmoid(ty) + i) * stride
                    bw = anchor_w * math.exp(tw)
                    bh = anchor_h * math.exp(th)
                    objectness = sigmoid(objectness_raw)
                    class_scores = sigmoid(class_raw)

                    # --- 2) Overall confidence = objectness × max_class_score ---
                    class_id = np.argmax(class_scores)
                    class_conf = class_scores[class_id]
                    score = objectness * class_conf

                    # Threshold check
                    if score > conf_thresh:
                        # Convert to top-left, bottom-right
                        x1 = bx - bw / 2.0
                        y1 = by - bh / 2.0
                        x2 = bx + bw / 2.0
                        y2 = by + bh / 2.0

                        # Scale up to original image size
                        scale_x = float(orig_w) / input_dim
                        scale_y = float(orig_h) / input_dim
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)

                        # Optional: clamp to image boundaries
                        x1 = max(0, x1)
                        y1 = max(0, y1)
                        x2 = min(orig_w - 1, x2)
                        y2 = min(orig_h - 1, y2)

                        detections.append([x1, y1, x2, y2, class_id, score])

    # --- 3) Apply Non-Maximum Suppression (NMS) ---
    if len(detections) == 0:
        return []

    boxes = np.array([det[:4] for det in detections])
    confidences = np.array([det[5] for det in detections])
    indices = cv2.dnn.NMSBoxes(
        boxes.tolist(), confidences.tolist(), conf_thresh, nms_thresh
    )
    if len(indices) > 0:
        indices = indices.flatten()
        detections = [detections[i] for i in indices]
    else:
        detections = []

    return detections

def main():
    # Configure camera
    if not post_encode_config(frame_config_encode(1, 1, 255, 0, 2, 7, 1, 0, 0)):
        print("Failed to configure camera")
        return

    # Set up TensorRT
    engine = get_engine(ENGINE_PATH)
    context = engine.create_execution_context()

    # Get tensor names and modes
    input_name = "000_net"  # From trtexec output
    output_names = ["030_convolutional", "037_convolutional"]  # From trtexec output

    # Verify tensors exist in engine
    for name in [input_name] + output_names:
        if not engine.get_tensor_shape(name):
            raise ValueError(f"Tensor {name} not found in engine")

    # Allocate memory for input
    input_shape = engine.get_tensor_shape(input_name)
    d_input = cuda.mem_alloc(trt.volume(input_shape) * np.dtype(np.float32).itemsize)

    # Prepare outputs
    h_outputs = {}
    d_outputs = {}
    for name in output_names:
        shape = context.get_tensor_shape(name)
        h_outputs[name] = cuda.pagelocked_empty(trt.volume(shape), dtype=np.float32)
        d_outputs[name] = cuda.mem_alloc(h_outputs[name].nbytes)

    stream = cuda.Stream()

    while True:
        frame_data = get_frame_from_http()
        if frame_data is None:
            print("Failed to get frame from camera")
            continue

        # Extract RGB image from frame data
        config = struct.unpack("<BBBBBBBBi", frame_data[16:16+12])
        frame_payload = frame_data[16+12:]
        deep_data_size, rgb_data_size = struct.unpack("<ii", frame_payload[:8])
        rgb_data = frame_payload[8 + deep_data_size:]

        # Decode RGB image
        frame = cv2.imdecode(np.frombuffer(rgb_data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            print("Failed to decode image from buffer")
            continue

         # Preprocess the image
        input_image = preprocess_image(frame)

        # Copy input data to device
        cuda.memcpy_htod_async(d_input, input_image.ravel(), stream)

        # Set tensor addresses
        context.set_tensor_address(input_name, int(d_input))
        for name in output_names:
            context.set_tensor_address(name, int(d_outputs[name]))

        # Execute inference
        context.execute_async_v3(stream.handle)

        # Copy outputs from device
        for name in output_names:
            cuda.memcpy_dtoh_async(h_outputs[name], d_outputs[name], stream)

        stream.synchronize()

        # Postprocess both outputs
        detections = postprocess(
            outputs=[h_outputs["030_convolutional"], h_outputs["037_convolutional"]], 
            image_shape=frame.shape
        )

        # Draw bounding boxes on the frame
        if detections is not None:
            for det in detections:
                x1, y1, x2, y2, class_id, conf = det

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"Class: {class_id}, Conf: {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the result
        cv2.imshow("YOLOv4-tiny Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()