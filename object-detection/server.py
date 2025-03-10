from flask import Flask, request, jsonify
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

app = Flask(__name__)

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
ENGINE_PATH = "yolov4-tiny-fp16.trt"

def load_engine(engine_path):
    with open(engine_path, 'rb') as f:
        runtime = trt.Runtime(TRT_LOGGER)
        return runtime.deserialize_cuda_engine(f.read())

def allocate_buffers(engine):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()
    
    for i in range(engine.num_io_tensors):
        tensor_name = engine.get_tensor_name(i)
        shape = engine.get_tensor_shape(tensor_name)
        dtype = engine.get_tensor_dtype(tensor_name)
        size = trt.volume(shape)
        host_mem = cuda.pagelocked_empty(size, trt.nptype(dtype))
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(device_mem))
        
        if engine.get_tensor_mode(tensor_name) == trt.TensorIOMode.INPUT:
            inputs.append({'host': host_mem, 'device': device_mem})
        else:
            outputs.append({'host': host_mem, 'device': device_mem})
    
    return inputs, outputs, bindings, stream

def preprocess(frame):
    img = cv2.resize(frame, (416, 416))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose((2, 0, 1)).astype(np.float32)
    img /= 255.0
    return np.ascontiguousarray(img)

def postprocess(output):
    detections = []
    num_dets = int(output[0][0])
    
    if num_dets > 0:
        boxes = output[0][1:1+num_dets*4].reshape(-1, 4)
        scores = output[0][1+num_dets*4:1+num_dets*5]
        class_ids = output[0][1+num_dets*5:1+num_dets*6].astype(int)

        for i in range(num_dets):
            x1, y1, x2, y2 = boxes[i]
            conf = scores[i]
            class_id = class_ids[i]
            detections.append([int(x1), int(y1), int(x2), int(y2), float(conf), int(class_id)])
    
    return detections

@app.route('/detect', methods=['POST'])
def detect():
    file = request.files['frame']
    nparr = np.frombuffer(file.read(), np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    processed_img = preprocess(frame)

    np.copyto(inputs[0]['host'], processed_img.ravel())
    
    cuda.memcpy_htod_async(inputs[0]['device'], inputs[0]['host'], stream)
    context.execute_async_v3(bindings=bindings, stream_handle=stream.handle)

    cuda.memcpy_dtoh_async(outputs[0]['host'], outputs[0]['device'], stream)
    stream.synchronize()

    detections = postprocess(outputs[0]['host'])

    return jsonify(detections)

if __name__ == "__main__":
    engine = load_engine(ENGINE_PATH)
    context = engine.create_execution_context()
    inputs, outputs, bindings, stream = allocate_buffers(engine)

    app.run(host="0.0.0.0", port=5000)
