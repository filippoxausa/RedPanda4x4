import os
import time
import cv2
import numpy as np
import requests
import threading

BASE = os.path.dirname(os.path.abspath(__file__))

# =========================
#      STREAM MJPEG
# =========================
STREAM_URL = "http://192.168.1.13/stream"
ROTATE_90_CW = True

def mjpeg_reader(url, shared, stop_event, timeout=(3, None), chunk_size=4096):
    while not stop_event.is_set():
        try:
            r = requests.get(url, stream=True, timeout=timeout)
            r.raise_for_status()
            buff = b""
            for chunk in r.iter_content(chunk_size=chunk_size):
                if stop_event.is_set():
                    break
                if not chunk:
                    continue
                buff += chunk
                a = buff.find(b"\xff\xd8")
                b = buff.find(b"\xff\xd9")
                if a != -1 and b != -1 and b > a:
                    jpg = buff[a:b+2]
                    buff = buff[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    if ROTATE_90_CW:
                        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    with shared["lock"]:
                        shared["frame"] = frame
                        shared["ts"] = time.time()
        except Exception:
            time.sleep(0.3)

# =========================
#          YOLO
# =========================
names_path = os.path.join(BASE, "coco.names")
model_path = os.path.join(BASE, "yolov8n.onnx")

with open(names_path, "rt") as f:
    classNames = [c.strip() for c in f.readlines() if c.strip()]

net = cv2.dnn.readNetFromONNX(model_path)

# Se hai OpenCV con CUDA (di solito NO nelle build pip), puoi provare:
# try:
#     net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
#     net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
# except Exception:
#     net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
#     net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

CONF_TH = 0.40
NMS_TH  = 0.45

# ---- IMPORTANT: deve matchare l'export del tuo ONNX ----
INP_SIZE = 640

# ---- SPEED KNOBS ----
YOLO_EVERY = 3     # inferenza ogni N frame (2-5)
MAX_DETS   = 80    # limita candidate prima della NMS (aiuta FPS)
DRAW_EVERY = 1     # se vuoi alleggerire il drawing: metti 2 (disegna ogni 2 frame)

def decode_detections(out, w, h):
    if out.ndim == 3:
        out = out[0]
    if out.shape[0] < out.shape[1]:
        out = out.T

    C = out.shape[1]
    has_obj = (C >= 85)

    x_factor = w / INP_SIZE
    y_factor = h / INP_SIZE

    rough = []
    for det in out:
        if has_obj:
            obj = float(det[4])
            scores = det[5:]
        else:
            obj = 1.0
            scores = det[4:]

        cls = int(np.argmax(scores))
        conf = obj * float(scores[cls])
        if conf >= CONF_TH:
            rough.append((conf, det, cls))

    if not rough:
        return [], [], []

    rough.sort(key=lambda x: x[0], reverse=True)
    rough = rough[:MAX_DETS]

    boxes, confs, class_ids = [], [], []
    for conf, det, cls in rough:
        a, b, c, d = map(float, det[:4])

        # xyxy se plausibile, altrimenti xywh
        xyxy_ok = (c > a) and (d > b) and (c <= INP_SIZE * 1.25) and (d <= INP_SIZE * 1.25)

        if xyxy_ok:
            x1 = int(a * x_factor); y1 = int(b * y_factor)
            x2 = int(c * x_factor); y2 = int(d * y_factor)
            x = x1; y = y1; bw = x2 - x1; bh = y2 - y1
        else:
            cx, cy, bw_in, bh_in = a, b, c, d
            x = int((cx - bw_in/2) * x_factor)
            y = int((cy - bh_in/2) * y_factor)
            bw = int(bw_in * x_factor)
            bh = int(bh_in * y_factor)

        if bw <= 1 or bh <= 1:
            continue

        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        bw = max(1, min(bw, w - x))
        bh = max(1, min(bh, h - y))

        if bw > 0.98 * w and bh > 0.98 * h:
            continue

        boxes.append([x, y, bw, bh])
        confs.append(float(conf))
        class_ids.append(int(cls))

    return boxes, confs, class_ids

def infer_yolo(frame):
    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (INP_SIZE, INP_SIZE), swapRB=True, crop=False)
    net.setInput(blob)
    out = net.forward()
    boxes, confs, class_ids = decode_detections(out, w, h)

    idxs = cv2.dnn.NMSBoxes(boxes, confs, CONF_TH, NMS_TH)
    dets = []
    if len(idxs) > 0:
        for i in idxs.flatten():
            x, y, bw, bh = boxes[i]
            dets.append((x, y, bw, bh, class_ids[i], confs[i]))
    return dets

def main():
    shared = {"frame": None, "ts": 0.0, "lock": threading.Lock()}
    stop_event = threading.Event()
    threading.Thread(target=mjpeg_reader, args=(STREAM_URL, shared, stop_event), daemon=True).start()

    win = "ESP32 MJPEG + YOLO (fast, fixed input)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    last_dets = []
    yolo_counter = 0
    draw_counter = 0

    # FPS
    last = time.time(); frames = 0; video_fps = 0.0
    y_last = time.time(); y_frames = 0; yolo_fps = 0.0

    while True:
        with shared["lock"]:
            frame = None if shared["frame"] is None else shared["frame"].copy()
        if frame is None:
            time.sleep(0.01)
            continue

        frames += 1
        now = time.time()
        if now - last >= 1.0:
            video_fps = frames / (now - last)
            frames = 0
            last = now

        # YOLO ogni N frame
        yolo_counter = (yolo_counter + 1) % YOLO_EVERY
        if yolo_counter == 0:
            last_dets = infer_yolo(frame)
            y_frames += 1
            y_now = time.time()
            if y_now - y_last >= 1.0:
                yolo_fps = y_frames / (y_now - y_last)
                y_frames = 0
                y_last = y_now

        # Disegno (se vuoi alleggerire: DRAW_EVERY=2)
        draw_counter = (draw_counter + 1) % DRAW_EVERY
        if draw_counter == 0:
            for x, y, bw, bh, cls, conf in last_dets:
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                name = classNames[cls] if 0 <= cls < len(classNames) else f"id{cls}"
                cv2.putText(frame, f"{name} {conf:.2f}", (x, max(0, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.putText(frame, f"VIDEO FPS: {video_fps:.1f} | YOLO FPS: {yolo_fps:.1f} | 640 every:{YOLO_EVERY}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        cv2.imshow(win, frame)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break

    stop_event.set()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
