import os
import cv2
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))

stream_url = "http://10.142.160.78/stream-lo"   # oppure /stream-lo per più FPS

names_path = os.path.join(BASE, "coco.names")
model_path = os.path.join(BASE, "yolov8n.onnx")

with open(names_path, "rt") as f:
    classNames = [c.strip() for c in f.readlines() if c.strip()]

net = cv2.dnn.readNetFromONNX(model_path)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

cap = cv2.VideoCapture(stream_url)
if not cap.isOpened():
    raise RuntimeError("Non riesco ad aprire lo stream MJPEG. Prova /stream-lo oppure VLC per testare.")

winName = "ESP32 MJPEG + YOLOv8"
cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)

CONF_TH = 0.40
NMS_TH  = 0.45
INP_SIZE = 640

while True:
    ok, frame = cap.read()
    if not ok or frame is None:
        continue

    # Se la tua camera è ruotata:
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    h, w = frame.shape[:2]

    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (INP_SIZE, INP_SIZE), swapRB=True, crop=False)
    net.setInput(blob)
    preds = net.forward()

    out = preds
    if out.ndim == 3:
        out = out[0]

    # YOLOv8 ONNX spesso: (84, 8400) -> trasponi a (8400, 84)
    if out.shape[0] < out.shape[1]:
        out = out.T

    boxes, confs, class_ids = [], [], []

    x_factor = w / INP_SIZE
    y_factor = h / INP_SIZE

    for det in out:
        scores = det[4:]
        cls = int(np.argmax(scores))
        conf = float(scores[cls])

        if conf < CONF_TH:
            continue

        cx, cy, bw, bh = float(det[0]), float(det[1]), float(det[2]), float(det[3])

        x = int((cx - bw/2) * x_factor)
        y = int((cy - bh/2) * y_factor)
        bw = int(bw * x_factor)
        bh = int(bh * y_factor)

        boxes.append([x, y, bw, bh])
        confs.append(conf)
        class_ids.append(cls)

    idxs = cv2.dnn.NMSBoxes(boxes, confs, CONF_TH, NMS_TH)

    if len(idxs) > 0:
        for i in idxs.flatten():
            x, y, bw, bh = boxes[i]
            cls = class_ids[i]
            conf = confs[i]

            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = f"{classNames[cls]} {conf:.2f}"
            cv2.putText(frame, label, (x, max(0, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow(winName, frame)
    if (cv2.waitKey(1) & 0xFF) == 27:
        break

cap.release()
cv2.destroyAllWindows()
