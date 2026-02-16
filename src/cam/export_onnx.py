from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12)   # crea yolov8n.onnx
print("Export done")
