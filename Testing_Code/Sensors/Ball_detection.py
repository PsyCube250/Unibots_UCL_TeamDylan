import cv2 as cv
from ultralytics import YOLO
import time

model = YOLO("yolo26n.pt") 
target_classes = 32

cap = cv.VideoCapture(0, cv.CAP_V4L2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("no frame returned")
        break
        
    t0 = time.time()
    
    # force to cuda and filter classes before nms
    results = model(frame, verbose=False, device="cuda", classes=[target_classes])
    
    boxes = results[0].boxes.xyxy.cpu().numpy()
    fps = 1 / (time.time() - t0)
    
    print(f"FPS: {fps:.1f} | Detections: {len(boxes)}")
    if len(boxes) > 0:
        print(f"Coordinates: {boxes}")

cap.release()