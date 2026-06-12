import cv2 as cv
from ultralytics import YOLO
import time

model = YOLO("yolo26n.pt") 
target_classes = 32

cap = cv.VideoCapture(0, cv.CAP_V4L2)

cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))

cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)

model.predict("https://ultralytics.com/images/bus.jpg", device="cuda", half=True, verbose=False,imgsz=320,vid_stride=2)

while True:
    t0 = time.time()
    
    ret, frame = cap.read()
    if not ret:
        print("no frame returned")
        break

    results = model.predict(frame, verbose=False, device="cuda", half=True, classes=[target_classes], conf=0.01, imgsz=640, stream=True)
    
    for r in results:
        boxes = r.boxes.xyxy.cpu().numpy()
        
        fps = 1 / (time.time() - t0)
        print(f"FPS: {fps:.1f} | Detections: {len(boxes)}")
        
        if len(boxes) > 0:
            print(f"Coordinates: {boxes}")

cap.release()