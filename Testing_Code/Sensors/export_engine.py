import cv2

# Inject dummy functions into OpenCV to bypass the Ultralytics headless bug
if not hasattr(cv2, 'imshow'):
    cv2.imshow = lambda *a, **k: None
    cv2.waitKey = lambda *a, **k: None
    cv2.destroyAllWindows = lambda *a, **k: None

# NOW it is safe to import YOLO
from ultralytics import YOLO

print("Compiling TensorRT Engine...")
model = YOLO("yolo26n.pt")
model.export(format="engine", half=True, workspace=1024)
print("Compilation Complete.")
