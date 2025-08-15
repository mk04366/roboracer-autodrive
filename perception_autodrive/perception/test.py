import cv2
import numpy as np
from ultralytics import YOLO
import json

i = 0
def bi(img_path):
    global i
    
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    _, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
    h, w = binary.shape
    cv2.imwrite('binary_image.jpg', binary)

    mask = np.zeros((h + 2, w + 2), dtype=np.uint8)
    flooded = binary.copy()
    seed_point = (0, 0)  
    cv2.floodFill(flooded, mask, seed_point, 128)
    outside_road_mask = np.where(flooded == 128, 0, 255).astype(np.uint8)
    cv2.imwrite('outside_road_mask.jpg', outside_road_mask)
    
    contours, _ = cv2.findContours(outside_road_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_image = cv2.cvtColor(outside_road_mask, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
    
    i += 1
    cv2.namedWindow('{}'.format(i), cv2.WINDOW_NORMAL)
    cv2.imshow('{}'.format(i), contour_image)
    cv2.imwrite('contour_image_{}.jpg'.format(i), contour_image)
    # print(contours)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

def yolov8(img_path):
    # Load YOLOv8 model
    model = YOLO('yolov8n.pt')  # n s m l x
    # model = YOLO('yolov8n-seg.pt')  # Use the segmentation model for better results
    # Read the image
    img = cv2.imread(img_path)
    # Perform inference
    # results = model(img)
    results = model(img, conf=0.1)
    # Process results
    output = []
    for r in results:
        for box in r.boxes:
            cls = int(box.cls.item())
            name = model.names[cls]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            output.append({
                'id': id(box),
                'class': name,
                'bbox': [x1, y1, x2, y2]
            })
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    print(json.dumps(output, indent=2))
    
    cv2.imshow("YOLO Detection", img)
    cv2.waitKey(1)


img1 = '/home/lyh/ros2_ws/images/camera_image.jpg'
# img2 = '/home/lyh/ros2_ws/front_camera_image.jpg'
img3 = '/home/lyh/car.jpeg'

# bi(img1)
# bi(img2)
yolov8(img1)

cv2.waitKey(0)
cv2.destroyAllWindows()
    