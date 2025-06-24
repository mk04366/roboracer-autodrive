import cv2
import numpy as np


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

img1 = '/home/lyh/ros2_ws/camera_image.jpg'
# img2 = '/home/lyh/ros2_ws/front_camera_image.jpg'

bi(img1)
# bi(img2)

# 等待按键并关闭窗口
cv2.waitKey(0)
cv2.destroyAllWindows()
    