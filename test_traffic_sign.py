# Import socket module
import os
import cv2
import numpy as np
from utils.traffic_sign_general import *





img_dir = r'C:\Users\Asus\Desktop\AI\AICAR\New-map\TRAFFIC_SIGN\traffic_sign_half_2\images'

traffic_sign_model, _ = load_model('best.pt')

for file in os.listdir(img_dir):
    img_file = os.path.join(img_dir, file)

    img = cv2.imread(img_file)
    visualized_image = img.copy()
    pred = detect(img, traffic_sign_model, imgsz = (320, 320), conf_thres = 0.7, iou_thres = 0.45)
    
    print(img.shape)

    
   


    boxes = get_boxes(pred)

    for box in boxes:
        visualize_img(visualized_image, box)
      
      


    cv2.imshow("Origin", visualized_image)
    cv2.waitKey(0)
