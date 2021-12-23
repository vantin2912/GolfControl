# Import socket module
import socket
import time
import cv2
import numpy as np
import os

from utils.traffic_sign_general import *

from src.parameters import Parameters


traffic_sign_model, _ = load_model(r'dataset/best.pt')
img_dir = r'C:\Users\Asus\Desktop\UIT_CAR_RACING\fail_final'


for file in os.listdir(img_dir):
    traffic_sign_image = cv2.imread(os.path.join(img_dir, file))

    pred = detect(traffic_sign_image, traffic_sign_model, imgsz = (320, 320), conf_thres = 0.4  , iou_thres = 0.45)
    boxes = get_boxes(pred)
    for box in boxes:
        traffic_sign_image = visualize_img(traffic_sign_image, box)

    cv2.imshow("traffic_sign_image", traffic_sign_image)
    cv2.waitKey(0)