import cv2
import numpy as np
import os

from net import Net
from src.util import adjust_fits, get_steer_angle, calcul_speed
from src.parameters import Parameters



net = Net()
p = Parameters()
net.load_model(r"C:\Users\Asus\Downloads\best.pkl")

dir = r'C:\Users\Asus\Desktop\GOLF_CAR\Image_3'


for file in os.listdir(dir):
    frame = cv2.imread(os.path.join(dir, file))

    image_resized = cv2.resize(frame,(512,256))

    y , x = net.predict(image_resized, warp = False)     
    fits = np.array([np.polyfit(_x,_y, 2) for _x, _y in zip(x,y)])
    fits = adjust_fits(fits)

    mask = net.get_mask_lane(fits)
    image_points_result = net.get_image_points()

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("image_points_result", image_points_result)
    cv2.waitKey(0)