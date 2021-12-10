import os
import cv2
import numpy as np
from net import Net
from src.util import adjust_fits, get_steer_angle, calcul_speed
from src.parameters import Parameters

net = Net()
p = Parameters()
net.load_model("dataset/golf_car.pkl")

MAX_SPEED = 90
MAX_ANGLE = 25
cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()

    if ret == True:
        image_resized = cv2.resize(frame,(512,256))

        y , x = net.predict(image_resized, warp = False)     
        fits = np.array([np.polyfit(_x,_y, 2) for _x, _y in zip(x,y)])
        fits = adjust_fits(fits)

        mask = net.get_mask_lane(fits)
        image_points_result = net.get_image_points()
        angle = get_steer_angle(fits)
        predicted_speed = calcul_speed(angle, MAX_SPEED, MAX_ANGLE)

        print("Angle: ", angle , " Speed: ", predicted_speed)
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.imshow("image_points_result", image_points_result)

        k = cv2.waitKey(1)
        if k == ord('q'):
            continue
    