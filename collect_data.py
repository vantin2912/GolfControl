import cv2
import numpy as np
import time
import keyboard 
import threading
import os

from utils.connect import Connection
from utils.controller import Controller

def Thread_collect_data():
    global count

    while cap.isOpened() and not DONE:
        ret, frame = cap.read()
        if ret == True:
            path = os.path.join(r'C:\Users\Asus\Desktop\GOLF_CAR\Image_nga_ba', str(count) + ".png")
            cv2.imwrite(path, frame)
            cv2.imshow("frame", frame)
            count += 1
            cv2.waitKey(10)




def Thread_read_key_board():
    global DONE
    while cap.isOpened() and not DONE:
        print("Car controller: ", car_controller.get_info())
        if keyboard.is_pressed('space') :
            car_controller.increase_mode()

        if keyboard.is_pressed('ctrl') :
            car_controller.decrease_mode()

        elif keyboard.is_pressed('w'):
            car_controller.increase_speed(2)

        elif keyboard.is_pressed('s') :   
            car_controller.decrease_speed(2)
        
        elif keyboard.is_pressed('d') :
            car_controller.turn_right(1)

        elif keyboard.is_pressed('a') :
            car_controller.turn_left(1)

        if keyboard.is_pressed('up'):
            car_controller.go_straight()  

        elif keyboard.is_pressed('down'):
            car_controller.go_reverse()  

        elif keyboard.is_pressed('enter'):
            car_controller.brake()  

        else:
            car_controller.nop()

        if keyboard.is_pressed('esc'):
            DONE = True

        time.sleep(0.1)

def init_thread():
    main_ThreadData = threading.Thread(target= Thread_collect_data)
    main_ThreadKeyBoard = threading.Thread(target= Thread_read_key_board)

    main_ThreadData.start()
    main_ThreadKeyBoard.start()


if __name__ == "__main__":
    MAX_SPEED = 100
    MAX_ANGLE = 25
    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture(r'C:\Users\Asus\Desktop\DoAnCV\video\test_video_2.mp4')

    global DONE 
    DONE = False
    global count
    count = 0

    connector = Connection("COM8", 115200, False)
    connector.Car_ChangeMaxSpeed(100)

    car_controller = Controller(connector)
    init_thread()

        

    