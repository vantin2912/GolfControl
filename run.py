import cv2
import numpy as np
import time
import keyboard 
import threading
import os

from simple_pid import PID
from net import Net
from src.util import adjust_fits, get_steer_angle, calcul_speed
from src.parameters import Parameters
from utils.connect import Connection
from utils.controller import Controller



class SimpleKalmanFilter:
  def __init__(self, mea_e, est_e, q):
    self.mea_e = mea_e
    self.est_e = est_e
    self.q = q
    self.last_est = 0
  
  def update(self, mea):
    kalman_gain = self.est_e/(self.est_e + self.mea_e)
    current_est = self.last_est + kalman_gain*(mea - self.last_est)

    self.est_e = (1.0 - kalman_gain) * self.est_e + np.fabs(self.last_est - current_est) * self.q
    self.last_est = current_est
    return current_est

def Thread_AI():
    global MODE
    global count
    while cap.isOpened() and not DONE:  
        ret, frame = cap.read()
        if ret == True:
            path = os.path.join(r'C:\Users\Asus\Desktop\GOLF_CAR\Image_uit_2', str(count) + ".png")
            #cv2.imwrite(path, frame)
            count += 1


            image_resized = cv2.resize(frame,(512,256))

            y , x = net.predict(image_resized, warp = False)     
            fits = np.array([np.polyfit(_x,_y, 2) for _x, _y in zip(x,y)])
            fits = adjust_fits(fits)

            mask = net.get_mask_lane(fits)
            image_points_result = net.get_image_points()
            angle = int(get_steer_angle(fits))
            angle = pid(-(angle))
            angle = km.update(angle)

            speed = int(calcul_speed(angle, MAX_SPEED, MAX_ANGLE)) 
            speed_ratio = speed/MAX_SPEED
            predicted_speed = speed_ratio * 5 + 60

            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            cv2.imshow("image_points_result", image_points_result)
    
            if MODE == 0:
                print("predicted_speed, angle: ", predicted_speed, angle)
                connector.Car_SetSpeedAngle(predicted_speed, angle , 0)

            cv2.waitKey(1) 


def Thread_read_key_board():
    global MODE
    global DONE
    while cap.isOpened() and not DONE:
        if MODE == 1:
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

        if keyboard.is_pressed('right shift') and MODE == 0:
            MODE = 1
            car_controller.brake()  

        if keyboard.is_pressed('tab'):
            print("HRERERERE")
            if MODE == 1:
                MODE = 0     
            else:          
                MODE = 1
                car_controller.brake()  

        if keyboard.is_pressed('esc'):
            DONE = True

        time.sleep(0.08)

def init_thread():
    main_ThreadAI = threading.Thread(target= Thread_AI)
    main_ThreadKeyBoard = threading.Thread(target= Thread_read_key_board)

    main_ThreadAI.start()
    main_ThreadKeyBoard.start()


if __name__ == "__main__":
    net = Net()
    p = Parameters()
    net.load_model(r"C:\Users\Asus\Downloads\best.pkl")
    km = SimpleKalmanFilter(1, 5, 5)
    kp = 0.5
    ki = 0
    kd = 0.1

    pid = PID(kp, ki, kd, setpoint= 0)
    pid.output_limits = (-25, 25)

    MAX_SPEED = 100
    MAX_ANGLE = 25
    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture(r'C:\Users\Asus\Desktop\DoAnCV\video\test_video_2.mp4')
    global count
    count = 0
    global DONE 
    DONE = False

    global MODE
    MODE = 0
    print("We are currently using AI mode")
    print("Press tab to switch to hand mode, else to bypass")

    if keyboard.read_key('tab'):
        if MODE == 1:
            MODE = 0        
        else:
            MODE = 1

    connector = Connection("COM8", 115200, False)
    connector.Car_ChangeMaxSpeed(100)

    car_controller = Controller(connector)
    init_thread()

        

    