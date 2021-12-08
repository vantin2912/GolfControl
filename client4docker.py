# Import socket module
import socket
import time
import cv2
import numpy as np


from src.util import get_steer_angle, calcul_speed, adjust_fits, get_speed
from net import Net
from simple_pid import PID
from utils.traffic_sign_general import *

from src.parameters import Parameters

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the port on which you want to connect
PORT = 54321
# connect to the server on local computer
s.connect(('host.docker.internal', PORT))

pre = time.time()

sendBack_angle = 0
sendBack_Speed = 0

using_pid = True
using_visualization = False

if using_pid:
    kp = 3
    ki = 0
    kd = 0.7

    pid = PID(kp, ki, kd, setpoint= 0)
    pid.output_limits = (-25, 25)


MAX_SPEED = 90
MAX_ANGLE = 25

statistic_path = r'dataset/statistic'

net = Net()
p = Parameters()
net.load_model(r'dataset/3_lane_better.pkl')

def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed


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

km = SimpleKalmanFilter(1, 5, 5)

traffic_sign_model, _ = load_model(r'dataset/best.pt')

save_img_dir = r'images_test'
count = 336

# 'do_not_go_straight', 'do_not_turn_left', 'do_not_turn_right', 'go_straight', 'turn_left', 'turn_right'
AREA_THRESHOLD = np.array([3400, 2500, 3000, 3000, 3000, 3000])

HANDLER_VARIABLE_HIGH_SPEED = np.array([[0, 15, 1],        # 'do_not_go_straight'
                             [10, 15, 1],        # 'do_not_turn_left'
                             [10, -15, 1],       # 'do_not_turn_right'
                             [50, 0, 2],        # 'go_straight'
                             [-150, -24, 1],    # 'turn_left'
                             [-150, 24, 1]])    # 'turn_right'


HANDLER_VARIABLE_LOW_SPEED = np.array([[20, 15, 1],        # 'do_not_go_straight'
                             [20, 15, 1],        # 'do_not_turn_left'
                             [20, -15, 1],       # 'do_not_turn_right'
                             [50, 0.2, 1.5],        # 'go_straight'
                             [0, -20, 1],    # 'turn_left'
                             [0, 20, 1]])    # 'turn_right'



def handler(speed, angle, time_delay):
    Control(angle , speed)
    message = bytes(f"1 {sendBack_angle} {sendBack_Speed}", "utf-8")
    s.sendall(message)
    s.recv(100000)
    time.sleep(time_delay)

try:
    while True:
        # Send data
        start_time = time.time()
        message_getState = bytes("0", "utf-8")
        s.sendall(message_getState)
        state_date = s.recv(100)
        
        try:
            current_speed, current_angle = state_date.decode(
                "utf-8"
                ).split(' ')
        except Exception as er:
            print(er)
            pass

        message = bytes(f"1 {sendBack_angle} {sendBack_Speed}", "utf-8")
        s.sendall(message)
        data = s.recv(100000)
        
        try:
            image = cv2.imdecode(np.frombuffer(data, np.uint8), -1)           
            traffic_sign_image = image[0: 140, 160 : 600]
            pred = detect(traffic_sign_image, traffic_sign_model, imgsz = (320, 320), conf_thres = 0.9  , iou_thres = 0.45)
            boxes = get_boxes(pred)
     
            confidence = 0 
            box = None
            area = 0

            for i, box in enumerate(boxes):
                if box[4] > confidence:
                    box = box
                    confidence = box[4]
       
            if confidence != 0:
                area = calculate_area(box)
                print(area)
                visualize_img(traffic_sign_image, box)
                if AREA_THRESHOLD[int(box[5])] < area:
                    if float(current_speed) > 40:
                        handler_speed, handler_angle, time_delay = HANDLER_VARIABLE_HIGH_SPEED[int(box[5])]
                    else:
                        handler_speed, handler_angle, time_delay = HANDLER_VARIABLE_LOW_SPEED[int(box[5])]
                    handler(handler_speed, handler_angle, time_delay)
                    continue

            image = image[160: 340, :]

            image_resized = cv2.resize(image,(512,256))

            y , x = net.predict(image_resized, warp = False)     
            fits = np.array([np.polyfit(_x,_y, 2) for _x, _y in zip(x,y)])
            fits = adjust_fits(fits)

            mask = net.get_mask_lane(fits)
            image_points_result = net.get_image_points()
            angle = get_steer_angle(fits)
            

            if using_pid:
                out_pid = pid(-(angle))
                predicted_speed = calcul_speed(out_pid, MAX_SPEED, MAX_ANGLE)
                speed = get_speed(out_pid, predicted_speed, float(current_speed), MAX_SPEED, area) 
                out_pid = km.update(out_pid)
                Control(out_pid ,speed)

            else:
                predicted_speed = calcul_speed(angle, MAX_SPEED, MAX_ANGLE)
                speed = get_speed(angle, predicted_speed, float(current_speed), MAX_SPEED, area)
                Control(angle ,speed)

            if using_visualization:
                cv2.imshow("traffic_sign_image", traffic_sign_image)
                cv2.imshow("points", image_points_result)
                cv2.imshow("mask", mask)
                cv2.imshow("IMG", image)
                cv2.waitKey(1)   

            print("FPS: ", 1.0/(time.time() - start_time))



        except Exception as er:
            print(er)
            pass

          
finally:
    print('closing socket')
    s.close()
