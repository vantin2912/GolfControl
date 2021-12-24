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
net.load_model(r'C:\Users\Asus\Downloads\lane.pkl')

def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed

def avoid_left():
    #print("avoid_left")
    for i in range(10, 0, -1):
        #print("angle: ", i)
        send(i, 20)
        time.sleep(0.06)

    for i in range(1, -16, -1):
        #print("angle: ", i)
        send(i, 20)
        time.sleep(0.03)

    for i in range(-13, 10, 1):
        send(i, 20)
        time.sleep(0.03)

def avoid_right():
    #print("avoid_right")
    for i in range(-10, 0, 1):
        send(i, 20)
        time.sleep(0.06)

    for i in range(0, 16, 1):
        send(i, 20)
        time.sleep(0.03)

    for i in range(13, -10, -2):
        send(i, 20)
        time.sleep(0.03)

        

def avoid_2right():
    #print("avoid_2right")
    for i in range(-5, 0, 1):
        send(i, 20)
        time.sleep(0.17)

    for i in range(-0, 10, 1):
        send(i, 20)
        time.sleep(0.1)

    for i in range(10, -4, -1):
        send(i, 20)
        time.sleep(0.03)


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

traffic_sign_model, _ = load_model(r'C:\Users\Asus\Downloads\best.pt')

save_img_dir = r'images_test'
count = 0

global instruction_count 
instruction_count = 0

#Speed_threshold = []

INSTRUCTION = [["go_straight",3000, 50, 0, 0.6], ["turn_right", 2000, 20, 25, 1.55], \
                ["turn_right", 2000, 20, 25, 1.4], ["turn_left", 2000, 20, -25, 1.467],\
                ["turn_left_switch", 2100, 0, -14, 1.5], ["go_straight_left", 2000, 50, -1, 0.8],\
                ["go_straight", 2000, 50, 0, 0.8], ["go_straight", 2000, 50, 0, 0.85], \
                ["turn_left", 2200, 20, -25, 1.5], ["turn_right", 2000, 0, 25, 1.5],\
                ["turn_right", 2000, 0, 25, 1.5], ["turn_right", 2000, 0, 25, 1.5],
                ["left_object", 3000], ["right_object", 3000],  
                ["left_object", 3000], ["right_2_object", 3000]]
                
                #  ["left_object", 0], \
                # ["right_object", 0], ["left_object", 0], ["right_object", 0]]


SPEED_THRESHOLD = [70, 80, 80, 70, 80, 80, 80, 70, 60, 80, 60, 60, 60, 60, 60, 90]

INSTRUCTION_OBJECT = [avoid_left, avoid_right, avoid_left, avoid_2right]

def send(angle, speed):
    message = bytes(f"1 {angle} {speed}", "utf-8")
    s.sendall(message)  
    s.recv(100000)

def turn_left_switch():
    send(0, 0)
    time.sleep(0.46)
    angle = -25
    time_delay = 1.6/np.abs(angle)
    for i in range(np.abs(angle)):
        send(angle, 0)
        time.sleep(time_delay)
        angle += 1



def handler(speed, angle, time_delay):

    if angle == 0:
        angle = 1
    time_delay = time_delay/np.abs(angle)

    direction = -1 if angle < 0 else 1

    for i in range(np.abs(angle)):
        send(angle, speed)
        time.sleep(time_delay)
        angle -= direction * 1

count = 0
img_dir = r'C:\Users\Asus\Desktop\UIT_CAR_RACING\fail_img'


try:
    while True:
        # Send data
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
            if instruction_count < 12:   
                traffic_sign_image = image[0: 140, 160 : 600]
            else:
                traffic_sign_image = image[100: 240, 160 : 600]
            
            pred = detect(traffic_sign_image, traffic_sign_model, imgsz = (320, 320), conf_thres = 0.8  , iou_thres = 0.45)
            boxes = get_boxes(pred)
     
            confidence = 0 
            box = None
            area = 0
            
            # if boxes != []:
            #     img_path = os.path.join(img_dir, str(count) + "_fail.png")
            #     cv2.imwrite(img_path, traffic_sign_image)
            #     count += 1

            for i, box in enumerate(boxes):
                if box[4] > confidence:
                    box = box
                    confidence = box[4]
       
            if confidence != 0 and instruction_count < 12:
                area = calculate_area(box)
                print(area)
                visualize_img(traffic_sign_image, box)
                if INSTRUCTION[instruction_count][1] < area:
                    if instruction_count == 4:
                        turn_left_switch()
                        print(INSTRUCTION[instruction_count][0])
                        MAX_SPEED = SPEED_THRESHOLD[instruction_count]
                        print("MAX_SPEED: ", MAX_SPEED)
                        instruction_count += 1
                        print("instruction_count: ", instruction_count)
                        continue

                    else:
                        handler_speed, handler_angle, time_delay = INSTRUCTION[instruction_count][2:]
                        print(INSTRUCTION[instruction_count][0])
                        MAX_SPEED = SPEED_THRESHOLD[instruction_count]
                        print("MAX_SPEED: ", MAX_SPEED)
                        instruction_count += 1
                        print("instruction_count: ", instruction_count)
                        handler(handler_speed, handler_angle, time_delay)
                        continue

            elif confidence != 0 and instruction_count >= 12:   
                area = calculate_area(box)
                print(area)
                visualize_img(traffic_sign_image, box)
                if INSTRUCTION[instruction_count][1] < area:
                    print("Object")
                    avoid_function = INSTRUCTION_OBJECT[instruction_count - 12]
                    avoid_function()
                    MAX_SPEED = SPEED_THRESHOLD[instruction_count]
                    print("MAX_SPEED: ", MAX_SPEED)
                    instruction_count += 1


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

        except Exception as er:
            print("Sup bitch")
            pass
    
finally:
    print('closing socket')
    s.close()
