# Import socket module
import socket
import time
import cv2
import numpy as np
import os

from src.util import get_steer_angle, calcul_speed, adjust_fits, get_speed
from net import Net
from simple_pid import PID


from src.parameters import Parameters
from src.processing_image import warp_image

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
using_statistic = False
using_visualization = True

if using_pid:
    kp = 3
    ki = 0
    kd = 0.7

    pid = PID(kp, ki, kd, setpoint= 0)
    pid.output_limits = (-25, 25)


MAX_SPEED = 150
MAX_ANGLE = 25

statistic_path = r'dataset/statistic'

net = Net()
p = Parameters()
net.load_model(r'dataset/best.pkl')

def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed

if using_statistic:
    angle_buffer = []  
    pid_angle_buffer = [] 
    speed_buffer = []

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
                speed = get_speed(out_pid, predicted_speed, float(current_speed), MAX_SPEED)

                out_pid = km.update(out_pid)
                Control(out_pid ,speed)

            else:
                predicted_speed = calcul_speed(angle, MAX_SPEED, MAX_ANGLE)
                speed = get_speed(angle, predicted_speed, float(current_speed), MAX_SPEED)
                Control(angle ,speed)

            print("current_speed, predicted_speed, speed: ", float(current_speed), predicted_speed, speed)
            print(angle, speed)

            if using_statistic:
                angle_buffer.append(angle)
                speed_buffer.append(speed)
                if using_pid:
                    pid_angle_buffer.append(out_pid)

            if using_visualization:
                cv2.imshow("points", image_points_result)
                cv2.imshow("mask", mask)
                cv2.imshow("IMG", image)

                cv2.waitKey(1)
        except Exception as er:
            print(er)
            pass
          


finally:

    if using_statistic:
        angle_buffer = np.array(angle_buffer)
        pid_angle_buffer = np.array(pid_angle_buffer)
        speed_buffer = np.array(speed_buffer)

        if using_pid:
            save_name = str(kp) + "_" + str(ki) + "_" + str(kd) + "_" + str(MAX_SPEED) + "_" 
        else:
            save_name = str(MAX_SPEED) + "_" 
        save_name = save_name.replace('.', '-')
        save_name = os.path.join(statistic_path, save_name)

        np.save(save_name + 'angle.npy', angle_buffer)
        np.save(save_name + 'pid_angle_buffer.npy', pid_angle_buffer)
        np.save(save_name + 'speed_buffer.npy', speed_buffer)

    print('closing socket')
    s.close()
