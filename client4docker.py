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
using_visualization = False

if using_pid:
    kp = 3
    ki = 0
    kd = 0.7

    pid = PID(kp, ki, kd, setpoint= 0)
    pid.output_limits = (-25, 25)


MAX_SPEED = 120
MAX_ANGLE = 25

statistic_path = r'dataset/statistic'

net = Net()
p = Parameters()
net.load_model(r'dataset/speed_120.pkl')

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
# count = 0
print("SIMPLE WITH OUT LE")

print("OOOOOOOOOO000000KKKKK000000KXXXXXXXXNXXNNNXXXXXXXX\n\
xxxxkkkkkkkOOOOkxddddooooodddxxkO0XXNXXNNNNNNNNNNN\n\
xxxkkkkkkkkkxdolcccccccllllcc::clodk0KXNNNNNNNNNNX\n\
xkkkkkkkkxdlcccllooloooddddollcc::::ldkKXNNNNNNNNX\n\
kkkkkkkxollllloodddxxkkkxxkkkkkxdoc:::cokKXXNNNNNX\n\
kkkkkkollcloddxkkOOO00OkOOOOOO00Okxdo:;cld0XXXXXNX\n\
kkkkkocllodddk0OOOOOxc''coxO00KK0OOOkxlccloOXXXXXX\n\
kkOkoclloxO0OOO000d;......':dOKK00K0OOkdlcloOXXXXX\n\
OOkdclloxO0OOO00x:...''.''...;x0O0K00OOkxocld0XXXX\n\
OOklclok00O0000O:.'codxxdol:,.,x000000OOkdllokXXXX\n\
OOxlcox0K0O000KO:':lddxOxdoc;',dKKKKOOOOOkoloxKXXX\n\
00xllokKK0O0000k:,:dxxxxoodoc,;xKKK0OOO00kdlld0XXX\n\
00koodOKX0OOOO0Oo::dOOkdooxkl;cO00K0OO0K0OdolxKXXX\n\
KKOddxOKKK00OOOOkl:lxxdlclodc;dK0OOOOkO0OkdookKXXX\n\
KK0kxxk0KXXXK000Odccldxxdool;:d0OOOO0OOOkxood0XXXX\n\
KKK0kxkOKXXXK00KOdlllooollc::cd0K00K0OOkxdodkKXXXX\n\
KKXK0kkO0KXXK0Okxoldxkkxxxolc::looxkOOkxdddkKXXXKX\n\
XXXXXKOOOkxdoooooodxxkkkkkxdoolc::::clddddOKXKXXKK\n\
XXXXXXKOdodxkxxkkkkkkkkkOOOkkxdddolc:,;lx0XKXXXKKK\n\
XXXXXXXkldOkdokKKKKK0OOO00000Okxxxdoc:::d0XXKKKKKK\n\
XXXXXXKdcdkxoox0K0KK00OOO00KK0xoxkdollolckXKXKKKKK\n\
XNNNXXKoldkOkdoxkxkO0OOOOO0K0Oddkkdllodocd0KKKKKKK\n\
NNNXNNOllxdxOkdkOxdxkOOOO000Okxxkxdlldxdl:xKKK0000\n\
XXNNNXd:lolldkxxkoldxkkOO0Okkkxxxdoodxkxl:d0KK0000\n\
NXNNNNxcoxolloolllldxkOOOOOkkkxxxdoodxkxl;:kK00000\n\
NNNNXXxcodolllccccldxkkOOOOOkkkkxooodxkxo;;x000000\n\
NNNNNXd:llloolc;,:codxkOOOOOOOOkdloddxkko:,o000OOO\n\
XXXXNKo;cclolc;'';cldxkOOOOOOOkdoloooxkOxc,lO0OOOO\n\
XXXXX0l;:cllc;..,:codxxkkkkOOxdooooodkOOko;ckOOOOO\n\
XXXXX0c,::;:,...,cldxkkkOOOkxdddddxxxdxxdl;;dOOOkk\n\
XXXXXKd;''''...,:loddddxkkkxxkxxdoxkkkxxdo;,dOOOkk\n\
XXKKK0OxooddooloxxxxxxxxdddxkkkkkkO000KKOkxdxkkkkk\n\
KKOOkkkkkOOkOOOO000OOOOOkxkOkk000000KKKKK000kxxxxk\n\
0O0KKKK000OkxdddxkkkOO000OOOO0000KKKKXXXXXKKOOOold\n\
OxxOkkOkxxxxdoooodddddxxO0OkkOkOO0000OOOOOOkOkoclo\n\
xolloloddoooooooooooddxkO0kxkkxxxxxxxxxxxdddllllcc\n\
koccccclllllllllodddddxkkkkkOOkkkxxxdddddoooool:cd\n\
OkdddolloolllcodddddodoodddkkkxkOOkkOOkxkxddlc:cox\n\
OkkOkxxxdxddooodoodolllllllodxdodxxkO0OkOOkkddooxx\n\
xxkkkddxddoooxxxxkkxdxxxddddxkkkkkkkkkxxxxkxxxkxxk\n\
")
print("Please, stay on the road")
from playsound import playsound

def play_music():
    playsound(r'music.mp3')

from threading import Thread

Thread(target= play_music).start()

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

            # print("current_speed, predicted_speed, speed: ", float(current_speed), predicted_speed, speed)
            # print(angle, speed)

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
