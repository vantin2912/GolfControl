import serial
import numpy as np

class Connection:
    def __init__(self, port, baudrate, test):
        self.test = test
        self.serial = self.connect(port, baudrate)


    def connect(self, port, baudrate):
        if not self.test:
            ser = serial.Serial(
                port= port,
                baudrate = baudrate,
            )
            if (ser):
                print("Serial communication success!!")
                return ser

        else:
            return 0

    def Car_ChangeMaxSpeed(self, Max_speed):
        Max_speed = np.clip(Max_speed, 0 , 100)
        Send = bytes([0xAA, 0x05, int(Max_speed), 0xEE])
        print(Send)

        if not self.test:
            self.serial.write(Send)

    def Car_SetSpeedAngle(self, speed, angle, allowRun):
        angle = np.clip(angle, -24, 24)
        speed = np.clip(speed, -100, 100)

        allowRun = np.clip(allowRun, 0, 1)

        sendAngle = int((angle + 25) * 4)
        sendSpeed = int(speed + 100)
        Send = bytes([0xAA, 0x04, allowRun, sendSpeed, sendAngle, 0xEE])    

        if not self.test:
            self.serial.write(Send)
            



# def startSerialCom(test):
#     if test:
#         return 0
#     else:
#         ser = serial.Serial(
#             port= 'COM8',
#             baudrate = 115200,
#         )
#         if (ser):
#             print("Serial communication success!!")
#             return ser


# def Car_ChangeMaxSpeed(ser, newMaxSpeed, test):
#     if newMaxSpeed > 100:
#         newMaxSpeed = 100
#     elif newMaxSpeed < 0:
#         newMaxSpeed = 0

#     Send = bytes([0xAA, 0x05, int(newMaxSpeed), 0xEE])
#     print(Send)

#     if not test:
#         ser.write(Send)



# def Car_SetSpeedAngle(ser, speed, angle, allowRun, test):
#     if angle > 25:
#         angle = 25
#     elif angle < -25:
#         angle = -25

#     if speed > 100:
#         speed = 100
#     elif speed < -100:
#         speed = -100
    
#     if allowRun > 1:
#         allowRun = 1

#     sendAngle = int((angle + 25) * 4)
#     sendSpeed = int(speed + 100)
#     Send = bytes([0xAA, 0x04, allowRun, sendSpeed, sendAngle, 0xEE])    
#     if not test:
#         ser.write(Send)
        