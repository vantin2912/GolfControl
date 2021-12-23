import serial
import time


def startSerialCom():
    ser = serial.Serial(
        port= 'COM8',
        baudrate = 115200,
    )
    if (ser):
        print("Serial communication success!!")
        return ser


def Car_ChangeMaxSpeed(ser, newMaxSpeed):
    if newMaxSpeed > 100:
        newMaxSpeed = 100
    elif newMaxSpeed < 0:
        newMaxSpeed = 0

    Send = bytes([0xAA, 0x05, int(newMaxSpeed), 0xEE])
    ser.write(Send)
    print(Send)


def Car_SetSpeedAngle(ser, speed, angle, allowRun):
    
    if angle > 25:
        angle = 25 
    elif angle < -25:
        angle = -25

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    
    if allowRun > 1:
        allowRun = 1

    sendAngle = int((angle + 25) * 4)
    print("Angle ",sendAngle)
    sendSpeed = int(speed + 100)
    Send = bytes([0xAA, 0x04, allowRun, sendSpeed, sendAngle, 0xEE])
    ser.write(Send)
    print([int(send) for send in Send])

ser = startSerialCom()
Car_ChangeMaxSpeed(ser, 100)

while True:
    Car_SetSpeedAngle(ser, 0, 0, 0)
    time.sleep(0.1)