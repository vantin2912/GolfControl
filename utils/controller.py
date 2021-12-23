import numpy as np
import time


class Controller(object):
    def __init__(self, serial):
        self.serial = serial
        self.mode = 0
        self.gear_box = {0: 0, 1:30, 2:40, 3:50, 4:60, 5:70, 6:80, 7:90, 8:100}
        self.direction = 0
        self.speed = 0
        self.angle = 0

    def update(self):
        self.speed = np.clip(self.speed, 0, self.gear_box[self.mode])

    def reset_angle(self):
        self.angle = 0

    def reset(self):
        self.direction = 0
        self.mode = 0
        self.speed = 0
        self.angle = 0

    def increase_mode(self):                               ## Tăng số
        self.mode = np.clip(self.mode + 1, 0, 8)
        self.update()

    def decrease_mode(self):                               ## Giảm số
        self.mode = np.clip(self.mode - 1, 0, 8)
        self.update()

    def increase_speed(self, step):                        ## Tăng tốc
        self.speed = np.clip(self.speed + step, 0, self.gear_box[self.mode])

    def decrease_speed(self, step):                        ## Giảm tốc
        self.speed = np.clip(self.speed - step, 0, self.gear_box[self.mode])
    
    def turn_right(self, step):                            ## Tăng gốc cua phải
        self.angle = np.clip(self.angle + step, -25, 25)

    def turn_left(self, step):                             ## Tăng góc cua trái
        self.angle = np.clip(self.angle - step, -25, 25)

    def get_info(self):
        return self.mode, self.direction, self.speed, self.angle

    def go_straight(self):
        if self.direction >= 0:               ## Đang đi thẳng
            self.serial.Car_SetSpeedAngle(self.speed, self.angle, 0)
            self.direction = 1

        else:                                 ## Đang đi lùi
            self.serial.Car_SetSpeedAngle(0, self.angle, 1)
            self.serial.Car_SetSpeedAngle(0, self.angle, 1)
            time.sleep(0.1)
            self.serial.Car_SetSpeedAngle(-self.speed, self.angle, 0)
            self.direction = 1

    def go_reverse(self):
        if self.direction > 0:               ## Đang đi thẳng
            self.serial.Car_SetSpeedAngle(0, self.angle, 1)
            self.serial.Car_SetSpeedAngle(0, self.angle, 1)
            time.sleep(0.1)
            self.serial.Car_SetSpeedAngle(-self.speed, self.angle, 0)
            self.direction = -1

        else:                                 ## Đang đi lùi
            self.serial.Car_SetSpeedAngle(-self.speed, self.angle, 0)  
            self.direction = -1

    def brake(self):
        print("BRAKE")
        self.serial.Car_SetSpeedAngle(0, self.angle, 1)
        self.reset()

    def nop(self):
        print("NOP")
        self.serial.Car_SetSpeedAngle(0, self.angle, 0)
