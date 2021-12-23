# Import socket module
import socket
import cv2
import numpy as np
from utils.traffic_sign_general import *
import os
global sendBack_angle, sendBack_Speed, current_speed, current_angle
sendBack_angle = 0
sendBack_Speed = 0
current_speed = 0
current_angle = 0

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the port on which you want to connect
PORT = 54321
# connect to the server on local computer
s.connect(('127.0.0.1', PORT))


def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed



count = 254
img_dir = r'C:\Users\Asus\Desktop\UIT_CAR_RACING\final_image\Images'

if __name__ == "__main__":

    #traffic_sign_model, _ = load_model('best.pt')

    try:
        while True:

            """
            - Chương trình đưa cho bạn 1 giá trị đầu vào:
                * image: hình ảnh trả về từ xe
                * current_speed: vận tốc hiện tại của xe
                * current_angle: góc bẻ lái hiện tại của xe

            - Bạn phải dựa vào giá trị đầu vào này để tính toán và
            gán lại góc lái và tốc độ xe vào 2 biến:
                * Biến điều khiển: sendBack_angle, sendBack_Speed
                Trong đó:
                    + sendBack_angle (góc điều khiển): [-25, 25]
                        NOTE: ( âm là góc trái, dương là góc phải)

                    + sendBack_Speed (tốc độ điều khiển): [-150, 150]
                        NOTE: (âm là lùi, dương là tiến)
            """

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
           
                traffic_sign_image = image[100: 240, 160 : 600]


                # pred = detect(traffic_sign_image, traffic_sign_model, imgsz = (320, 320), conf_thres = 0.7, iou_thres = 0.45)
                # boxes = get_boxes(pred)
                # for box in boxes:
                #     visualize_img(traffic_sign_image, box)

                img_path = os.path.join(img_dir, str(count) + ".png")
                cv2.imwrite(img_path, traffic_sign_image)

                count +=1
                
                cv2.imshow("traffic_sign_image", traffic_sign_image)
                cv2.waitKey(1)   
                

                #print("FPS: ", 1.0/(time.time() - start_time))

            except Exception as er:
                print(er)
                pass

    finally:
        print('closing socket')
        s.close()
