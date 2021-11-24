
# Import socket module 
import socket          
import cv2
import numpy as np
import os

# Create a socket object 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

# Define the port on which you want to connect 
port = 54321                
  
# connect to the server on local computer 
s.connect(('127.0.0.1', port))



output_dir = r'C:\Users\Asus\Desktop\AI\AICAR\New-map\map1\data'

angle = 50
speed = 100

count = 0

try:
    while True:
        # Send data
        print(angle, speed)
        message = bytes(f"{angle} {speed}", "utf-8")
        s.sendall(message)

        data = s.recv(1000000)
        # print(data)
        try:
            decoded = cv2.imdecode(np.frombuffer(data, np.uint8), -1)
            print(decoded.shape)
            cv2.imshow("IMG", decoded)
            key = cv2.waitKey(100)                   # the more this number is, the less image is captured

            img_file = os.path.join(output_dir, str(count) + ".png")
            cv2.imwrite(img_file, decoded)
            count += 1



        except Exception as er:
            print(er)
            pass
        
        


finally:
    print('closing socket')
    s.close()