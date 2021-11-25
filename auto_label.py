import os
import cv2
import json
import numpy as np
from net import Net
from src.util import adjust_fits
from src.parameters import Parameters

h_samples  = [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 126, 129, 132, 135, 138, 141, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177]
IMG_W, IMG_H = 640, 180

def mask(x, y):
    image = np.zeros((180,640,3))
    color_index = 0
    for i, j in zip(x, y):
        color_index += 1
        if color_index > 12:
            color_index = 12
        for index in range(len(i) - 1):
            # print(int(i[index]), int(j[index]))s
            image = cv2.line(image, (int(i[index]), int(j[index])), (int(i[index+1]), int(j[index+1])), p.color[color_index], 1)

    return image

def getLaneAnnotation(image):
    leftLane = [-2] * len(h_samples)
    rightLane = [-2] * len(h_samples)

    for i, h in enumerate(h_samples):
        if h < 80:
            continue
        # left
        for j in range(IMG_W):
            if image[h, j][0] == 255:
                leftLane[i] = abs(j)
                break

        for j in range(IMG_W - 1, 0, -1):
            if image[h, j][1] == 255:
                rightLane[i] = abs(j)
                break

    return [leftLane, rightLane]

def convert_to_original_size(x, y):
    # convert results to original size
    out_x = []
    out_y = []

    ratio_w = 512*1.0/640
    ratio_h = 256*1.0/180

    for i, j in zip(x,y):
        out_x.append((np.array(i)/ratio_w).tolist())
        out_y.append((np.array(j)/ratio_h).tolist())

    return out_x, out_y

if __name__ == "__main__":
    net = Net()
    p = Parameters()
    net.load_model("dataset/best5.pkl")

    #labelFile = open("dataset/train/overfit.json", "w")
    root = r"C:\Users\Asus\Desktop\AI\AICAR\New-map\overfit_data\\"

    for imagePath in os.listdir(root):
        if "desktop" in imagePath:
            continue
        
        imagePath = root + imagePath
        print(imagePath)
        image = cv2.imread(imagePath)

        image_resized = cv2.resize(image,(512,256))
        x, y = net.predict(image_resized, warp = False)

        x_, y_ = convert_to_original_size(x, y)
        lane_mask = mask(x_, y_)

        fits = np.array([np.polyfit(_y,_x, 2) for _y, _x in zip(y,x)])
        fits = adjust_fits(fits)
        lane_mask = net.get_mask_lane(fits)
        image_points_result = net.get_image_points()
        cv2.imshow("mask", lane_mask)
        cv2.imshow("image_points_result", image_points_result)

        k = cv2.waitKey(0)
        if k == ord('q'):
            continue
        if k == ord('d'):
            os.remove(imagePath)
        # cv2.imwrite(imagePath.replace("overfit", "inspector_overfit"), lane_mask)

        # label = {}
        # lanes = getLaneAnnotation(lane_mask)
        # raw_file = imagePath.replace("dataset/train/", "")
        # label.update({"lanes": lanes})
        # label.update({"h_samples": h_samples})
        # label.update({"raw_file": raw_file})   
        
        # json.dump(label, labelFile)
        # labelFile.write("\n")