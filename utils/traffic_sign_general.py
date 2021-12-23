
import time

import cv2
import numpy as np
from numpy.core.fromnumeric import amax
import torch
from utils.general import non_max_suppression, scale_coords
from utils.general import non_max_suppression
from models.experimental import attempt_load
from utils.augmentations import letterbox

COLOR_MAP_7_CLASS = ((255, 0 ,0))

CLASS_NAME = ['signal']




def detect(original_image, model, imgsz= (640,640), conf_thres = 0.5, iou_thres = 0.45, classes = None, \
                                    agnostic_nms = False, max_det = 1000, color_map = COLOR_MAP_7_CLASS, class_name = CLASS_NAME):

    visualize_image = original_image.copy()
    img = preprocess_image(visualize_image, imgsz)
    pred = model(img, augment= False, visualize= False)[0]
    new_pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det)
    return new_pred

def preprocess_image(original_image, size = (1280,1280), device = 'cuda'):
    original_image = letterbox(original_image, size, stride= 16, auto= False)[0]
    image = original_image[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    image = np.ascontiguousarray(image)
    image = torch.from_numpy(image).to(device)
    image = image.float()  
    image = image / 255.0 
    if image.ndimension() == 3:
        image = image.unsqueeze(0)

    return image
    

def visualize_img(img_src = None, box = None, color_map = COLOR_MAP_7_CLASS, class_name = CLASS_NAME):
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = color_map[int(box[5])]
    thickness = 3
    line_type = 0.5

    visualize_image = cv2.putText(img_src, "{} [{:.2f}]".format(str(class_name[int(box[5])]),float(box[4])), (int(box[2]), int(box[3])-10), font, line_type, color, thickness)
    visualize_image = cv2.rectangle(visualize_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color_map[int(box[5])] , thickness)
    return visualize_image

def calculate_area(box):
    return (box[2] - box[0]) * (box[3] - box[1])

def load_model(path, train = False):
    model = attempt_load(path, map_location='cuda')  # load FP32 model
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if train:
        model.train()
    else:
        model.eval()
    return model, names

def get_boxes(pred):
    boxes = []

    for det in pred:  
        det[:, :4] = scale_coords([320, 320], det[:, :4], [140, 440]).round()
        det = det[:, :6].cpu().detach().numpy()
        for box in det:
            boxes.append(box)
    return boxes

def scale_coordinate(boxes):
    original_shape = 140, 640
    resized_size = 320, 320

    print("original_shape, resized_size: ", original_shape, " ", resized_size)

    ratio = np.divide(original_shape, resized_size)
    
    for box in boxes:
        box[0] *= ratio[1]
        box[2] *= ratio[1]
        box[1] *= ratio[0]
        box[3] *= ratio[0]

    return boxes


def get_iou(box1, box2):
    # 1.get the coordinate of inters
    ixmin = max(box1[0], box2[0])
    ixmax = min(box1[2], box2[2])
    iymin = max(box1[1], box2[1])
    iymax = min(box1[3], box2[3])

    iw = np.maximum(ixmax-ixmin+1., 0.)
    ih = np.maximum(iymax-iymin+1., 0.)

    # 2. calculate the area of inters
    inters = iw*ih

    # 3. calculate the area of union
    uni = ((box1[2]-box1[0]+1.) * (box1[3]-box1[1]+1.) +
           (box2[2] - box2[0] + 1.) * (box2[3] - box2[1] + 1.) -
           inters)

    iou = inters / uni

    return iou

