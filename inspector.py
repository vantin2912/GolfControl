import os
import cv2               
import json                                                         
import numpy as np
                              
def getRandomColour():           
    return tuple(np.random.randint(0, 255, dtype=np.uint8, size=3))
                          
def walkThroughDataset():           
    trainJsonFilePath = r'dataset\train\overfit.json'

    labels = []
    with open(trainJsonFilePath, 'r') as f:
        for line in f:
            label = json.loads(line)
            labels.append(label)

    colorMapMat = np.zeros((6, 3), dtype=np.uint8)

    for i in range(0, 6):
        colorMapMat[i] = np.random.randint(0, 255, dtype=np.uint8, size=3)

    for label in labels:
        imagePath = label["raw_file"]
        print(imagePath)                                                                                      

        img_bgr = cv2.imread(imagePath)         
        segImage = np.zeros_like(img_bgr)        
        for laneIndex, laneXPoints in enumerate(label["lanes"], 0):
            laneLabelColor = colorMapMat[laneIndex]

            curveVertices = list(filter(lambda xyPair: xyPair[0] > 0, zip(                               
                laneXPoints, label["h_samples"])))
            
            for vertex1, vertex2 in zip(curveVertices[:-1], curveVertices[1:]):
                cv2.line(segImage, tuple(vertex1), tuple(vertex2), (int(colorMapMat[5][0]), int(colorMapMat[5][1]), int(colorMapMat[5][2])), 2)

            for node in curveVertices:         
                cv2.circle(segImage, tuple(node), 5, (int(laneLabelColor[0]), int(
                    laneLabelColor[1]), int(laneLabelColor[2])), -1)
            
        res = cv2.addWeighted(img_bgr, 1, segImage, 0.7, 0.4)           
        print(imagePath.replace("round1/auto_label/overfit", "round1/auto_label/inspector_overfit"))
        #cv2.imwrite(imagePath.replace("round1/auto_label/overfit", "round1/auto_label/inspector_overfit"), res)
 
if __name__ == "__main__":
    walkThroughDataset()
