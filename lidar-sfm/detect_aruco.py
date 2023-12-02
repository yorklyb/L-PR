import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import time
import os

img = cv2.imread('this.png',cv2.IMREAD_GRAYSCALE)
length = 0
step = 0.25
temp_ids = []
temp_corners = []
init = 0

# save_path = './imgs2/'

strat = time.time()
for i in range(600):

    threshold = step*i
    # print(threshold)
    ret, gray = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    # path = os.path.join(save_path,str(i)+'.png')
    # print(path)
    # cv2.imwrite(path,gray)
    # cv2.imshow('image',gray)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)


    if ids is not None:

        if init == 0:
            temp_ids = ids
            temp_corners = corners
            init = 1
        # print(threshold,len(ids))
        # lids = ids.tolist()
        # print(ids)
        s = set()
        for id in ids:
            # print(id[0])
            s.add(id[0])
        uids = set(s)
        
        if len(ids)>=length and len(ids) == len(uids) and max(ids)<100:

                # print('init',temp_ids)
            length = len(ids)
            # print(threshold,ids)
            
            if not (temp_ids== ids).all():
                new = [id for id in ids if not id in temp_ids]
                temp_ids = np.append(temp_ids, new)

                for i in new:
                    # print('here',new)
                    for j in range(len(ids)):
                        if ids[j] == new:
                            # print('here',j,new)
                            c = corners[j][0]

                            temp_corners = np.append(temp_corners,c)

            f = open("./this.txt", 'w')


            for i in range(len(temp_ids)):
                # content = str(temp_ids[i])
                # index = 8*(i+1)
                cc = temp_corners
                if type(cc) is not np.ndarray:
                    # print(cc)
                    content = str(temp_ids[i][0])
                    c = cc[i][0]
                    content = content + "," + str(c[3][0]) + "," + str(c[3][1])
                    content = content + "," + str(c[2][0]) + "," + str(c[2][1])
                    content = content + "," + str(c[1][0]) + "," + str(c[1][1])
                    content = content + "," + str(c[0][0]) + "," + str(c[0][1])

                    f.write(content)


                    f.write("\n")

                if type(cc) is np.ndarray:
                    c = cc
                    content = str(int(temp_ids[i]))
                    content = content + "," + str(c[i*8+6]) + "," + str(c[i*8+7])
                    content = content + "," + str(c[i * 8 + 4]) + "," + str(c[i * 8 + 5])
                    content = content + "," + str(c[i * 8 + 2]) + "," + str(c[i * 8 + 3])
                    content = content + "," + str(c[i * 8 + 0]) + "," + str(c[i * 8 + 1])

                   
                    f.write(content)
                    # print('done')

                    f.write("\n")

            
            f.close()


end = time.time()


