import apriltag
import cv2
import numpy as np
img = cv2.imread('this.png',cv2.IMREAD_GRAYSCALE)
detector = apriltag.Detector()

length = 0
step = 0.25

for i in range(600):

    threshold = step * i
    # print(threshold)
    ret, gray = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    result = detector.detect(gray)
    if result is not None:
        if len(result) >= length:
            length = len(result)
            #print(len(result))
            f = open("./this.txt", 'w')

            for i in range(len(result)):
                # print(i)
                corners = result[i].corners  # in apriltag-python, the order is left-top -> right-top; in C-apriltag, it is left-bottom -> right_bottom
                corners = np.flipud(corners)
                corners = np.resize(corners, (1, 8))
                # print(corners)
                id = result[i].tag_id
                content = str(id)
                for i in range(8):
                    content = content + "," + str(corners[0][i])
                f.write(content)
                f.write("\n")
            f.close()


