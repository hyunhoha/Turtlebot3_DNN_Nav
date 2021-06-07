import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

with open("/home/iasl/catkin_ws/src/opencv/label.txt","a") as f:
    for i in range(1,501):
        img_name = "/home/iasl/catkin_ws/src/opencv/image/camera_image" + str(i) + ".jpeg"
        curr_img = cv2.imread(img_name,cv2.IMREAD_UNCHANGED)
        name2 = str(i)
        cv2.namedWindow(name2, cv2.WINDOW_NORMAL)
        cv2.imshow(name2,curr_img)
            
        key = cv2.waitKey(100)

        curr_label = str(input())
        f.write(curr_label+" ")
        cv2.destroyAllWindows()
        time.sleep(0.1)

    