import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

with open("/home/iasl/catkin_ws/src/opencv/label.txt","a") as f:
    for i in range(1,501):
        img_name = "/home/iasl/catkin_ws/src/opencv/image/camera_image" + str(i) + ".jpeg"
        # print(img_name)
        curr_img = cv2.imread(img_name,cv2.IMREAD_UNCHANGED)
        # curr_img2 = cv2.resize(curr_img, dsize=(640,480), interpolation=cv2.INTER_AREA)
        print(curr_img.shape)
        # cv2.imshow(str(i), curr_img2)
        # plt.ion()
        # plt.
        name2 = str(i)
        cv2.namedWindow(name2, cv2.WINDOW_NORMAL)
        cv2.imshow(name2,curr_img)
        # plt.imshow(curr_img)
        # time.sleep(3)

        # while True:
            
        key = cv2.waitKey(100)
        # print(str(key))
        # if key == 27:
        #     print("good")
        #     pass
        # f.write(chr(key)+" ")
        # pass
        # if key == 48 or key==49 or key == 50: # 0, 1, 2
        #     print(chr(key))
        #     f.write(chr(key)+" ")
        #     pass

        # plt.imshow(curr_img)
        curr_label = str(input())
        # print(curr_label)
        # curr_label = key
        f.write(curr_label+" ")
        cv2.destroyAllWindows()
        time.sleep(0.1)

    