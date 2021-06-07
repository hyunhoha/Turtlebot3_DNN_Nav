import tensorflow as tf
import cv2
import numpy as np

model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
img = cv2.imread("/home/iasl/catkin_ws/src/knu_prj/src/camera_image32.jpeg", cv2.IMREAD_ANYCOLOR)
img_re = cv2.resize(img, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
image = np.array(img_re).reshape((1,640,480,1))

print(model.predict(image))