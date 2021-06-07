# Turtlebot3_DNN_Nav
DNN for steering

![image](https://user-images.githubusercontent.com/65225823/121050639-b43fea00-c7f3-11eb-90ee-a400f52824f9.png)

![image](https://user-images.githubusercontent.com/65225823/121050691-c15cd900-c7f3-11eb-91a7-4e88d392c61f.png)

![image](https://user-images.githubusercontent.com/65225823/121051393-611a6700-c7f4-11eb-9cef-d8cd64a399ba.png)


# Turtlebot3_DNN_Nav
DNN for steering

#### ROS noetic, Ubuntu20.04 environment
TURTLEBOT3 MODEL = waffle 사용

#### ROS TURTLEBOT3 GAZEBO에서 자동으로 Depth Image 받아온 것을 자동으로 저장하기
<Opencv / image.py>
rosrun opencv image.py

#### Depth Image에 대한 Control 명령을 쉽게 Labeling하는 코드
iasl_huno_real / Labeling.py

#### Model Labeling
0 : only rotation
1 : left turn
2 : straight
3 : right turn

#### DNN to move
###### knu_prj / turtlesim_depth_control.py 

depth image에서 control 명령 (0, 1, 2, 3)을 넘겨주는 코드
(recieve depth image and predict control command from DNN model)

###### knu_prj / turtlesim_depth_move.cpp

명령 (0, 1, 2, 3)을 받아서 실제로 로봇을 움직이는 코드 
(recieve control command and manipulate turtlebot)

#### Running

make catkin workspace -> git clone this repo -> catkin_make
roscore
roslaunch turtlebot3_gazebo [].launch
rosrun knu_prj turtlesim_depth_control.py
rosrun knu_prj turtlesim_depth_move

### etc.

You can adjust speed in depth_move.cpp (linear.x, angular.z)
You can Train it in iasl_huno / DNN_nav.ipynb

## To be better

down the image scale
Auto labeling by navigation package

Deep Learning for speed estimation control.
