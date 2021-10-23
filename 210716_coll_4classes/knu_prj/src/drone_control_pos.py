#! /usr/bin/python3

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

mavros.set_namespace()

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode

def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()

    while 1:
        now = rospy.get_rostime()
        if not current_state.armed:# and (now - last_request > rospy.Duration(5.)):
            # set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            # last_request = now
            arming_client(True)
            last_request = now
            rospy.loginfo("trying to arming")
            # local_pos_pub.publish(pose)
            # rate.sleep()
        else:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
            break
            # if not current_state.armed and (now - last_request > rospy.Duration(5.)):
            #    arming_client(True)
            #    last_request = now 
    while 1:
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD": # and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now
            local_pos_pub.publish(pose)
            rate.sleep()
        else:
            rospy.loginfo("Current mode: %s" % current_state.mode)
            break
            #    arming_client(True)
            #    last_request = now 
    

        # older versions of PX4 always return success==True, so better to check Status instead
        # if prev_state.armed != current_state.armed:
            
        # if prev_state.mode != current_state.mode: 
        # prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        # local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Hello")
        position_control()
    except rospy.ROSInterruptException:
        pass