#!/usr/bin/env python

import rospy
import math
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def move(velosity_publisher,speed,distance,is_forward):
    
    velocity_message = Twist()

    global x,y

    x0=x
    y0=y

    if(is_forward):
        velocity_message.linear.x=abs(speed) # positive (forward)
    else:
        velocity_message.linear.x=-abs(speed) # negative (backward)
    
    distance_moved = 0.0
    loop_rate=rospy.Rate(10)

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved=abs(math.sqrt(((x-x0)**2)+((y-y0)**2)))
        print(distance_moved)
        if not(distance_moved<distance):
            rospy.loginfo("reached")
            break # stop the robot

    velocity_message.linear.x=0
    velocity_publisher.publish(velocity_message)

def poseCallback(pose_message):
    
    global x,y,yaw

    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta


if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose',anonymous=True)

        cmd_vel_topic="/turtle1/cmd_vel"
        velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

        position_topic="/turtle1/pose"
        pose_subscriber=rospy.Subscriber(position_topic,Pose,poseCallback)
        time.sleep(2)

        move(velocity_publisher,1.0,4.0,True)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
