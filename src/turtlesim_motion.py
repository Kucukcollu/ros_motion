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

def rotate(velocity_publisher,angular_speed_degree,relative_angle_degree,clockwise):

    velocity_message = Twist()

    angular_speed = math.radians(abs(angular_speed_degree))

    if(clockwise):
        velocity_message.angular.z=-abs(angular_speed)
    else:
        velocity_message.angular.z=abs(angular_speed)
    
    loop_rate = rospy.Rate(20)
    t0=rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1=rospy.Time.now().to_sec()
        current_angle_degree=(t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if(current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break
    
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)

def go_to_goal_P(velocity_publisher,x_goal,y_goal):
    
    global x,y,yaw

    loop_rate=rospy.Rate(1)
    velocity_message=Twist()

    while True:
        K_linear=0.5
        distance=abs(math.sqrt(((x_goal-x)**2)+ ((y_goal-y)**2)))
        linear_speed=distance*K_linear

        K_angular=4.0
        desired_angle_goal=math.atan2(y_goal-y,x_goal-x)
        angular_speed=(desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print("x=",x,"y=",y,"distance to goal=",distance)

        if (distance<0.01):
            break

def go_to_goal_PID(velocity_publisher,x_goal,y_goal):
    # working on
    pass

def set_desired_orientation(velocity_publisher,speed_in_degree,desired_angle_degree):
    relative_angle_radians=math.radians(desired_angle_degree)-yaw
    clockwise=0

    if relative_angle_radians<0:
        clockwise=1
    else:
        clockwise=0
    
    print("relative_angle_radians:",math.degrees(relative_angle_radians))
    print("desired_angle_degree:",desired_angle_degree)
    rotate(velocity_publisher,speed_in_degree,math.degrees(abs(relative_angle_radians)),clockwise)

def spiral(velocity_publisher,wk,rk):
    
    velocity_message=Twist()
    loop_rate=rospy.Rate(1)

    while((x<10.5) and (y<10.5)):
        rk=rk+1
        velocity_message.linear.x=rk
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=wk
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
    
    velocity_message.linear.x=0
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message,0,2)

def clean(velocity_publisher):
    
    desired_pose=Pose()
    desired_pose.x=1
    desired_pose.y=1
    desired_pose.theta=0

    go_to_goal_P(velocity_publisher,1,1)

    set_desired_orientation(velocity_publisher,30,math.radians(desired_pose.theta))

    for i in range(5):
        move(velocity_publisher,2.0,1.0,True)
        rotate(velocity_publisher,20,90,False)
        move(velocity_publisher,2.0,9.0,True)
        rotate(velocity_publisher,20,90,True)
        move(velocity_publisher,2.0,1.0,True)
        rotate(velocity_publisher,20,90,True)
        move(velocity_publisher,2.0,9.0,True)
        rotate(velocity_publisher,20,90,False)
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose',anonymous=True)

        cmd_vel_topic="/turtle1/cmd_vel"
        velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

        position_topic="/turtle1/pose"
        pose_subscriber=rospy.Subscriber(position_topic,Pose,poseCallback)
        time.sleep(2)

        #move(velocity_publisher,1.0,4.0,True)
        #rotate(velocity_publisher,90,90,True)
        #go_to_goal_P(velocity_publisher,7.0,8.0)
        #set_desired_orientation(velocity_publisher,30,270)
        #spiral(velocity_publisher,2,0)
        clean(velocity_publisher)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
