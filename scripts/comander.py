#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Twist


MIN_DISTANCE = 1  # m


def build_cmd(msg):
    pos_x = lambda m, i: m.pose[i].pose.position.x
    pos_y = lambda m, i: m.pose[i].pose.position.y
    cmd = Twist()

    dx = pos_x(msg, 2) - pos_x(msg, 1) - MIN_DISTANCE
    dy = pos_y(msg, 2) - pos_y(msg, 1) - MIN_DISTANCE
    
    cmd.linear.x = dx
    cmd.linear.y = dy
    cmd.linear.z = 0
    
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0
    
    return cmd


def callback(msg):
    global chanel
    
    chanel.publish(build_cmd(msg))


def listener():
    rospy.loginfo("Start Commander!")
    print("Start Commander!")
    
    rospy.init_node('comander', anonymous=True)
    
    global chanel 
    chanel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/gazebo/model_states", ModelState, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

