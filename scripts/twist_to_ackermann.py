#!/usr/bin/python
# A script that converts Twist message to AckermannDrive message
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class RobotCommand():
    def __init__(self):
        '''
        Initialize the publisher and subscriber
        '''
        self.pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size = 10)
        self.sub = rospy.Subscriber('cmd_vel',Twist, self.callback)
        self.rate = rospy.Rate(100) # 100Hz

    def callback(self, msg):
        '''
        A callback function that recieves a Twist message and publishes an AckermannDrive message
        :param msg: A Twist message
        '''
        vx = msg.linear.x
        vy = msg.linear.y
        theta_dot = msg.angular.z
        d = 0.325

        speed = np.hypot(theta_dot*d, np.hypot(vx,vy))
        action = AckermannDriveStamped()
        theta = np.arctan2(d*theta_dot, np.hypot(vx,vy))
        action.drive.steering_angle = theta 
        action.drive.speed = speed
        self.pub.publish(action)
        self.rate.sleep()
    
if __name__=="__main__":
    # Initalize node
    rospy.init_node("bridge")
    # Create controller object
    robot_command = RobotCommand()
    rospy.spin()