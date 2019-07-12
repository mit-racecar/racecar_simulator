#!/usr/bin/env python

from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Ackermann Drive!
---------------------------
Moving around:
        i    
   j    k    l
Press any other key to stop.
CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    drive_topic = rospy.get_param("/racecar_simulator/drive_topic")

    pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
    rospy.init_node('keyboard_teleop')

    
    speed = 0
    steering_angle = 0

    try:
        print(msg)
        previous_speed = 0
        while(1):
            key = getKey()
            if key == 'i':
                speed = 2
                steering_angle = 0
            elif key == 'j':
                steering_angle = 0.5
                speed = previous_speed
            elif key == 'l':
                steering_angle = -0.5
                speed = previous_speed
            elif key == 'k':
                speed = -2
                steering_angle = 0
            else:
                speed = 0
                steering_angle = 0
                if (key == '\x03'):
                    break

            ack_drive = AckermannDriveStamped()
            ack_drive.drive.speed = speed
            ack_drive.drive.steering_angle = steering_angle
            pub.publish(ack_drive)
            previous_speed = speed

    except Exception as e:
        print(e)

    finally:
        ack_drive = AckermannDriveStamped()
        ack_drive.drive.speed = 0
        ack_drive.drive.steering_angle = 0
        pub.publish(ack_drive)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)