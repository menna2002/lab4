#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

key_mapping = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

    try:
        print("Ready to control the robot! Press keys to move.")
        while not rospy.is_shutdown():
            key = getKey()
            if key in key_mapping.keys():
                twist.linear.x = key_mapping[key][0]
                twist.angular.z = key_mapping[key][1]
                pub.publish(twist)
            else:
                if (key == '\x03'):
                    break
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
