#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

CENTER = 640
ANG_VEL_STEP_SIZE = 0.1

class Rotate():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.bounding_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.rotate, queue_size = 1)
        self.angular_z = 0.0
        self.sleep()


    def rotate(self, input):
        bounding_boxes = input.bounding_boxes

        bottle_b_box = None
        for b_box in bounding_boxes:
            if b_box.Class == "bottle":
                bottle_b_box = b_box
                break
        
        if bottle_b_box is None:
            return
        
        x_mid = (bottle_b_box.xmin + bottle_b_box.xmax) / 2
        
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0
	print(1111, x_mid, CENTER)

        if x_mid > CENTER - 10:
            self.angular_z = -0.05
        elif x_mid < CENTER + 10:
            self.angular_z = 0.05
        else:
            self.angular_z = 0
	twist.angular.z = self.angular_z
	twist.angular.x = self.angular_z
        
        self.cmd_pub.publish(twist)
    
    def sleep(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        

def main():
    rospy.init_node('turtlebot3_teleop')
    try:
        rotate = Rotate()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
