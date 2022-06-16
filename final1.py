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
from std_msgs.msg import Float64MultiArray
import time

CENTER = 640
ANG_VEL_STEP_SIZE = 0.1

class Rotate():
    def __init__(self):

        self.margin_of_centering = 20
        self.rotation_speed = 0.05
        self.area_criteria = 2500

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.grip_pub = rospy.Publisher('gripper_position', Float64MultiArray, queue_size=1)
        self.arm_pub = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size=1)

        self.bounding_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.algorithm, queue_size = 1)
        self.angular_z = 0.0
        self.step = 0

        self.step_count = 0

        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_pub.publish(twist)
        
        msg = Float64MultiArray()
        msg.data = [0.025]
        self.grip_pub.publish(msg)

        self.sleep()
    
    def rotate_and_stop(self, b_box, sensitivity):
        
        x_mid = (b_box.xmin + b_box.xmax) / 2
        
        twist = Twist()

        if x_mid > CENTER - self.margin_of_centering * sensitivity:
            self.angular_z = -self.rotation_speed * sensitivity
        elif x_mid < CENTER + self.margin_of_centering * sensitivity:
            self.angular_z = self.rotation_speed * sensitivity
        else:
            self.angular_z = 0
            self.step_count += 1
        
        twist.angular.z = self.angular_z
        
        self.cmd_pub.publish(twist)
    
    def go_and_stop(self, b_box):
        
        area = (b_box.xmax - b_box.xmin) * (b_box,ymax - b_box.ymin)
        if area <= self.area_criteria:
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)
        else:
            twist.linear.x = 0
            self.cmd_pub.publish(twist)
            self.step_count += 1
            
    def grip_and_lift(self):
        
        msg = Float64MultiArray()
        msg.data = [-1, 0.0, 0.68, 0.29, -0.81]
        self.arm_pub.publish(msg)
        time.sleep(5)

        msg = Float64MultiArray()
        msg.data = [-0.1]
        self.grip_pub.publish(msg)
        time.sleep(1)
        
        msg = Float64MultiArray()
        msg.data = [-1, 0.0, 0.0, 0.0, 0.0]
        self.arm_pub.publish(msg)

        self.step += 1


    def algorithm(self, input):

        bounding_boxes = input.bounding_boxes

        bottle_b_box = None
        for b_box in bounding_boxes:
            if b_box.Class == "bottle":
                bottle_b_box = b_box
                break
        
        if bottle_b_box is None:
            return
        
        if self.step == 1:
            self.rotate_and_stop(bottle_b_box)
        elif self.step == 2:
            self.rotate_and_stop(bottle_b_box)
            self.go_and_stop(bottle_b_box)
        elif self.step == 3:
            self.grip_and_lift()
        elif self.step == 4:
            print("-------------------- done! --------------------")
            self.step += 1
        
        if self.step_count > 5:
            self.step += 1
            self.step_count = 0
        
    
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
