#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

from math import sqrt, atan2, pi

class TurtlesimController:
    
    def __init__(self) -> None:
        # inicijalizacija node-a
        rospy.init_node("turtlesim_controller")
        
        self.cmd_vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.cmd_vel_rate = rospy.Rate(60)
        
        rospy.Subscriber("/turtle1/pose", Pose, callback=self.callback_turtle_pose)
        
        self.pose = Pose()
        self.flg_PoseUpdated = False
        
        self.reset_service = rospy.ServiceProxy("reset", Empty)
        
        self.reset_service()
        
        self.x_goal = 0
        self.y_goal = 0
        
    def callback_turtle_pose(self, msg : Pose):
        self.pose = msg
        self.flg_PoseUpdated = True
        
    def go_to(self, x_goal, y_goal):
        
        Kp_linear = 1.5
        Kp_angular = 5
        
        while not rospy.is_shutdown():
            
            distance_err = sqrt((x_goal - self.pose.x) ** 2 + (y_goal - self.pose.y)**2)
            linear_vel = distance_err * Kp_linear
            
            angle_between = atan2(y_goal - self.pose.y, x_goal - self.pose.x)
            angular_err = angle_between - self.pose.theta

            if angular_err > pi:
                angular_err = angular_err - 2*pi
            elif angular_err < -pi:
                angular_err = angular_err + 2*pi
                
            angular_vel = angular_err * Kp_angular
            
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = angular_vel
            
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.cmd_vel_rate.sleep()
    
    def run(self):
        linear_velocity = 4
        angular_velocity = 5
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        
        cmd_vel_msg.angular.z = angular_velocity
        
        while not rospy.is_shutdown():
        
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.cmd_vel_rate.sleep()
            
            
if __name__ == "__main__":
    
    node = TurtlesimController()
    node.go_to(1, 5)
        