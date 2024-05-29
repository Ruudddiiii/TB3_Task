
# Author : Rudresh Singh 

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile,qos_profile_sensor_data
from nav_msgs.msg import Odometry
import math
from typing import List, Tuple
import time


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x1 = 0.0
        self.last_pose_y1 = 0.0
        self.last_pose_x2 = 0.0
        self.last_pose_y2 = 0.0
        self.last_pose_x3 = 0.0
        self.last_pose_y3 = 0.0

        self.last_pose_x5 = 0.0
        self.last_pose_y5 = 0.0
        self.last_pose_x6 = 0.0
        self.last_pose_y6 = 0.0


        self.init_odom_state1 = False
        self.init_odom_state2 = False 
        self.init_odom_state3 = False 
        self.init_odom_state4 = False 
        self.init_odom_state5 = False 
        self.init_odom_state6 = False 
        self.init_odom_state7 = False 
        self.init_odom_state8 = False 
        self.init_odom_state9 = False 
        self.init_odom_state10 = False 


        self.last_pose_theta1 = 0.0
        self.last_pose_theta2 = 0.0
        self.last_pose_theta3 = 0.0

        self.last_pose_theta5 = 0.0
        self.last_pose_theta6 = 0.0

        qos = QoSProfile(depth=10)

        self.odom_sub1 = self.create_subscription(
            Odometry,
            '/tb3_1/odom',
            self.odom_callback1,
            qos)
        
        self.odom_sub2 = self.create_subscription(
            Odometry,
            'tb3_2/odom',
            self.odom_callback2,
            qos)

        self.odom_sub3 = self.create_subscription(
            Odometry,
            'tb3_3/odom',
            self.odom_callback3,
            qos)

        
        self.odom_sub5 = self.create_subscription(
            Odometry,
            'tb3_5/odom',
            self.odom_callback5,
            qos)
        
        self.odom_sub6 = self.create_subscription(
            Odometry,
            'tb3_6/odom',
            self.odom_callback6,
            qos)
        

        self.m = 0
        self.c = 0
        self.x_net = 0.0
        self.call_count = 0
        
        #Initialise publisher
        self.cmd_pub2 = self.create_publisher(
            Twist,
            'tb3_2/cmd_vel',
            qos)
        
        self.cmd_pub1 = self.create_publisher(
            Twist,
            'tb3_1/cmd_vel',
        
            qos)
        self.cmd_pub3 = self.create_publisher(
            Twist,
            'tb3_3/cmd_vel',
            qos)

        self.cmd_pub5 = self.create_publisher(
            Twist,
            'tb3_5/cmd_vel',
            qos)
            
        self.cmd_pub6 = self.create_publisher(
            Twist,     
            'tb3_6/cmd_vel',
            qos)
        

        
        self.timer1 = self.create_timer(0.05, self.computation)
        self.timer2 = self.create_timer(0.1, self.same_orien1)  
        self.timer3 = self.create_timer(0.1, self.same_orien2)  
        self.timer4 = self.create_timer(0.1, self.same_orien3)  
        self.timer6 = self.create_timer(0.1, self.same_orien5)
        self.timer7 = self.create_timer(0.1, self.same_orien6)


    def odom_callback1(self, msg):
        self.last_pose_x1 = msg.pose.pose.position.x
        self.last_pose_y1 = msg.pose.pose.position.y
        _, _, self.last_pose_theta1 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state1 = True
    
    def odom_callback2(self, msg):
        self.last_pose_x2 = msg.pose.pose.position.x
        self.last_pose_y2 = msg.pose.pose.position.y
        _, _, self.last_pose_theta2 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state5 = True
       
    def odom_callback3(self, msg):
        self.last_pose_x3 = msg.pose.pose.position.x
        self.last_pose_y3 = msg.pose.pose.position.y
        _, _, self.last_pose_theta3 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state6 = True
    
    def odom_callback5(self, msg):
        self.last_pose_x5 = msg.pose.pose.position.x
        self.last_pose_y5 = msg.pose.pose.position.y
        _, _, self.last_pose_theta5 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state7 = True

    def odom_callback6(self, msg):
        self.last_pose_x6 = msg.pose.pose.position.x
        self.last_pose_y6 = msg.pose.pose.position.y
        _, _, self.last_pose_theta6 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state8 = True



       
    

    def computation(self):
      

      if( self.init_odom_state1 == True and 
         self.init_odom_state5 == True and
         self.init_odom_state6 == True and
         self.init_odom_state7 == True and
         self.init_odom_state8 == True
                                             ):
        
        if self.call_count <= 2:
            print("hello")
     
            # Create a list of the pose values
            pose_values = [
                self.last_pose_x1,
                self.last_pose_x2,
                self.last_pose_x3,
                self.last_pose_x5,
                self.last_pose_x6
            ]
            
            # Find the minimum and maximum values
            min_value = min(pose_values)
            max_value = max(pose_values)
            
            self.x_net = (min_value + max_value)/2
        self.call_count += 1
            

            
        
    
    def same_orien1(self):
        twist = Twist()
        if(self.last_pose_x1 - self.x_net < 0.1):
            twist.linear.x = 0.13
            self.cmd_pub1.publish(twist)
        elif(self.last_pose_x1 - self.x_net > 0.15):
            twist.linear.x = -0.13
            self.cmd_pub1.publish(twist)
        else:
            twist.linear.x = -0.0
            self.cmd_pub1.publish(twist)


    def same_orien2(self):
        twist = Twist()
        if(self.last_pose_x2 - self.x_net < 0.1):
            twist.linear.x = 0.13
            self.cmd_pub2.publish(twist)
        elif(self.last_pose_x2 - self.x_net > 0.15):
            twist.linear.x = -0.13
            self.cmd_pub2.publish(twist)
        else:
            twist.linear.x = -0.0
            self.cmd_pub2.publish(twist)

    def same_orien3(self):
        twist = Twist()
        if(self.last_pose_x3 - self.x_net < 0.1):
            twist.linear.x = 0.13
            self.cmd_pub3.publish(twist)
        elif(self.last_pose_x3 - self.x_net > 0.15):
            twist.linear.x = -0.13
            self.cmd_pub3.publish(twist)
        else:
            twist.linear.x = -0.0
            self.cmd_pub3.publish(twist)

    def same_orien5(self):
        twist = Twist()
        if(self.last_pose_x5 - self.x_net < 0.1):
            twist.linear.x = 0.13
            self.cmd_pub5.publish(twist)
        elif(self.last_pose_x5 - self.x_net > 0.15):
            twist.linear.x = -0.13
            self.cmd_pub5.publish(twist)
        else:
            twist.linear.x = -0.0
            self.cmd_pub5.publish(twist)

    def same_orien6(self):
        twist = Twist()
        if(self.last_pose_x6 - self.x_net < 0.1):
            twist.linear.x = 0.13
            self.cmd_pub6.publish(twist)
        elif(self.last_pose_x6 - self.x_net > 0.15):
            twist.linear.x = -0.13
            self.cmd_pub6.publish(twist)
        else:
            twist.linear.x = -0.0
            self.cmd_pub6.publish(twist)

        
   



    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        z = quat.z
        w = quat.w

        roll = 0
        pitch = 0

        siny_cosp = 2 * w * z
        cosy_cosp = 1 - 2 * z * z
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    
    rclpy.init(args=args)
    turtlebot3_position_control = Turtlebot3PositionControl()
    rclpy.spin(turtlebot3_position_control)
    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
