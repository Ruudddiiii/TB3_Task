
#Authors : Rudresh Singh and Nitish Bishnoi

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        self.odom = Odometry()
        self.last_pose_x1 = 0.0
        self.last_pose_y1 = 0.0
        self.last_pose_x2 = 0.0
        self.last_pose_y2 = 0.0
        self.last_pose_x3 = 0.0
        self.last_pose_y3 = 0.0
        self.last_pose_theta = 0.0
        self.init_odom_state = False 
        self.last_pose_theta1 = 0.0 # To get the initial pose at the beginning
        self.last_pose_theta2 = 0.0
        self.last_pose_theta3 = 0.0
        qos = QoSProfile(depth=10)

        # Initialise subscribers
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

      #  self.timer2 = self.create_timer(1.0, self.print_odom2)  # Create a timer with a 1 Hz frequency

        # Initialise subscribers
        self.odom_sub1 = self.create_subscription(
            Odometry,
            'tb3_1/odom',
            self.odom_callback1,
            qos)
        
        #Initialise publisher
        self.cmd_pub1 = self.create_publisher(
            Twist,
            'tb3_1/cmd_vel',
           # self.same_orien,
            qos)
        self.cmd_pub2 = self.create_publisher(
            Twist,
            'tb3_2/cmd_vel',
           # self.same_orien,
            qos)
        self.cmd_pub3 = self.create_publisher(
            Twist,
            'tb3_3/cmd_vel',
           # self.same_orien,
            qos)

      #  self.timer1 = self.create_timer(1.0, self.print_odom1)
        self.timer3 = self.create_timer(0.1, self.com_pt1)
        self.timer4 = self.create_timer(0.1, self.com_pt2)    
        self.timer5 = self.create_timer(0.1, self.com_pt3)
          

    def odom_callback1(self, msg):
        self.last_pose_x1 = msg.pose.pose.position.x
        self.last_pose_y1 = msg.pose.pose.position.y
        _, _, self.last_pose_theta1 = self.euler_from_quaternion(msg.pose.pose.orientation)
    

    # def print_odom1(self):
    #     #if self.init_odom_state:
    #        # print(f"Pose X: {self1.last_pose_x}")
    #         print(f"Pose Theta of robo 1: {self.last_pose_theta1 * 180/math.pi}")
    
    def odom_callback2(self, msg):
        self.last_pose_x2 = msg.pose.pose.position.x
        self.last_pose_y2 = msg.pose.pose.position.y
        _, _, self.last_pose_theta2 = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def odom_callback3(self, msg):
        self.last_pose_x3 = msg.pose.pose.position.x
        self.last_pose_y3 = msg.pose.pose.position.y
        _, _, self.last_pose_theta3 = self.euler_from_quaternion(msg.pose.pose.orientation)
       

    # def print_odom2(self):
        
    #        # print(f"Pose X: {self1.last_pose_x}")
    #         print(f"Pose Theta of robo2: {self.last_pose_theta2 * 180/math.pi}")
    def com_pt1(self):
        twist = Twist()
        kp=0.22
       # print(x_net)
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3
        # if(abs(self.last_pose_x1 - x_net) > 0.01):
        if(self.last_pose_x1 < x_net):
            twist.linear.x =  kp*(x_net - self.last_pose_x1)
            self.cmd_pub1.publish(twist)
        else:
            twist.linear.x = kp*(x_net - self.last_pose_x1)
            self.cmd_pub1.publish(twist)
        # else:
        #     twist.linear.x = 0.0
        #     self.cmd_pub1.publish(twist)

    
    def com_pt2(self):
        twist = Twist()
        kp=0.22
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3
        # if(abs(self.last_pose_x2 - x_net) > 0.01):
        if(self.last_pose_x2 < x_net):
            twist.linear.x = kp*(x_net - self.last_pose_x2)
            self.cmd_pub2.publish(twist)
        else:
            twist.linear.x = kp*(x_net - self.last_pose_x2)
            self.cmd_pub2.publish(twist)
        # else:
        #     twist.linear.x = 0.0
        #     self.cmd_pub2.publish(twist)

    
    def com_pt3(self):
        twist = Twist()
        kp=0.22
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3
        # if(abs(self.last_pose_x3 - x_net) > 0.01):
        if(self.last_pose_x3 < x_net):
            twist.linear.x = +kp*(x_net - self.last_pose_x3)
            self.cmd_pub3.publish(twist)
        else:
            twist.linear.x = kp*(x_net - self.last_pose_x3)
            self.cmd_pub3.publish(twist)
        # else:
        #    twist.linear.x = 0.0
        #    self.cmd_pub3.publish(twist)


    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
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

