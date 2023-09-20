
#Author : Rudresh Singh

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('tpc')

        self.odom = Odometry()
        self.last_pose_x1 = 0.0
        self.last_pose_y1 = 0.0
        self.last_pose_x2 = 0.0
        self.last_pose_y2 = 0.0
        self.last_pose_x3 = 0.0
        self.last_pose_y3 = 0.0
        self.last_pose_theta1 = 0.0
        self.last_pose_theta2 = 0.0
        self.last_pose_theta3 = 0.0
        self.init_odom_state = False 
        qos = QoSProfile(depth=10)
        

      #  self.timer2 = self.create_timer(1.0, self.print_odom2)  # Create a timer with a 1 Hz frequency

        # Initialise subscribers
        self.odom_sub1 = self.create_subscription(
            Odometry,
            'tb3_1/odom',
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
        
        #Initialise publisher
        self.cmd_pub1 = self.create_publisher(
            Twist,
            'tb3_1/cmd_vel',
           # self.PID,
            qos)
        self.cmd_pub2 = self.create_publisher(
            Twist,
            'tb3_2/cmd_vel',
           # self.PID,
            qos)
        self.cmd_pub3 = self.create_publisher(
            Twist,
            'tb3_3/cmd_vel',
           # self.PID,
            qos)

      #  self.timer1 = self.create_timer(1.0, self.print_odom1)
        self.timer3 = self.create_timer(0.1, self.com_pt1)
        self.timer4 = self.create_timer(0.1, self.com_pt2)    
        self.timer5 = self.create_timer(0.1, self.com_pt3)
        # self.timer4 = self.create_timer(0.01, self.norm)


    def odom_callback1(self, msg):
        self.last_pose_x1 = msg.pose.pose.position.x
        self.last_pose_y1 = msg.pose.pose.position.y
        _, _, self.last_pose_theta1 = self.euler_from_quaternion(msg.pose.pose.orientation)
        # Kp = 0.22/math.sqrt((a)**2 + (b)**2)
        # Kp_z = 2.82/(-self.last_pose_theta1 + numpy.arctan((b)/(a)))
    
    def odom_callback2(self, msg):
        self.last_pose_x2 = msg.pose.pose.position.x
        self.last_pose_y2 = msg.pose.pose.position.y
        _, _, self.last_pose_theta2 = self.euler_from_quaternion(msg.pose.pose.orientation)
        # Kp = 0.22/math.sqrt((a)**2 + (b)**2)
        # Kp_z = 2.82/(-self.last_pose_theta1 + numpy.arctan((b)/(a)))
    
    def odom_callback3(self, msg):
        self.last_pose_x3 = msg.pose.pose.position.x
        self.last_pose_y3 = msg.pose.pose.position.y
        _, _, self.last_pose_theta3 = self.euler_from_quaternion(msg.pose.pose.orientation)
        global a,b
        a=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3
        b=(self.last_pose_y3 + self.last_pose_y2 + self.last_pose_y1)/3
    

    def com_pt1(self):
        twist = Twist()
        def Kd(x):
         return -0.0147157*x**4 + 0.0970346*x**3 - 0.174314*x**2 - 0.045563*x + 0.327559
       # print(x_net)
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3 - self.last_pose_x1
        y_net=(self.last_pose_y3 + self.last_pose_y2 + self.last_pose_y1)/3 - self.last_pose_y1
        net_dist = math.sqrt(x_net**2 + y_net**2)
        net_theta = -self.last_pose_theta1 + numpy.arctan2(y_net,x_net) 
        if(net_theta > math.pi):
            net_theta = -2*math.pi + net_theta
        elif(net_theta < -math.pi):
            net_theta = 2*math.pi + net_theta
        Kp_z = 0.5
        # print(self.last_pose_x1,self.last_pose_y1)
        er_x =   Kd(net_dist) * net_dist *0.8
        er_z = Kp_z * net_theta
        print(er_x , er_z , net_dist)
        
        twist.linear.x = er_x 
        twist.angular.z = er_z 
        self.cmd_pub1.publish(twist)
        if(net_dist <= 0.3):
            twist.linear.x =0.0
            twist.angular.z = 0.0
            self.cmd_pub1.publish(twist)
    
    def com_pt2(self):
        twist = Twist()
        def Kd(x):
         return -0.0147157*x**4 + 0.0970346*x**3 - 0.174314*x**2 - 0.045563*x + 0.327559
       # print(x_net)
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3 - self.last_pose_x2
        y_net=(self.last_pose_y3 + self.last_pose_y2 + self.last_pose_y1)/3 - self.last_pose_y2
        net_dist = math.sqrt(x_net**2 + y_net**2)
        net_theta = -self.last_pose_theta2 + numpy.arctan2(y_net,x_net) 
        if(net_theta > math.pi):
            net_theta = -2*math.pi + net_theta
        elif(net_theta < -math.pi):
            net_theta = 2*math.pi + net_theta
        Kp_z = 0.8
        # print(self.last_pose_x1,self.last_pose_y1)
        er_x =   Kd(net_dist) * net_dist * 0.8
        er_z = Kp_z * net_theta
        print(er_x , er_z , net_dist)
        
        twist.linear.x = er_x 
        twist.angular.z = er_z 
        self.cmd_pub2.publish(twist)
        if(net_dist <= 0.3):
            twist.linear.x =0.0
            twist.angular.z = 0.0
            self.cmd_pub2.publish(twist)

    def com_pt3(self):
        twist = Twist()
        def Kd(x):
         return -0.0147157*x**4 + 0.0970346*x**3 - 0.174314*x**2 - 0.045563*x + 0.327559
       # print(x_net)
        x_net=(self.last_pose_x3 + self.last_pose_x2 + self.last_pose_x1)/3 - self.last_pose_x3
        y_net=(self.last_pose_y3 + self.last_pose_y2 + self.last_pose_y1)/3 - self.last_pose_y3
        net_dist = math.sqrt(x_net**2 + y_net**2)
        net_theta = -self.last_pose_theta3 + numpy.arctan2(y_net,x_net) 
        if(net_theta > math.pi):
            net_theta = -2*math.pi + net_theta
        elif(net_theta < -math.pi):
            net_theta = 2*math.pi + net_theta
        Kp_z = 0.8
        # print(self.last_pose_x1,self.last_pose_y1)
        er_x =   Kd(net_dist) * net_dist * 0.8
        er_z = Kp_z * net_theta
        print(er_x , er_z , net_dist)
        
        twist.linear.x = er_x 
        twist.angular.z = er_z 
        self.cmd_pub3.publish(twist)
        if(net_dist <= 0.3):
            twist.linear.x =0.0
            twist.angular.z = 0.0
            self.cmd_pub3.publish(twist)

    
    

        

 

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
    tpc = Turtlebot3PositionControl()
    try:

        rclpy.spin(tpc)
    except KeyboardInterrupt: 
        tpc.Twist().linear.x = 0.0 
        tpc.Twist().angular.z = 0.0 
        tpc.cmd_pub1.publish(Twist())
        tpc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
