

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

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x1 = 0.0
        self.last_pose_y1 = 0.0
        self.last_pose_x2 = 0.0
        self.last_pose_y2 = 0.0
        self.last_pose_theta = 0.0
        self.init_odom_state = False 
        self.last_pose_theta1 = 0.0 # To get the initial pose at the beginning
        self.last_pose_theta2 = 0.0
        qos = QoSProfile(depth=10)

        # Initialise subscribers
        self.odom_sub2 = self.create_subscription(
            Odometry,
            'tb3_2/odom',
            self.odom_callback2,
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

      #  self.timer1 = self.create_timer(1.0, self.print_odom1)
        self.timer3 = self.create_timer(0.2, self.same_orien)  

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
       

    # def print_odom2(self):
        
    #        # print(f"Pose X: {self1.last_pose_x}")
    #         print(f"Pose Theta of robo2: {self.last_pose_theta2 * 180/math.pi}")

    def same_orien(self):
        twist = Twist()
        
        #spawnpos = 0.17
        #self.last_pose_x2 = self.last_pose_x2 - spawnpos
        #print(self.last_pose_x1 - self.last_pose_x2 , self.last_pose_y1 - self.last_pose_y2)
        #works fine for x_net and y_net both > 0
        x_net=self.last_pose_x1 - self.last_pose_x2
        y_net=self.last_pose_y1 - self.last_pose_y2
        #print(theta_net)
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta1*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            print('both +')
            if(theta_net < 175 or  theta_net > 185):
                # twist.angular.z = (abs(self.last_pose_theta2 - self.last_pose_theta1)*(180/math.pi))/150
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.2
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = +0.2
                 self.cmd_pub1.publish(twist)
            else:
               # print("yes")
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
               # print(dist_net)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            print('both -')
           # print((-180+numpy.arctan(y_net/x_net)*180/math.pi) - self.last_pose_theta1*180/math.pi )
           # print(numpy.arctan(self.last_pose_y2 / self.last_pose_x2)*(180/math.pi))
            if(theta_net > 5 or theta_net < -5):
                # twist.angular.z = (abs(self.last_pose_theta2 - self.last_pose_theta1)*(180/math.pi))/150
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = +0.2
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = -0.2
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
               # print(dist_net)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
           # print('yes')
            print('x + and y - ')
           # print((-180+numpy.arctan(y_net/x_net)*180/math.pi) - self.last_pose_theta1*180/math.pi )
           # print(numpy.arctan(self.last_pose_y2 / self.last_pose_x2)*(180/math.pi))
            if(theta_net > -175 or theta_net < -185):
                # twist.angular.z = (abs(self.last_pose_theta2 - self.last_pose_theta1)*(180/math.pi))/150
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = +0.2
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = -0.2
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
               # print(dist_net)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            print('x - and y + ')
           # print((-180+numpy.arctan(y_net/x_net)*180/math.pi) - self.last_pose_theta1*180/math.pi )
           # print(numpy.arctan(self.last_pose_y2 / self.last_pose_x2)*(180/math.pi))
            if(theta_net < -5 or theta_net > 5):
                # twist.angular.z = (abs(self.last_pose_theta2 - self.last_pose_theta1)*(180/math.pi))/150
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.2
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = +0.2
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
               # print(dist_net)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        
 

    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w] but roll and pitch is 0 if we are working on a plane surface
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
