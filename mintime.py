import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

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
        self.last_pose_x4 = 0.0
        self.last_pose_y4 = 0.0
        self.last_pose_x5 = 0.0
        self.last_pose_y5 = 0.0
        self.last_pose_theta = 0.0
        self.init_odom_state = False 
        self.last_pose_theta1 = 0.0
        self.last_pose_theta2 = 0.0
        self.last_pose_theta3 = 0.0
        self.last_pose_theta4 = 0.0
        self.last_pose_theta5 = 0.0
        qos = QoSProfile(depth=10)

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
        self.odom_sub4 = self.create_subscription(
            Odometry,
            'tb3_4/odom',
            self.odom_callback4,
            qos)
        
        self.odom_sub5 = self.create_subscription(
            Odometry,
            'tb3_5/odom',
            self.odom_callback5,
            qos)

        self.ans1 = [] 
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
        self.cmd_pub4 = self.create_publisher(
            Twist,
            'tb3_4/cmd_vel',
            qos)
        
        self.cmd_pub5 = self.create_publisher(
            Twist,
            'tb3_5/cmd_vel',
            qos)
        
     #   self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer3 = self.create_timer(0.1, self.same_orien) 
       # self.same_orien()
        self.timer4 = self.create_timer(0.1, self.same_orien1)  
        self.timer5 = self.create_timer(0.1, self.same_orien2)  
        self.timer6 = self.create_timer(0.1, self.same_orien3)  
        self.timer7 = self.create_timer(0.1, self.same_orien4)
        self.timer8 = self.create_timer(0.1, self.same_orien5) 
    
        
        


    def odom_callback1(self, msg):
        self.last_pose_x1 = msg.pose.pose.position.x
        self.last_pose_y1 = msg.pose.pose.position.y
        _, _, self.last_pose_theta1 = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def odom_callback2(self, msg):
        self.last_pose_x2 = msg.pose.pose.position.x
        self.last_pose_y2 = msg.pose.pose.position.y
        _, _, self.last_pose_theta2 = self.euler_from_quaternion(msg.pose.pose.orientation)
       
    def odom_callback3(self, msg):
        self.last_pose_x3 = msg.pose.pose.position.x
        self.last_pose_y3 = msg.pose.pose.position.y
        _, _, self.last_pose_theta3 = self.euler_from_quaternion(msg.pose.pose.orientation)

    def odom_callback4(self, msg):
        self.last_pose_x4 = msg.pose.pose.position.x
        self.last_pose_y4 = msg.pose.pose.position.y
        _, _, self.last_pose_theta4 = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def odom_callback5(self, msg):
        self.last_pose_x5 = msg.pose.pose.position.x
        self.last_pose_y5 = msg.pose.pose.position.y
        _, _, self.last_pose_theta5 = self.euler_from_quaternion(msg.pose.pose.orientation)
    

    def same_orien(self):
        if self.call_count <= 5:
         
            ans = []
            x1, y1 = self.last_pose_x1, self.last_pose_y1
            # print(x1,y1)
            x2, y2 = self.last_pose_x2, self.last_pose_y2
            x3, y3 = self.last_pose_x3, self.last_pose_y3
            x4, y4 = self.last_pose_x4, self.last_pose_y4
            x5, y5 = self.last_pose_x5, self.last_pose_y5
            p1 = (x1, y1)
            p2 = (x2, y2)
            p3 = (x3, y3)
            p4 = (x4, y4)
            p5 = (x5, y5)
            v = [p1, p2, p3, p4, p5]

            min_dist = 0
            point1 = (0, 0)

            for point in v:
                d = dist((0, 0), point)
                if d > min_dist:
                    min_dist = d
                    point1 = point

            mx = 0
            x3_net, y3_net = 0, 0
            point2 = None

            for point in v:
                if point == point1:
                    continue
                else:
                    x1, y1 = point1
                    x2, y2 = point

                    radi1 = -(math.sqrt(x1**2 + y1**2) * (x1**2 - 2 * x1 * x2 + x2**2 + y1**2 - 2 * y1 * y2 + y2**2)) / (2 * (-x1**2 + x2 * x1 - y1**2 + y2 * y1))

                    if radi1 >= mx:
                        mx = radi1
                        x3_net = (x1 * (x1**2 + y1**2 - y2**2 - x2**2)) / (2 * (x1**2 + y1**2 - x2 * x1 - y1 * y2))
                        y3_net = (y1 / x1) * x3_net
                        point2 = point

            if point2 is not None: 
                if (point1[0] + point2[0]) / 2 == x3_net and (point1[1] + point2[1]) / 2 == y3_net:
                    # print("No further iteration required, and the center is:")
                    ans = [x3_net, y3_net]
                else:
                    a = (point1[0] + point2[0]) / 2
                    b = (point1[1] + point2[1]) / 2
                    minn = 0
                    p1 = None
                    d = 0

                    for point in v:
                        c = dist((a, b), point)
                        if c > dist(point1, point2) / 2:
                            if c > minn:
                                p1 = point
                                minn = c
                            d = 9.0

                    if d == 0:
                        ans = [a , b]
                    
                    else:
                        avgp_x = (point1[0] + point2[0]) / 2
                        avgp_y = (point1[1] + point2[1]) / 2
                        m = (avgp_y - y3_net) / (avgp_x - x3_net)
                        c = y3_net - x3_net * m
                        x1, y1 = point1
                        x5_net = -(-x1**2 + p1[0]**2 - y1**2 + 2 * c * y1 + p1[1]**2 - 2 * c * p1[1]) / (2 * (x1 - p1[0] + m * y1 - m * p1[1]))
                        y5_net = m * (x5_net - x3_net) + y3_net
                        ans = [x5_net, y5_net]
            else:
             ans = [x3_net, y3_net]
            
            self.ans1 = ans
        self.call_count += 1
        
    
    def same_orien1(self):
        
        twist = Twist()
        ans = self.ans1
        print(ans)
        # print(self.last_pose_x1,self.last_pose_y1)
        x_net=self.last_pose_x1 - ans[0]
        y_net=self.last_pose_y1 - ans[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta1*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)
         
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub1.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub1.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub1.publish(twist)

                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        
    def same_orien2(self):
        twist = Twist()
        ans = self.ans1
        x_net=self.last_pose_x2 - ans[0]
        y_net=self.last_pose_y2 - ans[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta2*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub2.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub2.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub2.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub2.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub2.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub2.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub2.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub2.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub2.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub2.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub2.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub2.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub2.publish(twist)
         
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub2.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub2.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub2.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub2.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub2.publish(twist)

                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub2.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub2.publish(twist)
    
    def same_orien3(self):
        twist = Twist()
        ans = self.ans1
        x_net=self.last_pose_x3 - ans[0]
        y_net=self.last_pose_y3 - ans[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta3*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub3.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub3.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub3.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub3.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub3.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub3.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub3.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub3.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub3.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub3.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub3.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub3.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub3.publish(twist)
         
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub3.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub3.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub3.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub3.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub3.publish(twist)

                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub3.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub3.publish(twist)

    def same_orien4(self):
        twist = Twist()
        ans = self.ans1
        x_net=self.last_pose_x4 - ans[0]
        y_net=self.last_pose_y4 - ans[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta4*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub4.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub4.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub4.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub4.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub4.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub4.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub4.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub4.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub4.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub4.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub4.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub4.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub4.publish(twist)
         
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub4.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub4.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub4.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub4.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub4.publish(twist)

                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub4.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub4.publish(twist)
    
    def same_orien5(self):
        twist = Twist()
        ans = self.ans1
        x_net=self.last_pose_x5 - ans[0]
        y_net=self.last_pose_y5 - ans[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta5*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub5.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub5.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub5.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub5.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub5.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub5.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub5.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub5.publish(twist)
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub5.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub5.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub5.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub5.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub5.publish(twist)
         
                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub5.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub5.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub5.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub5.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub5.publish(twist)

                if(dist_net > 0.4 ):
                 twist.linear.x = 0.15
                 self.cmd_pub5.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub5.publish(twist)
    

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

