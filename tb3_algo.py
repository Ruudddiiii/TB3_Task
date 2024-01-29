
# Author : Rudresh Singh 

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import math
from typing import List, Tuple


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

        self.m = -1
        self.c = 0
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
        
        self.timer4 = self.create_timer(0.1, self.same_orien1)  
        self.timer5 = self.create_timer(0.1, self.same_orien2)  
        self.timer6 = self.create_timer(0.1, self.same_orien3)  
        self.timer7 = self.create_timer(0.1, self.same_orien4)
        self.timer8 = self.create_timer(0.1, self.same_orien5)
       # self.timer9 = self.create_timer(0.1, self.same_orien6)
        self.timer9 = self.create_timer(0.1, self.computation) 


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
    


    
    def computation(self):

      if self.call_count <= 5:
        Point = Tuple[float, float]
        def orientation(p: Point, q: Point, r: Point) -> int:
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0
            return 1 if val > 0 else 2

        # def dist(p1: Point, p2: Point) -> float:
        #     return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        def convexHull(points: List[Point]) -> List[Point]:
            n = len(points)
            hull = []
            l = 0
            for i in range(1, n):
                if points[i][0] < points[l][0]:
                    l = i
            p = l
            while True:
                hull.append(points[p])
                q = (p + 1) % n
                for i in range(n):
                    if orientation(points[p], points[i], points[q]) == 2:
                        q = i
                p = q
                if p == l:
                    break
            return hull

        
        points = [(self.last_pose_x1, self.last_pose_y1), 
                  (self.last_pose_x2, self.last_pose_y2), 
                  (self.last_pose_x3, self.last_pose_y3), 
                  (self.last_pose_x4, self.last_pose_y4), 
                  (self.last_pose_x5, self.last_pose_y5)]
        v = convexHull(points)

        vv = []
        dis = []
        mx = 0
        mxx = float('inf')
        e = 0
        for i in range(len(v) - 1):
            m = (v[i+1][1] - v[i][1]) / (v[i+1][0] - v[i][0])
            c = m * v[i][0] - v[i][1]
            p = None
            for j in range(len(v)):
                d = abs(v[j][1] - m * v[j][0] + c) / (math.sqrt(1 + m * m))
                if d > mx:
                    p = v[j]
                    mx = d
            vv.append((v[i], v[i+1], p))
            if mx < mxx:
                e = i
                mxx = mx
            dis.append(mx)
            mx = 0

        #print("\n\n\n")
        m = (vv[e][1][1] - vv[e][0][1]) / (vv[e][1][0] - vv[e][0][0])
        p1 = (vv[e][2][0] + vv[e][0][0]) / 2
        p2 = (vv[e][2][1] + vv[e][0][1]) / 2
        #print("y = {}x + {}".format(m, p2 - m * p1))
        self.m = m
        self.c = p2 - m * p1
        print(m,p2-m*p1)

      self.call_count += 1
        
    
    def same_orien1(self):
        
        twist = Twist()
        m=self.m
        c=self.c
        ans = [ (self.last_pose_x1  + m*(self.last_pose_y1 - c ))/(1+m*m) , (c+m*(self.last_pose_x1 + m*self.last_pose_y1 ))/(1+m*m)]
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
                if(dist_net > 0.05 ):
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
                if(dist_net > 0.05 ):
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
         
                if(dist_net > 0.05 ):
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

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.15
                 self.cmd_pub1.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub1.publish(twist)
        
    def same_orien2(self):
        twist = Twist()
        m=self.m
        c=self.c
        ans = [ (self.last_pose_x2  + m*(self.last_pose_y2 - c ))/(1+m*m) , (c+m*(self.last_pose_x2 + m*self.last_pose_y2 ))/(1+m*m)]
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
                if(dist_net > 0.05 ):
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
                if(dist_net > 0.05 ):
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
         
                if(dist_net > 0.05 ):
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

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.15
                 self.cmd_pub2.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub2.publish(twist)
    
    def same_orien3(self):
        twist = Twist()
        m=self.m
        c=self.c
        ans = [ (self.last_pose_x3  + m*(self.last_pose_y3 - c ))/(1+m*m) , (c+m*(self.last_pose_x3 + m*self.last_pose_y3 ))/(1+m*m)]
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
                if(dist_net > 0.05 ):
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
                if(dist_net > 0.05 ):
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
         
                if(dist_net > 0.05 ):
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

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.15
                 self.cmd_pub3.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub3.publish(twist)

    def same_orien4(self):
        twist = Twist()
        m=self.m
        c=self.c
        ans = [ (self.last_pose_x4  + m*(self.last_pose_y4 - c ))/(1+m*m) , (c+m*(self.last_pose_x4 + m*self.last_pose_y4 ))/(1+m*m)]
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
                if(dist_net > 0.05 ):
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
                if(dist_net > 0.05 ):
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
         
                if(dist_net > 0.05 ):
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

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.15
                 self.cmd_pub4.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub4.publish(twist)
    
    def same_orien5(self):
        twist = Twist()
        m=self.m
        c=self.c
        ans = [ (self.last_pose_x5  + m*(self.last_pose_y5 - c ))/(1+m*m) , (c+m*(self.last_pose_x5 + m*self.last_pose_y5 ))/(1+m*m)]
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
                if(dist_net > 0.05 ):
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
                if(dist_net > 0.05 ):
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
         
                if(dist_net > 0.05 ):
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

                if(dist_net > 0.05 ):
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
