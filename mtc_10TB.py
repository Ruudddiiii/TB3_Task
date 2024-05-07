
# Author : Rudresh Singh 


import random
import math

def dist(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)



import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy
import math
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

def dist(p1,p2):
  return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2
  




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
        self.last_pose_x6 = 0.0
        self.last_pose_y6 = 0.0
        self.last_pose_x7 = 0.0
        self.last_pose_y7 = 0.0
        self.last_pose_x8 = 0.0
        self.last_pose_y8 = 0.0
        self.last_pose_x9 = 0.0
        self.last_pose_y9 = 0.0
        self.last_pose_x10 = 0.0
        self.last_pose_y10 = 0.0
        self.last_pose_theta = 0.0

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
        self.last_pose_theta4 = 0.0
        self.last_pose_theta5 = 0.0
        self.last_pose_theta6 = 0.0
        self.last_pose_theta7 = 0.0
        self.last_pose_theta8 = 0.0
        self.last_pose_theta9 = 0.0
        self.last_pose_theta10 = 0.0
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
        
        self.odom_sub6 = self.create_subscription(
            Odometry,
            'tb3_6/odom',
            self.odom_callback6,
            qos)
        
        self.odom_sub7 = self.create_subscription(
            Odometry,
            'tb3_7/odom',
            self.odom_callback7,
            qos)

        self.odom_sub8 = self.create_subscription(
            Odometry,
            'tb3_8/odom',
            self.odom_callback8,
            qos)
        self.odom_sub9 = self.create_subscription(
            Odometry,
            'tb3_9/odom',
            self.odom_callback9,
            qos)
        
        self.odom_sub10 = self.create_subscription(
            Odometry,
            'tb3_10/odom',
            self.odom_callback10,
            qos)

        # self.a = 0
        # self.b = 0
        # self.r = 0
        self.ans1 = 0
        self.ans2 = 0
        self.ans3 = 0
        self.ans4 = 0
        self.ans5 = 0
        self.ans6 = 0
        self.ans7 = 0
        self.ans8 = 0
        self.ans9 = 0
        self.ans10 = 0
        self.c1 = 0
        self.c2 = 0
        self.c3 = 0
        self.c4 = 0
        self.c5 = 0
        self.c6 = 0
        self.c7 = 0
        self.c8 = 0
        self.c9 = 0
        self.c10 = 0
        self.cc = 0


        
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
        
        self.cmd_pub6 = self.create_publisher(
            Twist,
            'tb3_6/cmd_vel',
            qos)
        
        self.cmd_pub7 = self.create_publisher(
            Twist,
            'tb3_7/cmd_vel',
        
            qos)
        self.cmd_pub8 = self.create_publisher(
            Twist,
            'tb3_8/cmd_vel',
            qos)
        self.cmd_pub9 = self.create_publisher(
            Twist,
            'tb3_9/cmd_vel',
            qos)
        
        self.cmd_pub10 = self.create_publisher(
            Twist,
            'tb3_10/cmd_vel',
            qos)
        
        self.timer2 = self.create_timer(0.1, self.runner)  
        # self.timer3 = self.create_timer(0.1, self.computation)  
        # self.timer4 = self.create_timer(0.1, self.same_orien1)  
        # self.timer5 = self.create_timer(0.1, self.same_orien2)  
        # self.timer6 = self.create_timer(0.1, self.same_orien3)  
        # self.timer7 = self.create_timer(0.1, self.same_orien4)
        # self.timer8 = self.create_timer(0.1, self.same_orien5)
        # self.timer9 = self.create_timer(0.1, self.same_orien6)  
        # self.timer10 = self.create_timer(0.1, self.same_orien7)  
        # self.timer11 = self.create_timer(0.1, self.same_orien8)  
        # self.timer12 = self.create_timer(0.1, self.same_orien9)
        # self.timer13 = self.create_timer(0.1, self.same_orien10)  
    
        
        


    def odom_callback1(self, msg):
        self.last_pose_x1 = msg.pose.pose.position.x
        self.last_pose_y1 = msg.pose.pose.position.y
        _, _, self.last_pose_theta1 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state1 = True
    
    def odom_callback2(self, msg):
        self.last_pose_x2 = msg.pose.pose.position.x
        self.last_pose_y2 = msg.pose.pose.position.y
        _, _, self.last_pose_theta2 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state2 = True
       
    def odom_callback3(self, msg):
        self.last_pose_x3 = msg.pose.pose.position.x
        self.last_pose_y3 = msg.pose.pose.position.y
        _, _, self.last_pose_theta3 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state3 = True

    def odom_callback4(self, msg):
        self.last_pose_x4 = msg.pose.pose.position.x
        self.last_pose_y4 = msg.pose.pose.position.y
        _, _, self.last_pose_theta4 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state4 = True
    
    def odom_callback5(self, msg):
        self.last_pose_x5 = msg.pose.pose.position.x
        self.last_pose_y5 = msg.pose.pose.position.y
        _, _, self.last_pose_theta5 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state5 = True

    def odom_callback6(self, msg):
        self.last_pose_x6 = msg.pose.pose.position.x
        self.last_pose_y6 = msg.pose.pose.position.y
        _, _, self.last_pose_theta6 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state6 = True

    def odom_callback7(self, msg):
        self.last_pose_x7 = msg.pose.pose.position.x
        self.last_pose_y7 = msg.pose.pose.position.y
        _, _, self.last_pose_theta7 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state7 = True

    def odom_callback8(self, msg):
        self.last_pose_x8 = msg.pose.pose.position.x
        self.last_pose_y8 = msg.pose.pose.position.y
        _, _, self.last_pose_theta8 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state8 = True
    
    def odom_callback9(self, msg):
        self.last_pose_x9 = msg.pose.pose.position.x
        self.last_pose_y9 = msg.pose.pose.position.y
        _, _, self.last_pose_theta9 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state9 = True

    def odom_callback10(self, msg):
        self.last_pose_x10 = msg.pose.pose.position.x
        self.last_pose_y10 = msg.pose.pose.position.y
        _, _, self.last_pose_theta10 = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state10 = True


    def runner(self):
      if( self.init_odom_state1 is True and 
             self.init_odom_state2 is True and
             self.init_odom_state3 is True and 
             self.init_odom_state4 is True and 
             self.init_odom_state5 is True and 
             self.init_odom_state6 is True and 
             self.init_odom_state7 is True and 
             self.init_odom_state8 is True and 
             self.init_odom_state9 is True and 
             self.init_odom_state10 is True ):
            
        self.computation()
        self.same_orien1()
        self.same_orien2()
        self.same_orien3()  
        self.same_orien4()
        self.same_orien5()
        self.same_orien6()
        self.same_orien7()
        self.same_orien8()
        self.same_orien9()
        self.same_orien10()

    
        
    def computation(self):
            
            if(self.cc <= 10):
        
                v = [(self.last_pose_x1,self.last_pose_y1),
                    (self.last_pose_x2,self.last_pose_y2),
                    (self.last_pose_x3,self.last_pose_y3),
                    (self.last_pose_x4,self.last_pose_y4),
                    (self.last_pose_x5,self.last_pose_y5), 
                    (self.last_pose_x6,self.last_pose_y6),
                    (self.last_pose_x7,self.last_pose_y7),
                    (self.last_pose_x8,self.last_pose_y8),
                    (self.last_pose_x9,self.last_pose_y9),
                    (self.last_pose_x10,self.last_pose_y10)]

                print("Points are:")
                for point in v:
                    print("( {}, {} )".format(point[0], point[1]))

                # mx = 0
                # mxx = float('inf')
                b = 0
                finn = float('inf')
                radd = 0
                ans = (0, 0)

                for i in range(len(v)):
                    for j in range(len(v)):
                        for k in range(len(v)):
                            for l in range(len(v)):
                                if i != j and k != l and i != k and i != l and j != k and j != l:
                                    if((v[i][1] - v[j][1]) == 0 or v[k][1] - v[l][1] == 0):
                                     pass
                                    else:
                                        m1 = (v[j][0] - v[i][0]) / (v[i][1] - v[j][1])
                                        m2 = (v[l][0] - v[k][0]) / (v[k][1] - v[l][1])
                                        x1net = (v[i][0] + v[j][0]) / 2
                                        x2net = (v[k][0] + v[l][0]) / 2
                                        y1net = (v[i][1] + v[j][1]) / 2
                                        y2net = (v[k][1] + v[l][1]) / 2
                                        xfinal = ((y2net - y1net) + (m1 * x1net - m2 * x2net)) / (m1 - m2)
                                        yfinal = m1 * xfinal - m1 * x1net + y1net
                                        ff = (xfinal, yfinal)

                                        mx = 0
                                        mxx = float('inf')
                                        for m in range(len(v)):
                                            ndist = math.sqrt(dist(ff, v[m]))
                                            # print(ndist)
                                            if ndist > mx:
                                                mx = ndist
                                            if ndist < mxx:
                                                mxx = ndist

                                        # print(ff)

                                        mxxx = mx - (mx + mxx) / 2
                                        if mxxx < finn:
                                            finn = mxxx
                                            ans = ff
                                            radd = (mx + mxx) / 2
                                        

                                    

                print("Minimum distance from each point our algo:", finn)
                print("Radius:", radd, "Centre: (", ans[0], ",", ans[1], ")")
                self.a = ans[0]
                self.b = ans[1]
                self.r = radd
                print(ans[0], ans[1])

                self.cc+=1

           

       
    def same_orien1(self):

       
        twist = Twist()
        if(self.c1 <= 5):
            x1 = self.last_pose_x1
            y1 = self.last_pose_y1
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3) < dist(p2,p3)):
             self.ans1 = p1
            else:
             self.ans1 = p2
        self.c1+=1
        
        x_net=self.last_pose_x1 - self.ans1[0]
        y_net=self.last_pose_y1 - self.ans1[1]
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
        if(self.c2 <= 5):
            x1 = self.last_pose_x2
            y1 = self.last_pose_y2
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans2 = p1
            else:
             self.ans2  = p2
        self.c2+=1
        # print(self.ans2)
        x_net=self.last_pose_x2 - self.ans2[0]
        y_net=self.last_pose_y2 - self.ans2[1]
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
        if(self.c3 <= 5):
            x1 = self.last_pose_x3
            y1 = self.last_pose_y3
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans3 = p1
            else:
             self.ans3  = p2
        self.c3+=1
        x_net=self.last_pose_x3 - self.ans3[0]
        y_net=self.last_pose_y3 - self.ans3[1]
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
        if(self.c4 <= 5):
            x1 = self.last_pose_x4
            y1 = self.last_pose_y4
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans4 = p1
            else:
             self.ans4  = p2
        self.c4+=1
        x_net=self.last_pose_x4 - self.ans4[0]
        y_net=self.last_pose_y4 - self.ans4[1]
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
        if(self.c5 <= 5):
            x1 = self.last_pose_x5
            y1 = self.last_pose_y5
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans5 = p1
            else:
             self.ans5  = p2
        self.c5+=1
        x_net=self.last_pose_x5 - self.ans5[0]
        y_net=self.last_pose_y5 - self.ans5[1]
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
    
    def same_orien6(self):

       
        twist = Twist()
        if(self.c6 <= 5):
            x1 = self.last_pose_x6
            y1 = self.last_pose_y6
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans6 = p1
            else:
             self.ans6  = p2
        self.c5+=1
        x_net=self.last_pose_x6 - self.ans6[0]
        y_net=self.last_pose_y6 - self.ans6[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta6*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub6.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub6.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub6.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub6.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub6.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub6.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub6.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub6.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub6.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub6.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub6.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub6.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub6.publish(twist)
         
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub6.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub6.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub6.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub6.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub6.publish(twist)

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub6.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub6.publish(twist)

    

    def same_orien7(self):

       
        twist = Twist()
        if(self.c7 <= 5):
            x1 = self.last_pose_x7
            y1 = self.last_pose_y7
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans7 = p1
            else:
             self.ans7  = p2
        self.c7+=1
        x_net=self.last_pose_x7 - self.ans7[0]
        y_net=self.last_pose_y7 - self.ans7[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta7*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub7.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub7.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub7.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub7.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub7.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub7.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub7.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub7.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub7.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub7.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub7.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub7.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub7.publish(twist)
         
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub7.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub7.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub7.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub7.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub7.publish(twist)

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub7.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub7.publish(twist)



    def same_orien8(self):

       
        twist = Twist()
        if(self.c8 <= 5):
            x1 = self.last_pose_x8
            y1 = self.last_pose_y8
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans8 = p1
            else:
             self.ans8  = p2
        self.c8+=1
        x_net=self.last_pose_x8 - self.ans8[0]
        y_net=self.last_pose_y8 - self.ans8[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta8*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub8.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub8.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub8.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub8.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub8.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub8.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub8.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub8.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub8.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub8.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub8.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub8.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub8.publish(twist)
         
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub8.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub8.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub8.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub8.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub8.publish(twist)

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub8.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub8.publish(twist)
    
    def same_orien9(self):

       
        twist = Twist()
        if(self.c9 <= 5):
            x1 = self.last_pose_x9
            y1 = self.last_pose_y9
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans9 = p1
            else:
             self.ans9 = p2
        self.c9+=1
        x_net=self.last_pose_x9 - self.ans9[0]
        y_net=self.last_pose_y9 - self.ans9[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta9*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub9.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub9.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub9.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub9.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub9.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub9.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub9.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub9.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub9.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub9.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub9.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub9.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub9.publish(twist)
         
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub9.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub9.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub9.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub9.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub9.publish(twist)

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub9.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub9.publish(twist)


    
    def same_orien10(self):

       
        twist = Twist()
        if(self.c10 <= 5):
            x1 = self.last_pose_x10
            y1 = self.last_pose_y10
            m = (y1 - self.b)/(x1 - self.a)
            c = y1 - m * x1
            r = self.r
            a = self.a
            b = self.b
            a = self.a
            b = self.b
            a1 = -(c - (c + m*a + m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            a2 = -(c - (c + m*a - m*(- c**2 - 2*c*m*a + 2*c*b + (m**2)*r**2 - (m**2)*a**2 + 2*m*a*b + r**2 - b**2)**(1/2) + (m**2)*b)/(m**2 + 1))/m
            b1 = m*a1 + c
            b2 = m*a2 + c
            b1 = m*a1 + c
            b2 = m*a2 + c
            p1 = [a1,b1]
            p2 = [a2,b2]
            p3 = [x1,y1]
            if(dist(p1,p3)<=dist(p2,p3)):
             self.ans10 = p1
            else:
             self.ans10  = p2
        self.c10+=1
        x_net=self.last_pose_x10 - self.ans10[0]
        y_net=self.last_pose_y10 - self.ans10[1]
        theta_net = numpy.arctan(y_net/x_net)*180/math.pi - self.last_pose_theta10*180/math.pi  
        dist_net = abs(math.sqrt(x_net**2 + y_net**2))
        if(x_net > 0 and y_net >0 ):
            # print('both +')
            if(theta_net < 175 or  theta_net > 185):
                if(theta_net > 0  and theta_net < 180):
                 twist.angular.z = -0.5
                 self.cmd_pub10.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub10.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub10.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub10.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub10.publish(twist)
        elif(x_net < 0 and y_net < 0 ):
            # print('both -')
            if(theta_net > 5 or theta_net < -5):
                if(theta_net > 0 and theta_net < 180):
                 twist.angular.z = 0.5
                 self.cmd_pub10.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub10.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub10.publish(twist)
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub10.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub10.publish(twist)

        elif(x_net > 0 and y_net < 0 ):
            # print('x + and y - ')
            if(theta_net > -175 or theta_net < -185):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = 0.5
                 self.cmd_pub10.publish(twist)
                else:
                 twist.angular.z = -0.5
                 self.cmd_pub10.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub10.publish(twist)
         
                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub10.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub10.publish(twist)
        elif(x_net < 0 and y_net > 0 ):
            # print('x - and y + ')
            if(theta_net < -5 or theta_net > 5):
                if(theta_net < 0 and theta_net > -180):
                 twist.angular.z = -0.5
                 self.cmd_pub10.publish(twist)
                else:
                 twist.angular.z = 0.5
                 self.cmd_pub10.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub10.publish(twist)

                if(dist_net > 0.05 ):
                 twist.linear.x = 0.25
                 self.cmd_pub10.publish(twist)
                else:
                 twist.linear.x = 0.0
                 self.cmd_pub10.publish(twist)


    

   
    

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
