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
        self.last_pose_theta1 = 0.0
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
        
        #Initialise publisher
        self.cmd_pub1 = self.create_publisher(
            Twist,
            'tb3_1/cmd_vel',
           # self.PID,
            qos)

      #  self.timer1 = self.create_timer(1.0, self.print_odom1)
        self.timer3 = self.create_timer(0.1, self.PID)  
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


    def PID(self):
        twist = Twist()
        # print(self.last_pose_x1,self.last_pose_y1)

        x_net = self.last_pose_x2 - self.last_pose_x1
        y_net = self.last_pose_y2 - self.last_pose_y1
        net_dist = math.sqrt(x_net**2 + y_net**2)
        net_theta = -self.last_pose_theta1 + numpy.arctan2(y_net,x_net) 
        if(net_theta > math.pi):
            net_theta = -2*math.pi + net_theta
        elif(net_theta < -math.pi):
            net_theta = 2*math.pi + net_theta


        #if( x_net > 0 and y_net >0 or x_net >0 and y_net < 0):
         #   print(net_theta)
        if(net_dist < 1 ):
            Kp = 0.18
        if(net_dist < 1.5 and net_dist > 1):
            Kp = 0.13
        if(net_dist > 1.5 and net_dist < 3.2) :
            Kp = 0.065
        Kp_z = 0.8
        # print(self.last_pose_x1,self.last_pose_y1)
        er_x = Kp * net_dist
        er_z = Kp_z * net_theta
        print(er_x , er_z , net_dist)
        
        twist.linear.x = er_x 
        twist.angular.z = er_z 
        self.cmd_pub1.publish(twist)
        # elif( x_net <=0 and y_net >=0):
        #     if(net_dist < 1.5):
        #      Kp = 0.13
        #     if(net_dist > 1.5 and net_dist < 3.2) :
        #      Kp = 0.065
        #     Kp_z = 0.8
        #  #   print(self.last_pose_x1,self.last_pose_y1)
        #     er_x = Kp * net_dist
        #     er_z = Kp_z * net_theta
        #     # print(er_z)
        #     print(er_x , er_z)

        #     twist.linear.x = er_x
        #     twist.angular.z = er_z
        #     self.cmd_pub1.publish(twist)
        # else:
        #     # Kp = 0.10
        #     # Kp_z = 2.8
        #  #   print(self.last_pose_x1,self.last_pose_y1)
        #     er_x = Kp * math.sqrt((a - self.last_pose_x1)**2 + (b - self.last_pose_y1)**2)
        #     er_z = Kp_z * (-self.last_pose_theta1 +( -math.pi/2 - numpy.arctan((b-self.last_pose_y1)/(a-self.last_pose_x1))))
        #     print(er_z)

        #     twist.linear.x = er_x
        #     twist.angular.z = er_z
        #     self.cmd_pub1.publish(twist)

 

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
