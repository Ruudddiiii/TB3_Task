import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from pynput import keyboard

class Turtlebot3PositionControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_position_control')
        qos = QoSProfile(depth=10)

        # Initialize publishers
        self.cmd_pub1 = self.create_publisher(
            Twist,
            'tb3_1/cmd_vel',
            qos)
        self.cmd_pub2 = self.create_publisher(
            Twist,
            'tb3_2/cmd_vel',
            qos)
        self.cmd_pub3 = self.create_publisher(
            Twist,
            'tb3_3/cmd_vel',
            qos)
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0
        self.c = 0
        self.c2 = 0
        self.c3 = 0
        # Initialize Twist message

        #distance between the bot is 0.5m
        d = 0.5 
        
        # self.timer1 = self.create_timer(0.1, self.com_pt1)
        # self.timer2 = self.create_timer(0.1, self.com_pt2)
  
        # Create a listener for keyboard events for com_pt1
        self.keyboard_listener1 = keyboard.Listener(on_press=self.com_pt1)
        self.keyboard_listener1.start()

        # Create a listener for keyboard events for com_pt2
        self.keyboard_listener2 = keyboard.Listener(on_press=self.com_pt2)
        self.keyboard_listener2.start()

        self.keyboard_listener3= keyboard.Listener(on_press=self.com_pt3)
        self.keyboard_listener3.start()


    def com_pt1(self, key):
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        # Publish the Twist message periodically for robot 1
        try:
            # Handle key presses and adjust the Twist message
            if key.char == 'w':
                self.a1=self.a1+1
                self.twist.linear.x = self.a1*0.01
                self.twist.angular.z = self.b1*0.1
                self.cmd_pub1.publish(self.twist)
            elif key.char == 'x':
                # print(self.twist.linear.x , self.a12 )
                self.a1=self.a1-1
                self.twist.linear.x = self.a1*0.01
                self.twist.angular.z = self.b1*0.1
                self.cmd_pub1.publish(self.twist)

            elif key.char == 'a':
                self.b1=self.b1+1
                self.twist.angular.z += self.b1*0.1
                self.twist.linear.x = self.a1*0.01
                self.cmd_pub1.publish(self.twist)
            elif key.char == 'd':
                self.b1=self.b1-1
                self.twist.angular.z += self.b1*0.1
                self.twist.linear.x = self.a1*0.01
                self.cmd_pub1.publish(self.twist)
            elif key.char == 's':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.a1 = 0
                self.a2 = 0
                self.a3 = 0
                self.b1 = 0
                self.b2 = 0
                self.b3 = 0
                self.c = 0
                self.c2 = 0
                self.c3 = 0
                self.cmd_pub1.publish(self.twist)
        except AttributeError:
            pass
        
    
    def com_pt2(self, key):
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        # Publish the Twist message periodically for robot 2
        try:
            # Handle key presses and adjust the Twist message
            if key.char == 'w':
                self.a2=self.a2+1
                self.twist.linear.x += self.a2*0.01
                self.cmd_pub2.publish(self.twist)
                self.twist.angular.z += self.b2*0.1
                self.cmd_pub2.publish(self.twist)

            elif key.char == 'x':
                self.a2=self.a2-1
                self.twist.linear.x += self.a2*0.01
                self.cmd_pub2.publish(self.twist)
                self.twist.angular.z += self.b2*0.1
                self.cmd_pub2.publish(self.twist)

            elif key.char == 'a':
                self.twist.linear.x =  self.a2*0.01
                self.c2=self.c2+1
                self.twist.linear.x += self.c2*0.05
                self.b2=self.b2+1
                self.twist.angular.z += self.b2*0.1
                self.cmd_pub2.publish(self.twist)
  
            elif key.char == 'd':
                self.twist.linear.x =  self.a2*0.01
                self.c2=self.c2-1
                self.twist.linear.x += self.c2*0.05
                self.b2=self.b2-1
                self.twist.angular.z += self.b2*0.1
                self.cmd_pub2.publish(self.twist)

            elif key.char == 's':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.a1 = 0
                self.a2 = 0
                self.a3 = 0
                self.b1 = 0
                self.b2 = 0
                self.b3 = 0
                self.c = 0
                self.c2 = 0
                self.c3 = 0
                self.cmd_pub2.publish(self.twist)
        except AttributeError:
            pass

    def com_pt3(self, key):
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        # Publish the Twist message periodically for robot 2
        try:
            # Handle key presses and adjust the Twist message
            if key.char == 'w':
                self.a3=self.a3+1
                self.twist.linear.x += self.a3*0.01
                self.cmd_pub3.publish(self.twist)
                self.twist.angular.z += self.b3*0.1
                self.cmd_pub3.publish(self.twist)

            elif key.char == 'x':
                self.twist.linear.x -= self.a3*0.01
                self.a3=self.a3-1
                self.cmd_pub3.publish(self.twist)
                self.twist.angular.z += self.b3*0.1
                self.cmd_pub3.publish(self.twist)

            elif key.char == 'a':
                self.twist.linear.x =  self.a3*0.01
                self.c3=self.c3+1
                self.twist.linear.x -= self.c3*0.05
                self.b3=self.b3+1
                self.twist.angular.z += self.b3*0.1
                self.cmd_pub3.publish(self.twist)

            elif key.char == 'd':
                self.twist.linear.x =  self.a3*0.01
                self.c3=self.c3-1
                self.twist.linear.x -= self.c3*0.05
                self.b3=self.b3-1
                self.twist.angular.z += self.b3*0.1
                self.cmd_pub3.publish(self.twist)
                
            elif key.char == 's':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.a1 = 0
                self.a2 = 0
                self.a3 = 0
                self.b1 = 0
                self.b2 = 0
                self.b3 = 0
                self.c = 0
                self.c2 = 0
                self.c3 = 0
                self.cmd_pub3.publish(self.twist)
        except AttributeError:
            pass
        

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_position_control = Turtlebot3PositionControl()
    rclpy.spin(turtlebot3_position_control)

    turtlebot3_position_control.keyboard_listener1.stop()
    turtlebot3_position_control.keyboard_listener2.stop()
    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
