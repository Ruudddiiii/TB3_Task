# TB3_Task

In odoo.py we implemented tracking of turtlebot(tb3_2) by a 2nd turtlebot(tb3_1) using only the Odometry data from both turtlebots.

In consenKP.py we implemented 1D (average)consensus of 3 TurtleBots using the Kp controller.

In tra1p.py we applied the Kp controller to track one turtlebot3 by another , it follows a smooth trajectory to catch the other turtlebot, here we also incorporated different values of Kp w.r.t. the distance between the bots, a function that outputs the value of Kp when we give it the value of distance between the bots is what we are typing to build.


