# TB3_Task

1) In odoo.py we implemented tracking of turtlebot(tb3_2) by a 2nd turtlebot(tb3_1) using only the Odometry data from both turtlebots.

2) In consenKP.py we implemented 1D (average)consensus of 3 TurtleBots using the Kp controller.

3) In tra1p.py we applied the Kp controller to track one turtlebot3 by another , it follows a smooth trajectory to catch the other turtlebot, here we also incorporated different values of Kp w.r.t. the distance between the bots, a function that outputs the value of Kp when we give it the value of distance between the bots is what we are typing to build and implement that in the future.

4) In 2Dconsen.py we are trying to reach a average point of 3 turtlebots that we have spawned. 


5) In intersect.cpp , we have 3 circles with different centres and they are growing with a same rate(radius), and we are interested in coordinate which lies in all the circles and for that coordinate the radius is smallest possible.

6) intersectFAST.cpp , we now have 4 circle which are growing. I have optimised the code using set , which has log(n) time complexity of finding pair in itself.

[Screencast from 09-28-2023 07:43:40 PM.webm](https://github.com/Ruudddiiii/TB3_Task/assets/107204888/dc37f253-b985-4658-a635-9c8a4a01bbd7)

Here suppose i have spawned 4 turtlebot in a 6m x 6m space and trying to find the intersection point of the circles, for this specific case i have : 
x1=-2.5;  y1=2.5; x2=-0.9; y2=-2.1; x3=2.0; y3=-2.1; x4=2.9; y4=0.3;

and intersection of this circles given by intersectFAST.cpp is : 
"Found the point (x,y) : -0.3 , 0.1
Radius is : 10.89" . 
In testc package i have integrated the intersectFAST.cpp code with the 4 turtlebot to get their point of interestion of their growing circle, and further using cmd_vel topic we can steer robot to that point(steer is yet to be implemented) .

7) In teleop3TB.py i am trying to control 3 turtlebot3 together using (W,S,D,A,X) keys so that they always stay on a stright line and move together everywhere.


https://github.com/Ruudddiiii/TB3_Task/assets/107204888/ab345082-4b1f-47d2-a008-72d89a9d9d08

