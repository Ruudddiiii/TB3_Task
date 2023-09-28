# TB3_Task

In odoo.py we implemented tracking of turtlebot(tb3_2) by a 2nd turtlebot(tb3_1) using only the Odometry data from both turtlebots.

In consenKP.py we implemented 1D (average)consensus of 3 TurtleBots using the Kp controller.

In tra1p.py we applied the Kp controller to track one turtlebot3 by another , it follows a smooth trajectory to catch the other turtlebot, here we also incorporated different values of Kp w.r.t. the distance between the bots, a function that outputs the value of Kp when we give it the value of distance between the bots is what we are typing to build and implement that in the future.

In 2Dconsen.py we are trying to reach a average point of 3 turtlebots that we have spawned. 


In intersect.cpp , we have 3 circles with different centres and they are growing with a same rate(radius), and we are interested in coordinate which lies in all the circles and for that coordinate the radius is smallest possible.

intersectFAST.cpp , we now have 4 circle which are growing. I have optimised the code using set , which has log(n) time complexity of finding pair in itself.


[Screencast from 09-28-2023 07:43:40 PM.webm](https://github.com/Ruudddiiii/TB3_Task/assets/107204888/dc37f253-b985-4658-a635-9c8a4a01bbd7)

Here suppose i have spawned 4 turtlebot in a 3m x 3m space and trying to find the interection point of the circles, for this specific case i have : 
x1=-2.5;  y1=2.5; x2=-0.9; y2=-2.1; x3=2.0; y3=-2.1; x4=2.9; y4=0.3;

and intersection of this circles given by intersectFAST.cpp is 
Found the point (x,y) : -0.3 , 0.1
Radius is : 10.89
