# TB3_Task

1) In odoo.py we implemented tracking of turtlebot(tb3_2) by a 2nd turtlebot(tb3_1) using only the Odometry data from both turtlebots. 
   

https://github.com/Ruudddiiii/TB3_Task/assets/107204888/543551e3-9b13-406c-980c-95a2b5b662ed


2) In consenKP.py we implemented 1D (average)consensus of 3 TurtleBots using the Kp controller.

3) In tra1p.py we applied the Kp controller to track one turtlebot3 by another , it follows a smooth trajectory to catch the other turtlebot, here we also incorporated different values of Kp w.r.t. the distance between the bots, a function that outputs the value of Kp when we give it the value of distance between the bots is what we are typing to build and implement that in the future.

4) In 2Dconsen.py we are trying to reach a average point of 3 turtlebots that we have spawned. We are using Kp controller.

https://github.com/Ruudddiiii/TB3_Task/assets/107204888/ed6f2b29-702a-4d0e-9e6a-9b73656107b0

[Screencast from 09-21-2023 11 17 49 PM_compressed.webm](https://github.com/Ruudddiiii/TB3_Task/assets/107204888/8e42fa09-6e73-4f82-bd05-e84b04fe936c)



5) In intersect.cpp , we have 3 circles with different centres and they are growing with a same rate(radius), and we are interested in coordinate which lies in all the circles and for that coordinate the radius is smallest possible.

6) intersectFAST.cpp , we now have 4 circle which are growing. I have optimised the code using set , which has log(n) time complexity of finding pair in itself.

7) In min_circle_brute.cpp i have created a random number generator function and it gives the 5 different set of coordinates and for that coordinates the MEC's centre is given as output, Time complexity is O(n^4).

[Screencast from 09-28-2023 07:43:40 PM.webm](https://github.com/Ruudddiiii/TB3_Task/assets/107204888/dc37f253-b985-4658-a635-9c8a4a01bbd7)

Here suppose i have spawned 4 turtlebot in a 6m x 6m space and trying to find the intersection point of the circles, for this specific case i have : 
x1=-2.5;  y1=2.5; x2=-0.9; y2=-2.1; x3=2.0; y3=-2.1; x4=2.9; y4=0.3;

and intersection of this circles given by intersectFAST.cpp is : 
"Found the point (x,y) : -0.3 , 0.1
Radius is : 10.89" . 
In testc package i have integrated the intersectFAST.cpp code with the 4 turtlebot to get their point of interestion of their growing circle, and further using cmd_vel topic we can steer robot to that point(steer is yet to be implemented) .

8) In teleop3TB.py i am trying to control 3 turtlebot3 together using (W,S,D,A,X) keys so that they always stay on a stright line and move together.


https://github.com/Ruudddiiii/TB3_Task/assets/107204888/ab345082-4b1f-47d2-a008-72d89a9d9d08

9) In sect_test.cpp and sect_test.py we have written the raw code for minimum time consensus for multiple mobile robot, which runs in O(n) time and gives accurate estimate of the point as compared to intersectFASt.cpp which was just a brute force technique. Further implementation with multiple turtlebot is yet to be accomplished. (Some error in code -> to be corrected)

10) In mintime.py we have 5 turtlebot3 which are spawned randomly anywhere in a empty world and will try to converge to a point in minimum time using the sect_test output. (Some error in code -> to be corrected)
    
11) In str_l.py we are giving value of m and c of y = mx + c line as input and 5 TBs will reach this line in minimum time.

12) In circle_consen.py we are giving value of (a,b) centre and radius r of circle and the 5 TBs will try to reach there in minimum time.

13) In sect_new_test.py i am using Welzl's algorithm for finding the minimum enclosing circle(MEC) which works in O(n).

14) In hull_brute_min_line.cpp i am trying to find the line on which randomly spawned robots can align in minimum time, it is just a raw code that returns the equation of line in form y = mx + c and the code works in O(n^2).

15) In min_line_5TBs.py i am spawning 5 TBs and calculating the min_line using the Algorithm defined in (14).

16) In min_line_brute.cpp using random number generator i am generating the values of (m,c) , So that we have 10^8 different lines, and using brute force we can check which line is minimum amongst them.

17) In compare_algo_min_line.cpp i am comapring our Algorithm with the brute force Algorithm for minimum time line.


