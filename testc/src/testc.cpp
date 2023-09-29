
// Author : Rudresh Singh

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <bits/stdc++.h>
using namespace std;
#define pb push_back
#define mp make_pair
#define mod 1000000007
#define B begin()
#define E end()
#define F first
#define S second
#define pie 3.1415926535897932384626433832795
#define ll  long long

class Turtlebot3PositionControl : public rclcpp::Node {
public:
    Turtlebot3PositionControl() : Node("tpc") {

    // double last_pose_x1_;
    // double last_pose_y1_;
    // double last_pose_theta1_;
    // double last_pose_x2_;
    // double last_pose_y2_;
    // double last_pose_theta2_;
    // double last_pose_x3_;
    // double last_pose_y3_;
    // double last_pose_theta3_;
    // double last_pose_x4_;
    // double last_pose_y4_;
    // double last_pose_theta4_;
    //  double last_pose_x1, last_pose_y1 , last_pose_theta1, last_pose_x2, last_pose_y2 , last_pose_theta2;
        // Initialize subscribers
        odom_sub1 = create_subscription<nav_msgs::msg::Odometry>(
            "tb3_1/odom", 10, std::bind(&Turtlebot3PositionControl::odom_callback1, this, std::placeholders::_1));

     //   Initialize subscribers
        odom_sub2 = create_subscription<nav_msgs::msg::Odometry>(
            "tb3_2/odom", 10, std::bind(&Turtlebot3PositionControl::odom_callback2, this, std::placeholders::_1));
        
        odom_sub3 = create_subscription<nav_msgs::msg::Odometry>(
            "tb3_3/odom", 10, std::bind(&Turtlebot3PositionControl::odom_callback3, this, std::placeholders::_1));

        odom_sub4 = create_subscription<nav_msgs::msg::Odometry>(
            "tb3_4/odom", 10, std::bind(&Turtlebot3PositionControl::odom_callback4, this, std::placeholders::_1));

        // Initialize publishers
        cmd_pub1_ = create_publisher<geometry_msgs::msg::Twist>("tb3_1/cmd_vel", 10);

        // Create timers
        timer3_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Turtlebot3PositionControl::com_pt1, this));

        //Initialize variables
        // last_pose_x1_ = 0.0;
        // last_pose_y1_ = 0.0;
        // last_pose_theta1_ = 0.0;
        // last_pose_x2_ = 0.0;
        // last_pose_y2_ = 0.0;
        // last_pose_theta2_ = 0.0;
        // last_pose_x3_ = 0.0;
        // last_pose_y3_ = 0.0;
        // last_pose_theta3_ = 0.0;
        // last_pose_x4_ = 0.0;
        // last_pose_y4_ = 0.0;
        // last_pose_theta4_ = 0.0;

    }

private:
    void odom_callback1(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Process odom data for turtle 1
        last_pose_x1_ = msg->pose.pose.position.x;
        last_pose_y1_ = msg->pose.pose.position.y;
     //  cout<<last_pose_x1_<<" "<<last_pose_y1_<<"\n";
        // Extract orientation and calculate last_pose_theta1_
        // ...
    }

    void odom_callback2(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Process odom data for turtle 1
        last_pose_x2_ = msg->pose.pose.position.x;
        last_pose_y2_ = msg->pose.pose.position.y;
      //  cout<<last_pose_x2_<<" "<<last_pose_y2_<<"\n";
        // Extract orientation and calculate last_pose_theta1_
        // ...
    }

    void odom_callback3(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Process odom data for turtle 1
        last_pose_x3_ = msg->pose.pose.position.x;
        last_pose_y3_ = msg->pose.pose.position.y;
       // cout<<last_pose_x3_<<" "<<last_pose_y3_<<"\n";

    }

    void odom_callback4(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Process odom data for turtle 1
        last_pose_x4_ = msg->pose.pose.position.x;
        last_pose_y4_ = msg->pose.pose.position.y;
      //  cout<<last_pose_x4_<<" "<<last_pose_y4_<<"\n";

    }

    void com_pt1() {
           // geometry_msgs::msg::Twist twist;
            double x1,y1,x2,y2,x3,y3,x4,y4,i,j,k,d;
            vector<pair<double,double>> v1,v2,v3;
            set<pair<double,double>> se1,se2,se3,se4;
            x1=last_pose_x1_;  // centre of different cirles
            y1=last_pose_y1_;
            x2=last_pose_x2_;
            y2=last_pose_y2_;
            x3=last_pose_x3_;
            y3=last_pose_y3_;
            x4=last_pose_x4_;
            y4=last_pose_y4_;
            cout<<x1<<" "<<y1<<"\n";
            cout<<x2<<" "<<y2<<"\n";
            cout<<x3<<" "<<y3<<"\n";
            cout<<x4<<" "<<y4<<"\n";

            for(i=0;i<=10;i=i+0.1)
            {d=0;

            for(j=-3;j<=3;j=j+0.1)
            {
            for(k=-3;k<=3;k=k+0.1)
            {
            if((j-x1)*(j-x1) + (k-y1)*(k-y1) - i*i < 0)
            {
            se1.insert(mp(j,k));
            }

            }

            }
            for(j=-3;j<=3;j=j+0.1)
            {
            for(k=-3;k<=3;k=k+0.1)
            {
            if((j-x2)*(j-x2) + (k-y2)*(k-y2) - i*i < 0)
            {
            se2.insert(mp(j,k));
            }

            }

            }
            for(j=-3;j<=3;j=j+0.1)
            {
            for(k=-3;k<=3;k=k+0.1)
            {
            if((j-x3)*(j-x3) + (k-y3)*(k-y3) - i*i < 0)
            {
            se3.insert(mp(j,k));
            }

            }

            }
            for(j=-3;j<=3;j=j+0.1)
            {
            for(k=-3;k<=3;k=k+0.1)
            {
            if((j-x4)*(j-x4) + (k-y4)*(k-y4) - i*i < 0)
            {
            se4.insert(mp(j,k));
            }

            }

            }
            for(auto it : se1)
            {
            if(se2.count(it)!=0 && se3.count(it)!=0 && se4.count(it)!=0)
            {
            cout<<"Found the point (x,y) : ";
            cout<<it.F<<" , "<<it.S<<"\n";
            d=9;
            break;
            }

            }
            if(d==9)
            {

            cout<<"Radius is : ";
            cout<<i*i<<"\n";
            break;
            }
            se1.clear();
            se2.clear();
            se3.clear();
            se4.clear();
            d=0;

            }
      //  cmd_pub1_->publish(twist);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub3;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub4;
    rclcpp::TimerBase::SharedPtr timer3_;


    double last_pose_x1_;
    double last_pose_y1_;
    double last_pose_theta1_;
    double last_pose_x2_;
    double last_pose_y2_;
    double last_pose_theta2_;
    double last_pose_x3_;
    double last_pose_y3_;
    double last_pose_theta3_;
    double last_pose_x4_;
    double last_pose_y4_;
    double last_pose_theta4_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtlebot3PositionControl>());
    rclcpp::shutdown();
    return 0;
}
