#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ROSARIA/BumperState.h"
#include "sensor_msgs/Imu.h"
#include <signal.h>
#include <termios.h>

using geometry_msgs::Twist;
using namespace std;

ros::Publisher chatter_pub;
ros::Time t1;
Twist vel;
int kfd = 0;
struct termios cooked, raw;
unsigned int temp=0;
float x;
float y;
float z;

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// Callback function for pose message
void poseMessageReceived(const nav_msgs::Odometry& msg){
    std::cout << std::setprecision(2) << std::fixed << /* output the pose information using standard output */
    "Current position=(" << msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ") " << 
    "Current direction=" << std::setprecision(2) << std::fixed << msg.pose.pose.orientation.w<<"\r";
    std::flush(std::cout);
}

int main(int argc, char** argv){
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "ARteleop"); ros::NodeHandle nh;

    //Create subscriber objects
    ros::Subscriber pose, bumper_state, battery_state_of_charge, battery_voltage, battery_charge_state, motors_state;
    pose = nh.subscriber("RosAria/pose", 1000, &poseMessageReceived);
    
    //Create publisher objects
    ros::Publisher pub;

    //Add loop
    Update();

    return (0);
}

void Update(){
    while(ros::ok()){
        //system call
        //system("rosrun rosaria_client ~~~") or add wifiscanning systemcall
    }
}