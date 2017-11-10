#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
//my custom message
#include <rosaria/PathName.h>
//my custom service
#include <rosaria/GetWayPoints.h>

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void scanWifi();
    ros::NodeHandle nh;
    ros::Subscriber points;
    rosaria::PathName msg; 
    std::map<std::string, int> APmap;
    double cur_posx, cur_posy;
};

void pathWaypointsMessageReceived(const rosaria::PathName &msg, std::map<std::string, int> &APmap){
  
    for(int i=0; i<10; i++){
        std::cout<< msg.points[i] <<std::endl;
        APmap.insert(std::pair<std::string, int>(msg.points[i], i));
    }

//    ROS_INFO_STREAM("sibal");
}

void pathfinding(){

}

int main(int argc, char** argv){
    std::FILE* fp;
    ros::init(argc, argv, "teleop");
    
    MyP3AT myrobot;
    myrobot = MyP3AT();
    //myrobot.points = myrobot.nh.subscribe("pathwaypoints", 1000, &pathWaypointsMessageReceived);
    boost::shared_ptr<rosaria::PathName const> sharedPtr;

    sharedPtr = ros::topic::waitForMessage<rosaria::PathName>("pathwaypoints", myrobot.nh);
    if(sharedPtr == NULL){
        ROS_ERROR("Failed to get waypoints!");
        return 1;
    }
    else{
        myrobot.msg = *sharedPtr;
        pathWaypointsMessageReceived(myrobot.msg, myrobot.APmap);
    }

    myrobot.scanWifi();
    
    fp = std::fopen("scan.txt", "rw");
    if(fp == NULL){
        std::cout << "File open error" <<std::endl;
        return -1;
    }

    //myrobot.Loop();
    std::fclose(fp);
    ros::spin();
    return (0);
}

void MyP3AT::Loop(){
    
}

void MyP3AT::scanWifi(){
    char a;

    a = system("iwlist wlp2s0 scanning | grep 'ESSID\|Signal level' > scan.txt");
}
