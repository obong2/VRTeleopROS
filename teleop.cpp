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
typedef struct {
    std::string SSID;
    double posX;
    double posY;
    double sigLevel;
}AP;

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void scanWifi();
//private:
    ros::NodeHandle nh;
    ros::Publisher pose, sonar;
    ros::Subscriber points;
    std::map<std::string, double> map;
    double cur_posx, cur_posy;
};

void pathWaypointsMessageReceived(const rosaria::PathName &msg){
    int length = sizeof(msg.points)/sizeof(std::string);
    
    for(int i=0; i<length; i++){
        std::cout<<msg.points[i];
    }
}

int main(int argc, char** argv){
    std::FILE* fp;
    ros::init(argc, argv, "teleop");
    MyP3AT myrobot;

    myrobot.points = myrobot.nh.subscribe("pathwaypoints", 1000, &pathWaypointsMessageReceived);
    myrobot.scanWifi();
    //signal(SIGINT,quit);
    fp = std::fopen("scan.txt", "rw");
    if(fp == NULL){
        std::cout << "File open error" <<std::endl;
        return -1;
    }

    //myrobot.Loop();
    std::fclose(fp);
    return (0);
}

void MyP3AT::Loop(){
    while(true){

    }
}

void MyP3AT::scanWifi(){
    char a;

    a = system("iwlist wlp2s0 scanning | grep 'ESSID\|Signal level' > scan.txt");
}
