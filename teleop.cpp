#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
//my custom message
#include <rosaria/PathName.h>
//my custom service
#include <rosaria/GetWayPoints.h>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void scanWifi();                            // Scan near wifi
    void serialSetup(std::string port);         // Serial Port setup
    
    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Publisher cmdvel;                      // publisher to move a robot
    rosaria::PathName msg;                      // custom message 
    std::map<std::string, int> requestedAPs;
    std::map<std::string, double> nearNetworks; // store near wifi with SSID + Signal level
    double cur_posx, cur_posy;
    int mc;                                     // variable for a serial communication with a motor controller
};

void pathWaypointsMessageReceived(const rosaria::PathName &msg, std::map<std::string, int> &requestedAPs){
  
    for(int i=0; i<10; i++){
        std::cout<< msg.points[i] <<std::endl;
        requestedAPs.insert(std::pair<std::string, int>(msg.points[i], i));
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
    myrobot.cmdvel = myrobot.nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    boost::shared_ptr<rosaria::PathName const> sharedPtr;
/*
    sharedPtr = ros::topic::waitForMessage<rosaria::PathName>("pathwaypoints", myrobot.nh);
    
    if(sharedPtr == NULL){
        ROS_ERROR("Failed to get waypoints!");
        return 1;
    }
    else{
        myrobot.msg = *sharedPtr;
        pathWaypointsMessageReceived(myrobot.msg, myrobot.requestedAPs);
    }
*/
    if(argc < 2) {
        ROS_ERROR("%s", "Add port number as a command argument!");
        return (1);
    }
    else{
        myrobot.serialSetup(argv[1]);
    }
    
    ROS_INFO("%s", "Start scanning near APs...");
    //myrobot.scanWifi();
    ROS_INFO("%s", "Done scanning near APs...");
    myrobot.Loop();
    
    close(myrobot.mc);
    ros::spin();
    return (0);
}

void MyP3AT::Loop(){
    std::string cur_waypoint = "Boiler";
    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    msg.linear.x = 0;
    //std::map<std::string, double>::iterator it = nearNetworks.find(cur_waypoint);
    std::string line;
    
    signed int cur_sig;
    std::vector<double> total_sig;
    int maxidx;

    write(mc, "$A1M2ID1-085", 13);
    system("sleep 5.0");
    //Follow the cur waypoint until the robot arrives
    while(1){
        total_sig.clear();
        maxidx = 0;
        for(int i=0; i<180; i++){
            std::string motorcommand = "$A1M2ID1";
            int angle = i - 85;
            if(angle < 0){
                if(angle > -10) motorcommand += "-00" + patch::to_string(abs(angle));
                else if(angle > -100) motorcommand += "-0" + patch::to_string(abs(angle));
                else motorcommand+="-" + patch::to_string(abs(angle));
            }
            else{
                if(angle<10) motorcommand += "+00" + patch::to_string(angle);
                else if(angle<100){
                    motorcommand += "+0" + patch::to_string(angle);
                }
                else{
                    motorcommand += "+" + patch::to_string(angle);
                }
            }
            
            //std::cout<<motorcommand<<std::endl;
            write(mc, motorcommand.c_str(), motorcommand.length());
            system("sleep 0.01");

            system("iwconfig wlx00c0ca577641 | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt");
            std::ifstream tempfile("/home/yeonju/catkin_ws/sig_temp.txt");
            while(!tempfile.eof()){
                std::getline(tempfile, line, '\n');
                std::stringstream convertor(line);
                convertor >> cur_sig;
                //std::cout << "["<< i <<"]"<< " " << line << " " << cur_sig <<std::endl;
            }
            total_sig.push_back(cur_sig);
            if(cur_sig < total_sig[maxidx]) maxidx = i;
        }
        msg.linear.x = 1;
        msg.angular.z = (maxidx-85)/100;
        cmdvel.publish(msg);
    }

    //msg.angular.z = 0;
    //msg.linear.x = 0;
    //cmdvel.publish(msg);
}

void MyP3AT::scanWifi(){
    char a; //system call handler
    
    std::string line, line2;
    std::string ESSID;
    double siglevel;
    std::size_t found, end;
    
    //TODO: Replace your password and wireless adapter name with yours...
    a = system("echo iuer9895 | sudo -S iwlist wlx00c0ca577641 scanning | (grep ESSID) > ssid.txt");
    a = system("echo iuer9895 | sudo -S iwlist wlx00c0ca577641 scanning | (grep 'Signal') > signal.txt");
    
    //TODO: Change the location of files to your own paths.
    std::ifstream fp("/home/yeonju/catkin_ws/ssid.txt");
    std::ifstream fd("/home/yeonju/catkin_ws/signal.txt");
    
    if(fp == NULL){
        std::cout << "[FP]File open error" <<std::endl;
        return;
    }
    if(fd == NULL){
        std::cout << "[FD]File open error" <<std::endl;
        return;
    }

    while(std::getline(fp,line) && std::getline(fd, line2)){
        //std::cout<<line<<std::endl;
        
        found = line2.find("Signal level=");
        
        if(found!=std::string::npos){
            std::stringstream convertor(line2.substr(found+13,3));
            convertor >> siglevel;
        }
        
        found = line.find("ESSID:\"");
        //std::remove(line.begin(), line.end(), ' ');
        end = line.find("\"");
    //std::cout<<found << " " <<end << std::endl;
        if(found!=std::string::npos && end!=std::string::npos){
            std::stringstream convertor2(line.substr(found+7, end-found));
            convertor2 >> ESSID;
        }
        //std::cout<<ESSID <<"   "<<siglevel<<std::endl;
        if(nearNetworks.find(ESSID) == nearNetworks.end()){ //register new AP
            std::cout << "New near network registered: " << ESSID <<std::endl;
             nearNetworks.insert(std::pair<std::string, double>(ESSID, siglevel));
        }
    }

    return;
}

void MyP3AT::serialSetup(std::string port){
    struct termios newtio;
    std::string cmd = "stty -F " + port + " 57600";
    
    system(cmd.c_str());
    mc = open(port.c_str(), O_RDWR | O_NOCTTY);
  
    newtio.c_cflag = B57600;
    newtio.c_cflag |= CS8;
    newtio.c_cflag |= CLOCAL;
    newtio.c_cflag |= CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
  
    tcflush (mc, TCIFLUSH);
    tcsetattr(mc, TCSANOW, &newtio);

    ROS_INFO("%s", "Serial Port is opend!");
}