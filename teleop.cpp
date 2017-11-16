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
#include <deque>
//my custom message
#include <rosaria/PathName.h>
//my custom service
#include <rosaria/GetWayPoints.h>

#define WINDOWSIZE 3
#define RANGE 180
#define PI           3.14159265358979323846  /* pi */

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
    //void scanWifi();                            // Scan near wifi
    void serialSetup(std::string port);         // Serial Port setup
    
    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Publisher cmdvel;                      // publisher to move a robot
    rosaria::PathName msg;                      // custom message 
    std::map<std::string, int> requestedAPs;
    std::map<std::string, double> nearNetworks; // store near wifi with SSID + Signal level
    double cur_posx, cur_posy;
    int mc;                                     // variable for a serial communication with a motor controller
    std::vector<std::deque<double> > window;     // for moving window average
    std::vector<double> movingaverage;          // current average
};

void pathWaypointsMessageReceived(const rosaria::PathName &msg, std::map<std::string, int> &requestedAPs){
  
    for(int i=0; i<10; i++){
        std::cout<< msg.points[i] <<std::endl;
        requestedAPs.insert(std::pair<std::string, int>(msg.points[i], i));
    }

}

void pathfinding(){

}

double movingwindowaverage(std::deque<double> &data){
    double sum = 0;
    std::deque<double>::reverse_iterator j;
    std::cout<<"--------------------------"<<std::endl;
    for(j = data.rbegin(); j != data.rend(); j++){
        std::cout<<*j<<std::endl;
        sum = sum + *j;
    }

    data.pop_front();
    
    return sum/WINDOWSIZE;
}

int findstrongestsignal(std::vector<double> arr){
    int minidx;
    double min;
    min = arr[0];
    minidx = 0;
    for(int i=0; i<RANGE; i++){
        //std::cout<< arr[i] <<std::endl;
        if(arr[i] < min){
            min = arr[i];
            minidx = i;
        }
    }
    return minidx;
}

int main(int argc, char** argv){
    std::FILE* fp;
    ros::init(argc, argv, "teleop");
    
    MyP3AT myrobot;
    myrobot = MyP3AT();
    //myrobot.points = myrobot.nh.subscribe("pathwaypoints", 1000, &pathWaypointsMessageReceived);
    myrobot.cmdvel = myrobot.nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
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

    for(int i=0; i<RANGE; i++){
        myrobot.window.push_back(std::deque<double>()); //add a deque
        for(int j=0; j<WINDOWSIZE-1; j++) myrobot.window[i].push_back(0);
        myrobot.movingaverage.push_back(100);
    }

    myrobot.Loop();
    
    close(myrobot.mc);
    ros::spin();
    return (0);
}

void MyP3AT::Loop(){
    std::string cur_waypoint = "";
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
        write(mc, "$A1M2ID1-085", 13);
        system("sleep 5.0");
        total_sig.clear();
        maxidx = 0;
        for(int i=0; i<RANGE; i++){
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
                std::cout << "["<< i <<"]"<< " " << cur_sig <<std::endl;
            }
            //total_sig.push_back(cur_sig);
            //ROS_INFO("%s", "before pushing back window[i]");
            window[i].push_back(cur_sig);
            //ROS_INFO("%s", "after pushing back window[i]");
            movingaverage[i] = movingwindowaverage(window[i]);
            //ROS_INFO("%s", "after finding moving average of window[i]");
        }
        maxidx = findstrongestsignal(movingaverage);
        std::cout << "Current strongest signal is at: " << maxidx << " " << movingaverage[maxidx]<< std::endl;
        msg.linear.x = 1;
        msg.angular.z = -(maxidx-85)*PI/180; // angular.z > 0 : anti-clockwise in radians
        cmdvel.publish(msg);
    }

}
/*
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
*/
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