#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
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
#include <queue>
#include <math.h>
#include <rosaria/PathName.h>
#include <rosaria/GetWayPoints.h>

using namespace std;
struct vertex {
    typedef pair<int, vertex*> ve;
    vector<ve> adj; //cost of edge, destination vertex
    string name;
    int x;
    int y;
    vertex(string s, int posx, int posy) {
        name = s;
        x = posx;
        y = posy;
    }
};

class Graph{
public:
    typedef map<string, vertex *> vmap;
    vmap work;
    void addvertex(const string&, const int&, const int&);
    void addedge(const string& from, const string& to, double cost);
};

void Graph::addvertex(const string &name, const int &posx, const int &posy)
{
    vmap::iterator itr = work.find(name);
    if (itr == work.end())
    {
        vertex *v;
        v = new vertex(name, posx, posy);
        work[name] = v;
        return;
    }
    cout << "\nVertex already exists!";
}

void Graph::addedge(const string& from, const string& to, double cost)
{
    vertex *f = (work.find(from)->second);
    vertex *t = (work.find(to)->second);
    pair<int, vertex *> edge = make_pair(cost, t);
    f->adj.push_back(edge);
}

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void Init(char*);
    void Terminate();
    void pathWaypointsMessageReceived(const rosaria::PathName& msg, map<string, double> requestedAPs);
    void sonarMessageReceived(const sensor_msgs::PointCloud &msg);
    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Subscriber sonar;                      // subscriber to get sonar sensor values
    ros::Publisher cmdvel;                      // publisher to move a robot
    
private:
    //void scanWifi();                            // Scan near wifi
    void serialSetup(string port);         // Serial Port setup
    void setupAPGraph(); 
    vector<string> pathfinding(string, string);

    rosaria::PathName waypoints;                // custom message 
    
    map<string, int> requestedAPs;
    map<string, double> nearNetworks; // store near wifi with SSID + Signal level
    
    int mc;                                     // variable for a serial communication with a motor controller
    
    vector<deque<double> > window;     // for moving window average
    vector<double> DOA;          // current average

    Graph initialAPs;  
};

namespace patch
{
    template < typename T > string to_string( const T& n )
    {
        ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}