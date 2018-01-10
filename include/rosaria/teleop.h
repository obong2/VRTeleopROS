#include <rosaria/PathName.h>
#include <rosaria/graph.h>
#include <rosaria/GetWayPoints.h>
using namespace std;

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void Init(char*);
    void Terminate();
    void pathWaypointsMessageReceived(const rosaria::PathName& msg, map<string, double> requestedAPs);
    void sonarMessageReceived(const sensor_msgs::PointCloud &msg);
    void poseMessageReceived(const nav_msgs::Odometry &msg);
public:
    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber sub_points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Subscriber sub_sonar;                      // subscriber to get sonar sensor values
    ros::Subscriber sub_pose;
    ros::Publisher pub_cmdvel;                      // publisher to move a robot
    
//private:
    double scanWifi(string);                            // Scan near wifi
    void serialSetup(string port);         // Serial Port setup
    void setupAPGraph(); 
    void pathfinding(string);

    rosaria::PathName waypoints;                // custom message 
    
    pair<double, double> currentpose;

    pair<string, int> destinationAP;
    
    
    

    
public:
    static deque<vector<double> > window;     // for moving window average
    vector<double> sonar;
    vector<double> DOA;          // current average
    queue<vertex> path;
    Graph initialAPs;
    int mc;                                   // variable for a serial communication with a motor controller
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