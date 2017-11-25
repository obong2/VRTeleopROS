#include <rosaria/PathName.h>
#include <rosaria/GetWayPoints.h>
#include <rosaria/graph.h>

using namespace std;

class MyP3AT {
public:
    MyP3AT(){};
    void Loop();
    void Init(char*);
    void Terminate();
    void pathWaypointsMessageReceived(const rosaria::PathName& msg, map<string, double> requestedAPs);
    void sonarMessageReceived(const sensor_msgs::PointCloud &msg);
    
    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber sub_points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Subscriber sub_sonar;                      // subscriber to get sonar sensor values
    ros::Publisher pub_cmdvel;                      // publisher to move a robot
    
private:
    double scanWifi(string);                            // Scan near wifi
    void serialSetup(string port);         // Serial Port setup
    void setupAPGraph(); 
    void pathfinding(string, string);

    rosaria::PathName waypoints;                // custom message 
    
    map<string, int> requestedAPs;
    queue<vertex> path;
    
    int mc;                                     // variable for a serial communication with a motor controller
    
    vector<deque<double> > window;     // for moving window average
    vector<double> sonar;
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