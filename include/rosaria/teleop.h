#include <rosaria/PathName.h>
//#include <rosaria/Measurements.h>
#include <rosaria/GetWayPoints.h>
#include <rosaria/graph.h>
#include <atomic>
#include <thread>
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

    ros::NodeHandle nh;                         // Node handler
    ros::Subscriber sub_points;                     // subscriber to get waypoints from COM2 with PathName type message
    ros::Subscriber sub_sonar;                      // subscriber to get sonar sensor values
    ros::Subscriber sub_pose;
    ros::Publisher pub_cmdvel;                      // publisher to move a robot
    
private:
    double scanWifi(string);                            // Scan near wifi
    void serialSetup(string port);         // Serial Port setup
    void setupAPGraph(); 
    void pathfinding(string);

    rosaria::PathName waypoints;                // custom message 
    
    pair<double, double> currentpose;

    pair<string, int> destinationAP;
    
    queue<vertex> path;
    
    int mc;                                     // variable for a serial communication with a motor controller
    
    vector<deque<double> > window;     // for moving window average
    vector<double> sonar;
    vector<double> DOA;          // current average

    Graph initialAPs;  
};

class threadRSSI{
public:
    threadRSSI(){
        condition.store(true);
    }
    ~threadRSSI(){
        condition.store(false);
        if(tid.joinable()){
            tid.join();
        }
    }
    void run(){
        tid = std::thread(std::bind(&threadRSSI::measure, this));
    }

private:
    void measure(){
        int cur_sig;
        string line;

        string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";
        //cout << "Thread RSSI has started" <<endl;
        while(condition){
            system(cmd.c_str());
            this_thread::sleep_for(chrono::seconds(0.1));
            ifstream tempfile("/home/yeonju/catkin_ws/sig_temp.txt");
            while(!tempfile.eof()){
                std::getline(tempfile, line, '\n');
                std::stringstream convertor(line);
                convertor >> cur_sig;
                cout << cur_sig << endl;
            }

            
        }

        //cout << "Thread RSSI has terminated" <<endl;
    }

private:
    thread tid;

public:
    atomic_bool condition;
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