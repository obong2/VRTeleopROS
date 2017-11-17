#include <rosaria/teleop.h>

#define WINDOWSIZE 3
#define RANGE      180
#define PI         3.14159265358979323846  /* pi */
#define kP         1                       //P gain
#define kD         0.3                     //D gain
#define ALPHA      0.7                     //smoothing parameter
#define SIGTHD     10                      // Threshold
#define SONARTHD   4                       // Sonar safe zone threshold (m)

double movingwindowaverage(std::deque<double> &data){
    double sum = 0;
    std::deque<double>::reverse_iterator j;
    //std::cout<<"--------------------------"<<std::endl;
    for(j = data.rbegin(); j != data.rend(); j++){
        //std::cout<<*j<<std::endl;
        sum = sum + *j;
    }
    return sum/WINDOWSIZE;
}

double movingwindowvariance(std::deque<double> &data, double avg){
    double sum = 0;

    std::deque<double>::reverse_iterator j;
    //std::cout<<"--------------------------"<<std::endl;
    for(j = data.rbegin(); j != data.rend(); j++){
        //std::cout<<*j<<std::endl;
        sum = sum + pow((*j-avg),2);
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

//TODO
void MyP3AT::pathWaypointsMessageReceived(const rosaria::PathName &msg, std::map<std::string, double> requestedAPs){
    
      for(int i=0; i<10; i++){
          std::cout<< msg.points[i] <<std::endl;
          requestedAPs.insert(std::pair<std::string, int>(msg.points[i], i));
      }

      

}

//TODO
void MyP3AT::sonarMessageReceived(const sensor_msgs::PointCloud &msg){
    for(int i=0; i<msg.points.size(); i++){
        std::cout<<msg.points[i].x<< "  " << msg.points[i].y << "  " << msg.points[i].z <<std::endl;
    }
}
//TODO: Which Algorithm?
std::vector<std::string> MyP3AT::pathfinding(std::string from, std::string to){
    //Find suboptimal paths and concatenate all together... (recursive???? hmm..m.m.m.m.m.m..)
    Graph::vmap::iterator it_from = initialAPs.work.find(from);
    Graph::vmap::iterator it_to = initialAPs.work.find(to);
    priority_queue<vertex, double> pq;
    std::vector<std::string> result;
    if(it_from != initialAPs.work.end() && it_to != initialAPs.work.end()){
        
    }
    return result;
}

void MyP3AT::Init(char * argv){
    serialSetup(argv);
    
    sonar = nh.subscribe("RosAria/sonar", 1, &MyP3AT::sonarMessageReceived, this);
    cmdvel = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
    boost::shared_ptr<rosaria::PathName const> sharedPtr;

    initialAPs = Graph();
    setupAPGraph();
    /*
    sharedPtr = ros::topic::waitForMessage<rosaria::PathName>("pathwaypoints", nh);
    
    if(sharedPtr == NULL){
        ROS_ERROR("Failed to get waypoints!");
        return 1;
    }
    else{
        waypoints = *sharedPtr;
        pathWaypointsMessageReceived(waypoints, requestedAPs);
    }
    */

    for(int i=0; i<RANGE; i++){
        window.push_back(std::deque<double>()); //add a deque
        for(int j=0; j<WINDOWSIZE-1; j++) window[i].push_back(0);
        DOA.push_back(100);
    }
}

void MyP3AT::Terminate(){
    close(mc);
    //ros::spin();
}

void MyP3AT::Loop(){
    
    std::string cur_waypoint = "";
    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    msg.linear.x = 0;
    //std::map<std::string, double>::iterator it = nearNetworks.find(cur_waypoint);
    std::string line;
    
    signed int cur_sig;
    int maxidx;

    write(mc, "$A1M2ID1-085", 13);
    system("sleep 2.0");
    //Follow the cur waypoint until the robot arrives
    while(1){ //TODO: Change the threshold...
        write(mc, "$A1M2ID1-085", 13);
        system("sleep 5.0");
        
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
                //std::cout << "["<< i <<"]"<< " " << cur_sig <<std::endl;
            }
            
            window[i].push_back(cur_sig);

            // Find DOA in 180 degree range
            double avg_temp, var_temp;
            avg_temp = movingwindowaverage(window[i]);
            var_temp = movingwindowvariance(window[i],avg_temp);
            DOA[i] = ALPHA*var_temp + (1-ALPHA)*avg_temp;
        }
        ros::spinOnce();
        maxidx = findstrongestsignal(DOA);
        std::cout << "Current strongest signal is at: " << maxidx << " " << DOA[maxidx]<< std::endl;
        //TODO: PID Control
        msg.linear.x = 1; // fixed linear velocity
        msg.angular.z = -(maxidx-85)*PI/180; // angular.z > 0 : anti-clockwise in radians
        //cmdvel.publish(msg);
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

void MyP3AT::setupAPGraph(){
    std::string line;
    std::string name;
    int posx, posy;

    std::ifstream aplistfile("/home/yeonju/catkin_ws/src/ARTeleOpROS/aplist.txt");
    while(!aplistfile.eof()){
        std::getline(aplistfile, line, '\n');
        std::stringstream convertor(line);
        convertor >> name >> posx >> posy;
        initialAPs.addvertex(name, posx, posy);
    }

    std::string from;
    std::string to;
    double cost;
    std::ifstream edgefile("/home/yeonju/catkin_ws/src/ARTeleOpROS/edge.txt");
    while(!edgefile.eof()){
        std::getline(edgefile, line, '\n');
        std::stringstream convertor(line);
        convertor >> from >> to >> cost;
        initialAPs.addedge(from, to, cost);
    }
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "teleop");
    
    MyP3AT myrobot;
    myrobot = MyP3AT();
    
    //myrobot.points = myrobot.nh.subscribe("pathwaypoints", 1000, &pathWaypointsMessageReceived);

    if(argc < 2) {
        ROS_ERROR("%s", "Add port number as a command argument!");
        return (1);
    }
    else{
        myrobot.Init(argv[1]);
        ROS_INFO("%s", "Initialization is done!");
    }
 
    myrobot.Loop();    
    
    myrobot.Terminate();
    return (0);
}