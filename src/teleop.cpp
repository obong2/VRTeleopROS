#include <rosaria/teleop.h>
#include <rosaria/RSSImeasure.h>

#define WINDOWSIZE 3
#define RANGE      180
#define PI         3.14159265358979323846  /* pi */
#define kP         1                       //P gain
#define kD         0.3                     //D gain
#define ALPHA      0.5                     //smoothing parameter
#define SIGTHD     3                       // Threshold
#define SAFEZONE   2                       // Sonar threshold
#define INF        99999                          // Initial edge cost
#define ALPHA2     0.5

//static const std::string WADAPTER = "wlx00c0ca590adb";
//static const std::string WADAPTER = "wlan0";

double movingwindowaverage(std::deque<double> &data){
    double sum = 0;
    std::deque<double>::iterator j;
    
    std::cout<<"--------------------------"<<std::endl;
    for(j = data.begin(); j != data.end(); j++){
        std::cout<<*j<<std::endl;
        //raw.push_back(*j);
        sum = sum + *j;
    }
    return sum/WINDOWSIZE;
}

double movingwindowvariance(std::deque<double> &data, double avg){
    double sum = 0;

    std::deque<double>::iterator j;
    
    //std::cout<<"--------------------------"<<std::endl;
    for(j = data.begin(); j != data.end(); j++){
        sum = sum + pow((*j-avg),2);
    }

    return sum/WINDOWSIZE;
}

int findstrongestsignal(std::vector<double> arr){
    int minidx;
    double min;
    min = arr[RANGE-1];
    minidx = RANGE-1;
    for(int i=RANGE-1; i>=0; i--){
        //std::cout<< arr[i] <<std::endl;
        if(arr[i] < min){
            min = arr[i];
            minidx = i;
        }
    }
    return minidx;
}

int sensorfusion(int cur_max, std::vector<double> sonar_sensor){
    int i=cur_max;
    int j=cur_max;
    int result = -1;
    while(i>=0 || j<RANGE){
        //std::cout<<sonar_sensor[i]<<" " << sonar_sensor[j] <<std::endl;
        if(i>=0 && sonar_sensor[i] <= SAFEZONE) {
            result = i;
            std::cout<<"HERE1: " <<result<<std::endl;
            break;
        }
        if(j<RANGE && sonar_sensor[j] <= SAFEZONE) {
            result = j;
            std::cout<<"HERE2: " <<result<<std::endl;
            break;
        }
        i--;
        j++;
    }

    return result; //if we cannot find any possible idx
}

void connectap(std::string id, std::string password){
    int a;

    //std::string cmd_passphrase = "sudo wpa_passphrase " + id + " \"" + password + "\" " + "> /home/yeonju/catkin_ws/wireless.conf";
    //std::string cmd_supplicant = "sudo wpa_supplicant -B -c /home/yeonju/catkin_ws/wireless.conf -i " + WADAPTER;
    std::string cmd_simple = "sudo iwconfig " + WADAPTER + " essid " + id;
    //std::string cmd_supplicant = "sudo wpa_supplicant -B -Dwext -i "+ WADAPTER +" -c /home/yeonju/catkin_ws/wireless.conf";
    
    /*
    a = system(cmd_passphrase.c_str());
    if(a == -1){
        ROS_ERROR("%s", "Fail to create passphrase!");
        return;
    }
    */
    a = system(cmd_simple.c_str());
    
    system("sleep 3.0");
    ROS_INFO("%s", "Connected to AP...");
}

void disconnectap(){
    std::string tempcmd = "sudo iwconfig " + WADAPTER + " down";
    system(tempcmd.c_str());
    tempcmd = "sudo iwconfig " + WADAPTER + " up";
    system(tempcmd.c_str());
}

void MyP3AT::poseMessageReceived(const nav_msgs::Odometry &msg){
    std::cout<< "Pose: x = " << msg.pose.pose.position.x << "y = " << msg.pose.pose.position.y <<std::endl;
    currentpose.first = msg.pose.pose.position.x;
    currentpose.second = msg.pose.pose.position.y;
}
//TODO
void MyP3AT::pathWaypointsMessageReceived(const rosaria::PathName &msg, std::map<std::string, double> requestedAPs){
    
      for(int i=0; i<10; i++){
          std::cout<< msg.points[i] <<std::endl;
          requestedAPs.insert(std::pair<std::string, int>(msg.points[i], i));
      }

}

void MyP3AT::sonarMessageReceived(const sensor_msgs::PointCloud &msg){
    double data_prev, data_next;
    for(int i=0; i<8; i++){
        if(msg.points[i].x == 0 && msg.points[i].y == 0) continue; //ignore useless data
        else{
            //std::cout<<msg.points[i].x<< "  " << msg.points[i].y << "  " << msg.points[i].z <<std::endl;
            data_prev = sqrt(pow(msg.points[i].x, 2) + pow(msg.points[i].y, 2));
            data_next = sqrt(pow(msg.points[i+1].x, 2) + pow(msg.points[i+1].y, 2));
            //linear interpolation...
            double frag = (data_next - data_prev)/23;
            for(int j=0; j<23; j++){
                sonar[i*23 + j] = data_prev + j*frag;
                //std::cout<<i*23 + j << ": " << sonar[i*23+j]<<std::endl;
            }
        }
    }
    std::reverse(sonar.begin(), sonar.end());
}

void MyP3AT::pathfinding(std::string to){
    initialAPs.pathfindingFloyd();
    string from = initialAPs.closestNode(currentpose.first, currentpose.second);
    path = initialAPs.returnPath(from, to);
}

void MyP3AT::Init(char * argv){
    serialSetup(argv);
    
    sub_sonar = nh.subscribe("RosAria/sonar", 1, &MyP3AT::sonarMessageReceived, this);
    sub_pose = nh.subscribe("RosAria/pose", 1, &MyP3AT::poseMessageReceived, this);
    pub_cmdvel = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
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
    currentpose.first = 0; currentpose.second = 0;

    pathfinding("SMARTAP3");
    /*
    for(int i=0; i<RANGE; i++){
        window.push_back(std::deque<double>()); //add a deque
        for(int j=0; j<WINDOWSIZE-1; j++) window[i].push_back(0);
        //DOA.push_back(100);
    }
*/
}

void MyP3AT::Terminate(){
    close(mc);
    //ros::spin();
}

void MyP3AT::Loop(){
    int maxidx;
    double cur_doa = 100;

    geometry_msgs::Twist msg;
      
    msg.angular.z = 0;
    msg.linear.x = 0;
           
    //Follow the cur waypoint until the robot arrives
    while(cur_doa > SIGTHD){
        write(mc, "$A1M2ID1+090", 13);
        threadRSSI th;
        th.run();
        std::this_thread::sleep_for(chrono::seconds(1));
        th.condition.store(false);
        std::this_thread::sleep_for(chrono::seconds(1));
        /*    
            
            window[i].push_back(cur_sig);

            // Find DOA in 180 degree range
            double avg_temp, var_temp;
            avg_temp = movingwindowaverage(window[i]);
            var_temp = movingwindowvariance(window[i],avg_temp);
            window[i].pop_front();
            DOA[i] = ALPHA*var_temp + (1-ALPHA)*avg_temp;
        */
        ros::spinOnce(); //recei        ve sonar sensor msg
        
        maxidx = findstrongestsignal(DOA);
        //maxidx = sensorfusion(maxidx, sonar);
            
        if(maxidx == -1) { //based on sonar sensor, if there is no available direction...
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub_cmdvel.publish(msg);

            system("sleep 0.5");
            write(mc, "$A1M2ID1-090", 13);
            continue;
        }
        system("sleep 0.5");
        write(mc, "$A1M2ID1-090", 13);
        
        std::cout << "Current strongest signal is at: " << maxidx << " " << DOA[maxidx]<< std::endl;
        
        //TODO: PID Control
        msg.linear.x = 1; // fixed linear velocity
        msg.angular.z = -(maxidx-85)*PI/180; // angular.z > 0 : anti-clockwise in radians
        this->pub_cmdvel.publish(msg);

        system("sleep 0.5");
    }

/*
        std::string tempcmd = "sudo ifconfig " + WADAPTER + " down";
        system(tempcmd.c_str());
        tempcmd = "sudo pkill -9 wpa_supplicant";
        system(tempcmd.c_str());
        tempcmd = "sudo ifconfig " + WADAPTER + " up";
        system(tempcmd.c_str());
        */
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

    //change speed
    write(mc, "$A1M3ID1-400", 13);
    
}

void MyP3AT::setupAPGraph(){
    std::string line;
    std::string name;
    std::string password;
    int posx, posy;

    //Read vertex file
    std::ifstream aplistfile("/home/yeonju/catkin_ws/src/ARTeleOpROS/aplist.txt");
    while(!aplistfile.eof()){
        std::getline(aplistfile, line, '\n');
        std::stringstream convertor(line);
        convertor >> name >> posx >> posy >> password;
        initialAPs.addvertex(name, password, posx, posy);
    }
    //Initializing the floyd 2x2 vector
    for(int i=0; i<initialAPs.count; i++){
        vector<double> element(initialAPs.count);
        vector<int> element2(initialAPs.count);
        initialAPs.floyd.push_back(element);
        initialAPs.path.push_back(element2);
    }
    //std::cout<<"Test2"<<std::endl;
    for(int i=0; i<initialAPs.count; i++){
        for(int j=0; j<initialAPs.count; j++){
            initialAPs.floyd[i][j] = INF;
            initialAPs.path[i][j] = 0;
        }
    }
    //std::cout<<"Test3"<<std::endl;
    //Read edge file
    int from;
    int to;
    double cost;
    std::ifstream edgefile("/home/yeonju/catkin_ws/src/ARTeleOpROS/edge.txt");
    while(!edgefile.eof()){
        std::getline(edgefile, line, '\n');
        std::stringstream convertor(line);
        convertor >> from >> to >> cost;
        initialAPs.addedge(from, to, cost);
    }
//std::cout<<"Test4"<<std::endl;
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

    while(!myrobot.path.empty()){
        int maxidx;

        write(myrobot.mc, "$A1M2ID1-090", 13); // move antenna to the most left side
        system("sleep 1.0");

        // Establish new connection
        vertex cur_waypoint = myrobot.path.front();
        connectap(cur_waypoint.name, cur_waypoint.password);
        // Re-initialize local variables
        maxidx = 0;
        myrobot.DOA.clear();
        myrobot.sonar.clear();
        /*
        for(int i=0; i<RANGE; i++){
            myrobot.DOA.push_back(100);
            myrobot.sonar.push_back(0);
            myrobot.window[i].clear();
            for(int j=0; j<WINDOWSIZE-1; j++) myrobot.window[i].push_back(0);
        }*/

        myrobot.Loop();

        disconnectap();
        myrobot.path.pop();
    }
    
    ROS_INFO("%s", "Your robot finished the navigation.");

    myrobot.Terminate();
    return (0);
}