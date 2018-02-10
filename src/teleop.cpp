//#include <rosaria/teleop.h>
#include <rosaria/RSSImeasure.h>

#define PI         3.14159265358979323846  /* pi */
#define kP         1                       //P gain
#define kD         0.3                     //D gain
#define ALPHA      0.5                     //smoothing parameter
#define SIGTHD     3                       // Threshold
#define SAFEZONE   4                       // Sonar threshold
#define INF        99999                          // Initial edge cost

static const bool isTesting = false;
vector<vector<double> > MyP3AT::window;

void pop_front(){
    assert(!MyP3AT::window.empty());
    MyP3AT::window.erase(MyP3AT::window.begin());
}

vector<double> movingwindowaverage(){
    double sum = 0;
    int size = MyP3AT::window[0].size();
    //vector<double>::iterator j;
    //vector<vector<double> >::iterator i;
    int i, j;
    vector<double> result;
    //cout<<"--------------------------"<< MyP3AT::window[0].size() << "    "<< MyP3AT::window.size() <<endl;
    for(i=0; i<size; i++){
        sum = 0;
        for(j=0; j<MyP3AT::window.size(); j++){
            sum+= MyP3AT::window[j][i];
        }
        result.push_back(sum/size);
    }
    
    return result;
}

vector<double> movingwindowvariance(vector<double> avg){
    double sum = 0;
    int size = MyP3AT::window[0].size();
    int k;
    int i, j;
    vector<double> result;
    
    //cout<<"--------------------------"<< size << endl;
    
    for(i=0,k=0; i<size; i++, k++){
        sum = 0;
        for(j=0;j<MyP3AT::window.size(); j++){
            sum+= pow((MyP3AT::window[j][i]-avg[k]),2);
        }
        result.push_back(sum/size);
    }
    //cout<<"--------------------------"<< endl;
    return result;
}

int findstrongestsignal(vector<double> arr){
    int minidx;
    double min;
    //vector<double>::iterator j;
    min = arr[0];
    minidx = 0;
    for(int i=1; i < arr.size(); i++){
        //std::cout<< arr[i] <<std::endl;
        if(arr[i] < min){
            min = arr[i];
            minidx = i;
        }
    }
    return minidx;
}

int sensorfusion(int cur_max, vector<double> sonar_sensor){
    int i=cur_max;
    int j=cur_max;
    int result = -1;
    
    while(i>=0 || j<MyP3AT::window[0].size()){
        cout<<sonar_sensor[i]<<" " << sonar_sensor[j] <<endl;
        if(i>=0 && sonar_sensor[i] >= SAFEZONE) {
            result = i;
            cout<<"HERE1: " <<result<<endl;
            break;
        }
        if(j<MyP3AT::window[0].size() && sonar_sensor[j] >= SAFEZONE) {
            result = j;
            cout<<"HERE2: " <<result<<endl;
            break;
        }
        i--;
        j++;
    }
    
    return result; //if we cannot find any possible idx
}

void connectap(string id, string password){
    int a;

    //string cmd_passphrase = "sudo wpa_passphrase " + id + " \"" + password + "\" " + "> /home/yeonju/catkin_ws/wireless.conf";
    //string cmd_supplicant = "sudo wpa_supplicant -B -c /home/yeonju/catkin_ws/wireless.conf -i " + WADAPTER;
    string cmd_simple = "sudo iwconfig " + WADAPTER + " essid " + id;
    //string cmd_supplicant = "sudo wpa_supplicant -B -Dwext -i "+ WADAPTER +" -c /home/yeonju/catkin_ws/wireless.conf";
    
    /*
    a = system(cmd_passphrase.c_str());
    if(a == -1){
        ROS_ERROR("%s", "Fail to create passphrase!");
        return;
    }
    */
    a = system(cmd_simple.c_str());
    
    cmd_simple = "sudo ifconfig " + WADAPTER + " down";
    system(cmd_simple.c_str());
    cmd_simple = "sudo ifconfig " + WADAPTER + " up";
    system(cmd_simple.c_str());

    system("sleep 3.0");
    ROS_INFO("%s", "Connected to AP...");
}

void disconnectap(){
    string tempcmd = "sudo ifconfig " + WADAPTER + " down";
    system(tempcmd.c_str());
    tempcmd = "sudo ifconfig " + WADAPTER + " up";
    system(tempcmd.c_str());
}

void MyP3AT::poseMessageReceived(const nav_msgs::Odometry &msg){
    cout<< "Pose: x = " << msg.pose.pose.position.x << "y = " << msg.pose.pose.position.y <<endl;
    currentpose.first = msg.pose.pose.position.x;
    currentpose.second = msg.pose.pose.position.y;

    ofstream POSErecordfile("/home/yeonju/catkin_ws/poserecord.txt", ios::app);
    //ostream_iterator<double> sonaroutput_iterator(POSErecordfile, "\t");

    POSErecordfile << currentpose.first << "\t" << currentpose.second << "\n";
    POSErecordfile.flush();

    POSErecordfile.close();
}

void MyP3AT::pathWaypointsMessageReceived(const std_msgs::String &msg, pair<string, int> destinationAP){
    cout<< msg.data <<endl;
    //destinationAP.first = msg.data;
    //destinationAP.second = 1;
    //pair <string, int> test;
    //test = make_pair(msg.data, 1);
    //destinationAP = test;
    pathfinding(msg.data);
}

void MyP3AT::movingMessageReceived(const std_msgs::Bool &msg){
    isMoving = msg.data;
}

void MyP3AT::sonarMessageReceived(const sensor_msgs::PointCloud &msg){
    double data_prev, data_next;
    ofstream SONARrecordfile("/home/yeonju/catkin_ws/sonarmeasure.txt", ios::app);
    ostream_iterator<double> sonaroutput_iterator(SONARrecordfile, "\t");

    sonar.clear();
    sonar_new.clear();
    
    //sonar_new.reserve(20);

    for(int i=1; i<9; i++){
        if(msg.points[i-1].x == 0 && msg.points[i-1].y == 0) continue; //ignore useless data
        else{
            
            data_prev = sqrt(pow(msg.points[i-1].x, 2) + pow(msg.points[i-1].y, 2));
            data_next = sqrt(pow(msg.points[i].x, 2) + pow(msg.points[i].y, 2));
            //cout<<sqrt(pow(msg.points[i-1].x, 2) + pow(msg.points[i-1].y, 2))<<endl;
            //linear interpolation...
            double frag = (data_next - data_prev)/22.5;
            for(int j=0; j<23; j++){
                //sonar[i*23 + j] = data_prev + j*frag;
                sonar.push_back(data_prev + j*frag);
                //cout<<i*23 + j << ": " << sonar[i*23+j]<<endl;
            }
        }
    }
    //cout<< "SONAR SIZE ============ "<< sonar.size() << "LAST : " << sonar[sonar.size()-1]<< endl;
    //reverse(sonar.begin(), sonar.end());
    for(int i=0; i<MyP3AT::window[0].size(); i++){
        sonar_new.push_back(sonar[i*5]);
        //cout<< "sonar_new["<<i<<"] " <<sonar_new[i] <<endl;
    }
    copy(sonar_new.begin(), sonar_new.end(), sonaroutput_iterator);
    SONARrecordfile << '\n';
    SONARrecordfile.close();
}

void MyP3AT::pathfinding(string to){
    initialAPs.pathfindingFloyd();
    //cout << currentpose.first << "\t" << currentpose.second << endl;
    string from = initialAPs.closestNode(currentpose.first, currentpose.second);
    //cout<< "STARTFROM: " << from <<endl;
    path = initialAPs.returnPath(from, to);
}

void MyP3AT::Init(char * argv){
    serialSetup(argv);
    //sub_dest = nh.subscribe("/pathwaypoints", 1000, &pathWaypointsMessageReceived, this);
    sub_sonar = nh.subscribe("/RosAria/sonar", 1, &MyP3AT::sonarMessageReceived, this);
    sub_pose = nh.subscribe("/RosAria/pose", 1, &MyP3AT::poseMessageReceived, this);
    //sub_isMoving = nh.subscribe("/isMoving", 1, &MyP3AT::movingMessageReceived, this);
    pub_cmdvel = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
    boost::shared_ptr<std_msgs::String const> sharedPtr;

    initialAPs = Graph();
    
    setupAPGraph();
    
    if(isTesting){
        sharedPtr = ros::topic::waitForMessage<std_msgs::String>("/pathwaypoints", nh);
        ROS_INFO("%s", "Waiting for the destination input...");
        if(sharedPtr == NULL){
            ROS_ERROR("Failed to get waypoints!");
            return;
        }
        else{
            waypoints = *sharedPtr;
            pathWaypointsMessageReceived(waypoints, destinationAP);
            //cout<<"cur dest: " << destinationAP.first << endl;
            //pathfinding(destinationAP.first);
        }
    }
    else{
        pathfinding("SMARTAP1");
    }
    currentpose.first = 0; currentpose.second = 0;
}

void MyP3AT::Terminate(){
    close(mc);
    //ros::spin();
}

void MyP3AT::Loop(){
    int maxidx;
    double cur_doa = 100;
    ofstream DOArecordfile("/home/yeonju/catkin_ws/doarecord.txt", ios::app);
    

    geometry_msgs::Twist msg;
      
    msg.angular.z = 0;
    msg.linear.x = 0;
    vector<int> doa_temp_record;
    //Follow the cur waypoint until the robot arrives
    while(cur_doa > SIGTHD){
        doa_temp_record.clear();
        
        //if(!isMoving) continue;
        
        write(mc, "$A1M2ID1+090", 13);
        threadRSSI th;
        th.run();

        this_thread::sleep_for(chrono::seconds(1));
        th.condition.store(false);
        this_thread::sleep_for(chrono::milliseconds(500));
        cout<< "window size: " << MyP3AT::window.size() <<endl;
        
        ros::spinOnce();    
        
        // Find DOA in 180 degree range
        vector<double> avg_temp, var_temp;
        avg_temp = movingwindowaverage();
        var_temp = movingwindowvariance(avg_temp);
        
        for(int l=0; l<avg_temp.size(); l++){
            DOA.push_back(ALPHA*avg_temp[l] + (1-ALPHA)*var_temp[l]);
            //cout<< avg_temp[l] << "    " << var_temp[l] <<endl;
        }
        //cout<<"--------------------------3" << endl;
        
        maxidx = findstrongestsignal(DOA);
        doa_temp_record.push_back(maxidx*(180/MyP3AT::window[0].size()));
        //DOArecordfile << maxidx*(180/MyP3AT::window[0].size()) << '\t';
        maxidx = sensorfusion(maxidx, sonar_new);
        doa_temp_record.push_back(maxidx*(180/MyP3AT::window[0].size()));
        //DOArecordfile << maxidx*(180/MyP3AT::window[0].size()) << '\n';
        //cout<<"--------------------------4" << endl;
        if(maxidx == -1) { //based on sonar sensor, if there is no available direction...
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub_cmdvel.publish(msg);

            system("sleep 0.5");
            write(mc, "$A1M2ID1-090", 13);
            continue;
        }
        //cout<<"--------------------------5" << endl;
        
        write(mc, "$A1M2ID1-090", 13);
        
        cout << "Current strongest signal is at: " << maxidx*(180/MyP3AT::window[0].size()) << " " << DOA[maxidx]<< endl;
        
        //TODO: PID Control
        msg.linear.x = 1; // fixed linear velocity
        msg.angular.z = -(maxidx*5-90)*PI/180; // angular.z > 0 : anti-clockwise in radians
        pub_cmdvel.publish(msg);

        //system("sleep 0.5");

        
        this_thread::sleep_for(chrono::seconds(1));
        if(MyP3AT::window.size() >= 3){
            pop_front();
        }else if(MyP3AT::window.size() == 0){
            continue;
        }
        DOA.clear();
        ostream_iterator<int> doaoutput_iterator(DOArecordfile, "\t");
        copy(doa_temp_record.begin(), doa_temp_record.end(), doaoutput_iterator);
        DOArecordfile<<"\n";
        DOArecordfile.flush();
    }
    DOArecordfile.close();
}

void MyP3AT::serialSetup(string port){
    struct termios newtio;
    string cmd = "stty -F " + port + " 57600";

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

    if (mc < 0){
        ROS_INFO("%s", "ERROR");
        return;
    }
    tcflush (mc, TCIFLUSH);
    tcsetattr(mc, TCSANOW, &newtio);


    //change speed
    write(mc, "$A1M3ID1-400", 13);

    ROS_INFO("%s", "Serial Port is opend!");

}

void MyP3AT::setupAPGraph(){
    string line;
    string name;
    string password;
    int posx, posy;

    //Read vertex file
    ifstream aplistfile("/home/yeonju/catkin_ws/src/ARTeleOpROS/aplist.txt");
    while(!aplistfile.eof()){
        getline(aplistfile, line, '\n');
        stringstream convertor(line);
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
    //cout<<"Test2"<<std::endl;
    for(int i=0; i<initialAPs.count; i++){
        for(int j=0; j<initialAPs.count; j++){
            initialAPs.floyd[i][j] = INF;
            initialAPs.path[i][j] = 0;
        }
    }
    //out<<"Test3"<<endl;
    //Read edge file
    int from;
    int to;
    double cost;
    ifstream edgefile("/home/yeonju/catkin_ws/src/ARTeleOpROS/edge.txt");
    while(!edgefile.eof()){
        getline(edgefile, line, '\n');
        stringstream convertor(line);
        convertor >> from >> to >> cost;
        initialAPs.addedge(from, to, cost);
    }
//cout<<"Test4"<<endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "teleop");

    MyP3AT myrobot;
    myrobot = MyP3AT();

    

    if(argc < 2) {
    ROS_ERROR("%s", "Add port number as a command argument!");
    return (1);
    }
    else{
        //cout<<argv[1]<<endl;
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