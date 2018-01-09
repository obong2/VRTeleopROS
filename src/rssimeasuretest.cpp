#include <rosaria/teleop.h>
using namespace std;

static const string WADAPTER = "wlan0";
//static const string WADAPTER = "wlx00c0ca590adb";

void Init(char* argv){
    SerialSetup(argv);
}

void SerialSetup(char* port){
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

void RSSI(){
    int cur_sig;
    string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";

    system(cmd.c_str());

    ifstream tempfile("/home/yeonju/catkin_ws/sig_temp.txt");
    while(!tempfile.eof()){
        std::getline(tempfile, line, '\n');
        std::stringstream convertor(line);
        convertor >> cur_sig;
        cout << cur_sig << endl;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rssimeasure");

    Init(argv[1]);
    int count = 0;
    

    while(1){
        write(mc, "$A1M2ID1-090", 13);
        cout<< "--------------------- [" << count << "] -----------------------" <<endl;
        
        cout<< "---------------------------------------------------------------" <<endl;
        system("sleep 1.0");
        write(mc, "$A1M2ID1-090", 13);
        system("sleep 1.0");
    }
}