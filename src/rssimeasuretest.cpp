#include <rosaria/teleop.h>
#include <atomic>
#include <thread>
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

void RSSI(atomic< bool >& run){
    int cur_sig;
    string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";
    cout << "Thread RSSI has started" <<endl;
    while(run){
        system(cmd.c_str());
        this_thread::sleep_for(0.01s);
        ifstream tempfile("/home/yeonju/catkin_ws/sig_temp.txt");
        while(!tempfile.eof()){
            std::getline(tempfile, line, '\n');
            std::stringstream convertor(line);
            convertor >> cur_sig;
            cout << cur_sig << endl;
        }
    }

    cout << "Thread RSSI has terminated" <<endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rssimeasure");

    atomic < bool > run{ true };
    thread th{ RSSI, ref(run) };
    run = false;
    
    Init(argv[1]);
    int count = 0;
    

    while(1){
        write(mc, "$A1M2ID1+090", 13);
        cout<< "--------------------- [" << count << "] -----------------------" <<endl;
        run = true;
        
        cout<< "---------------------------------------------------------------" <<endl;
        system("sleep 1.0");
        //Or use
        //this_thread::sleep_for(1.0s);
        run = false;
        write(mc, "$A1M2ID1-090", 13);
        system("sleep 1.0");
        //this_thread::sleep_for(1.0s);
    }
    run = false;
    th.join();

    return 0;
}