#include <rosaria/teleop.h>
#include <atomic>
#include <thread>
#include <iostream>
using namespace std;

static const string WADAPTER = "wlan0";
//static const string WADAPTER = "wlx00c0ca590adb";
static int mc;
class hello{
public:
    hello()
	{
		condition.store(true);
	}
	~hello()
	{
		condition.store(false);
		if(tid.joinable())
		{
			tid.join();
		}
	}
    void run(){
        tid = std::thread(std::bind(&hello::RSSI, this));
    }

private:
    void RSSI(){
        int cur_sig;
        string line;

        string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";
        cout << "Thread RSSI has started" <<endl;
        while(condition){
            system(cmd.c_str());
            //this_thread::sleep_for(chrono::seconds(1));
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

private:
    thread tid;
public:
    atomic_bool condition;
};
/*
void RSSI(){
    int cur_sig;
    string line;

    string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";
    cout << "Thread RSSI has started" <<endl;
    while(run){
        system(cmd.c_str());
        this_thread::sleep_for(chrono::seconds(1));
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
*/
void myInit(string port){
    //serialSetup(mc, argv);
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

int main(int argc, char** argv){
    ros::init(argc, argv, "rssimeasure");
 
    myInit(argv[1]);
    int count = 0;

    while(1){
        hello h;
        cout<< "--------------------- [" << count << "] -----------------------" <<endl;
        h.run();
        write(mc, "$A1M2ID1+090", 13);
        
        
        //system("sleep 1.0");
        //Or use
        this_thread::sleep_for(chrono::seconds(1));
        
        cout<< "---------------------------------------------------------------" <<endl;
        h.condition.store(false); //exit/destroy thread
        write(mc, "$A1M2ID1-090", 13);
        
        //system("sleep 1.0");
        this_thread::sleep_for(chrono::seconds(1));
        count++;
    }

    return (0);
}