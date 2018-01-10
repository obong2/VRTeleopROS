#include <iostream>
#include <atomic>
#include <thread>
#include <deque>
#include <queue>
#include <string>
using namespace std;

//static const std::string WADAPTER = "wlx00c0ca590adb";
static const std::string WADAPTER = "wlan0";

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
        double cur_sig;
        string line;
        
        string cmd = "iwconfig " + WADAPTER + " | grep Signal| cut -d - -f2 | cut -d ' ' -f 1 > sig_temp.txt";
        //cout << "Thread RSSI has started" <<endl;
        while(condition){
            system(cmd.c_str());
            this_thread::sleep_for(chrono::milliseconds(100));
            ifstream tempfile("/home/yeonju/catkin_ws/sig_temp.txt");
            while(!tempfile.eof()){
                std::getline(tempfile, line, '\n');
                std::stringstream convertor(line);
                convertor >> cur_sig;
                //cout << cur_sig << endl;
                buff.push_back(cur_sig);
            }
        }

        //curwindow.push_back(buff);
        //cout << "Thread RSSI has terminated" <<endl;
    }

private:
    thread tid;
    vector<int> buff;

public:
    atomic_bool condition;
};