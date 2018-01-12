#include <iostream>
#include <atomic>
#include <thread>
#include <queue>
#include <string>
#include <rosaria/teleop.h>
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
            //cout<<"aaaa"<<endl;
        }

        MyP3AT::window.push_back(buff);
        cout<<" window size in thread: " << MyP3AT::window.size() << endl;
        cout<< "current buff size: "<< buff.size() <<endl;
        
        //cout << "Thread RSSI has terminated" <<endl;
    }

private:
    thread tid;
    vector<double> buff;

public:
    atomic_bool condition;
};