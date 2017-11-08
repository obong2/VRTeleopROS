#include <signal.h>
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#define KEYCODE_1 0x31 
#define KEYCODE_2 0X32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_Q 0X71

void quit(){
    ros::shutdown();
    exit(1);
}

int main(int argc, char** argv){
    //Initialize ros node
    ros::init(argc, argv, "ARTeleop");
    ros::NodeHandle nh;

    /* greet user and display selection options */
	std::cout  	
    << "******************************************************************" << std::endl
    << "*                   ROSARIA CLIENT INTERFACE                     *" << std::endl
    << "*                                                                *" << std::endl
    << "*     This is ROSARIA interface for ARTeleoperation project!     *" << std::endl
    << "*                                                                *" << std::endl
    << "*       [1] start and request                                    *" << std::endl
    //<< "*       [2] spin_clockwise                                       *" << std::endl
    //<< "*       [3] spin_counterclockwise                                *" << std::endl
    //<< "*       [4] teleop                                               *" << std::endl
    //<< "*       [5] enable/disable print_state                           *" << std::endl
    //<< "*       [6] enable_motors                                        *" << std::endl
    << "*       Press [Q] to close the interface                         *" << std::endl 
    << "******************************************************************" << std::endl;

    char select, a, b;
    
    while(ros::ok()){
        std::cout << "Please select a program to run, or hit q to quit: "<< std::endl; /* prompt user at start of every loop */
        std::cin >> select;	/* use standard input to select program to run */

        switch(select){
            case KEYCODE_1:
                pid_t pid;
                switch(pid = fork()){ //fork the process
                    case 0: /* when pid == 0, we have the child process */
                    b = system("rosrun rosaria print_state"); /*run the print_state function */
                    exit(1); /* exit the child process */
                    break; 
                }
                a = system("rosrun rosaria teleop"); /* run option 1*/
                a = system("rosnode kill /print_aria_state "); /*kill the ros print_state node */
                break;

            case KEYCODE_Q:
                quit(); // exit the program
                return false;
                break;

            default:
                std::cout << "Invalid Option. Please select the option listed obove or Q to quit."<<std::endl;
                break;
        }
    }
    return 1;
}
