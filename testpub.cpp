#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosaria/PathName.h"
#include <sstream>
#include <vector>
#include <string>

#include "serial/serial.h"
#include <unistd.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<rosaria::PathName>("pathwaypoints", 1000);

  if(argc < 2){
    std::cout<<"argc!"<<std::endl;
    return 0;
  } 
  std::string port(argv[1]);
  if(port == "-e"){
    //enumerate_ports();
    return 0;

  }
  else if(argc < 3){
    std::cout <<"argc2!"<<std::endl;
    return 1;
  }

  unsigned long baud = 0;
  #if defined(WIN32) && !defined(__MINGW32__)
    std::sscanf_s(argv[2], "%lu", &baud);
  #else
    std::sscanf(argv[2], "%lu", &baud);
  #endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
  
    std::cout << "Is the serial port open?";
    if(my_serial.isOpen())
      std::cout << " Yes." << std::endl;
    else
      std::cout << " No." << std::endl;
  
  
  ros::Rate loop_rate(10);
  

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    rosaria::PathName msg;
    std::vector<std::string> test;
    
    for(int i=0; i<10; i++){
      std::stringstream ss;
      ss << "hello World " << i;
      test.push_back(ss.str());
      //ROS_INFO("%s", test[i].c_str());  
    }
    msg.points = test;

  //  std::write()
    //strcpy(temp, msg.points);
    std::cout<<msg<<std::endl;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    my_serial.write("$A2M3ID1+00400");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  
  return 0;
}
