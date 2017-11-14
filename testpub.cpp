#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosaria/PathName.h"
#include <sstream>
#include <vector>
#include <string>

#include <unistd.h>
#include <fcntl.h>

#include <termios.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<rosaria::PathName>("pathwaypoints", 1000);
  int fd;
  struct termios newtio;

  system("stty -F /dev/ttyUSB0 57600");
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

  newtio.c_cflag = B57600;
  newtio.c_cflag |= CS8;
  newtio.c_cflag |= CLOCAL;
  newtio.c_cflag |= CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;

  tcflush (fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  ros::Rate loop_rate(1000);
  write(fd, "#", 1);

  int count = 0;
  while (ros::ok())
  {
    unsigned char bytes;
    unsigned char buf[1];

    rosaria::PathName msg;
    std::vector<std::string> test;
    
    for(int i=0; i<10; i++){
      std::stringstream ss;
      ss << "hello World " << i;
      test.push_back(ss.str());
      //ROS_INFO("%s", test[i].c_str());  
    }
    msg.points = test;

    chatter_pub.publish(msg);
    
    write(fd, "$A1M2ID1-0085", 13);
    bytes = read(fd, buf, sizeof(buf));
    std::string data = "";
    for(int j=0; j<bytes; j++){
      data.append(1, buf[j]);
      
    }
    ROS_INFO("%s", data);
    //my_serial.write("$A1M3ID1+0400");
    //my_serial.write("$A3M3ID1+0400");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  close(fd);
  return 0;
}
