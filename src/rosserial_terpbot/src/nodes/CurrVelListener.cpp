#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <rosserial_terpbot/MessageParser.h>


int main(int argc, char **argv) {
    auto uart0_filestream = ::open("/dev/ttyACM3", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    if (uart0_filestream == -1)
    {
      //ERROR - CAN'T OPEN SERIAL PORT
      printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
      return -1;
    }

    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B2000000 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

    ros::init(argc, argv, "vel_listener");
    ros::NodeHandle nh;
    ros::Publisher curr_vel_pub = nh.advertise<geometry_msgs::Pose2D>("CURR_VEL", 1000);

    geometry_msgs::Pose2D curr_vel_msg;

    std::string line;
    std::ifstream tty("/dev/ttyACM3");

    auto currRate = terpbot::msgs::CurrentTickRate();

    if (tty.is_open()) {
      std::cout << "tty is open" << std::endl;
        while (getline(tty, line)&&ros::ok()) {
            currRate.deserialize((unsigned char*)line.c_str());
            curr_vel_msg.x = currRate.leftTickRate;  
            curr_vel_msg.y = currRate.rightTickRate;  
            curr_vel_pub.publish(curr_vel_msg);
            ros::spinOnce();
        }
        tty.close();
        close(uart0_filestream);
    }else{
      close(uart0_filestream);
    }
    return 0;
}