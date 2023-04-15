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
#include <rpi-rt/rt.hpp>
#include <cmath>
float leftTicks = 0;
float rightTicks = 0;
static const constexpr float alpha = 0.7;
static inline float filterVelocity(const float raw, float& filter) {
  return filter = filter + alpha * ((raw - filter));
}
geometry_msgs::Pose2D curr_odom;
void publishOdom(geometry_msgs::Pose2D& data,ros::Publisher& curr_odom_publisher){
    auto ticks = data;

    auto velX = filterVelocity(ticks.x,leftTicks);
    auto velY = filterVelocity(ticks.y,rightTicks);

    curr_odom.x = curr_odom.x + (((6.45/2)/2)* (2*3.14159/495) *(velX + velY)*cos(curr_odom.theta));
    curr_odom.y = curr_odom.y + (((6.45/2)/2)* (2*3.14159/495) *(velX + velY)*sin(curr_odom.theta));
    curr_odom.theta = curr_odom.theta + (((6.45/2)/19.2)*(2*3.14159/495) * (velY - velX));

    curr_odom.theta = atan2(sin(curr_odom.theta),cos(curr_odom.theta));


    //# Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom);
}
int main(int argc, char **argv) {
    rpi_rt::rt_settings rt(rpi_rt::CPUS::CPU3, 99, 100);
    rt.applyAffinity();
    // rt.applyPriority();
    auto uart0_filestream = ::open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
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
    ros::Publisher curr_odom_pub =  nh.advertise<geometry_msgs::Pose2D>("CURR_ODOM", 1000);

    geometry_msgs::Pose2D curr_vel_msg;

    std::string line;
    std::ifstream tty("/dev/ttyACM0");

    auto currRate = terpbot::msgs::CurrentTickRate();

    if (tty.is_open()) {
      std::cout << "tty is open" << std::endl;
        while (getline(tty, line)&&ros::ok()) {
            if (line.length()==11) {
                currRate.deserialize((unsigned char*)line.c_str());
                curr_vel_msg.x = currRate.leftTickRate;  
                curr_vel_msg.y = currRate.rightTickRate;  
                curr_vel_pub.publish(curr_vel_msg);
                ros::spinOnce();
                publishOdom(curr_vel_msg,curr_odom_pub);
                ros::spinOnce();
            }
        }
        tty.close();
        close(uart0_filestream);
    }else{
      close(uart0_filestream);
    }
    return 0;
}