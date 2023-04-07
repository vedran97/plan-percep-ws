#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

class CustomMsg
{
public:
  virtual uint8_t serialize(unsigned char *outbuffer) const = 0;
  virtual uint8_t deserialize(unsigned char *data) = 0;
};
enum Tags{TARGET,GAINS,CURRENT};

static const unsigned char tags[] = {'T','G','C'};

class CurrentTickRate:public CustomMsg{
  public:
  float leftTickRate=0;
  float rightTickRate=0;
  virtual uint8_t serialize(unsigned char *outbuffer) const override{
    uint8_t offset = 0;

    *(outbuffer+offset) = '{';
    offset+=1;

    *(outbuffer+offset) = tags[Tags::CURRENT];
    offset+=1;

    union {
      float real;
      uint32_t base;
    } u_i_leftMotorTicks;
    u_i_leftMotorTicks.real = this->leftTickRate;

    *(outbuffer + offset + 0) = (u_i_leftMotorTicks.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_leftMotorTicks.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_leftMotorTicks.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_leftMotorTicks.base >> (8 * 3)) & 0xFF;

    offset += sizeof(this->leftTickRate);

    union {
      float real;
      uint32_t base;
    } u_i_rightMotorTicks;
    u_i_rightMotorTicks.real = this->rightTickRate;

    *(outbuffer + offset + 0) = (u_i_rightMotorTicks.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_rightMotorTicks.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_rightMotorTicks.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_rightMotorTicks.base >> (8 * 3)) & 0xFF;
    
    offset += sizeof(this->rightTickRate);


    *(outbuffer+offset) = '}';
    offset+=1;

    *(outbuffer+offset) = '\n';
    offset+=1;

    return offset;
  }
  virtual uint8_t deserialize(unsigned char *inbuffer) override{
    uint8_t offset = 0;
    offset+=2;
    if ((inbuffer[1] !=  tags[Tags::CURRENT])){
        return 0;
    }
    union {
      float real;
      uint32_t base;
    } u_leftTarg;
    u_leftTarg.base = 0;
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->leftTickRate = u_leftTarg.real;
    offset += sizeof(this->leftTickRate);

    union {
      float real;
      uint32_t base;
    } u_rightTarg;
    u_rightTarg.base = 0;
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->rightTickRate = u_rightTarg.real;
    offset += sizeof(this->rightTickRate);

    offset+=2;
    return offset;
  }
};


int main(int argc, char **argv) {
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

    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;
    ros::Publisher curr_vel_pub = nh.advertise<geometry_msgs::Pose2D>("CURR_VEL", 1000);

    geometry_msgs::Pose2D curr_vel_msg;

    std::string line;
    std::ifstream tty("/dev/ttyACM0");
    auto currRate = CurrentTickRate();

    if (tty.is_open()) {
      std::cout << "tty is open" << std::endl;
        while (getline(tty, line)&&ros::ok()) {
            currRate.deserialize((unsigned char*)line.c_str());
            curr_vel_msg.x = currRate.leftTickRate;  
            curr_vel_msg.y = currRate.rightTickRate;  
            curr_vel_msg.theta = 0.0;  
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