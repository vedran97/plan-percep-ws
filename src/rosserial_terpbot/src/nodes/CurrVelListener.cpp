#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>

using namespace std;


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
    if (! inbuffer[1] ==  tags[Tags::CURRENT]){
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


int main() {
    string line;
    ifstream tty("/dev/ttyACM0");
    auto currRate = CurrentTickRate();
    if (tty.is_open()) {
        while (getline(tty, line)) {
            currRate.deserialize((unsigned char*)line.c_str());
            cout << line.length() << endl;
            cout << currRate.leftTickRate << endl;
            cout << currRate.rightTickRate << endl;
        }
        tty.close();
    }
    return 0;
}