#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <thread> // for std::this_thread::sleep_for
#include <chrono> // for std::chrono::milliseconds

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <rosserial_terpbot/MessageParser.h>
#include <rpi-rt/rt.hpp>
#include <signal.h>

#define USE_LOCAL_TRAJECTORY 0
static const constexpr int NO_OF_WAYPOINTS = 350;
static const constexpr float CONTROL_FREQ = 75;
static const constexpr float TICKS_PER_REV= 495;
static const constexpr float DIA_WHEEL = 64.5/1000; // #(converting it to meter);
static const constexpr float MAX_RPM = 140;
static const constexpr float DOUBLE_PI = 2*M_PI;
static const constexpr float VMAX = ((float)MAX_RPM/60)*DOUBLE_PI*(DIA_WHEEL/2);

static inline constexpr float convertLinearToRPS(const float& linearVelocity){
    return linearVelocity/(DIA_WHEEL/2);
}
static inline constexpr float convertRPSToLinear(const float& RPS){
    return RPS*(DIA_WHEEL/2);
}
static inline constexpr float getXLinearVelocity(const float&  t,const float& T){
    return  t*(4*VMAX/T)+(-(4*VMAX/(T*T))*t*t);
}
static inline constexpr float getT(const float&  xInitial,const float& xFinal){
    return (6.0/(4.0*VMAX))*(xFinal - xInitial);
}

class Trajectory{
    public:
    std::vector<float> wayPoints;
    int noOfWayPoints=0;
    float totalTime=0;
    Trajectory(){
        auto time = 0.0;
        float loopRate = CONTROL_FREQ;
        auto timeIncrement = 1/loopRate;
        auto xInit = 0.0;  
        auto xFinal = 1.00; //#// 1 rotation of the wheel //0.20263272615
        this->totalTime = getT(xInit,xFinal);
        int i=0;
        while (true){
            this->wayPoints.push_back(convertLinearToRPS(getXLinearVelocity(time,this->totalTime))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ));
            time += timeIncrement;
            if(time>this->totalTime){
                this->wayPoints.push_back(0.0);
                break;
            }
            else{
                i+=1;
            }
        }
        this->noOfWayPoints = i;
    }
};

inline int publishToUART(int& uart0_filestream, uint8_t* outbuf,int data_length){
    return ::write(uart0_filestream,outbuf,data_length);
}

template<typename T>
inline int publishToUart(int& uart0_filestream, uint8_t* outbuf , T& input){
    auto dl = input.serialize(outbuf);
    auto bytes_written = publishToUART(uart0_filestream,outbuf,dl);
    if (bytes_written < 0) {
        std::cerr << "Error: could not start controller\n";
        return 1;
    }
    if(bytes_written!=dl){
        return 1;
    }
    return 0;
}
int sendGains(int& uart0_filestream,terpbot::msgs::Gains& leftGain,terpbot::msgs::Gains& rightGain){
    /**
    Left Motor Gains
    */
    uint8_t outbuf[50];
    for(int i=0;i<5;i++){
        publishToUart(uart0_filestream,outbuf,leftGain);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    /**
    Right Motor Gains
    */
    for(int i=0;i<5;i++){
        publishToUart(uart0_filestream,outbuf,rightGain);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
int sendLocalTrajectory(const Trajectory& trajectory,int& uart0_filestream){
    std::cout<<"No of waypoints:"<<(int)trajectory.wayPoints.size()<<std::endl;
    std::cout<<"Time:"<<trajectory.totalTime<<std::endl;

    auto target = terpbot::msgs::Target();

    uint8_t outbuf[50];
    /**
    * Ensure controller has started
    */
    for(int i=0;i<5;i++){
        target.leftMotorTarget = 0;
        target.rightMotorTarget = 0;
        target.theta = 1;
        publishToUart(uart0_filestream,outbuf,target);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    /**
    Start sending trajectories
    */
    for(const auto waypt:trajectory.wayPoints){
        target.leftMotorTarget = waypt;
        target.rightMotorTarget = waypt;
        target.theta = 1;
        publishToUart(uart0_filestream,outbuf,target);
        std::this_thread::sleep_for(std::chrono::milliseconds(13));
    }

    /**
    * Ensure controller has stopped
    */
    for(int i=0;i<5;i++){
        target.leftMotorTarget = 0;
        target.rightMotorTarget = 0;
        target.theta = 0;
        publishToUart(uart0_filestream,outbuf,target);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}

// ## ROTATION GAINS:
// # left_KU = 35
// # left_TU = 0.1481
// # left_kd_coeff = 0.085

// # right_KU = 42
// # right_TU = 0.1481
// # right_kd_coeff = 0.085
static const constexpr float left_KU = 32.5;
static const constexpr float left_TU = 0.1481;
static const constexpr float left_kd_coeff = 0.075;

static const constexpr float right_KU = 39;
static const constexpr float right_TU = 0.1481;
static const constexpr float right_kd_coeff = 0.085;

static const Trajectory trajectory;

void initGains(terpbot::msgs::Gains& leftGain,terpbot::msgs::Gains& rightGain){
    leftGain.kp = 0.6*left_KU;
    leftGain.ki = 1.2*left_KU/left_TU;
    leftGain.kd = left_kd_coeff*left_KU*left_TU;
    leftGain.iSat = 200.0;
    leftGain.isLeft = true;

    rightGain.kp =  0.6*right_KU;
    rightGain.ki = 1.2*right_KU/right_TU;
    rightGain.kd = right_kd_coeff*right_KU*right_TU;
    rightGain.iSat = 200.0;
    rightGain.isLeft = false;
}

void setupSerialPort(const int& uart0_filestream){
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B2000000 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
}

int uart0_filestream;
uint8_t outbuf[50];
terpbot::msgs::Target target;

static void sendPoint(const terpbot::msgs::Target& targ,int& uart0_filestream){
    publishToUart(uart0_filestream,outbuf,targ);
}

static inline void rot2wheel(float vel,float ang_vel){
    auto r = 6.45/2.0;
    auto L = 19.2;
    target.leftMotorTarget = 0.5* ( (2*vel/r) + (L*ang_vel/r))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ);
    target.rightMotorTarget = 0.5* ( (2*vel/r) - (L*ang_vel/r))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ);
}

void targetCB(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    rot2wheel(msg->x,msg->y);
    sendPoint(target,uart0_filestream);
}

void sigint_handler(int sig)
{
   ros::shutdown();
}

int main(int argc, char **argv){
    #if USE_LOCAL_TRAJECTORY
        rpi_rt::rt_settings rt(rpi_rt::CPUS::CPU4, 99, 100);
        rt.applyAffinity();
        rt.applyPriority();
    #endif  

    signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "target_sender");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("TARG_VEL", 10, targetCB);

    uart0_filestream = ::open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    if (uart0_filestream == -1)
    {
      //ERROR - CAN'T OPEN SERIAL PORT
      printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
      return -1;
    }
    terpbot::msgs::Gains leftGain;
    terpbot::msgs::Gains rightGain;
    setupSerialPort(uart0_filestream);
    initGains(leftGain,rightGain);
    sendGains(uart0_filestream,leftGain,rightGain);
    #if USE_LOCAL_TRAJECTORY
        sendLocalTrajectory(trajectory,uart0_filestream);
    #endif
    #if !USE_LOCAL_TRAJECTORY
        std::thread ros_thread([&]() {
            rpi_rt::rt_settings rt(rpi_rt::CPUS::CPU4, 99, 100);
            rt.applyAffinity();
            rt.applyPriority();
            while (ros::ok()) {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        ros_thread.join();
    #endif
    ::close(uart0_filestream);
    return 0;
}