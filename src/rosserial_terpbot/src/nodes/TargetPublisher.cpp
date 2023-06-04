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

/// @brief Macro to generate a local trajectory and send it to the controller
#define USE_LOCAL_TRAJECTORY 0

/// @brief Control freq in Hz
static const constexpr float CONTROL_FREQ = 75;
/// @brief Ticks per revolution of the wheel
static const constexpr float TICKS_PER_REV= 495;
/// @brief Diameter of the wheel in meters
static const constexpr float DIA_WHEEL = 64.5/1000; // #(converting it to meter);
/// @brief Maximum RPM of the wheel
static const constexpr float MAX_RPM = 140;
/// @brief 2*PI
static const constexpr float DOUBLE_PI = 2*M_PI;
/// @brief Maximum linear velocity of the wheel in m/s
static const constexpr float VMAX = ((float)MAX_RPM/60)*DOUBLE_PI*(DIA_WHEEL/2);

/// @brief Converts linear velocity to Radians per second
/// @param linearVelocity m/s
/// @return Angular vel in Radians per second
static inline constexpr float convertLinearToRPS(const float& linearVelocity){
    return linearVelocity/(DIA_WHEEL/2);
}
/// @brief Converts angular velocity to linear velocity
/// @param RPS radians per sec
/// @return linear velocity in m/s
static inline constexpr float convertRPSToLinear(const float& RPS){
    return RPS*(DIA_WHEEL/2);
}
/// @brief Parabolic velocity profile
/// @param t time in the trajectory
/// @param T total time of the trajectory
/// @return linear velocity in m/s
static inline constexpr float getXLinearVelocity(const float&  t,const float& T){
    return  t*(4*VMAX/T)+(-(4*VMAX/(T*T))*t*t);
}
/// @brief Returns the total time of the trajectory
/// @param xInitial initial position of the trajectory
/// @param xFinal final position of the trajectory
static inline constexpr float getT(const float&  xInitial,const float& xFinal){
    return (6.0/(4.0*VMAX))*(xFinal - xInitial);
}
/// @brief Parabolic trajectory class
class Trajectory{
    public:
    /// @brief Vector of waypoints
    std::vector<float> wayPoints;
    /// @brief Number of waypoints
    int noOfWayPoints=0;
    /// @brief Total time of the trajectory
    float totalTime=0;
    /// @brief Constructs a trajectory
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

/// @brief Publishes data in outbuf of length data_length to the uart port
/// @param uart0_filestream Uart filestream
/// @param outbuf Buffer with the data
/// @param data_length length of the data
/// @return 0 if successful
inline int publishToUART(int& uart0_filestream, uint8_t* outbuf,int data_length){
    return ::write(uart0_filestream,outbuf,data_length);
}

template<typename T>
/// @brief Serializes the input data and publishes it to the uart port
/// @param uart0_filestream Uart stream
/// @param outbuf buffer where data has to be serialized
/// @param input input data
/// @return 0 if successful,1 otherwise
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
/// @brief Sends the gains to the controller
/// @param uart0_filestream Uart filestream FD
/// @param leftGain Left motor gain
/// @param rightGain Right motor gain
/// @return 0 if successful,1 otherwise
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
/// @brief Sends the local trajectory to the controller
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


/**
 * \brief Left Motor Controller Gain variables
 * \name Left motor gains
 * @{
 */
static const constexpr float left_KU = 32.5; ///<  Left Ku
static const constexpr float left_TU = 0.1481; ///<  Left Tu
static const constexpr float left_kd_coeff = 0.075; ///<  Left Kd coeff
/** @} */

/**
 * \brief Right Motor Controller Gain variables
 * \name Right Motor gains
 * @{
 */
static const constexpr float right_KU = 36; ///<  Right Ku
static const constexpr float right_TU = 0.1481; ///<  Right Tu
static const constexpr float right_kd_coeff = 0.075; ///<  Right Kd coeff
/** @} */ 

/// @brief Local trajectory object
static const Trajectory trajectory;

/// @brief Uart filestream FD
int uart0_filestream;
/// @brief Buffer to store the serialized data
uint8_t outbuf[50];
/// @brief Variable to store target as wheel tickrate
terpbot::msgs::Target target;

/// @brief Enables the control action
void enable_controller(){
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
}

/// @brief Disables the control action
void disable_controller(){
    auto target = terpbot::msgs::Target();

    uint8_t outbuf[50];
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
}

/// @brief Initializes the gains
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

/// @brief Sets up the serial port
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

/// @brief Serializes the target and publishes it to the uart port
static void sendPoint(const terpbot::msgs::Target& targ,int& uart0_filestream){
    publishToUart(uart0_filestream,outbuf,targ);
}

/// @brief Converts linear and angular velocity to wheel tickrate
/// @param vel Linear velocity in cm/s
/// @param ang_vel Angular velocity in rad/s
static inline void rot2wheel(float vel,float ang_vel){
    auto r = 6.45/2.0;
    auto L = 19.2;
    target.leftMotorTarget = 0.5* ( (2*vel/r) + (L*ang_vel/r))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ);
    target.rightMotorTarget = 0.5* ( (2*vel/r) - (L*ang_vel/r))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ);
    target.theta = 1.0;
}

/// @brief Callback function for the target velocity subscriber
/// @details Converts the linear and angular velocity to wheel tickrate and sends it to the controller
/// @param msg Contains linear and angular velocity of needed from the robot
void targetCB(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    rot2wheel(msg->x,msg->y);
    sendPoint(target,uart0_filestream);
}

/// @brief Signal handler for SIGINT
/// @details Shuts down the ros node
void sigint_handler(int sig)
{
   ros::shutdown();
}

/// @brief Main function
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
        enable_controller();
        std::thread ros_thread([&]() {
            rpi_rt::rt_settings rt(rpi_rt::CPUS::CPU4, 99, 100);
            rt.applyAffinity();
            // rt.applyPriority();
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