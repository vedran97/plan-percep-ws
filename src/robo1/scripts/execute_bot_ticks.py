#!/usr/bin/env python3


# for battery voltage 16.7V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.6 radians per sec  9.52 reverse
# for battery voltage 16.25V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.25 radians per sec
# for batt volt 16.7V left to right ratio  of 1.206 and additional turn ratio of 1.1, pwm 75,98  turning angular vel = 5 right turn
# for batt volt 16.7V , pwm 85,90.45  turning angular vel = 5 left turn

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np

curr_ticks = 0
target_pose = Pose2D()
state = 0
flagState = True
flagTurn = True
flagGo = False

turnTicks = 368
goTicks = 1221

# STATE is running variable, for keeping track of operations
# -1 -> Next position coordinates given (this will be changed by some other node to 1)
# 0  -> Home/start state
# 1  -> Waiting for next position coordinates, reached last position
# 2  -> Rotating 
# 3  -> Done rotating, will start translation next
# 4  -> Translating
# 5  -> Done Translating, will start waiting for next coordinate


def curr_ticks_callback(msg):
    global curr_ticks

    global turnTicks
    global goTicks
    global flagState

    global flagTurn
    global flagGo

    # print("State: ",flagState,"Turning: ",flagTurn,"Moving: ",flagGo,"\nTicks",
    #       int(curr_ticks)," out of ",int(turnTicks),"/",int(goTicks)," [Turning / Moving]")
    
    curr_ticks = curr_ticks + (abs(msg.x) + abs(msg.y)) /2 
    print(curr_ticks)

    if flagState:
        if flagTurn:
            if curr_ticks < abs(turnTicks):
                #print("Turning ",curr_ticks,' / ',turnTicks)
                runBot_turn(np.sign(turnTicks))
                return
            else:
                runBot_stop()
                print("Angle done, error in ticks :",abs(abs(turnTicks)-curr_ticks))
                flagTurn = False
                flagGo = True

                rospy.sleep(1)
                curr_ticks = 0
                return
            return
        if flagGo:
            if curr_ticks < goTicks:
                #print("Going ",curr_ticks,' / ',goTicks)
                runBot_forward()
                return
            else:
                runBot_stop()
                print("Linear done, error in ticks :",(goTicks-curr_ticks))

                flagGo = False
                flagTurn = True

                rospy.sleep(1)
                curr_ticks = 0
                return
            return
        return
    return
                
# Callback function for the /NEXT_POS topic
def next_pos_callback(msg):
    global target_pose
    global flagState

    global turnTicks
    global goTicks

    if not flagState:
        target_pose = msg
        dx = target_pose.x
        dy = target_pose.y

        distance = ((dx ** 2) + (dy ** 2)) ** 0.5
        angle = np.arctan2(dy,dx)

        print("Angle is: ",angle)

        turnTicks = (495/(2*3.141596))*angle*(19.2/6.45)
        goTicks = (distance/(6.45/2)) * (495/(2*3.141596))

        flagState = True


def runBot_turn( sign ):
    pose = Pose2D()
    pose.x = -3 * sign
    pose.y = 3 * sign
    pose.theta = 0

    targ_vel_pub.publish(pose)

def runBot_stop():
    pose = Pose2D()
    pose.x = 0
    pose.y = 0
    pose.theta = 0

    targ_vel_pub.publish(pose)

def runBot_forward():

    pose = Pose2D()
    pose.x = 6
    pose.y = 6
    pose.theta = 0

    targ_vel_pub.publish(pose)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_follower')

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    #next_pos_sub = rospy.Subscriber('/NEXT_POS', Pose2D, next_pos_callback)
    curr_odom_sub = rospy.Subscriber('/CURR_VEL', Pose2D, curr_ticks_callback)
    

    rospy.spin()