#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import numpy as np
import rospkg
import csv
from std_msgs.msg import Float32MultiArray
import cv2

# Construct path to CSV file
rospack = rospkg.RosPack()
csv_path = rospack.get_path('robo1') + '/scripts_planner/vel_coord.csv'
csv_file_path = open(csv_path,'r')
data_raw = np.array(list(csv.reader(csv_file_path, delimiter=",")),dtype=float)
data = data_raw - data_raw[0]

# Import map and dot-ify it
map_path = rospack.get_path('robo1') + '/scripts_planner/map_bloated.jpg'
map_ = cv2.bitwise_not(cv2.imread(map_path,cv2.IMREAD_GRAYSCALE))
map_ = cv2.Canny(map_, 100, 200) 

# List of obsticles from map
map_x = np.tile( np.linspace(0,map_.shape[1]/3,num=map_.shape[1]) , (map_.shape[0],1)) - data_raw[0][0]
map_y = np.tile( np.linspace(map_.shape[0]/3,0,num=map_.shape[0]).reshape(-1, 1) , (1,map_.shape[1])) - data_raw[0][1]
mapObs = np.hstack( (np.reshape(map_x[map_ > 0], (-1,1)) , np.reshape(map_y[map_ > 0], (-1,1)) ) )


###############################################################################
# Function to get target points list
def targetFinder(odom,obst,data):
    T = data[:,0]* ((odom[0]-obst[0])/(obst[1]-odom[1])) + obst[1] - (obst[0]*(odom[0]-obst[0])/(obst[1]-odom[1]))
    - data[:,1]
    
    k = np.where(T > 0, T, np.inf).argmin()
    
    targP = []
    dist_cent = np.sqrt (np.sum(((data - obst)[:,:-1])**2,axis=1))
    while len(targP) < 4:
        if dist_cent[k] > obst_radius:
            targP.append(data[k])
        k=k+1
    
    targL = np.array([])
    for i in range(len(targP)-1):
        A = np.linspace(targP[i], targP[i+1],10)
        if targL.shape == (0,):
            targL = A
        else:
            targL = np.vstack((targL,A))
    return targL[:,:-1]

# Function to calculate force at any position
def forceCalc(Q,odom,PointSet):
    odom = np.array([odom[0],odom[1]])
    
    R = np.sum(np.square(PointSet - odom),axis=1)**1.5
    R = np.hstack((R[:,np.newaxis],R[:,np.newaxis]))
    F = Q * np.divide( (PointSet - odom) , R)
    F = np.sum(F,axis=0)
    
    return F

# Function to get obsticle point list
def obsticle(obst):
    detObs_x = obst[0] + obst_radius*np.cos(np.linspace(0, 2*np.pi,150))
    detObs_y = obst[1] + obst_radius*np.sin(np.linspace(0, 2*np.pi,150))
    detObs = np.hstack( (detObs_x[:,np.newaxis],detObs_y[:,np.newaxis]) )
    
    return detObs

# Function to convert x,y to pixels (to plot)
def coord2pixel(point,mapN):
    pixel = (int(3*point[0]), mapN.shape[0]-int(3*point[1]))
    return pixel
def pixel2point(pixel,mapN):
    point = (pixel[0]/3 , (pixel[1]-mapN.shape[0])/3)
    return point
# Function to get force vector end point on image (for plotting)
def dirxPlotter(force,odom):
    scale = np.sqrt(np.sum(force**2))
    force = force/np.sqrt(np.sum(force**2))
    end_point = ( int(coord2pixel(odom + data_raw[0],map_)[0]+ scale*1000*force[0]),
                 int(coord2pixel(odom + data_raw[0],map_)[1]- scale*1000*force[1]) )
    
    return end_point

###############################################################################
# Global variables
line_idx = 0
r = 6.45/2
L = 19.2
obst_radius = 10 + 19
obst = None
detObs = None
targObs = None

global_planner = True
init_local = True

rospy.init_node('pose_follower')

###############################################################################
# Global planner
def global_planner(msg):
    global global_planner
    global target_pose
    global line_idx

    if not global_planner:
        local_planner(msg)
        return
    
    if line_idx > len(data)-1:
        publish_this(0,0)
        print("reached!")
        return
    
    target_pose = np.float32(data[line_idx])
    if ((target_pose[1] - msg.y)**2 + (target_pose[0] - msg.x)**2)**0.5 < 4:
        line_idx = line_idx + 1
        print("temp goal ", line_idx, " reached!")

    curr_heading = msg.theta
    need_heading = np.arctan2(target_pose[1] - msg.y , target_pose[0] - msg.x)
    angle_diff = (need_heading - curr_heading)

    vel = 12 if np.cos(angle_diff) > 0 else 0

    publish_this( vel , -1.0* angle_diff)


###############################################################################  
# local planner
runner = 0

def local_planner(msg):

    global init_local
    global detObs
    global targObs
    global global_planner
    global line_idx
    global runner

    if runner % 10 != 0:
        runner = runner + 1
        return
    runner = runner + 1

    odom = [msg.x , msg.y , msg.theta]

    if init_local:
        detObs = obsticle(obst)
        targObs = targetFinder(odom,obst,data)
        init_local =  False
    
    if min(np.sum(np.square(targObs - np.array(odom)[:-1]),axis=1)) > 10:

        force1 = forceCalc(-0.3,odom,detObs)
        force2 = forceCalc(-0.13,odom,mapObs)
        force3 = forceCalc(70  ,odom,targObs)

        forceNet = force1 + force2 + force3

        need_heading = np.arctan2(forceNet[1],forceNet[0])
        curr_heading = msg.theta
        angle_diff = (need_heading - curr_heading)
        print(angle_diff)

        vel = 5 if np.cos(angle_diff) > 0 else 0
        publish_this( vel , -0.4* angle_diff)

    else:
        dist = (np.sum((data - np.array(odom))**2, axis=1))**0.5
        line_idx = int(np.argmin(dist) + 1)

        global_planner = True


###############################################################################

def publish_this(vel,ang_vel):
    targ_vel = Pose2D()
    targ_vel.x = vel
    targ_vel.y = ang_vel
    targ_vel_pub.publish(targ_vel)


def obst_callback(msg):
    global obst
    global global_planner
    global init_local

    obst = [msg.x , msg.y, msg.theta]
    global_planner = False
    init_local = True

if __name__ == '__main__':

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)

    curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, global_planner)

    obstacle_sub = rospy.Subscriber('/positions', Pose2D, obst_callback)

    rospy.spin()