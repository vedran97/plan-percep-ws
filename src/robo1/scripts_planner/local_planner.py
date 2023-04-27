#!/usr/bin/env python3


import numpy as np
import matplotlib.pyplot as plt
import csv
import cv2
import copy
import time


# Construct path to CSV file
csv_path = 'vel_coord.csv'
csv_file_path = open(csv_path,'r')
data_raw = np.array(list(csv.reader(csv_file_path, delimiter=",")),dtype=float)
data = data_raw - data_raw[0]

map_path = 'map_bloated.jpg'
map_ = cv2.bitwise_not(cv2.imread(map_path,cv2.IMREAD_GRAYSCALE))

map_canny = cv2.Canny(map_, 100, 200) 


#%%
                
obst_radius = 10 + 19

map_x = np.tile( np.linspace(0,map_canny.shape[1]/3,num=map_canny.shape[1]) , (map_canny.shape[0],1)) - data_raw[0][0]
map_y = np.tile( np.linspace(map_canny.shape[0]/3,0,num=map_canny.shape[0]).reshape(-1, 1) , (1,map_canny.shape[1])) - data_raw[0][1]

mapObs = np.hstack( (np.reshape(map_x[map_canny > 0], (-1,1)) , np.reshape(map_y[map_canny > 0], (-1,1)) ) )


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

def forceCalc(Q,odom,PointSet):
    odom = np.array([odom[0],odom[1]])
    
    R = np.sum(np.square(PointSet - odom),axis=1)**1.5
    R = np.hstack((R[:,np.newaxis],R[:,np.newaxis]))
    F = Q * np.divide( (PointSet - odom) , R)
    F = np.sum(F,axis=0)
    
    return F
#%%
# Global variables
line_idx = 0
r = 6.45/2
L = 19.2

global_planner = True
init_local = True



def obsticle(obst, odom):
    detObs_x = obst[0] + obst_radius*np.cos(np.linspace(0, 2*np.pi,150))
    detObs_y = obst[1] + obst_radius*np.sin(np.linspace(0, 2*np.pi,150))
    detObs = np.hstack( (detObs_x[:,np.newaxis],detObs_y[:,np.newaxis]) )
    
    return detObs

# local planner
def local_planner(msg):
    global init_local
    global obst
    
    x = msg[0]
    y = msg[1]
    theta = msg[2]
    
    if init_local:

        init_local =  False


#%%
def coord2pixel(point,mapN):
    pixel = (int(3*point[0]), mapN.shape[0]-int(3*point[1]))
    return pixel
def pixel2point(pixel,mapN):
    point = (pixel[0]/3 , (pixel[1]-mapN.shape[0])/3)
    return point
def dirxPlotter(force,odom):
    scale = np.sqrt(np.sum(force**2))
    force = force/np.sqrt(np.sum(force**2))
    end_point = ( int(coord2pixel(odom + data_raw[0],map_)[0]+ scale*1000*force[0]),
                 int(coord2pixel(odom + data_raw[0],map_)[1]- scale*1000*force[1]) )
    
    return end_point
    
#%%
line_idx = 13
odom = [ data[line_idx-1][0] + 1 , data[line_idx-1][1] - 1 , 0 ]
obst = [180,-100,0] 


mapN = np.copy(map_)
mapN = np.dstack((mapN,mapN,mapN))

# for i in data_raw:
#     mapN = cv2.circle(mapN,coord2pixel(i,map_),2,(0,0,255),-1)
    
mapN = cv2.circle(mapN,coord2pixel(odom + data_raw[0],map_),2,(0,255,255),-1)

for i in data:
    mapN = cv2.circle(mapN,coord2pixel(i + data_raw[0],map_),2,(0, 0, 255),-1)

start_time = time.time()
detObs = obsticle(obst,odom)
Targ = targetFinder(odom,obst,data)

for i in detObs:
    i = np.append(i,0)
    mapN = cv2.circle(mapN,coord2pixel(i + data_raw[0],map_),2,(255,0,0),-1)
    
# for i in Targ:
#     i = np.append(i,0)
#     mapN = cv2.circle(mapN,coord2pixel(i + data_raw[0],map_),2,(255,255,0),-1)


i = 1
odomList = []
while min(np.sum(np.square(Targ - np.array(odom)[:-1]),axis=1)) > 5:  
#for i in range(1000):
    #mapN_ = np.copy(mapN)
         
    force1 = forceCalc(-0.3,odom,detObs)
    # end_point = dirxPlotter(force1,odom)
    #mapN_ = cv2.arrowedLine(mapN_, coord2pixel(odom + data_raw[0],map_), end_point,(0,255,255), 1)  
    
    force2 = forceCalc(-0.13,odom,mapObs)   
    # end_point = dirxPlotter(force2,odom)
    #mapN_ = cv2.arrowedLine(mapN_, coord2pixel(odom + data_raw[0],map_), end_point,(255,0,255), 1)    
     
    force3 = forceCalc(30  ,odom,Targ)
    # end_point = dirxPlotter(force3,odom)
    #mapN_ = cv2.arrowedLine(mapN_, coord2pixel(odom + data_raw[0],map_), end_point,(0,255,0), 1)
    
    forceNet = force1 + force2 + force3
    # end_point = dirxPlotter(forceNet,odom)
    #mapN_ = cv2.arrowedLine(mapN_, coord2pixel(odom + data_raw[0],map_), end_point,(0,0,255), 2)
    
    forceNet = forceNet/np.sqrt(np.sum(forceNet**2))
    
    odom[0] = odom[0] + 2*forceNet[0]
    odom[1] = odom[1] + 2*forceNet[1]

    odom_ = copy.copy(odom)

    if i%5 == 0:
        odomList.append(odom_)
    i = i + 1
    
    # cv2.imshow('window_name', mapN_)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

dist = (np.sum((data - np.array(odom))**2, axis=1))**0.5
line_idx_end = int(np.argmin(dist) + 1)

data = np.delete(data, slice(line_idx,line_idx_end-1),0)
odomList = np.array(odomList)
data = np.insert(data,line_idx,odomList,axis=0)

print("--- %s seconds ---" % (time.time() - start_time))

for i in data:
    mapN = cv2.circle(mapN,coord2pixel(i + data_raw[0],map_),2,(0, 222, 49),-1)

cv2.imshow('window_name', mapN)
cv2.waitKey(0)
cv2.destroyAllWindows()

#%%
# def make_sparse(binary_image, k):
#     # Find all nonzero points in the binary image
#     nonzero_pts = np.nonzero(binary_image)
#     pts = list(zip(nonzero_pts[1], nonzero_pts[0]))
    
#     # Create a list to hold the sparse points
#     sparse_pts = []
    
#     # Iterate through all points and add every k-th point to the sparse list
#     for i in range(len(pts)):
#         if i % k == 0:
#             sparse_pts.append(pts[i])
    
#     # Create a blank image with the same shape as the input image
#     sparse_image = np.zeros_like(binary_image)
    
#     # Plot the sparse points on the new image
#     for pt in sparse_pts:
#         cv2.circle(sparse_image, pt, 2, (255, 255, 255), -1)
    
#     # Display the new image
#     plt.imshow(cv2.cvtColor(sparse_image, cv2.COLOR_BGR2RGB))
#     plt.show()

# make_sparse(map_,10)
#%%
# def publish_this(vel,ang_vel):
#     targ_vel = Pose2D()
#     targ_vel.x = vel
#     targ_vel.y = ang_vel
#     targ_vel_pub.publish(targ_vel)


# if __name__ == '__main__':

#     # Set up a publishers for the /TARG_VEL topic and state
#     targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)

#     curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, global_planner)

#     rospy.spin()