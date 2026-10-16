"""Body-to-world orientation, ZYX Euler convention (degrees)."""
import numpy as np


def yaw_rotation(degrees):
    a=np.radians(degrees)
    return np.array([[np.cos(a),-np.sin(a),0],
                     [np.sin(a),np.cos(a),0],[0,0,1]])


def imu_rotation(roll, pitch, yaw):
    r,p=np.radians([roll,pitch])
    rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    return yaw_rotation(yaw) @ ry @ rx
