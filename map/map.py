from DroneExperiments.vision.cam2drone import get_T_DC
T_DC = get_T_DC()
import numpy as np
from DroneExperiments.utils.transformations import quaternion_from_matrix, euler_from_quaternion
import time

class markerDB:
    'Database of detected markers'

    def __init__(self):
        self.marker_recorded = {}
        pass

    def addMarker(self, marker):
        if marker.id not in self.marker_recorded:
            pass
        else:
            pass

    def getArduPilotNED(self, radians=False):
        '''get the vehicle's current position in ArduPilot NED format,
        noting that Apriltag is coords are right, fwd, up (ENU)
        This assumes camera is on top of vehicle, bottom of camera facing fwd'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld
        posn = getPos(T_VehToWorld)
        rotn = getRotation(T_VehToWorld, radians)

        #return tuple of rotation and position
        return ((posn[1], posn[0], -posn[2]), (rotn[1], rotn[0], -rotn[2]))

    def getArduPilotNEDDelta(self, radians=False):
        '''get the vehicle's delta (current - prev frame) position in ArduPilot NED format,
        noting that Apriltag is coords are right, fwd, up (ENU)
        This assumes camera is on top of vehicle, bottom of camera facing fwd'''
        T_VehToWorldDelta = (self.T_CamtoVeh @ self.T_CamToWorld) - (self.T_CamtoVeh @ self.T_CamToWorldPrev)
        posn = getPos(T_VehToWorldDelta)
        rotn = getRotation(T_VehToWorldDelta, radians)

        #return tuple of rotation and position
        return ((posn[1], posn[0], -posn[2]), (rotn[1], rotn[0], -rotn[2]))

    def getPose(self, Ts, ids):
        for i in range(len(ids)):
            pose_C = Ts[i]
            pose_D = T_DC.dot(pose_C)
            pose = np.linalg.inv(pose_D)
            translation = pose[:3, 3]
            q = quaternion_from_matrix(pose)

            print("t:", translation)
            print("R:", np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.)

            time_usec = int(time.time() * 1e6)

            return time_usec, translation, q
 
