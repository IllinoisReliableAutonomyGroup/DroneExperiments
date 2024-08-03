'''
Geometrical functions
'''

import math
import numpy
import numpy as np

from transforms3d.euler import mat2euler
from DroneExperiments.vision.cam2drone import get_T_DC
from DroneExperiments.utils.transformations import quaternion_from_matrix, euler_from_quaternion, euler_from_matrix
T_DC = get_T_DC()

def getTransform(TS_indv):
    '''tag pose to transformation matrix'''

    pose_C = TS_indv
    pose_D = T_DC.dot(pose_C)
    # pose = numpy.linalg.inv(pose_D)
    # translation = pose[:3, 3]

    return pose_D

def getPos(T_tag):
    '''output the transformation matrix position as xyz tuple'''
    return numpy.array([T_tag[0, 3], T_tag[1, 3], T_tag[2, 3]])

def getRotation(T_Tag, useRadians=False):
    q = quaternion_from_matrix(T_Tag)
    print("R:", np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.)
    return q

def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def mag(x):
    return math.sqrt(sum(i**2 for i in x))

class tagDB:
    '''Database of all detected tags'''

    def __init__(self, debug=True):
        self.T_CamToWorld = numpy.array(numpy.eye((4)))
        self.T_CamToWorldPrev = numpy.array(numpy.eye((4)))
        self.tagPlacement = {}
        self.tagnewT = {}
        self.tagDuplicatesT = {}
        self.T_CamtoVeh = numpy.array(numpy.eye((4)))
        self.debug = debug
        #self.P_CamtoVeh = numpy.array([deltax, deltay, deltaz])

    def newFrame(self):
        '''Reset the duplicates for a new frame of tags'''
        self.tagDuplicatesT = {}
        self.tagnewT = {}

    def addTag(self, Ts_indv, id):
        '''Add tag to database'''
        # self.debug = True
        # pose_C = Ts_indv
        # pose_D = T_DC.dot(pose_C)
        # pose = np.linalg.inv(pose_D)
        # translation = pose[:3, 3]
        # print("translation",np.round(translation, 2))
        # q = quaternion_from_matrix(pose)
        # print("R:", np.round(np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.  ,2))

        if id not in self.tagPlacement:
            # tag is in cur camera frame
            T_TagToCam = getTransform(Ts_indv)
            #T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            self.tagnewT[id] = T_TagToCam
            if self.debug:
                print("New Tag ID {0} at pos {1}, rot {2}".format(id, getPos(self.tagnewT[id]).round(3), getRotation(self.tagnewT[id]).round(1)))
        else:
            # get tag's last pos, in camera frame
            T_TagToCam = getTransform(Ts_indv)
            #T_TagToCam[0:3, 3] = T_TagToCam[0:3, 3]
            
            # save tag positions in Camera frame at time t for the duplicate
            self.tagDuplicatesT[id] = T_TagToCam
            if self.debug:
                print("Duplicate Tag ID {0} at pos {1}, rot {2}".format(id, getPos(self.tagDuplicatesT[id]).round(3), getRotation(self.tagDuplicatesT[id]).round(1)))

    def debug_NED(self,Ts_indv):
        T_VehToWorld = self.T_CamtoVeh.dot(Ts_indv)
        T_VehToWorld = np.round(T_VehToWorld,3)
        # print("before inv \n",T_VehToWorld)
        T_VehToWorld = numpy.linalg.inv(T_VehToWorld)
        # print("after inv \n" , T_VehToWorld)

        # print("EULER FROM MATRIX",numpy.array(euler_from_matrix(T_VehToWorld[:3,:3], 'sxyz')) / numpy.pi  * 180.)

        posn = getPos(T_VehToWorld)
        rotn = getRotation(T_VehToWorld)
        # print("debug posn",np.round(posn,2))
        print("debug rotn", np.round(rotn,2))


    def getArduPilotNED(self, radians=False):
        '''get the vehicle's current position in ArduPilot NED format,
        noting that Apriltag is coords are right, fwd, up (ENU)
        This assumes camera is on top of vehicle, bottom of camera facing fwd'''

        # T_VehToWorld = self.T_CamtoVeh.dot(self.T_CamToWorld)
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld
        T_VehToWorld = numpy.linalg.inv(T_VehToWorld)

        # print("EULER FROM MATRIX",numpy.array(euler_from_matrix(T_VehToWorld[:3,:3], 'sxyz')) / numpy.pi  * 180.)

        posn = getPos(T_VehToWorld)
        qx, qy, qz, qw = getRotation(T_VehToWorld, radians)
  
        return ((posn[0], posn[1], posn[2]), (qw, qy, qx, -qz))
    

    def getArduPilotNEDDelta(self, radians=False):
        '''get the vehicle's delta (current - prev frame) position in ArduPilot NED format,
        noting that Apriltag is coords are right, fwd, up (ENU)
        This assumes camera is on top of vehicle, bottom of camera facing fwd'''
        T_VehToWorldDelta = (self.T_CamtoVeh @ self.T_CamToWorld) - (self.T_CamtoVeh @ self.T_CamToWorldPrev)
        # print("rot_cur",np.round(getRotation(self.T_CamToWorld),2))
        # print("rot_prev",np.round(getRotation(self.T_CamToWorldPrev),2))
        # print("rot_calc",np.round(getRotation(self.T_CamToWorld-self.T_CamToWorldPrev),2))
        # print("DELTA PRIOR \n", np.round(T_VehToWorldDelta,2))
              # if numpy.linalg.det(T_VehToWorldDelta)==0:

        # for i in range(3):
        #     for j in range(4):
        #         if i==j:
        #             pass
        #         else:
        #             T_VehToWorldDelta[i][j] = -1*T_VehToWorldDelta[i][j]

        # T_VehToWorldDelta = numpy.linalg.inv(T_VehToWorldDelta)
        # print("DELTA AFT \n",np.round(T_VehToWorldDelta,2))
        posn = getPos(T_VehToWorldDelta)
        qx, qy, qz, qw = getRotation(T_VehToWorldDelta, radians)

        #return tuple of rotation and position
        return ((posn[1], posn[0], -posn[2]), (qw, qy, qx, -qz))

    def getCurrentPosition(self):
        '''get the vehicle's current position in xyz'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld
        T_VehToWorld = numpy.linalg.inv(T_VehToWorld)
        #[0:3, 3]
        return getPos(T_VehToWorld)

    def getCurrentRotation(self, radians=False):
        '''get the vehicle's current position in xyz'''
        T_VehToWorld = self.T_CamtoVeh @ self.T_CamToWorld
        T_VehToWorld = numpy.linalg.inv(T_VehToWorld)
        return getRotation(T_VehToWorld, radians)

    def getTagdb(self):
        '''get coords of all tags by axis'''
        #xcoord = []
        #for tagid, tag in self.tagPlacement.items():
        #    xcoord.append(tag[0, axis])
        return self.tagPlacement

    def getTagPoints(self, axis):
        xcoord = []
        for tagid, tag in self.tagPlacement.items():
            xcoord.append(tag[axis,3])
        return xcoord

    def getBestTransform(self):
        '''Given the current self.tagDuplicatesT, what is the best fitting transform
        back to self.tagPlacement'''
        # get the least cost transform from the common points at time t-1 to time t
        # cost is the sum of position error between the common points, with the t-1 points
        # projected forward to t
        if len(self.tagDuplicatesT) == 0 and len(self.tagPlacement) != 0:
            print("WARNING: No tags in view")
        elif len(self.tagDuplicatesT) == 1:
            # print("WARNING: Only 1 duplicate tag in view")
            pass
            
        if len(self.tagDuplicatesT) > 0:
            # print("RANNN")
            bestTransform = numpy.array( numpy.eye((4)) )
            lowestCost = 999
            # Use each tag pair as a guess for the correct transform - lowest cost wins
            for tagid, tagT in self.tagDuplicatesT.items():
                if self.debug:
                    print("Trying tag {0}".format(tagid))
                # t is the time now, t-1 is the previous frame - where T_CamToWorld is at this point
                # tag is the same world position at both orig and duplicate
                # World frame is time-independent
                # TOrig(World<-Tag) = T(World <- Cam_t) * TDup(Cam_t<-Tag)
                # and:
                # T(World <- Cam_t-1) = T(World <- Cam_t) * T(Cam_t <- Cam_t-1)
                #
                # Thus
                # T(World <- Cam_t) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1
                # then
                # TOrig(World<-Tag) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1 * TDup(Cam_t<-Tag)
                # Thus
                # T(Cam_t <- Cam_t-1) = TDup(Cam_t<-Tag) * TOrig(World<-Tag)^-1 * T(World <- Cam_t-1)
                Ttprevtocur =  tagT @ numpy.linalg.inv(self.tagPlacement[tagid]) @ self.T_CamToWorld
                # print("tagT =\n{0}".format(tagT))
                #print("self.tagPlacement[tagid]^-1 =\n{0}".format(numpy.linalg.inv(self.tagPlacement[tagid])))
                #print("self.T_CamToWorld(old) =\n{0}".format(self.T_CamToWorld))
                # and figure out summed distances between transformed new point to old (in world frame)
                summeddist = 0
                for tagidj, tagTj in self.tagDuplicatesT.items():
                    # PosDelta = TOrig(World<-Tag) * [0,0,0] - T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1 * TDup(Cam_t<-Tag) * [0,0,0]
                    PosDelta = self.tagPlacement[tagidj] @ [[0],[0],[0],[1]] - self.T_CamToWorld @ numpy.linalg.inv(Ttprevtocur) @ tagTj @ [[0],[0],[0],[1]]
                    #print(self.tagPlacement[tagidj] - (self.T_CamToWorld @ numpy.linalg.inv(Ttprevtocur) @ tagTj))
                    summeddist += mag(PosDelta)
                    
                #print("Tag rot (Tag {1})= {0} deg".format(getRotation(Ttprevtocur), tagid))
                #print("Tag T (Tag {1})= {0} deg".format([Ttprevtocur[0,3],Ttprevtocur[1,3],Ttprevtocur[2,3]], tagid))
                if lowestCost > summeddist: #and mag(getPos(Ttprevtocur)) < 2:
                    if self.debug:
                        print("Using tag {0} with error {1:.3f}m".format(tagid, summeddist))
                    lowestCost = summeddist
                    bestTransform = Ttprevtocur
                    
            #is it a based transform? If more than 5cm delta or summed 15deg rot
            # print((euler_angles_from_rotation_matrix(self.T_CamToWorld)))
            #print((euler_angles_from_rotation_matrix(numpy.linalg.inv(bestTransform))))
            #print(mag(getPos(bestTransform)))
            #print("self.bestTransform^-1 =\n{0}".format(numpy.linalg.inv(bestTransform)))
            #print("Pos {0}, Rot = {1}".format(getPos(bestTransform), getRotation(bestTransform)))
            #print("Pos {0}, Rot = {1}".format(getPos(numpy.linalg.inv(bestTransform)), getRotation(numpy.linalg.inv(bestTransform))))
            #print("self.bestTransform^-1 =\n{0}".format(numpy.linalg.inv(bestTransform)))
            #f mag(getPos(bestTransform)) > 0.1:
            #    print("Bad Translation")
            #    bestTransform = numpy.array( numpy.eye((4)) )
            #else:
            #    print("Tag =\n{0}".format(bestTransform))
            #    print("Tag =\n{0}".format(numpy.linalg.inv(bestTransform)))                    
            #we have the lowest cost transform (need inverse)
            # T(World <- Cam_t) = T(World <- Cam_t-1) * T(Cam_t <- Cam_t-1)^-1
            #print(numpy.linalg.inv(bestTransform))
            if lowestCost > 1:
                # print("WARNING: bad position estimate. Ignoring this frame.")
                # change
                pass
            else:
                self.T_CamToWorldPrev = self.T_CamToWorld
                self.T_CamToWorld = self.T_CamToWorld @ numpy.linalg.inv(bestTransform)
                #print("self.T_CamToWorld(new) =\n{0}".format(self.T_CamToWorld))
                #print((euler_angles_from_rotation_matrix(self.T_CamToWorld)))
                if self.debug:
                    print("Delta {0}, Rot = {1}".format(getPos(bestTransform).round(3), getRotation(bestTransform).round(1)))
            if self.debug:
                print("New Pos {0}, Rot = {2} with {1} tags".format(self.getCurrentPosition().round(3), len(self.tagDuplicatesT), self.getCurrentRotation().round(1)))

        # finally add any new tags
        for tagid, tagT in self.tagnewT.items(): 
            # T(World <- tag) = T(World <- Cam_t) * T(Cam_t <- tag)
            self.tagPlacement[tagid] = self.T_CamToWorld @ tagT
            # print("camtoworld",self.T_CamToWorld)
            #dist = math.sqrt(math.pow(tagT[0,3], 2) + math.pow(tagT[1,3], 2) + math.pow(tagT[2,3], 2))
            #print("Added Tag ID {0} at {2:.3}, T(world) =\n {1}".format(tagid, self.tagPlacement[tagid].round(3), dist))
            #print("Added Tag ID {0} at pos {1}, rot {2}".format(tagid, getPos(self.tagPlacement[tagid]), getRotation(self.tagPlacement[tagid])))
            if self.debug:
                print("Added Tag ID {0} at pos {1}, rot {2}".format(tagid, getPos(self.tagPlacement[tagid]).round(3), getRotation(self.tagPlacement[tagid]).round(1)))
            #         print("Pos {0}, Rot = {3} in {1} with {4}/{2} tags".format(tagPlacement.getCurrentPosition().round(3), file, len(tags), tagPlacement.getCurrentRotation().round(0), tagsused))
