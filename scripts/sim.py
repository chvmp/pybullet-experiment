import rospy
from sensor_msgs.msg import JointState

import pybullet as p
import time
import pybullet_data

class Champ:
    def __init__(self):
        rospy.Subscriber("joint_states", JointState, self.jointsCallback)

        self.rosStarted = False
        self.jointPositions = []
        self.jointNames = []

        while not self.rosStarted:
            rospy.loginfo("Waiting for ros to start")
        
        #source https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        champStartPos = [0,0,1]
        champStartOrientation = p.getQuaternionFromEuler([0,0,0])
        champ = p.loadURDF("../urdf/champ.urdf",champStartPos, champStartOrientation)
        numJoints = p.getNumJoints(champ)
        
        bulletJointIndices = [0] * 12

        #iterate all joints in the URDF
        for jointIndex in range(numJoints):
            jointInfo= p.getJointInfo(champ, jointIndex)
            jointName = jointInfo[1]
            jointName = jointName.decode("utf-8")
        
            if jointName in self.jointNames:
                #if the current joint in the URDF is one of the joint name received from controller
                #save the index of the that joint in the received joint name's index
                bulletJointIndices[self.jointNames.index(jointName)] = jointIndex

        while True:
            p.setJointMotorControlArray(champ, bulletJointIndices, p.POSITION_CONTROL, self.jointPositions)
            p.stepSimulation()
            time.sleep(1./240.)

        p.disconnect()

    def jointsCallback(self, joints):
        if self.rosStarted == False:
            self.rosStarted = True
            self.jointNames = joints.name

        self.jointPositions = joints.position

if __name__ == '__main__':
    rospy.init_node('bullet_sim', anonymous=True)

    c = Champ()
    rospy.spin()