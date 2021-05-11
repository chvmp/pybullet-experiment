import pybullet as p
import numpy as np

import time
import pybullet_data
from champ.types import Pose, GaitConfig, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.body_controller import PoseGenerator
from champ.kinematics import Kinematics
from champ.robots.profile import AnymalC as anymal_c

class Champ:
    def __init__(self):

 

        quadruped = Base(anymal_c)

        controller = CheetahOne(quadruped, anymal_c.gait_config)
        ik = Kinematics(quadruped, anymal_c.gait_config.knee_orientation)

        req_pose = Pose()
        req_pose.position.z = anymal_c.gait_config.nominal_height

        req_vel = Velocities()

        self.jointPositions = []
        self.jointNames = ['LF_HAA',
                            'LF_HFE',
                            'LF_KFE',
                            'RF_HAA',
                            'RF_HFE',
                            'RF_KFE',
                            'LH_HAA',
                            'LH_HFE',
                            'LH_KFE',
                            'RH_HAA',
                            'RH_HFE',
                            'RH_KFE']


        #source https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        champStartPos = [0,0, 0.6]
        champStartOrientation = p.getQuaternionFromEuler([0,0,0])
        champ = p.loadURDF(anymal_c.urdf,champStartPos, champStartOrientation)
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

        roll_gen = PoseGenerator(0.4, 0.5)
        pitch_gen = PoseGenerator(0.3, 0.5, phase_shift=1.5708)
        yaw_gen = PoseGenerator(0.3, 0.5)
        for i in range(3500):
         
            if i == 0:
                print("Linear Velocity X")
                req_vel.linear.x = 1.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 0.0
            elif i == 500:
                print("Linear Velocity Y")
                req_vel.linear.x = 0.0
                req_vel.linear.y = 1.0
                req_vel.angular.z = 0.0
            elif i == 1000:
                print("Angular Velocity Z")
                req_vel.linear.x = 0.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 1.0
            elif i == 1500:
                print("Turning")
                req_vel.linear.x = 1.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 1.0
            elif i == 2000:
                print("Linear Velocity XZ")
                req_vel.linear.x = 1.0
                req_vel.linear.y = 1.0
                req_vel.angular.z = 0.0
            elif i ==  2500:
                print("Pose Command")
            elif i > 2500: 
                req_vel.linear.x = 0.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 0.0
                req_pose.orientation.roll = roll_gen.sine()
                req_pose.orientation.pitch = pitch_gen.sine()
                req_pose.orientation.yaw = yaw_gen.sine()


            foot_positions = controller.walk(req_pose, req_vel)
            joint_positions = ik.inverse(foot_positions)

            p.setJointMotorControlArray(champ, bulletJointIndices, p.POSITION_CONTROL, list(joint_positions))
            p.stepSimulation()
            # time.sleep(1./240.)

        p.disconnect()


if __name__ == '__main__':
    c = Champ()
