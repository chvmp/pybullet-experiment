import pybullet as p
import numpy as np

import time
import pybullet_data
from champ.types import Pose, GaitConfig, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.kinematics import Kinematics

class Champ:
    def __init__(self):

        gait_config = GaitConfig()
        gait_config.pantograph_leg = False
        gait_config.max_linear_velocity_x = 0.5
        gait_config.max_linear_velocity_y = 0.25
        gait_config.max_angular_velocity_z = 1.0
        gait_config.swing_height = 0.04
        gait_config.stance_depth = 0.0
        gait_config.stance_duration = 0.25
        gait_config.nominal_height = 0.2
        gait_config.knee_orientation = ">>"

        quadruped = Base()
        quadruped.gait_config = gait_config

        quadruped.lf.hip.set_origin(0.175, 0.105, 0)
        quadruped.lf.upper_leg.set_origin(0, 0.06, 0)
        quadruped.lf.lower_leg.set_origin(0, 0, -0.141)
        quadruped.lf.foot.set_origin(0, 0, -0.141)
        
        quadruped.rf.hip.set_origin(0.175, -0.105, 0)
        quadruped.rf.upper_leg.set_origin(0, -0.06, 0)
        quadruped.rf.lower_leg.set_origin(0, 0, -0.141)
        quadruped.rf.foot.set_origin(0, 0, -0.141)

        quadruped.lh.hip.set_origin(-0.175, 0.105, 0)
        quadruped.lh.upper_leg.set_origin(0, 0.06, 0)
        quadruped.lh.lower_leg.set_origin(0, 0, -0.141)
        quadruped.lh.foot.set_origin(0, 0, -0.141)

        quadruped.rh.hip.set_origin(-0.175, -0.105, 0)
        quadruped.rh.upper_leg.set_origin(0, -0.06, 0)
        quadruped.rh.lower_leg.set_origin(0, 0, -0.141)
        quadruped.rh.foot.set_origin(0, 0, -0.141)

        controller = CheetahOne(quadruped, gait_config)
        ik = Kinematics(quadruped)

        req_pose = Pose()
        req_pose.position.z = gait_config.nominal_height

        req_vel = Velocities()
        # req_vel.linear.x = 1.0
        req_vel.linear.y = 1.0
        # req_vel.angular.z = 1.0

        self.jointPositions = []
        self.jointNames = ['lf_hip_joint',
                            'lf_upper_leg_joint',
                            'lf_lower_leg_joint',
                            'rf_hip_joint',
                            'rf_upper_leg_joint',
                            'rf_lower_leg_joint',
                            'lh_hip_joint',
                            'lh_upper_leg_joint',
                            'lh_lower_leg_joint',
                            'rh_hip_joint',
                            'rh_upper_leg_joint',
                            'rh_lower_leg_joint']

        #source https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        champStartPos = [0,0, 0.282]
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
            foot_positions = controller.walk(req_pose, req_vel)
            joint_positions = ik.inverse(foot_positions)

            p.setJointMotorControlArray(champ, bulletJointIndices, p.POSITION_CONTROL, list(joint_positions))
            p.stepSimulation()
            # time.sleep(1./240.)

        p.disconnect()


if __name__ == '__main__':
    c = Champ()
