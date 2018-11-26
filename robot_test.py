import numpy as np
import random
import pybullet as p
import pybullet_data
from time import sleep
import os
from math import *

JOINT_ID = [1, 3, 6, 8, 9, 12, 14]


class BipedRobot(object):

    def __init__(self, init_pos, step=0.01, is_gui=True):
        """
        init the BipedRobot simulation object
        :param IsGUI: bool, True, open the graphical interface, False, do not open the graphical interface
        """
        # connect the client
        if is_gui:
            self.physicsClient = p.connect(p.GUI, options="--opengl3")
        else:
            self.physicsClient = p.connect(p.DIRECT)

        # add the ground into the simulation environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF to load the plane
        self.planeId = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(os.getcwd())
        self.biped_robot = p.loadURDF("Real_robot.urdf", init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        # self.biped_robot = p.loadURDF("biped_robot_mirror.urdf", init_pos, p.getQuaternionFromEuler([0, 0, 0]))
        p.setGravity(0, 0, -10)  # set the gravity of the simulation environment
        self.step = step  # set the time step, the default value is 0.01s
        p.setTimeStep(self.step)
        self.index = 0  # index increase every time simulation, and if index == frequency, clear to zero
        self.init_pos = init_pos

    def take_action(self, angle_control):
        """
        this method is used to drive the biped robot, control mode is position control
        :param angle_control: a angle list, [left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle]
        :return: none
        """
        self.index += 1
        """
        p.setJointMotorControl2(self.biped_robot, 0, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[0])  # left hip joint
        p.setJointMotorControl2(self.biped_robot, 2, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[1])  # left knee joint
        p.setJointMotorControl2(self.biped_robot, 4, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[2])  # left ankle joint
        p.setJointMotorControl2(self.biped_robot, 6, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[3])  # right hip joint
        p.setJointMotorControl2(self.biped_robot, 8, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[4])  # right knee joint
        p.setJointMotorControl2(self.biped_robot, 10, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[5])  # right ankle joint
        """
        p.setJointMotorControlArray(self.biped_robot, JOINT_ID,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=angle_control)
        p.stepSimulation()
        sleep(self.step)

    def reset_joint_state(self, init_angle):
        """
        this is used to reset the biped robot joint
        :param init_angle: initial joint angle
        :return: none
        """
        self.index = 0
        # p.resetJointState(self.biped_robot,)
        # p.resetJointState(self.biped_robot, 0, init_angle[0])
        # p.resetJointState(self.biped_robot, 2, init_angle[1])
        # p.resetJointState(self.biped_robot, 4, init_angle[2])
        # p.resetJointState(self.biped_robot, 6, init_angle[3])
        # p.resetJointState(self.biped_robot, 8, init_angle[4])
        # p.resetJointState(self.biped_robot, 10, init_angle[5])
        for i in range(len(JOINT_ID)):
            p.resetJointState(self.biped_robot, JOINT_ID[i], init_angle[i])

    def attack_define(self, frequency, number, strength):
        self.attack_frequency = frequency
        self.attack_number = number
        self.attack_strength = strength

    def single_attack(self):
        V = 3
        robot_base_position = p.getBasePositionAndOrientation(self.biped_robot)
        robot_base_position = robot_base_position[0]
        robot_base_position = [robot_base_position[0] + self.init_pos[0],
                               robot_base_position[1] + self.init_pos[1],
                               robot_base_position[2] + self.init_pos[2] - 0.3]
        Velocity = [random.random() * 5 - 2.5, random.random() * 5 - 2.5, random.random() * 5 - 2.5]
        mass = self.attack_strength / sqrt(Velocity[0] ** 2 + Velocity[1] ** 2 + Velocity[2] ** 2)
        mass = mass / V
        sphereRadius = 0.02 * random.randint(2, 4)
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = -1
        # basePosition = robot_base_position + Velocity
        basePosition = [robot_base_position[0] + Velocity[0],
                        robot_base_position[1] + Velocity[1],
                        robot_base_position[2] + Velocity[2]]
        baseOrientation = [0, 0, 0, 1]
        sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition, baseOrientation)
        p.changeDynamics(sphereUid, -1, spinningFriction=0.001, rollingFriction=0.001, linearDamping=0.0,
                         contactStiffness=10000, contactDamping=0)
        p.resetBaseVelocity(sphereUid,
                            [-V * Velocity[0] * random.uniform(0.8, 1.2),
                             -V * Velocity[1] * random.uniform(0.8, 1.2),
                             -V * Velocity[2] * random.uniform(0.8, 1.2)])

    def take_action_with_sphere_attack(self, angle_control):
        self.index += 1
        p.setJointMotorControl2(self.biped_robot, 0, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[0])  # left hip joint
        p.setJointMotorControl2(self.biped_robot, 2, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[1])  # left knee joint
        p.setJointMotorControl2(self.biped_robot, 4, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[2])  # left ankle joint
        p.setJointMotorControl2(self.biped_robot, 6, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[3])  # right hip joint
        p.setJointMotorControl2(self.biped_robot, 8, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[4])  # right knee joint
        p.setJointMotorControl2(self.biped_robot, 10, controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_control[5])  # right ankle joint

        if not (self.index % self.attack_frequency):
            self.index = 0  # clear index to zero
            for i in range(self.attack_number):
                self.single_attack()

        p.stepSimulation()
        sleep(self.step)

    def JointMotorTorque(self):

        motortorque = np.zeros(6)
        joint_index = [0, 2, 4, 6, 8, 10]
        for i in range(6):
            temp = p.getJointState(self.biped_robot, joint_index[i])
            motortorque[i] = temp[3]

        print(motortorque)


def main():
    h0 = 0.66
    init_pos = [0, 0, h0 + 0.1]
    robot = BipedRobot(init_pos, step=0.01, is_gui=True)
    angle_control = np.array([0, 0, 0, 0, 0, 0, -1])
    robot.reset_joint_state(angle_control)
    while True:
        pass


main()

