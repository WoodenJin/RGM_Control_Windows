import tinyik as ik
import pybullet as p
import pybullet_data
from time import sleep, time
import os
import numpy as np
from scipy import interpolate
import random
from math import *


PI = pi
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


def get_current_path(t):
    """
    return current time angle trajectory point
    :param t:
    :return:
    """
    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle, tck_upper
    left_hip = interpolate.splev(t, tck_leg1_hip, der=0)
    left_knee = interpolate.splev(t, tck_leg1_knee, der=0)
    left_ankle = interpolate.splev(t, tck_leg1_ankle, der=0)
    right_hip = interpolate.splev(t, tck_leg2_hip, der=0)
    right_knee = interpolate.splev(t, tck_leg2_knee, der=0)
    right_ankle = interpolate.splev(t, tck_leg2_ankle, der=0)
    upper = interpolate.splev(t, tck_upper, der=0)
    return np.array([upper, -left_hip, -left_knee, -left_ankle, -right_hip, -right_knee, -right_ankle])


def trajectory_produce(h0, a, b, period, amp, phase):
    """
    This is used to produce a trajectory function for leg robot
    :param h0: the height of the hip
    :param a: the step size along the z direction
    :param b: the time step along the x direction
    :param period: time period of one gait
    :param amp:  Amp is the upper body swing amplitude
    :return: interpolation function
    """

    # ============================================
    # define the robot kinematic model
    thigh_length = 0.353  # length of the thigh, the static parameter
    shank_length = 0.350  # length of the shank, the static parameter

    # suppose leg1 is the right leg, supporting leg
    # suppose leg2 is the left leg, swinging leg
    leg1 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg1.angles = [-0.01, 0.01]  # init the configuration of the leg1

    leg2 = ik.Actuator(['y', [0, 0, -thigh_length], 'y', [0, 0, -shank_length]])
    leg2.angles = [-0.01, 0.01]  # init the configuration of the leg2

    sample_num = 10  # number of the sampling points in half cycle

    # the first half cycle
    leg1_aim_x = np.linspace(0, -b, sample_num)
    leg1_aim_y = np.zeros(sample_num)
    leg1_aim_z = np.ones(sample_num) * -h0
    leg1_aim = np.stack((leg1_aim_x, leg1_aim_y, leg1_aim_z), axis=-1)
    leg1_angle = np.zeros((sample_num, 2))
    theta_temp = np.linspace(0, pi, sample_num)
    curve_x = a * np.sin(theta_temp)
    curve_y = -b * np.cos(theta_temp)

    leg2_aim_x = leg1_aim_x + curve_y
    leg2_aim_y = leg1_aim_y
    leg2_aim_z = leg1_aim_z + curve_x
    leg2_aim = np.stack((leg2_aim_x, leg2_aim_y, leg2_aim_z), axis=-1)
    leg2_angle = np.zeros((sample_num, 2))

    for i in range(sample_num):
        leg1.ee = leg1_aim[i, :]
        leg1_angle[i, :] = leg1.angles
        leg2.ee = leg2_aim[i, :]
        leg2_angle[i, :] = leg2.angles

    leg1_angle = np.stack((leg1_angle[:, 0], leg1_angle[:, 1]), axis=-1)
    leg2_angle = np.stack((leg2_angle[:, 0], leg2_angle[:, 1]), axis=-1)
    leg1_hip = leg1_angle[:, 0]
    leg1_knee = leg1_angle[:, 1]
    leg1_ankle = -(leg1_angle[:, 0] + leg1_angle[:, 1])
    leg2_hip = leg2_angle[:, 0]
    leg2_knee = leg2_angle[:, 1]
    leg2_ankle = -(leg2_angle[:, 0] + leg2_angle[:, 1])
    angle_control = np.stack((leg1_hip, leg1_knee, leg1_ankle, leg2_hip, leg2_knee, leg2_ankle), axis=-1)

    # the second half cycle
    angle_control_2 = np.hstack((angle_control[:, 3:6], angle_control[:, 0:3]))

    # total period
    angle_control = np.vstack((angle_control, angle_control_2))

    # mapping to the real robot configuration
    angle_control[:, 0:3] = -angle_control[:, 0:3]
    angle_control[:, 0] = -angle_control[:, 0]
    angle_control[:, 3] = -angle_control[:, 3]
    temp = np.copy(angle_control[:, 0:3])
    angle_control[:, 0:3] = np.copy(angle_control[:, 3:6])
    angle_control[:, 3:6] = np.copy(temp)
    angle_control = angle_control

    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle, tck_upper

    # interpolation
    time_array = np.linspace(0, period, sample_num * 2)
    tck_leg1_hip = interpolate.splrep(time_array, angle_control[:, 0], s=0)
    tck_leg1_knee = interpolate.splrep(time_array, angle_control[:, 1], s=0)
    tck_leg1_ankle = interpolate.splrep(time_array, angle_control[:, 2], s=0)
    tck_leg2_hip = interpolate.splrep(time_array, angle_control[:, 3], s=0)
    tck_leg2_knee = interpolate.splrep(time_array, angle_control[:, 4], s=0)
    tck_leg2_ankle = interpolate.splrep(time_array, angle_control[:, 5], s=0)
    tck_upper = interpolate.splrep(time_array, amp * np.sin(2 * pi * time_array / period + phase), s=0)

    return None


global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle, tck_upper


def main():
    # produce trajectory
    h0 = 0.66
    # r0 = 0.05
    r0 = 0.08
    # a = 0.07
    # b = 0.20
    a = 0.03
    b = 0.04
    period = 2
    step = 0.02
    amp = pi / 12  # unit is degree
    phase = pi / 6  # the phase between 6-dof leg motion and the torso motion
    # angle_control = trajectory_produce(h0, r0, period, step)
    trajectory_produce(h0, a, b, period, amp, phase)
    global tck_leg1_hip, tck_leg1_knee, tck_leg1_ankle, tck_leg2_hip, tck_leg2_knee, tck_leg2_ankle, tck_upper

    # control robot in the simulation environment
    init_pos = [0, 0, h0 + 0.033]
    # robot = BipedRobot(init_pos, step=0.01, is_gui=False)
    robot = BipedRobot(init_pos, step=0.005, is_gui=True)
    angle_control = get_current_path(0)
    # print(angle_control)
    robot.reset_joint_state(angle_control)
    start = time()
    # sleep(100)

    # data buffer, used to save the simulation result.
    # the structure is [time, pos_x, pos_y, pos_z, ori_1, ori_2, ori_3, ori_4]
    pos, ori = p.getBasePositionAndOrientation(robot.biped_robot)
    euler = p.getEulerFromQuaternion(ori)
    motion_data = np.array([0, pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]])

    while True:
        current_time = time() - start
        now = current_time % period
        angle_control = get_current_path(now)
        # angle_control[0] = 0
        robot.take_action(list(angle_control))
        sleep(0.01)
        pos, ori = p.getBasePositionAndOrientation(robot.biped_robot)
        euler = p.getEulerFromQuaternion(ori)
        motion_data = np.vstack((motion_data, np.array([current_time, pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]])))
        # only record 10 seconds data
        if current_time > 100:
            break
    # np.save('motion_data/phase_'+str(phase)+'amp_'+str(amp),motion_data)


if __name__ == '__main__':
    main()
