"""
Class of n-link arm in 3D
Author: Takayuki Murooka (takayuki5168)
Reference: https://github.com/AtsushiSakai/PythonRobotics
"""
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random
from tqdm import tqdm
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import geometry_msgs.msg


PLOT_AREA = 0.5


class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self):
        # theta = self.joint_angle_
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]

        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])

        return trans

    @staticmethod
    def basic_jacobian(trans_prev, ee_pos):
        pos_prev = np.array(
            [trans_prev[0, 3], trans_prev[1, 3], trans_prev[2, 3]])
        z_axis_prev = np.array(
            [trans_prev[0, 2], trans_prev[1, 2], trans_prev[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis_prev, ee_pos - pos_prev), z_axis_prev))
        return basic_jacobian


class NLinkArm:
    def __init__(self, dh_params_list):
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

    @staticmethod
    def convert_joint_angles_sim_to_mycobot(joint_angles):
        """convert joint angles simulator to mycobot

        Args:
            joint_angles ([float]): [joint angles(radian)]

        Returns:
            [float]: [joint angles calculated(radian)]
        """
        conv_mul = [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
        conv_add = [0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, 0.0]

        joint_angles = [joint_angles * conv_mul for (joint_angles, conv_mul) in zip(joint_angles, conv_mul)]
        joint_angles = [joint_angles + conv_add for (joint_angles, conv_add) in zip(joint_angles, conv_add)]

        joint_angles_lim = []
        for joint_angle in joint_angles:
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles_lim.append(joint_angle)

        return joint_angles_lim

    def transformation_matrix(self):
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
        return trans

    def forward_kinematics(self, plot=False):
        trans = self.transformation_matrix()

        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        alpha, beta, gamma = self.euler_angle()

        if plot:
            self.fig = plt.figure()
            self.ax = Axes3D(self.fig)

            x_list = []
            y_list = []
            z_list = []

            trans = np.identity(4)

            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])
            for i in range(len(self.link_list)):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
                x_list.append(trans[0, 3])
                y_list.append(trans[1, 3])
                z_list.append(trans[2, 3])

            self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                         mew=0.5)
            self.ax.plot([0], [0], [0], "o")

            self.ax.set_xlim(-PLOT_AREA, PLOT_AREA)
            self.ax.set_ylim(-PLOT_AREA, PLOT_AREA)
            self.ax.set_zlim(-PLOT_AREA, PLOT_AREA)

            plt.show()

        return [x, y, z, alpha, beta, gamma]

    def basic_jacobian(self):
        ee_pos = self.forward_kinematics()[0:3]
        basic_jacobian_mat = []

        trans = np.identity(4)
        for i in range(len(self.link_list)):
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
            trans = np.dot(trans, self.link_list[i].transformation_matrix())

        return np.array(basic_jacobian_mat).T

    def inverse_kinematics(self, ref_ee_pose, plot=False):
        for cnt in range(500):
            ee_pose = self.forward_kinematics()
            diff_pose = np.array(ref_ee_pose) - ee_pose

            basic_jacobian_mat = self.basic_jacobian()
            alpha, beta, gamma = self.euler_angle()

            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            self.update_joint_angles(theta_dot / 100.)

        if plot:
            self.fig = plt.figure()
            self.ax = Axes3D(self.fig)

            x_list = []
            y_list = []
            z_list = []

            trans = np.identity(4)

            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])
            for i in range(len(self.link_list)):
                trans = np.dot(trans, self.link_list[i].transformation_matrix())
                x_list.append(trans[0, 3])
                y_list.append(trans[1, 3])
                z_list.append(trans[2, 3])

            self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                         mew=0.5)
            self.ax.plot([0], [0], [0], "o")

            self.ax.set_xlim(-PLOT_AREA, PLOT_AREA)
            self.ax.set_ylim(-PLOT_AREA, PLOT_AREA)
            self.ax.set_zlim(-PLOT_AREA, PLOT_AREA)

            self.ax.plot([ref_ee_pose[0]], [ref_ee_pose[1]], [ref_ee_pose[2]],
                         "o")
            plt.show()

    def euler_angle(self):
        trans = self.transformation_matrix()

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def send_angles(self, joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] = joint_angle_list[i]

    def update_joint_angles(self, diff_joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] += diff_joint_angle_list[i]

    def get_angles(self):
        joint_angles = []
        for i in range(len(self.link_list)):
            joint_angle = self.link_list[i].dh_params_[0]
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles.append(joint_angle)

        return joint_angles

    def plot(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                     mew=0.5)
        self.ax.plot([0], [0], [0], "o")

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.set_xlim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_ylim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_zlim(-PLOT_AREA, PLOT_AREA)
        plt.show()


def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)

import multiprocessing as mp

## added
def calculate_position(angle):
    mycobot_sim = NLinkArm([[0., math.pi / 2, 0, 0.13156],
                            [0., 0., -0.1104, 0.],
                            [0., 0., -0.096, 0.],
                            [0., math.pi / 2, 0., 0.06639],
                            [0., -math.pi / 2, 0., 0.07318],
                            [0., 0., 0., 0.0436]])

    mycobot_sim.send_angles(
        mycobot_sim.convert_joint_angles_sim_to_mycobot(angle))
    return mycobot_sim.forward_kinematics()

def publish_points(points):
    pub = rospy.Publisher('mycobot_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        markerArray = MarkerArray()

        for i, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "link1"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.005
            marker.scale.y = 0.005
            marker.scale.z = 0.005
            marker.color.a = 0.6
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.id = i

            markerArray.markers.append(marker)

        pub.publish(markerArray)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('mycobot_points', anonymous=True)

   # define the number of processes
    n_processes = mp.cpu_count()

    # create a pool of processes
    pool = mp.Pool(processes=n_processes)

    # create a list of joint angles to evaluate
    joint_angles = [[random_val(-math.pi, math.pi) for _ in range(6)] for _ in range(1000)]

    # calculate reachable space in parallel
    results = pool.map(calculate_position, joint_angles)

    # the results is a list of reachable positions, we can use it to draw a 3d plot
    # convert results to numpy array for convenience
    results = np.array(results)

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(results[:,0], results[:,1], results[:,2], alpha=0.1, edgecolors='b', s=20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

    publish_points(results)
