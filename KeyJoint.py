import numpy as np


class KeyJoint:
    def __init__(self, joint_name=""):
        self.global_pos = []
        self.local_axis = np.array([0, 0, 0, 0])
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)
        self.joint_name = joint_name

        # For second step (Only for a...)
        self.local_axis2 = np.array([0, 0, 0, 0])
        self.diff_angle2 = 0

    def reset(self):
        self.global_pos = []
        self.local_axis = np.array([0, 0, 0, 0])
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)

        self.local_axis2 = np.array([0, 0, 0, 0])
        self.diff_angle2 = 0

    def set_global_position(self):
        self.global_pos = self.get_global_position()

    def get_global_position(self, test_x=0):
        ret = self.transformation_matrix.T @ np.array([test_x, 0, 0, 1]).T
        ret = ret[:-1]
        return ret

    def get_local_axis(self, global_axis):
        local_axis = global_axis @ self.transformation_matrix.T

        local_axis.squeeze()
        return local_axis
