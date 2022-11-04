import numpy as np
from enum import Enum
class Step(Enum):
    FIRST = 1
    SECOND = 2

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

    def set_transformation_matrix(self, transformation_matrix):
        self.transformation_matrix = transformation_matrix
        self.global_pos = self.get_global_position()

    def set_global_position(self):
        global_position = self.transformation_matrix.T @ np.array([0, 0, 0, 1]).T
        global_position = global_position[:-1]
        self.global_pos = global_position

    def get_global_position(self, test_x=0):
        return self.global_pos

    def set_local_axis(self, global_axis, step: Step):
        local_axis = global_axis @ self.transformation_matrix.T

        local_axis.squeeze()
        if step == Step.FIRST:
            self.local_axis = local_axis
        else:
            self.local_axis2 = local_axis
    def get_local_axis(self, step: Step):
        if step == Step.FIRST:
            return self.local_axis
        else:
            return self.local_axis2
