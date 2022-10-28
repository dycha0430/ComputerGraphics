class Skeleton:
    def __init__(self):
        self.joints = []

    def push_joint(self, joint):
        self.joints.append(joint)

    def get_root_joint(self):
        return self.joints[0]
