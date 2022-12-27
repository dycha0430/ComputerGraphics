from Motion.Posture import Posture
from Motion.Joint import Joint
from Motion.Skeleton import Skeleton
from abc import *


class Motion(metaclass=ABCMeta):
    def __init__(self, file_name):
        self.name = file_name
        self.skeleton = Skeleton()
        self.postures = []
        self.frame_time = 0
        self.joint_list = []
        if file_name != "":
            self.parse()

    def get_value_in_frame(self, frame, idx):
        return self.postures[frame].get_value(idx)

    def get_frame_num(self):
        return len(self.postures)

    def push_joint(self, joint):
        self.skeleton.push_joint(joint)

    def get_posture(self, frame):
        return self.postures[frame]

    def get_root(self):
        return self.skeleton.get_root_joint()

    @abstractmethod
    def parse(self):
        pass


class BvhMotion(Motion, metaclass=ABCMeta):
    def __init__(self, file_name):
        self.channels = []
        Motion.__init__(self, file_name)

    def parse(self):
        # if file extension is not bvh, return error message
        file_name = self.name.split('\\')[-1]
        extension = file_name.split('.')[-1]
        if extension != 'bvh':
            return False
        f = open(self.name, 'r')
        lines = f.readlines()
        lines = [line.rstrip() for line in lines]
        line_num = 0
        f.close()

        is_end_effector = False

        cur_joint = Joint()
        num_frames = 0
        joint_name = ""
        while True:
            if line_num == len(lines): break
            line = lines[line_num]
            line_num += 1

            words = line.split()
            if len(words) == 0: continue

            if words[0] == 'Frames:':
                num_frames = int(words[1])
            elif words[0] == 'Frame' and words[1] == 'Time:':
                self.frame_time = float(words[2])
                for i in range(0, num_frames):
                    line = lines[line_num]
                    line_num += 1
                    posture = Posture(line)
                    self.postures.append(posture)
            elif words[0] == 'JOINT' or words[0] == 'ROOT':
                self.joint_list.append(words[1])
                joint_name = words[1]
            elif words[0] == '{':
                new_joint = Joint()
                cur_joint.add_child(new_joint)
                new_joint.set_parent(cur_joint)
                cur_joint = new_joint
            elif words[0] == '}':
                cur_joint = cur_joint.parent
            elif words[0] == 'End':
                is_end_effector = True
                joint_name = "End effector"
            elif words[0] == 'OFFSET':
                cur_joint.set_offset(float(words[1]), float(words[2]), float(words[3]))
                if is_end_effector:
                    cur_joint.isEndEffector = True
                    is_end_effector = False
                cur_joint.name = joint_name
                self.push_joint(cur_joint)
            elif words[0] == "CHANNELS":
                num_channels = int(words[1])
                channel = []
                for i in range(0, num_channels):
                    chan_type = words[2 + i].upper()
                    channel.append(chan_type)

                self.channels.append(channel)

        self.print_info()
        return True

    def print_info(self):
        print("File name : ", self.name.split('\\')[-1])
        print("Number of frames : ", len(self.postures))
        print("FPS (1/FrameTime) : ", 1 / self.frame_time)
        print("Number of joints : ", len(self.joint_list))
        print("List of all joint names : ", self.joint_list)