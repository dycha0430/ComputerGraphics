class Joint:
    def __init__(self):
        self.offset = []
        self.children = []
        self.parent: Joint = None
        self.isEndEffector = False

    def add_child(self, joint):
        self.children.append(joint)

    def get_children(self):
        return self.children

    def set_parent(self, parent):
        self.parent = parent

    def set_offset(self, x, y, z):
        self.offset = [x, y, z]
