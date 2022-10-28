class Posture:
    def __init__(self, line):
        self.values = []
        values_str = line.split()
        for value in values_str:
            self.values.append(float(value))

    def get_value(self, idx):
        return self.values[idx]