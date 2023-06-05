class PathSmoother:
    def __init__(self, strength) -> None:
        self.strength = strength

    def smooth(self, path):
        new_path = []
        for index, node in enumerate(path):
            prior = path[max(index - 1, 0)]
            next = path[min(index + 1, len(path) - 1)]

            avg_x = (node[0] + prior[0] * self.strength + next[0] * self.strength) / (1 + self.strength * 2)
            avg_y = (node[1] + prior[1] * self.strength + next[1] * self.strength) / (1 + self.strength * 2)

            new_path.append([avg_x, avg_y])

        return new_path
