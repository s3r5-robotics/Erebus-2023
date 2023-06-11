from agent.agent_interface import PositionFinderInterface
from algorithms.bfs import BFSAlgorithm, NavigatingBFSAlgorithm
from data_structures.vectors import Position2D
from mapping.mapper import Mapper


class PositionFinder(PositionFinderInterface):
    """
    Finds the best position for the robot to go to, with the objective of exploring the maze.
    """

    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        self.closest_unseen_finder = NavigatingBFSAlgorithm(found_function=lambda x: x == False,
                                                            traversable_function=lambda x: x == False,
                                                            max_result_number=1)

        self.closest_free_point_finder = BFSAlgorithm(lambda x: x == 0)

        self.closest_unseen_grid_index = None

        self.__still_reachable_bfs = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)

    def update(self, force_calculation=False):
        """
        Calculate the closest unseen position if the objective is no longer traversable
        or if it's told to do it with the 'force' parameter.
        """
        if self.__is_objective_untraversable() or \
                not self.__is_grid_index_still_reachable(self.closest_unseen_grid_index) or \
                force_calculation:
            self.closest_unseen_grid_index = self.__get_closest_unseen_grid_index()

    def get_target_position(self):
        if self.target_position_exists():
            coords = self.__mapper.pixel_grid.grid_index_to_coordinates(self.closest_unseen_grid_index)
            return Position2D(coords)

    def target_position_exists(self) -> bool:
        return self.closest_unseen_grid_index is not None

    def __is_objective_untraversable(self):
        if self.target_position_exists():
            closest_unseen_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(
                self.closest_unseen_grid_index)
            return self.__mapper.pixel_grid.arrays["traversable"][
                closest_unseen_array_index[0], closest_unseen_array_index[1]]
        else:
            return False

    def __is_grid_index_still_reachable(self, grid_index) -> bool:
        if grid_index is None:
            return False
        start_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(grid_index)

        if self.__mapper.pixel_grid.arrays["traversable"][start_array_index[0], start_array_index[1]]:
            return False

        results = self.__still_reachable_bfs.bfs(self.__mapper.pixel_grid.arrays["traversed"],
                                                 self.__mapper.pixel_grid.arrays["traversable"], start_array_index)

        return bool(len(results))

    def __get_closest_unseen_grid_index(self):
        robot_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(self.__mapper.robot_position)
        start_node = self.__get_closest_traversable_array_index(robot_array_index)

        closest_unseen_array_indexes = self.closest_unseen_finder.bfs(
            found_array=self.__mapper.pixel_grid.arrays["discovered"],
            traversable_array=self.__mapper.pixel_grid.arrays["traversable"],
            start_node=start_node)

        if len(closest_unseen_array_indexes):
            # print("Found undiscovered area.")
            return self.__mapper.pixel_grid.array_index_to_grid_index(closest_unseen_array_indexes[0])
        else:
            print("No undiscovered area found.")
            return None

    def __get_closest_traversable_array_index(self, array_index):
        if self.__mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return self.closest_free_point_finder.bfs(array=self.__mapper.pixel_grid.arrays["traversable"],
                                                      start_node=array_index)
        else:
            return array_index
