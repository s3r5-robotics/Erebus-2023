from agent.agent_interface import AgentInterface, SubagentInterface
from agent.subagents.follow_walls.follow_walls_subagent import FollowWallsAgent
from agent.subagents.go_to_non_discovered.go_to_non_discovered_subagent import GoToNonDiscoveredAgent
from agent.subagents.return_to_start_subagent import ReturnToStartAgent
from data_structures.vectors import Position2D
from flow_control.state_machine import StateMachine
from mapping.mapper import Mapper
from robot.robot import Robot
import flags


class SubagentPriorityCombiner(SubagentInterface):
    """
    Tries different strategies until one returns a position.
    """

    def __init__(self, agents: list) -> None:
        self.__agent_list = agents
        self.__current_agent_index = 0
        self.__previous_agent_index = 0

    def update(self, force_calculation=False) -> None:
        agent: SubagentInterface
        for index, agent in enumerate(self.__agent_list):
            agent.update(force_calculation=self.__agent_changed() or force_calculation)
            if agent.target_position_exists():
                self.__previous_agent_index = self.__current_agent_index
                self.__current_agent_index = index
                break

    def get_target_position(self) -> Position2D:
        return self.__agent_list[self.__current_agent_index].get_target_position()

    def target_position_exists(self) -> bool:
        return self.__agent_list[self.__current_agent_index].target_position_exists()

    def __agent_changed(self) -> bool:
        return self.__previous_agent_index != self.__current_agent_index


class Agent(AgentInterface):
    def __init__(self, mapper: Mapper, robot: Robot) -> None:
        self.__mapper = mapper
        self.__robot = robot

        self.__navigation_agent = SubagentPriorityCombiner([FollowWallsAgent(self.__mapper),
                                                            GoToNonDiscoveredAgent(self.__mapper)])

        self.__return_to_start_agent = ReturnToStartAgent(self.__mapper)

        self.__stage_machine = StateMachine("explore", self.__set_force_calculation)
        self.__stage_machine.create_state(name="explore", function=self.__stage_explore,
                                          possible_changes={"return_to_start"})
        self.__stage_machine.create_state(name="return_to_start", function=self.__stage_return_to_start)

        self.do_force_calculation = False
        self.end_reached_distance_threshold = 0.04

        self.__target_position = None

    def update(self) -> None:
        self.__stage_machine.run()

    def get_target_position(self) -> Position2D:
        return self.__target_position

    def do_end(self) -> bool:
        return_to_start = self.__stage_machine.state == "return_to_start" and \
            self.__mapper.robot_position.get_distance_to(
                self.__mapper.start_position) < self.end_reached_distance_threshold
        no_time_remaining = flags.DO_EARLY_QUIT and self.__robot.comunicator.remaining_time < 5

        return return_to_start or no_time_remaining

    def __stage_explore(self, change_state_function):
        self.__navigation_agent.update(force_calculation=self.do_force_calculation)
        self.do_force_calculation = False

        if not self.__navigation_agent.target_position_exists():
            # If there's no target position, we've explored the whole map, thus we should return to start
            change_state_function("return_to_start")

        else:
            self.__target_position = self.__navigation_agent.get_target_position()

    def __stage_return_to_start(self, _):
        self.__return_to_start_agent.update(force_calculation=self.do_force_calculation)
        self.do_force_calculation = False

        if self.__return_to_start_agent.target_position_exists():
            self.__target_position = self.__return_to_start_agent.get_target_position()

    def __set_force_calculation(self):
        self.do_force_calculation = True
