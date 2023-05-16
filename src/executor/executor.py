from data_structures.angle import Angle
from executor.stuck_detector import StuckDetector

from flow_control.sequencer import Sequencer
from flow_control.state_machine import StateMachine
from flow_control.delay import DelayManager


from robot.robot import Robot
from robot.drive_base import Criteria as RotationCriteria

from mapping.mapper import Mapper

from agents.granular_navigation_agent.granular_navigation_agent import Agent

from fixture_detection.fixture_clasification import FixtureClasiffier

from flags import SHOW_DEBUG, DO_SLOW_DOWN, SLOW_DOWN_S

import time

class Executor:
    def __init__(self, agent: Agent, mapper: Mapper, robot: Robot) -> None:
        self.agent = agent # Tells the executor what to do
        self.mapper = mapper # Maps everything
        self.robot = robot # Low level movement and sensing

        self.delay_manager = DelayManager()
        self.stuck_detector = StuckDetector() # Detects if the wheels of the robot are moving or not

        self.state_machine = StateMachine("init") # Manages states
        self.state_machine.create_state("init", self.state_init, {"explore",}) # This state initializes and calibrates the robot
        self.state_machine.create_state("explore", self.state_explore, {"end", "detect_fixtures"}) # This state follows the position returned by the agent
        self.state_machine.create_state("end", self.state_end)
        self.state_machine.create_state("detect_fixtures", self.state_detect_fixtures, {"explore", "report_fixture"})
        self.state_machine.create_state("report_fixture", self.state_report_fixture, {"explore"})

        self.sequencer = Sequencer(reset_function=self.delay_manager.reset_delay) # Allows for asynchronous programming

        self.fixture_detector = FixtureClasiffier()
        
        # Flags
        self.mapping_enabled = False
        self.victim_reporting_enabled = True

        # Sequential functions used frequently
        self.seq_print =           self.sequencer.make_simple_event( print)
        self.seq_move_wheels =     self.sequencer.make_simple_event( self.robot.move_wheels)

        self.seq_rotate_to_angle = self.sequencer.make_complex_event(self.robot.rotate_to_angle)
        self.seq_rotate_slowly_to_angle = self.sequencer.make_complex_event(self.robot.rotate_slowly_to_angle)
        self.seq_move_to_coords =  self.sequencer.make_complex_event(self.robot.move_to_coords)
        self.seq_delay_seconds =   self.sequencer.make_complex_event(self.delay_manager.delay_seconds)

        self.__previous_position = 0

        self.letter_to_report = None

    def run(self):
        """Advances the simulation, updates all components and executes the state machine."""
        
        while self.robot.do_loop():
            self.robot.update()  # Updates robot position and rotation, sensor positions and values, etc.

            self.delay_manager.update(self.robot.time)
            self.do_mapping()
            self.state_machine.run()

            self.stuck_detector.update(self.robot.position,
                                       self.__previous_position,
                                       self.robot.drive_base.get_wheel_direction())
            self.__previous_position = self.robot.position

            if DO_SLOW_DOWN:
                time.sleep(SLOW_DOWN_S)
            if SHOW_DEBUG:
                print("state:", self.state_machine.state)
            
    def do_mapping(self):
        """Updates the mapper is mapping is enabled."""

        if self.mapping_enabled:
                # Floor and lidar mapping
                self.mapper.update(self.robot.get_point_cloud(), 
                                   self.robot.get_out_of_bounds_point_cloud(),
                                   self.robot.get_lidar_detections(),
                                   self.robot.get_camera_images(), 
                                   self.robot.position,
                                   self.robot.orientation)
        else:
            # Only position and rotation
            self.mapper.update(robot_position=self.robot.position, 
                               robot_orientation=self.robot.orientation)
            
    # STATES
    def state_init(self, change_state_function):
        """Initializes and calibrates the robot."""

        self.sequencer.start_sequence() # Starts the sequence
        self.seq_delay_seconds(0.5)

        self.sequencer.simple_event(self.calibrate_position_offsets) # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        
        self.sequencer.simple_event(self.mapper.register_start, self.robot.position) # Informs the mapping components of the starting position of the robot
        
        self.seq_calibrate_robot_rotation() # Calibrates the rotation of the robot using the gps

        # Starts mapping walls
        if self.sequencer.simple_event():
            self.mapping_enabled = True
            self.victim_reporting_enabled = True

        self.seq_delay_seconds(0.5)
        self.sequencer.complex_event(self.robot.rotate_to_angle, angle=Angle(90, Angle.DEGREES), direction=RotationCriteria.LEFT)
        self.sequencer.complex_event(self.robot.rotate_to_angle, angle=Angle(180, Angle.DEGREES), direction=RotationCriteria.LEFT)
        self.seq_delay_seconds(0.5)

        self.sequencer.simple_event(change_state_function, "explore") # Changes state
        self.sequencer.seq_reset_sequence() # Resets the sequence

        self.stuck_detector = StuckDetector()

    def state_explore(self, change_state_function):
        """Follows the instructions of the agent."""

        self.sequencer.start_sequence() # Starts the sequence

        self.agent.update()

        self.seq_move_to_coords(self.agent.get_target_position())

        # Detect if the robot is stuck
        if self.stuck_detector.is_stuck():
            # What do we do here :confused_emoji:
            self.robot.drive_base.move_to_position()

        self.sequencer.seq_reset_sequence() # Resets the sequence but doesn't change state, so it starts all over again.

        if SHOW_DEBUG:
            print("rotation:", self.robot.orientation)
            print("position:", self.robot.position)
        
        if self.agent.do_end():
            self.state_machine.change_state("end")

        if self.agent.do_report_victim():
            change_state_function("detect_fixtures")

    def state_end(self, change_state_function):
        self.robot.comunicator.send_map(self.mapper.get_grid_for_bonus())
        self.robot.comunicator.send_end_of_play()

    def state_detect_fixtures(self, change_state_function):
        self.sequencer.start_sequence()
        self.seq_print("entered_detect_fixture")
        self.seq_move_wheels(0, 0)

        self.sequencer.complex_event(self.robot.rotate_slowly_to_angle, angle=Angle(90, Angle.DEGREES), direction=RotationCriteria.LEFT)
        self.sequencer.complex_event(self.robot.rotate_slowly_to_angle, angle=Angle(180, Angle.DEGREES), direction=RotationCriteria.LEFT)

        self.seq_print("exiting_detect_fixture")
        self.sequencer.simple_event(change_state_function, "explore")
        self.sequencer.seq_reset_sequence() # Resets the sequence
        
        images = self.robot.get_camera_images()
        if self.victim_reporting_enabled and images is not None:
           
            fixtures = self.fixture_detector.find_fixtures(images[1].image)   
            if len(fixtures):
                change_state_function("report_fixture")
                self.sequencer.reset_sequence() # Resets the sequence

    def state_report_fixture(self, change_state_function):
        self.sequencer.start_sequence()
        self.seq_print("entered_report_fixture")
        self.seq_move_wheels(0, 0)
        
        if self.sequencer.simple_event():
            
            images = self.robot.get_last_camera_images()
            if self.victim_reporting_enabled:
                fixtures = self.fixture_detector.find_fixtures(images[1].image)      
                if len(fixtures):
                    self.letter_to_report = self.fixture_detector.classify_fixture(fixtures[0])
                    
        if self.letter_to_report is not None:
            self.seq_move_wheels(0.3, 0.3)
            self.seq_delay_seconds(0.2)
            self.seq_move_wheels(0, 0)
            self.seq_delay_seconds(2)

        if self.sequencer.simple_event():
            if self.letter_to_report is not None:
                print("sending letter:", self.letter_to_report)
                self.robot.comunicator.send_victim(self.robot.position, self.letter_to_report)
        
        if self.sequencer.simple_event():
            self.letter_to_report = None
        self.sequencer.simple_event(change_state_function, "explore")
        self.sequencer.seq_reset_sequence() # Resets the sequence

    def calibrate_position_offsets(self):
        """Calculates offsets in the robot position, in case it doesn't start perfectly centerd."""
        print("robot_position:", self.robot.position)
        self.robot.position_offsets = self.robot.position % (self.mapper.quarter_tile_size * 2)
        print("positionOffsets: ", self.robot.position_offsets)
        

    def seq_calibrate_robot_rotation(self):
        """ Calibrates the robot rotation using the gps."""

        if self.sequencer.simple_event():
            self.robot.auto_decide_orientation_sensor = False
        self.seq_move_wheels(-1, -1)
        self.seq_delay_seconds(0.1)
        if self.sequencer.simple_event(): 
            self.robot.orientation_sensor = self.robot.GPS
        self.seq_move_wheels(1, 1)
        self.seq_delay_seconds(0.1)
        if self.sequencer.simple_event(): 
            self.robot.orientation_sensor = self.robot.GYROSCOPE
        self.seq_delay_seconds(0.1)
        self.seq_move_wheels(0, 0)
        self.seq_move_wheels(-1, -1)
        self.seq_delay_seconds(0.1)
        self.seq_move_wheels(0, 0)
        if self.sequencer.simple_event():
            self.robot.auto_decide_orientation_sensor = True

            





    


