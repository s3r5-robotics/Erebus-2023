import typing
import warnings
from types import NoneType
from typing import Optional, Type, Tuple

import controller
import debug
from devices.devices import DeviceType, Camera, ColorSensor, Emitter, GPS, InertialUnit, LED, Motor, Receiver, \
    DistanceSensor, Gyro
from mapping.mapper import Mapper
from devices.lidar import Lidar
from movement import Drivetrain


class Robot(controller.Robot):
    # Robot dimensions in millimeters (https://cyberbotics.com/doc/guide/epuck#e-puck-model)
    DIAMETER = 71
    # Tile size in millimeters
    TILE_SIZE = 120

    # noinspection PyTypeChecker
    def __init__(self, time_step: Optional[int] = None) -> None:
        super().__init__()
        self.time_step = round(time_step or self.basic_time_step)
        print(f"Robot {self.name} running with time step: {self.time_step}")

        # Sensors
        self.imu = self._get_device("inertial_unit", InertialUnit)
        self.gps = self._get_device("gps", GPS)
        self.gyro = self._get_device("gyro", Gyro)
        self.camera_l = self._get_device("camera_left", Camera)
        self.camera_r = self._get_device("camera_right", Camera)
        self.color_sensor = self._get_device("colour_sensor", ColorSensor)
        self.distance_l = self._get_device("distance_sensor_left", Optional[DistanceSensor])
        self.distance_f = self._get_device("distance_sensor_front", Optional[DistanceSensor])
        self.distance_r = self._get_device("distance_sensor_right", Optional[DistanceSensor])
        # Lidar has vertical field of view of 0.2 rad (11.5 degrees), separated into 4 layers:
        # - Layer 0 is tilted upwards by 0.1 rad (5.7 degrees), causing it to see over the walls,
        #   reporting 'inf' as distance almost all the time.
        # - Layer 1 is tilted upwards by 1.9 degrees, reporting 'inf' for most of the time except
        #   when the robot is very close to a wall.
        # - Layer 2 is tilted downwards by 1.9 degrees and is generally the most/only useful one.
        # - Layer 3 is tilted downwards by 0.1 rad, causing it to see the floor and report some distances
        #   even when there is no obstacle until the other end of the maze.
        self.lidar = Lidar(self._get_device("lidar", controller.Lidar), self.time_step, use_single_layer=None)
        # Wheels
        motor_l = Motor(self._get_device("wheel_left motor", controller.Motor),
                        self._get_device("wheel_left sensor", controller.PositionSensor),
                        self.time_step)
        motor_r = Motor(self._get_device("wheel_right motor", controller.Motor),
                        self._get_device("wheel_right sensor", controller.PositionSensor),
                        self.time_step)
        # Other, built-in
        self.led0 = self._get_device("led8", LED)
        self.led1 = self._get_device("led9", LED)
        self.receiver = self._get_device("receiver", Receiver)
        self.emitter = self._get_device("emitter", Emitter)

        if self.devices:
            print("Unused devices:\n" + "\n".join(f"- {name}: {type(device)}" for name, device in self.devices.items()))

        if self.imu.noise:
            warnings.warn(f"{type(self.imu).__name__} '{self.imu.name}' is noisy - {self.imu.noise}!"
                          " Consider adding some filtering or implement sensor fusion.")

        # Higher level combined devices using peripheral devices retrieved above
        self.drive = Drivetrain(motor_l, motor_r, self.imu, self.gps)
        self.mapper = Mapper(tile_size=self.TILE_SIZE * .001, robot_diameter=self.DIAMETER * .001)

        # States
        self.step_counter = 0

    def _get_device(self, name: str, cls: Type[DeviceType]) -> DeviceType:
        """
        Get a robot device instance by name and remove it from the internal `devices` dictionary.

        :param name:     Name of the device as defined in Webots robot JSON.
        :param cls:      Class to instantiate the device as, can be Optional[...] to allow the device to be missing.

        :return: Instantiated subclass of Device (DeviceType) or None if cls was provided as Optional[] and the
                 device is not present on this robot.

        :raises AttributeError: If `name` device is not present on the robot and `cls` is not wrapped in `Optional[]`.
        """
        # If cls is wrapped in typing class (Optional[...], Union[...]), get_args will get all inner types
        optional = NoneType in typing.get_args(cls)  # Optional[...] is actually Union[..., None]
        # Get the first non-None type from the list of inner types (the actual class). If cls was not wrapped in
        # Optional[], get_args() will return empty tuple, and next() will return the default, cls itself.
        cls = next((t for t in typing.get_args(cls) if t is not NoneType), cls)

        if name not in self.devices:
            if optional:
                # Type checker cannot handle our advanced Optional[TypeVar] logic
                # noinspection PyTypeChecker
                return None
            raise AttributeError(f"Device '{name}' not present on robot {self.name}")

        # self.devices dict is only used for getDevice() and getDeviceByIndex(), so it is OK to delete the
        # entry after retrieving it, enabling detection of left-over (unused) devices.
        device = self.devices.pop(name)
        # If instance is already of the correct class (probably controller.Device), no need to wrap it
        if isinstance(device, cls):
            return device
        return cls(device, time_step=self.time_step)

    def _map(self):
        self.mapper(
            in_bounds_point_cloud=self.lidar.point_cloud,
            out_of_bounds_point_cloud=self.lidar.out_of_bounds_point_cloud,
            robot_position=self.gps.position.position2d,
            robot_orientation=None
        )

    def step(self, _=None) -> bool:
        """
        Run one simulation step and increase the step counter

        :param _:  Ignored argument to satisfy PyMethodOverriding inspection.

        :return: True if the simulation should continue, False if Webots is about to terminate the controller.
        """
        self.step_counter += 1
        # -1 indicates that Webots is about to terminate the controller
        return super().step(self.time_step) != -1

    @property
    def distances(self) -> Tuple[float, float, float]:
        """Get the current left, front, right distance readings (mm) the 3 onboard distance sensors"""
        sensors: list[DistanceSensor] = [self.distance_l, self.distance_f, self.distance_r]
        readings: Tuple[float, float, float] = tuple(map(lambda ds: ds.mm, sensors))

        return readings

    def __call__(self) -> bool:
        """
        Run one simulation step and process all sensors and actuators

        :return: True if the simulation should continue, False if Webots is about to terminate the controller.
        """

        self.lidar()
        self._map()
        self.drive()

        return True
