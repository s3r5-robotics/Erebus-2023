import typing
import warnings
from types import NoneType
from typing import Optional, Type

import controller
from devices import (DeviceType, Camera, ColorSensor, DistanceSensor, Emitter, GPS, InertialUnit,
                     LED, Lidar, Motor, Receiver)
from movement import Drivetrain


class Robot(controller.Robot):

    # noinspection PyTypeChecker
    def __init__(self, time_step: Optional[int] = None) -> None:
        super().__init__()
        self.time_step = round(time_step or self.basic_time_step)
        print(f"Robot {self.name} running with time step: {self.time_step}")

        # Sensors
        self.imu = self._get_device("inertial_unit", InertialUnit)
        self.gps = self._get_device("gps", GPS)
        self.camera_l = self._get_device("camera_left", Camera)
        self.camera_r = self._get_device("camera_right", Camera)
        self.color_sensor = self._get_device("colour_sensor", ColorSensor)
        self.lidar = self._get_device("lidar", Lidar)
        # Distance sensors
        self.distance_l = self._get_device("distance_sensor_left", DistanceSensor)
        self.distance_f = self._get_device("distance_sensor_front", DistanceSensor)
        self.distance_r = self._get_device("distance_sensor_right", DistanceSensor)
        # Wheels
        motor_l = Motor(self._get_device("wheel1 motor", controller.Motor),
                        self._get_device("wheel1 sensor", controller.PositionSensor),
                        self.time_step)
        motor_r = Motor(self._get_device("wheel2 motor", controller.Motor),
                        self._get_device("wheel2 sensor", controller.PositionSensor),
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

    def __call__(self) -> bool:
        """
        Run one simulation step, process all sensors and actuators

        :return: True if the simulation should continue, False if Webots is about to terminate the controller.
        """
        if self.step(self.time_step) == -1:  # -1 indicates that Webots is about to terminate the controller
            return False
        self.step_counter += 1

        self.drive()

        return True
