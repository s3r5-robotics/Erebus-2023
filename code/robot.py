import typing
import warnings
from types import NoneType
from typing import Optional, Type

import controller

from flow.state_machine import StateMachine
from devices import (DeviceType, Accelerometer, Camera, ColorSensor, DistanceSensor, Emitter, GPS, Gyro, InertialUnit,
                     LED, Lidar, Motor, Receiver)


class Robot(controller.Robot):

    # noinspection PyTypeChecker
    def __init__(self, time_step: Optional[int] = None) -> None:
        super().__init__()
        self.time_step = round(time_step or self.basic_time_step)
        print(f"Robot {self.name} running with time step: {self.time_step}")

        self.__init_devices()
        self.__init_state()

        if self.imu.noise:
            warnings.warn(f"{type(self.imu).__name__} '{self.imu.name}' is noisy - {self.imu.noise}!"
                          " Consider adding some filtering or implement sensor fusion.")

    def __init_devices(self):
        # Sensors (Optional[] devices may not be present on all versions of SERSy-1)
        self.gyro = self._get_device("gyro", Optional[Gyro])
        self.imu = self._get_device("inertial_unit", InertialUnit)
        self.gps = self._get_device("gps", GPS)
        self.camera_l = self._get_device("camera_left", Camera)
        self.camera_f = self._get_device("camera_front", Optional[Camera])
        self.camera_r = self._get_device("camera_right", Camera)
        self.color_sensor = self._get_device("colour_sensor", ColorSensor)
        self.accel = self._get_device("accelerometer", Optional[Accelerometer])
        self.lidar = self._get_device("lidar", Lidar)
        # Distance sensors
        self.distance_l = self._get_device("distance_sensor_left", DistanceSensor)
        self.distance_f = self._get_device("distance_sensor_front", DistanceSensor)
        self.distance_r = self._get_device("distance_sensor_right", DistanceSensor)
        # Wheels
        self.ml = Motor(self._get_device("wheel1 motor", controller.Motor),
                        self._get_device("wheel1 sensor", controller.PositionSensor),
                        self.time_step)
        self.mr = Motor(self._get_device("wheel2 motor", controller.Motor),
                        self._get_device("wheel2 sensor", controller.PositionSensor),
                        self.time_step)
        # Other, built-in
        self.led0 = self._get_device("led8", LED)
        self.led1 = self._get_device("led9", LED)
        self.receiver = self._get_device("receiver", Receiver)
        self.emitter = self._get_device("emitter", Emitter)

        if self.devices:
            print("Unused devices:\n" + "\n".join(f"- {name}: {type(device)}" for name, device in self.devices.items()))

    def __init_state(self):
        self.state = StateMachine("init")

        self.state.create_state("init", self.state_init)

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

    def _should_run(self):
        """Run one simulation step, process all sensors and actuators"""
        if self.step(self.time_step) == -1:  # -1 indicates that Webots is about to terminate the controller
            return False

        # TODO: main loop condition logic

        return True

    def state_init(self):
        pass

    def run(self) -> bool:
        while self._should_run():
            yaw, pitch, roll = self.imu.yaw_pitch_roll_dg

            print(f"L|F|R   {self.distance_l.value:.3f} | {self.distance_f.value:.3f} | {self.distance_r.value:.3f}    "
                  f"Y|P|R   {yaw:.3f} | {pitch:.3f} | {roll:.3f}")
            self.ml.target_velocity = 0.4
            self.mr.target_velocity = 0.5

            self.state.run()
