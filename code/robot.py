from typing import Optional

import controller
from devices import (Device, Accelerometer, Camera, ColorSensor, DistanceSensor, Emitter, GPS, LED, Lidar, Motor,
                     Receiver)


class Robot(controller.Robot):

    # noinspection PyTypeChecker
    def __init__(self, time_step: Optional[int] = None) -> None:
        super().__init__()
        self.time_step = round(time_step or self.basic_time_step)
        print(f"Robot {self.name} running with time step: {self.time_step}")

        # Sensors
        # Not present on SERSy-1: self.gyro = Gyro(self._get_device("gyro"), self.time_step)
        # Not present on SERSy-1: self.inertial_unit = InertialUnit(self._get_device("inertial_unit"), self.time_step)
        self.gps = GPS(self._get_device("gps"), self.time_step)
        self.camera_l = Camera(self._get_device("camera_left"), self.time_step)
        # Not present on SERSy-1: self.camera_f = Camera(self._get_device("camera_front"), self.time_step)
        self.camera_r = Camera(self._get_device("camera_right"), self.time_step)
        self.color_sensor = ColorSensor(self._get_device("colour_sensor"), self.time_step)
        self.accel = Accelerometer(self._get_device("accelerometer"), self.time_step)
        self.lidar = Lidar(self._get_device("lidar"), self.time_step)
        # Distance sensors
        self.distance_l = DistanceSensor(self._get_device("distance_sensor_left"), self.time_step)
        self.distance_f = DistanceSensor(self._get_device("distance_sensor_front"), self.time_step)
        self.distance_r = DistanceSensor(self._get_device("distance_sensor_right"), self.time_step)
        # Wheels
        self.ml = Motor(self._get_device("wheel1 motor"), self._get_device("wheel1 sensor"), self.time_step)
        self.mr = Motor(self._get_device("wheel2 motor"), self._get_device("wheel2 sensor"), self.time_step)
        # Other, built-in
        self.led0 = LED(self._get_device("led8"), self.time_step)
        self.led1 = LED(self._get_device("led9"), self.time_step)
        self.receiver = Receiver(self._get_device("receiver"), self.time_step)
        self.emitter = Emitter(self._get_device("emitter"), self.time_step)

        if self.devices:
            print("Unused devices:\n" + "\n".join(f"- {name}: {type(device)}" for name, device in self.devices.items()))

    def _get_device(self, name: str) -> Device:
        # self.devices dict is only used for getDevice() and getDeviceByIndex(), so it is OK to delete the
        # entry after retrieving it, enabling detection of unused devices (devices left in the dict).
        return self.devices.pop(name)

    def run(self) -> bool:
        """Run one simulation step, process all sensors and actuators"""
        if self.step(self.time_step) == -1:  # -1 indicates that Webots is about to terminate the controller
            return False

        # TODO: Main loop

        return True
