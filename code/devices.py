import inspect
import math
import struct
import sys
import typing
from typing import Tuple

import numpy as np

import controller.device
import controller.sensor
from controller.wb import wb
from utils import InstanceSubclass, Angle, Point


# Support all devices, and only devices from Erebus Robot Customisation 2023 webpage
# https://v23.robot.erebus.rcj.cloud/


class Device:
    """Base class for all devices"""

    def __init__(self, time_step: int) -> None:
        self.time_step = time_step


class Sensor(Device, controller.sensor.Sensor):
    """Base class for all sensors"""

    def __init__(self, time_step: int) -> None:
        Device.__init__(self, time_step)
        self.enable(time_step)


DeviceType = typing.TypeVar("DeviceType", Device, controller.device.Device)
"""
Type alias for any type which is subclass of Device or controller.Device

TypeVar is used in function typing instead of just the base-class to enable static type checking of the return type
of a function, based on the arguments.
https://medium.com/@steveYeah/using-generics-in-python-99010e5056eb
https://stackoverflow.com/questions/69184577/return-type-based-on-type-argument
https://stackoverflow.com/questions/68650071/type-annotations-for-base-and-inherited-classes-is-generic-and-typevar-the-rig
"""


# region Sensors

class Gyro(InstanceSubclass, Sensor, controller.Gyro):
    def __init__(self, _: controller.Gyro, time_step: int):
        Sensor.__init__(self, time_step)


class InertialUnit(InstanceSubclass, Sensor, controller.InertialUnit):
    def __init__(self, _: controller.InertialUnit, time_step: int):
        Sensor.__init__(self, time_step)

    @property
    def yaw_pitch_roll(self) -> Tuple[Angle, Angle, Angle]:
        """Get yaw, pitch, roll"""
        r, p, y = map(Angle, self.roll_pitch_yaw)
        return y, p, r

    @property
    def yaw(self) -> Angle:
        """Get yaw/heading/rotation"""
        return Angle(self.roll_pitch_yaw[2])

    @property
    def pitch(self) -> Angle:
        """Get pitch/inclination/front-tilt"""
        return Angle(self.roll_pitch_yaw[1])

    @property
    def roll(self) -> Angle:
        """Get roll/side-tilt"""
        return Angle(self.roll_pitch_yaw[0])


class GPS(InstanceSubclass, Sensor, controller.GPS):

    def __init__(self, _: controller.GPS, time_step: int):
        Sensor.__init__(self, time_step)

    @property
    def position(self) -> Point:
        """Get current position"""
        return Point.from_xyz(self.value)


class Camera(InstanceSubclass, Sensor, controller.Camera):

    def __init__(self, _: controller.Camera, time_step: int):
        Sensor.__init__(self, time_step)


class ColorSensor(InstanceSubclass, Sensor, controller.Camera):

    def __init__(self, _: controller.Camera, time_step: int):
        Sensor.__init__(self, time_step)


class Accelerometer(InstanceSubclass, Sensor, controller.Accelerometer):

    def __init__(self, _: controller.Accelerometer, time_step: int):
        Sensor.__init__(self, time_step)


class Lidar(InstanceSubclass, Sensor, controller.Lidar):
    # noinspection SpellCheckingInspection
    WbLidarPoint = struct.Struct("fffif")  # float x, float y, float z, int layer_id, float time
    """https://cyberbotics.com/doc/reference/lidar?tab-language=c#wblidarpoint"""

    def __init__(self, _: controller.Lidar, time_step: int, use_single_layer: int = None):
        """
        :param _:                Unused reference to the wrapped object.
        :param time_step:        Time step of the simulation.
        :param use_single_layer: If not None, only the layer with this index is used instead of all layers.
        """
        Sensor.__init__(self, time_step)
        self.enablePointCloud()
        # Set maximum possible scanning rate
        self.frequency = self.max_frequency

        # Update time-stamps are saved to prevent unnecessary parsing when not new data is available
        self._update_period = 1 / self.frequency
        self._last_update_timestamp = -self._update_period  # Ensure that the first update is immediate

        # For performance improvements, pre-allocate the point cloud array to avoid memory reallocation on
        # every update. See more at https://github.com/cyberbotics/webots/issues/2283 and connected issues.
        if use_single_layer is None:
            self._point_cloud = np.empty((self.number_of_layers, self.horizontal_resolution), dtype=float)
            """2D array of shape (number of layers, number of points per layer)"""
        else:
            assert 0 <= use_single_layer < self.number_of_layers, \
                f"Invalid layer index {use_single_layer} for lidar with {self.number_of_layers} layers"
            self._point_cloud = np.empty((self.horizontal_resolution,), dtype=float)
            """Array of points for selected layer"""
        self._single_layer = use_single_layer

        # Variables below are used when parsing point cloud data, but are declared here to avoid unnecessary
        # re-calculation on every update - they are constant for the given lidar instance.
        self._point_data_len = self.WbLidarPoint.size
        self._layer_data_len = self._point_data_len * self.horizontal_resolution

        # The vertical_fov defines the vertical repartition of the layers (angle between first and last layer).
        # So for 4 layers and vertical_fov 0.2, the first layer is at 0.1, last at -0.1 and the 2 others at
        # equally spaced between them with 0.2/3 spacing (there are 3 spaces between 4 layers, not 4).
        if self.number_of_layers > 1:
            # Number of spaces between layers is 1 less than the number of layers
            layer_angle = self.vertical_fov / (self.number_of_layers - 1)
            # Half of the layers are above the horizontal plane, half below
            layer_angle_offset = -self.vertical_fov / 2
            # Save the tilt angles for each layer for use when parsing point cloud data
            layer_angles = tuple(i * layer_angle + layer_angle_offset for i in range(self.number_of_layers))
        else:
            layer_angles = (0,)
        # If wb_lidar_get_[layer]_range_image is used, self._layer_dist_k are used to get the "planar"
        # distance from a tilted image using the formula:
        #   corrected_dist = image_dist[layer] * self._layer_dist_k[layer]
        self._layer_dist_k = tuple(math.cos(a) for a in layer_angles)

        angles = ", ".join(f"{Angle(a, normalize='-pi').deg:.1f}" for a in layer_angles)
        print(f"Lidar '{self.name}' with {self.number_of_layers}x{self.horizontal_resolution} points and"
              f" {angles} degree layer angles, update rate {1000 * self._update_period:.0f} ms"
              f" (every {math.ceil(1000 * self._update_period / time_step)} steps)")

    def _distance_from_wb_point(self, data: bytes, offset: int) -> float:
        x, y, z = self.WbLidarPoint.unpack_from(data, offset)[:3]
        # The X, Y and Z coordinates are relative to the Lidar node origin.
        return Point.from_xyz(x, y, z).distance

    def __call__(self, current_time: float) -> bool:
        """
        Update the lidar measurement data

        :param current_time:  Current time in seconds. As lidar has limited update frequency, this is
                              used to determine if a new cloud point measurement is available or not.

        :return: True if new scan has been parsed and saved to internal buffer, False otherwise.
        """
        if current_time - self._last_update_timestamp < self._update_period:
            return False
        self._last_update_timestamp = current_time

        # https://cyberbotics.com/doc/reference/lidar?tab-language=c#wb_lidar_get_point_cloud
        # Unfortunately, data_type='buffer' have no effect.
        def parse_layer(layer_index: int, distance_array: np.ndarray):
            # Also check controller.Lidar.getPointCloud() for example.
            layer_data = bytes(wb.wb_lidar_get_layer_point_cloud(self._tag, layer_index)[:self._layer_data_len])
            for i_point, offset in enumerate(range(0, self._layer_data_len, self._point_data_len)):
                distance_array[i_point] = self._distance_from_wb_point(layer_data, offset)

        if self._single_layer is None:
            # Get reference to every row/layer in the internal point cloud buffer
            for i_layer, array_for_this_layer in enumerate(self._point_cloud):  # type: int, np.ndarray
                parse_layer(i_layer, array_for_this_layer)
        else:
            parse_layer(self._single_layer, self._point_cloud)

        return True

    @property
    def point_clouds(self) -> np.ndarray[float]:
        """
        Get the point cloud data for all layers (only if initialized without `use_only_layer_index`)

        :return: 2D array of shape (num layers, num layer points) of type Lidar.Point
        """
        assert len(self._point_cloud.shape) == 2, "Lidar initialized with single layer index, point cloud is not 2D"
        return self._point_cloud

    @property
    def point_cloud(self) -> np.ndarray[float]:
        """
        Get the point cloud data for selected layer

        :return: Array of type Lidar.Point
        """
        assert len(self._point_cloud.shape) == 1, "Lidar initialized without single layer index, point cloud is 2D"
        return self._point_cloud


# endregion Sensors

# region Distance Sensors

class DistanceSensor(InstanceSubclass, Sensor, controller.DistanceSensor):

    def __init__(self, _: controller.DistanceSensor, time_step: int):
        Sensor.__init__(self, time_step)

    @property
    def mm(self) -> int:
        """Get distance in millimeters"""
        return round(self.value * 1000)


# endregion Distance Sensors

# region Wheels

class PositionSensor(InstanceSubclass, Sensor, controller.PositionSensor):

    def __init__(self, _: controller.PositionSensor, time_step: int):
        Sensor.__init__(self, time_step)


class Motor(InstanceSubclass, Device, controller.Motor):

    def __init__(self, _: controller.Motor, sensor: controller.PositionSensor, time_step: int):
        Device.__init__(self, time_step)
        # Prefer integrated sensor if available
        self.sensor = PositionSensor(self.position_sensor or sensor, time_step)
        # Enable velocity control instead of position control
        self.target_position = float("inf")
        self.target_velocity = 0

    @property
    def velocity(self) -> Angle:
        """Get angular velocity (ω, omega) in radians/s"""
        return Angle(self.target_velocity, normalize=None)

    @velocity.setter
    def velocity(self, omega: Angle) -> None:
        """Set angular velocity (ω, omega) in radians/s"""
        self.target_velocity = omega


# endregion Wheels

# region Other, built-in

class LED(InstanceSubclass, Device, controller.LED):

    def __init__(self, _: controller.LED, time_step: int):
        Device.__init__(self, time_step)


class Receiver(InstanceSubclass, Sensor, controller.Receiver):

    def __init__(self, _: controller.Receiver, time_step: int):
        Sensor.__init__(self, time_step)


class Emitter(InstanceSubclass, Device, controller.Emitter):

    def __init__(self, _: controller.Emitter, time_step: int):
        Device.__init__(self, time_step)


# endregion Other, built-in


def _check_implementations():
    to_implement = set(cls[0] for cls in inspect.getmembers(controller) if inspect.isclass(cls[1]))
    implemented = set(cls[0] for cls in inspect.getmembers(sys.modules[__name__]) if inspect.isclass(cls[1]))
    missing = to_implement - implemented
    if missing:
        print("Missing implementations for the following controller devices:", ", ".join(missing))


if __name__ == "__main__":
    _check_implementations()
