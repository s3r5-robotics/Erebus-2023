


#######################
# FILE: run.py
#######################

absolute_dir = r'C:\Programming\RoboCup_Erebus\Other Teams\rescate_laberinto-ale-new_navigation\src'
import sys

sys.path.append(absolute_dir)


#######################
# FILE: utilities.py
#######################

import math
import cv2 as cv
import numpy as np
import os
from typing import Callable, List
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from controller import Robot as WebotsRobot
from copy import copy, deepcopy
from heapq import heappop, heappush


script_dir = os.path.dirname(__file__)
image_dir = os.path.join(script_dir, "images")

def save_image(image, filename):
    cv.imwrite(os.path.join(image_dir, filename), image)


def normalizeRads(rad):
    rad %= 2 * math.pi
    if rad < 0:
        rad += 2 + math.pi
    return rad

# Converts from degrees to radians
def degsToRads(deg):
    return deg * math.pi / 180

# Converts from radians to degrees
def radsToDegs(rad):
    return rad * 180 / math.pi

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)

# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


def multiplyLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 * item2)
    return finalList

def sumLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 + item2)
    return finalList

def substractLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 - item2)
    return finalList

def divideLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 / item2)
    return finalList


def draw_grid(image, square_size, offset = [0,0], color=255):
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if (y + offset[1]) % square_size == 0 or (x + offset[0]) % square_size == 0:
                if len(image.shape) == 3:
                    image[y][x][:] = color
                else:
                    image[y][x] = color

def draw_poses(image, poses, color=255, back_image=None, xx_yy_format=False):
    if xx_yy_format:
        if back_image is not None:
            in_bounds_x = (poses[0] < min(image.shape[0], back_image.shape[0]) - 1) & (poses[0] > 0)
            in_bounds_y = (poses[1] < min(image.shape[1], back_image.shape[1]) - 1) & (poses[1] > 0)
        else:
            in_bounds_x = (poses[0] < image.shape[0] - 1) & (poses[0] > 0)
            in_bounds_y = (poses[1] < image.shape[1] - 1) & (poses[1] > 0)
        
        poses = (poses[0][in_bounds_x & in_bounds_y], poses[1][in_bounds_x & in_bounds_y])

        if back_image is None:
            image[poses[1], poses[0], :] = color
        else:
            image[poses[1], poses[0], :] = back_image[poses[1], poses[0], :]
        
    else:
        in_bounds = (poses[:, 0] >= 0) & (poses[:, 0] < image.shape[1]) & (poses[:, 1] >= 0) & (poses[:, 1] < image.shape[0])
        poses = poses[in_bounds]

        if back_image is None:
            image[poses[:, 1], poses[:, 0], :] = color
        else:
            image[poses[:, 1], poses[:, 0], :] = back_image[poses[:, 1], poses[:, 0], :]
            

def draw_squares_where_not_zero(image, square_size, offsets, color=(255, 255, 255)):
    ref_image = image.copy()
    for y in range(image.shape[0] // square_size):
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size)        + (square_size - offsets[1]),
                ((y + 1) * square_size)  + (square_size - offsets[1]), 
                (x * square_size)        + (square_size - offsets[0]),
                ((x + 1) * square_size)  + (square_size - offsets[0])]
            square = ref_image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
            non_zero_count = np.count_nonzero(square)
            if non_zero_count > 0:
                #print("Non zero count: ", non_zero_count)
                #print("max: ", np.max(square))
                cv.rectangle(image, (square_points[2], square_points[0]), (square_points[3], square_points[1]), color, 3)

def get_squares(image, square_size, offsets):
    grid = []
    for y in range(image.shape[0] // square_size):
        row = []
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size)        + (square_size - offsets[1]),
                ((y + 1) * square_size)  + (square_size - offsets[1]), 
                (x * square_size)        + (square_size - offsets[0]),
                ((x + 1) * square_size)  + (square_size - offsets[0])]
            row.append(square_points)
        grid.append(row)
    return grid

def resize_image_to_fixed_size(image, size):
    if image.shape[0] > size[0]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]))
    
    elif image.shape[1] > size[1]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height))
    
    elif image.shape[1] >= image.shape[0]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height), interpolation=cv.INTER_NEAREST)
    
    elif image.shape[0] >= image.shape[1]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]), interpolation=cv.INTER_NEAREST)
    
    return final_image


def divide_into_chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]



#######################
# FILE: flags.py
#######################


SHOW_FIXTURE_DEBUG = False
SHOW_DEBUG = False

SHOW_GRANULAR_NAVIGATION_GRID = True
SHOW_PATHFINDING_DEBUG = False
SHOW_BEST_POSITION_FINDER_DEBUG = False

PRINT_MAP_AT_END = True

DO_WAIT_KEY = True


#######################
# FILE: data_structures/angle.py
#######################

import math

class Angle:
    RADIANS = 0
    DEGREES = 1
    def __init__(self, value, unit=RADIANS):
        if unit == self.RADIANS:
            self.__radians = float(value)
        else:
            self.degrees = value

    @property
    def radians(self, value):
        self.__radians = value

    @radians.getter
    def radians(self):
        return float(self.__radians)
    
    @property
    def degrees(self):
        return float(self.__radians * 180 / math.pi)
    
    @degrees.setter
    def degrees(self, value):
        self.__radians = value * math.pi / 180

    def normalize(self):
        self.__radians %= 2 * math.pi

        if self.__radians < 0:
            self.__radians += 2 + math.pi
    
    def get_absolute_distance_to(self, angle):
        angle = copy(angle)
        angle.normalize()
        min_ang = min(self.radians, angle.radians)
        max_ang = max(self.radians, angle.radians)

        clockwise_distance = max_ang - min_ang
        counterclockwise_distance = (math.pi * 2 + min_ang) - max_ang

        return Angle(min(clockwise_distance, counterclockwise_distance))
    
    def __str__(self):
        return str(self.degrees)
    
    def __add__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians + other.radians)
        return Angle(self.radians + other)
    
    def __radd__(self, other):
        return self.__add__(other)
    
    def __sub__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians - other.radians)
        return Angle(self.radians - other)
    
    def __rsub__(self, other):
        return self.__sub__(other)
    
    def __mul__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians * other.radians)
        return Angle(self.radians * other)
    
    def __rmul__(self, other):
        return self.__mul__(other)
    
    def __truediv__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians / other.radians)
        return Angle(self.radians / other)
    
    def __rtruediv__(self, other):
        return self.__truediv__(other)
    
    def __floordiv__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians // other.radians)
        return Angle(self.radians // other)
    
    def __rfloordiv__(self, other):
        return self.__floordiv__(other)
    
    def __mod__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians % other.radians)
        return Angle(self.radians % other)
    
    def __rmod__(self, other):
        return self.__mod__(other)
    
    def __divmod__(self, other):
        if isinstance(other, Angle):
            return (Angle(self.radians // other.radians), Angle(self.radians % other.radians))
        return (Angle(self.radians // other), Angle(self.radians % other))
    
    def __rdivmod__(self, other):
        return self.__divmod__(other)
    
    def __pow__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians ** other.radians)
        return Angle(self.radians ** other)
    
    def __rpow__(self, other):
        return self.__pow__(other)
    
    def __neg__(self):
        return Angle(-self.radians)

    def __pos__(self):
        return self

    def __abs__(self):
        return Angle(abs(self.radians))

    def __eq__(self, other):
        if isinstance(other, Angle):
            return self.radians == other.radians
        return self.radians == other

    def __ne__(self, other):
        if isinstance(other, Angle):
            return self.radians != other.radians
        return self.radians != other

    def __lt__(self, other):
        if isinstance(other, Angle):
            return self.radians < other.radians
        return self.radians < other

    def __le__(self, other):
        if isinstance(other, Angle):
            return self.radians <= other.radians
        return self.radians <= other

    def __gt__(self, other):
        if isinstance(other, Angle):
            return self.radians > other.radians
        return self.radians > other

    def __ge__(self, other):
        if isinstance(other, Angle):
            return self.radians >= other.radians
        return self.radians >= other

    def __int__(self):
        return int(self.radians)

    def __float__(self):
        return float(self.radians)

    def __complex__(self):
        return complex(self.radians)
    
    def __round__(self, ndigits=None):
        return Angle(round(self.__radians, ndigits))
    



#######################
# FILE: data_structures/compound_expandable_grid.py
#######################

import numpy as np
import cv2 as cv
import copy
import math


class CompoundExpandableGrid:
    def __init__(self, initial_shape, pixel_per_m):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets # Maximum grid index
        self.grid_index_min = self.offsets * -1 # Minimum grid index

        self.arrays = {}

        self.resolution = pixel_per_m # resolution of the grid with regards to the coordinate system of the gps / the world
    
    # Index conversion
    def coordinates_to_grid_index(self, coordinates: np.ndarray) -> np.ndarray:
        coords = (coordinates * self.resolution).astype(int)
        return np.flip(coords)

    def grid_index_to_coordinates(self, grid_index: np.ndarray) -> np.ndarray:
        index = (grid_index.astype(float) / self.resolution)
        return np.flip(index)

    def array_index_to_grid_index(self, array_index: np.ndarray) -> np.ndarray:
        return array_index - self.offsets
    
    def grid_index_to_array_index(self, grid_index: np.ndarray) -> np.ndarray:
        return grid_index + self.offsets
    
    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)
    
    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # Grid expansion
    def expand_to_grid_index(self, grid_index: np.ndarray):
        """
        Expands all arrays to the specified index. 
        Note that all array_idexes should be recalculated after this operation.
        """

        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)

        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)

        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)
    
    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        
        for key in self.arrays:
            self.arrays[key] = self.__add_end_row_to_grid(self.arrays[key], size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_row_to_grid(self.arrays[key], size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_end_column_to_grid(self.arrays[key], size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_column_to_grid(self.arrays[key], size)

    def __add_end_row_to_grid(self, grid, size):
        grid = np.vstack((grid, np.zeros((size, self.array_shape[1]), dtype=grid.dtype)))
        return grid
    
    def __add_begining_row_to_grid(self, grid, size):
        grid = np.vstack((np.zeros((size, self.array_shape[1]), dtype=grid.dtype), grid))
        return grid
    
    def __add_end_column_to_grid(self, grid, size):
        grid = np.hstack((grid, np.zeros((self.array_shape[0], size), dtype=grid.dtype)))
        return grid

    def __add_begining_column_to_grid(self, grid, size):
        grid = np.hstack((np.zeros((self.array_shape[0], size), dtype=grid.dtype), grid))
        return grid
    
    # Debug
    def get_colored_grid(self):
       pass
    


#######################
# FILE: flow_control/sequencer.py
#######################




class Sequencer:
    """
    Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy.
    For example, one can make the robot execute a series of pre-programmed functions with delays and so on, without interrupting
    a sensor that must run continously. 
    This functions basically as an alternative to multithreading or multiprocessing.
    """
    def __init__(self, reset_function=None):
        self.line_identifier = 0
        self.line_pointer = 1
        self.done = False
        self.reset_function = reset_function

    def reset_sequence(self):
        """
        Resets the sequence and makes it start from the first event.
        """
        if self.reset_function is not None:
            self.reset_function()
        self.line_pointer = 1
        if SHOW_DEBUG:
            print("----------------")
            print("reseting sequence")
            print("----------------")

    def seq_reset_sequence(self):
        if self.check():
            self.reset_sequence()
            
            return True
        return False

    def start_sequence(self):
        """
        Starts the sequence. This must be at the start of any sequence of events.
        """
        self.line_identifier = 0
        self.done = False


    def check(self):
        """
        Returns if the line pointer and identifier match and increases the identifier.
        Must be included at the end of any sequential function.
        """
        self.done = False
        self.line_identifier += 1
        return self.line_identifier == self.line_pointer

    def next_seq(self):
        """
        Changes to the next event.
        """
        self.line_pointer += 1
        self.done = True

    def seq_done(self):
        """
        returns if the sequence has reached its end
        """
        return self.done

    def simple_event(self, function=None, *args, **kwargs):
        """
        Can be used to make a function sequential or used with an if statement to make a code block sequential.
        """
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.next_seq()
            return True
        return False

    def complex_event(self, function, *args, **kwargs):
        """
        Can be used to make a function sequential. The function inputted must return True when it ends
        """
        if self.check():
            if function(*args, **kwargs):
                self.next_seq()
                return True
        return False
    
    def make_simple_event(self, function):
        """
        When inpuuted any function it returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.next_seq()
                return True
            return False
        return event

    def make_complex_event(self, function):
        """
        When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence.
        """
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.next_seq()
                    return True
            return False
        return event



#######################
# FILE: flow_control/state_machine.py
#######################


class StateMachine:
    """
    A simple state machine.
    """
    def __init__(self, initial_state, function_on_change_state=lambda:None):
        self.state = initial_state
        self.current_function = lambda:None

        self.change_state_function = function_on_change_state

        self.state_functions = {}
        self.allowed_state_changes = {}
        self.possible_states = set()

    def create_state(self, name: str, function: Callable, possible_changes = set()):
        if name in self.possible_states:
            raise ValueError("Failed to create new state. State already exists.")
        self.possible_states.add(name)
        self.state_functions[name] = function
        self.allowed_state_changes[name] = possible_changes
        if name == self.state:
            self.current_function = self.state_functions[self.state]

    def change_state(self, new_state):
        """Sets the state the specified value."""
        if new_state not in self.possible_states:
            raise ValueError("Can't change state. New state doesn't exist.")
        
        if new_state in self.allowed_state_changes[self.state]:
            self.change_state_function()
            self.state = new_state
            self.current_function = self.state_functions[self.state]

        else:
            raise RuntimeWarning("Can't change state. New state is not in the possible changes for old state.")
        return True

    def check_state(self, state):
        """Checks if the state corresponds the specified value."""
        return self.state == state
    
    def run(self):
        return self.current_function(self.change_state)


#######################
# FILE: flow_control/delay.py
#######################



class DelayManager:
    def __init__(self) -> None:
        self.time = 0
        self.delay_first_time = True
        self.delay_start = 0
    
    def update(self, time):
        self.time = time

    def delay_seconds(self, delay):
            if SHOW_DEBUG:
                print("Current delay: ", delay)
            if self.delay_first_time:
                self.delay_start = self.time
                self.delay_first_time = False
            else:
                if self.time - self.delay_start >= delay:
                    
                    self.delay_first_time = True
                    return True
            return False
    
    def reset_delay(self):
         self.delay_first_time = True


#######################
# FILE: data_structures/vectors.py
#######################

import math
import numpy as np


class Position2D:
    def __init__(self, *args, **kwargs):
        """
        Takes either two values or an iterable with at least two indices.
        """
        if len(args) == 0:
            self.x = None
            self.y = None
        elif len(args) == 1:
            self.x = args[0][0]
            self.y = args[0][1]
        elif len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            raise TypeError()

    
    def __iter__(self):
        yield self.x
        yield self.y
    
    def __array__(self, *args, **kwargs):
        return np.array([self.x, self.y], *args, **kwargs)
        
    def __repr__(self):
        return f"Position2D({self.x}, {self.y})"
    
    def __eq__(self, other):
        if isinstance(other, Position2D):
            return self.x == other.x and self.y == other.y
        else:
            return False
    
    def __add__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x + other.x, self.y + other.y)
        else:
            return Position2D(self.x + other, self.y + other)
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x - other.x, self.y - other.y)
        else:
            return Position2D(self.x - other, self.y - other)
    
    def __rsub__(self, other):
        return -self + other
    
    def __mul__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x * other.x, self.y * other.y)
        else:
            return Position2D(self.x * other, self.y * other)
    
    def __rmul__(self, other):
        return self * other
    
    def __truediv__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x / other.x, self.y / other.y)
        else:
            return Position2D(self.x / other, self.y / other)
    
    def __rtruediv__(self, other):
        return Position2D(other / self.x, other / self.y)
    
    def __floordiv__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x // other.x, self.y // other.y)
        return Position2D(self.x // other, self.y // other)
    
    def __rfloordiv__(self, other):
        return self.__floordiv__(other)
    
    def __mod__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x % other.x, self.y % other.y)
        else:
            return Position2D(self.x % other, self.y % other)
    
    def __rmod__(self, other):
        return self.__mod__(other)
    
    def __divmod__(self, other):
        return self.__floordiv__(other), self.__mod__(other)
    
    
    def __rdivmod__(self, other):
        return self.__divmod__(other)
    
    def __pow__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x ** other.x, self.y ** other.y)
        else:
            return Position2D(self.x ** other, self.y ** other)
    
    def __rpow__(self, other):
        return self.__pow__(other)
    
    def __neg__(self):
        return Position2D(-self.x, -self.y)
    
    def __pos__(self):
        return Position2D(self.x, self.y)
    
    def __abs__(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise IndexError("Vector index out of range")
    
    def __setitem__(self, index, value):
        if index == 0:
            self.x = value
        elif index == 1:
            self.y = value
        else:
            raise IndexError("Vector index out of range")
        
    def astype(self, dtype: type):
        return self.apply_to_all(dtype)
    
    def apply_to_all(self, function):
        return Position2D(function(self.x), function(self.y))
    
    def get_distance_to(self, other):
        return abs(self - other)
    
    def get_angle_to(self, other):
        delta = self - other 
        result = Angle(math.atan2(delta.x, delta.y)) + Angle(180, Angle.DEGREES)
        result.normalize()
        return result
    

    def to_vector(self):
        m = Position2D(0, 0).get_distance_to(self)
        a = Position2D(0, 0).get_angle_to(self)
        return Vector2D(a, m)
       
class Vector2D:
    def __init__(self, direction:Angle=None, magnitude=None):
        self.direction = direction
        self.magnitude = magnitude
        
    def __repr__(self):
        return f"Vector2D(direction={self.direction}, magnitude={self.magnitude})"
    
    def __eq__(self, other):
        if isinstance(other, Vector2D):
            return self.direction == other.direction and self.magnitude == other.magnitude
        else:
            return False
    
    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.direction + other.direction, self.magnitude + other.magnitude)
        else:
            raise TypeError("Argument must be of type Vector2D")
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.direction - other.direction, self.magnitude - other.y)
        else:
            raise TypeError("Argument must be of type Vector2D")
    
    def __rsub__(self, other):
        return -self + other
    
    
    def __neg__(self):
        return Vector2D(-self.direction, -self.magnitude)
    
    def __pos__(self):
        return Vector2D(self.direction, self.magnitude)
    
    
    def to_position(self):
        y = float(self.magnitude * math.cos(self.direction.radians))
        x = float(self.magnitude * math.sin(self.direction.radians))
        return Position2D(x, y)
    


#######################
# FILE: executor/stuck_detector.py
#######################


class StuckDetector:
    """Checks if the robot is rotating the wheels but not actually moving."""
    def __init__(self) -> None:
        self.stuck_counter = 0

        self.stuck_threshold = 50
        self.minimum_distance_traveled = 0.00001

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)
        self.__wheel_direction = 0

    def update(self, position, previous_position, wheel_direction):
        self.__wheel_direction = wheel_direction
        self.__position = position
        self.__previous_position = previous_position

        # Check if the robot is not moving
        if self.__is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0    

    def is_stuck(self):
        return self.stuck_counter > self.stuck_threshold
    
    def __is_stuck_this_step(self):
        distance_traveled = self.__position.get_distance_to(self.__previous_position)
        is_rotating_wheels = self.__wheel_direction > 0
        return is_rotating_wheels and distance_traveled < self.minimum_distance_traveled


   


#######################
# FILE: flow_control/step_counter.py
#######################

class StepCounter:
    """
    Allows to execute actions every n number of timesteps. This can be useful for performance, as it enables the program
    to execute taxing tasks sparsely while not interrupting actions that must run constantly.
    """

    def __init__(self, interval):
        self.__current_step = 0
        self.interval = interval

    def increase(self):
        self.__current_step += 1
        if self.__current_step == self.interval:
            self.__current_step = 0
    
    def check(self):
        return self.__current_step == 0


#######################
# FILE: robot/devices/wheel.py
#######################

# Controlls a wheel
class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.velocity = 0
        self.wheel.setPosition(float("inf"))
        self.wheel.setVelocity(0)

    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < -1:
            ratio = -1
        self.velocity = ratio * self.maxVelocity
        self.wheel.setVelocity(self.velocity)


#######################
# FILE: robot/devices/sensor.py
#######################


class Sensor(ABC):
    def __init__(self, webots_device, time_step):
        self.time_step = time_step
        self.device = webots_device
        self.device.enable(time_step)

    def update(self):
        pass

class TimedSensor(Sensor):
    def __init__(self, webots_device, time_step, step_counter):
        super().__init__(webots_device, time_step)
        self.step_counter = step_counter

    def update(self):
        self.step_counter.increase()



#######################
# FILE: robot/devices/camera.py
#######################

import numpy as np
import cv2 as cv




import math

@dataclass
class CameraData:
    height: int
    width: int
    vertical_fov: Angle
    horizontal_fov: Angle
    relative_vertical_orientation: Angle
    relative_horizontal_orientation: Angle
    vertical_orientation: Angle
    horizontal_orientation: Angle
    distance_from_center: float

class CameraImage:
    def __init__(self) -> None:
        self.image: np.ndarray = None
        self.data: CameraData = None

# Captures images and processes them
class Camera(TimedSensor):
    def __init__(self, webots_device, time_step, step_counter: StepCounter, orientation: Angle, distance_from_center: float, rotate180=False):
        super().__init__(webots_device, time_step, step_counter)
        self.rotate180 = rotate180
        self.height = self.device.getHeight()
        self.width = self.device.getWidth()
        self.horizontal_fov = Angle(self.device.getFov())
        self.vertical_fov = Angle(2 * math.atan(math.tan(self.horizontal_fov * 0.5) * (self.height / self.width)))
        self.image = CameraImage()
        
        self.horizontal_orientation_in_robot = orientation
        self.vertical_orientation_in_robot = Angle(0)

        self.horizontal_orientation = orientation
        self.vertical_orientation = Angle(0)
        self.distance_from_center = distance_from_center

    # Returns the camera image
    def get_image(self):
        if self.step_counter.check():
            return self.image
        
    def get_data(self):
        data = CameraData(self.height,
                          self.width,
                          self.vertical_fov,
                          self.horizontal_fov,
                          self.vertical_orientation_in_robot,
                          self.horizontal_orientation_in_robot,
                          self.vertical_orientation,
                          self.horizontal_orientation,
                          self.distance_from_center)
        return data
    
    def update(self, robot_orientation: Angle):
        super().update()

        self.horizontal_orientation = self.horizontal_orientation_in_robot + robot_orientation
        
        # Do evey n steps
        if self.step_counter.check():
            # Extract image from buffer
            image_data = self.device.getImage()
            self.image.image = np.array(np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4)))

            if self.rotate180:
                self.image.image = np.rot90(self.image.image, 2, (0, 1))

            self.image.orientation = self.horizontal_orientation

            self.image.data = self.get_data()

            


#######################
# FILE: robot/devices/lidar.py
#######################

import math

import utilities


# Returns a point cloud of the detctions it makes
class Lidar(TimedSensor):
    def __init__(self, webots_device, time_step, step_counter, layers_used=range(4)):
        super().__init__(webots_device, time_step, step_counter)
        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = Angle(0)
        
        self.horizontal_fov = self.device.getFov()
        self.vertical_fov = self.device.getVerticalFov()

        self.horizontal_resolution = self.device.getHorizontalResolution()
        self.vertical_resolution = self.device.getNumberOfLayers()

        self.radian_per_detection_horizontally = self.horizontal_fov / self.horizontal_resolution
        self.radian_per_layer_vertically = self.vertical_fov / self.vertical_resolution

        self.rotation_offset = 0

        self.max_detection_distance = 0.06 * 8
        self.min_detection_distance = 0.06 * 0.6

        self.is_point_close = False
        self.is_point_close_threshold = 0.03
        self.is_point_close_range = (0, 360)

        self.distance_bias = 0#0.06 * 0.12

        self.layers_used = layers_used

        self.__point_cloud = None
        self.__out_of_bounds_point_cloud = None
        self.__distance_detections = None

    # Returns the in-bounds point cloud
    def get_point_cloud(self):
        if self.step_counter.check():
            return self.__point_cloud
    
    # Returns a point cloud with all the out of bounds detections as points with a fixed distance
    # to the center.
    def get_out_of_bounds_point_cloud(self):
        if self.step_counter.check():
            return self.__out_of_bounds_point_cloud
        
    def get_detections(self):
        if self.step_counter.check():
            return self.__distance_detections

    def set_orientation(self, angle):
        self.orientation = angle


    def update(self):
        super().update()

        # Do every n steps
        if self.step_counter.check():
            self.__update_point_clouds()


    # Create point clouds from detections and check if a point is close
    def __update_point_clouds(self):
        self.is_point_close = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        self.__point_cloud = []
        self.__out_of_bounds_point_cloud = []
        self.__distance_detections = []

        total_depth_array = self.device.getRangeImage()
        total_depth_array = divide_into_chunks(total_depth_array, self.horizontal_resolution)
        #print(total_depth_array)
        
        for layer_number, layer_depth_array in enumerate(total_depth_array):
            if layer_number not in self.layers_used:
                continue

            vertical_angle = layer_number * self.radian_per_layer_vertically + self.vertical_fov / 2
            horizontal_angle = self.rotation_offset + ((2 * math.pi) - self.orientation.radians)

            for item in layer_depth_array:
                # Item is out of bounds
                if item >= self.max_detection_distance or item == float("inf") or item == float("inf") *-1:
                    
                    # Corrects for vertical rotation and adds offset
                    distance = self.__normalize_distance(self.max_detection_distance, vertical_angle)
                    # Calculates 2d point from distance and horizontal angle
                    point = utilities.getCoordsFromRads(horizontal_angle, distance)
                    self.__out_of_bounds_point_cloud.append(self.__normalize_point(point))
                
                # Item is in bounds
                else:
                    if item >= self.min_detection_distance:
                        # Corrects for vertical rotation and adds offset
                        distance = self.__normalize_distance(item, vertical_angle)
                        # Calculates 2d point from distance and horizontal angle
                        point = utilities.getCoordsFromRads(horizontal_angle, distance)
                        self.__point_cloud.append(self.__normalize_point(point))

                        v = Vector2D(Angle(horizontal_angle), distance)
                        v.direction = Angle(math.pi) - v.direction
                        v.direction.normalize()
                        self.__distance_detections.append(v)
                        
                        #Check if point is close
                        if self.__in_range_for_close_point(horizontal_angle) and distance < self.is_point_close_threshold:
                            self.is_point_close = True

                horizontal_angle += self.radian_per_detection_horizontally
        
        if len(self.__out_of_bounds_point_cloud) == 0:
            self.__out_of_bounds_point_cloud = [[0, 0]]
        
        if len(self.__point_cloud) == 0:
            self.__point_cloud = [[0, 0]]
    
    def __in_range_for_close_point(self, horizontal_angle):
        return utilities.degsToRads(self.is_point_close_range[0]) > horizontal_angle > utilities.degsToRads(self.is_point_close_range[1])
    
    def __normalize_distance(self, distance, vertical_angle):
        # Correct for vertical inclination
        distance = distance * math.cos(vertical_angle)
        # Add offset
        distance += self.distance_bias
        return distance

    def __normalize_point(self, point):
            return [point[0], point[1] * -1]




#######################
# FILE: robot/devices/gps.py
#######################



class Gps(Sensor):
    """
    Tracks global position and rotation.
    """
    def __init__(self, webots_device, time_step, coords_multiplier=1):
        super().__init__(webots_device, time_step)
        self.multiplier = coords_multiplier
        self.__prev_position = Position2D()
        self.position = self.get_position()

    def update(self):
        """
        Updates gps, must run every timestep.
        """
        self.__prev_position = self.position
        self.position = self.get_position()

    def get_position(self):
        """
        Returns the global position.
        """
        vals = self.device.getValues()
        return Position2D(vals[0] * self.multiplier, vals[2] * self.multiplier)

    def get_orientation(self):
        """
        Returns the global orientation according to gps. This is calculated from the difference in angle from the current position
        to the position of the previous time_step (The robot must be driving perfectly straight for it to work).
        """
        if self.__prev_position != self.position:
            accuracy = abs(self.position.get_distance_to(self.__prev_position))
            if accuracy > 0.001:
                angle = self.__prev_position.get_angle_to(self.position)
                angle.normalize()
                return angle
        return None


#######################
# FILE: robot/devices/gyroscope.py
#######################



class Gyroscope(Sensor):
    """
    Tracks global rotation.
    """
    def __init__(self, webots_device, index, time_step):
        super().__init__(webots_device, time_step)
        self.index = index
        self.orientation = Angle(0)
        self.angular_velocity = Angle(0)

    def update(self):
        """
        Do on every timestep.
        """
        time_elapsed = self.time_step / 1000
        sensor_y_value = self.device.getValues()[self.index]
        self.angular_velocity = Angle(sensor_y_value * time_elapsed)
        self.orientation += self.angular_velocity
        self.orientation.normalize()

    def get_angular_velocity(self):
        """
        Gets the current angular velocity without direction data.
        """
        return abs(self.angular_velocity)

    def get_orientation(self):
        return self.orientation
    
    def set_orientation(self, angle):
        self.orientation = angle


#######################
# FILE: robot/devices/comunicator.py
#######################

import utilities
import struct


class Comunicator(Sensor):
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lack_of_progress = False
        self.do_get_world_info = True
        self.game_score = 0
        self.remaining_time = 0

    def send_victim(self, position, victimtype):
        self.do_get_world_info = False
        letter = bytes(victimtype, "utf-8")
        position = utilities.multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)
        self.do_get_world_info = False

    def send_lack_of_progress(self):
        self.do_get_world_info = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)
        self.do_get_world_info = False

    def send_end_of_play(self):
        self.do_get_world_info = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        print("Ended!!!!!")

    def send_map(self, np_array):
        # Get shape
        print(np_array)
        s = np_array.shape
        # Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        # Flattening the matrix and join with ','
        flatMap = ','.join(np_array.flatten())
        # Encode
        sub_bytes = flatMap.encode('utf-8')
        # Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        # Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.do_get_world_info = False

    def request_game_data(self):
        if self.do_get_world_info:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

    def update(self):
        if self.do_get_world_info:
            self.request_game_data()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                received_data = self.receiver.getBytes()
                if len(received_data) > 2:
                    tup = struct.unpack('c f i', received_data) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.game_score = tup[1]
                        self.remaining_time = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet

            self.lack_of_progress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                received_data = self.receiver.getBytes()
                print(received_data)
                if len(received_data) < 2:
                    tup = struct.unpack('c', received_data)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lack_of_progress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
        else:
            self.do_get_world_info = True


#######################
# FILE: robot/pose_manager.py
#######################




class PoseManager:
    GPS = 0
    GYROSCOPE = 1

    def __init__(self, gps: Gps, gyroscope: Gyroscope, position_offsets=Position2D(0, 0)) -> None:
        self.maximum_angular_velocity_for_gps = 0.00001

        self.gps = gps
        self.gyroscope = gyroscope

        self.orientation = Angle(0)
        self.previous_orientation = Angle(0)

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)

        self.orientation_sensor = self.GYROSCOPE
        self.automatically_decide_orientation_sensor = True

        self.position_offsets = position_offsets
    
    def update(self, wheel_direction):
        # Gyro and gps update
        self.gps.update()
        self.gyroscope.update()
        
        # Get global position
        self.__previous_position = self.position
        self.__position = self.gps.get_position()

        # Decides wich sensor to use for orientation detection
        if self.automatically_decide_orientation_sensor:
            self.decide_orientation_sensor(wheel_direction)

        # Remembers the corrent rotation for the next timestep
        self.previous_orientation = self.orientation

        self.calculate_orientation()

    def decide_orientation_sensor(self, wheel_direction):
        """if the robot is going srtaight it tuses the gps. If not it uses the gyro."""
        if self.robot_is_going_straight(wheel_direction):
                self.orientation_sensor = self.GPS
        else:
            self.orientation_sensor = self.GYROSCOPE

    def robot_is_going_straight(self, wheel_direction):
        return self.gyroscope.get_angular_velocity() < self.maximum_angular_velocity_for_gps and wheel_direction >= 0

    def calculate_orientation(self):
        
         # Gets global rotation
        gps_orientation = self.gps.get_orientation()

        if self.orientation_sensor == self.GYROSCOPE or gps_orientation is None:
            self.orientation = self.gyroscope.get_orientation()
            if SHOW_DEBUG: print("USING GYRO")
        else:
            self.orientation = gps_orientation
            self.gyroscope.set_orientation(self.orientation)
            if SHOW_DEBUG: print("USING GPS")

    @property
    def position(self):
        return self.__position + self.position_offsets
    
    @property
    def previous_position(self):
        return self.__previous_position + self.position_offsets
        




#######################
# FILE: robot/drive_base.py
#######################

import math

class Criteria(Enum):
    LEFT = 1
    RIGHT = 2
    CLOSEST = 3
    FARTHEST = 4

class DriveBase:
    def __init__(self, left_wheel, right_wheel, max_wheel_velocity) -> None:
        self.max_wheel_velocity = max_wheel_velocity
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.rotation_manager = RotationManager(self.left_wheel, self.right_wheel)
        #self.movement_manager = MovementToCoordinatesManager(self.left_wheel, self.right_wheel)
        self.movement_manager = SmoothMovementToCoordinatesManager(self.left_wheel, self.right_wheel)

    # Moves the wheels at the specified Velocity
    def move_wheels(self, left_ratio, right_ratio):
        self.left_wheel.move(left_ratio)
        self.right_wheel.move(right_ratio)
    
    def rotate_to_angle(self, angle:Angle, criteria:Criteria.CLOSEST) -> bool:
        self.rotation_manager.rotate_to_angle(angle, criteria)
        return self.rotation_manager.finished_rotating
    
    def move_to_position(self, position:Position2D) -> bool:
        self.movement_manager.move_to_position(position)
        return self.movement_manager.finished_moving
    
    @property
    def position(self) -> Position2D:
        return self.movement_manager.current_position
    
    @position.setter
    def position(self, value:Position2D):
        self.movement_manager.current_position = value

    @property
    def orientation(self) -> Angle:
        return self.rotation_manager.current_angle
    
    @orientation.setter
    def orientation(self, value:Angle):
        self.movement_manager.current_angle = value
        self.rotation_manager.current_angle = value


    def get_wheel_direction(self):
        if self.right_wheel.velocity + self.left_wheel.velocity == 0:
            return 0
        return (self.right_wheel.velocity + self.left_wheel.velocity) / 2




class RotationManager:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.Directions = Enum("Directions", ["LEFT", "RIGHT"])
        
        self.right_wheel = right_wheel
        self.left_wheel = left_wheel

        self.initial_angle = Angle(0)
        self.current_angle = Angle(0)

        self.first_time = True
        self.finished_rotating = True

        self.max_velocity_cap = 1
        self.min_velocity_cap = 0.2

        self.max_velocity = 1
        self.min_velocity = 0.2

        self.velocity_reduction_threshold = Angle(10, Angle.DEGREES)

        self.accuracy = Angle(2, Angle.DEGREES)

    def rotate_to_angle(self, target_angle, criteria=Criteria.CLOSEST):
        if self.first_time:
            self.initial_angle = self.current_angle
            self.first_time = False
            self.finished_rotating = False

        if self.is_at_angle(target_angle):
            self.finished_rotating = True
            self.first_time = True
            self.left_wheel.move(0)
            self.right_wheel.move(0)

        absolute_difference = self.current_angle.get_absolute_distance_to(target_angle)
        velocity = mapVals(absolute_difference.degrees, self.accuracy.degrees, 90, self.min_velocity, self.max_velocity)

        if absolute_difference < self.velocity_reduction_threshold:
            velocity *= 0.5

        velocity = min(velocity, self.max_velocity_cap)
        velocity = max(velocity, self.min_velocity_cap)


        direction = self.__get_direction(target_angle, criteria)
        
        if direction == self.Directions.RIGHT:
            self.left_wheel.move(velocity * -1)
            self.right_wheel.move(velocity)
        elif direction == self.Directions.LEFT:
            self.left_wheel.move(velocity)
            self.right_wheel.move(velocity * -1)
    
    def is_at_angle(self, angle) -> bool:
        return self.current_angle.get_absolute_distance_to(angle) < self.accuracy

    def __get_direction(self, target_angle, criteria):
        if criteria == Criteria.CLOSEST:
            angle_difference = self.current_angle - target_angle

            if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
                return self.Directions.RIGHT
            else:
                return self.Directions.LEFT

        elif criteria == Criteria.FARTHEST:
            angle_difference = self.initial_angle - target_angle
            if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
                return self.Directions.LEFT
            else:
                return self.Directions.RIGHT

        elif criteria == Criteria.LEFT: return self.Directions.LEFT
        elif criteria == Criteria.RIGHT: return self.Directions.RIGHT


class MovementToCoordinatesManager:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.current_position = Position2D()

        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.rotation_manager = RotationManager(self.left_wheel, self.right_wheel)

        self.error_margin = 0.0005
        self.desceleration_start = 0.5 * 0.12

        self.max_velocity_cap = 1
        self.min_velocity_cap = 0.8

        self.max_velocity = 1
        self.min_velocity = 0.1

        self.finished_moving = False

    @property
    def current_angle(self) -> Angle:
        return self.rotation_manager.current_angle
    
    @current_angle.setter
    def current_angle(self, value):
        self.rotation_manager.current_angle = value

    
    def move_to_position(self, target_position:Position2D):

        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.position)

        dist = abs(self.current_position.get_distance_to(target_position))

        if SHOW_DEBUG: print("Dist: "+ str(dist))

        if dist < self.error_margin:
            # self.robot.move(0,0)
            if SHOW_DEBUG: print("FinisehedMove")
            self.finished_moving = True
        else:
            self.finished_moving = False
            ang = self.current_position.get_angle_to(target_position)

            if self.rotation_manager.is_at_angle(ang):

                velocity = mapVals(dist, 0, self.desceleration_start, self.min_velocity, self.max_velocity)
                velocity = min(velocity, self.max_velocity_cap)
                velocity = max(velocity, self.min_velocity_cap)

                self.right_wheel.move(velocity)
                self.left_wheel.move(velocity)

            else:
                
                self.rotation_manager.rotate_to_angle(ang)


class SmoothMovementToCoordinatesManager:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.current_position = Position2D()

        self.left_wheel = left_wheel
        self.right_wheel = right_wheel

        self.current_angle = Angle(0)

        self.error_margin = 0.003

        self.velocity = 1

        self.distance_weight = 5
        self.angle_weight = 5

        self.turning_speed_multiplier = 1.5

        self.finished_moving = False

        self.angle_error_margin = Angle(2, Angle.DEGREES)

        self.strong_rotation_start = Angle(45, Angle.DEGREES)

    def move_to_position(self, target_position:Position2D):
        dist = abs(self.current_position.get_distance_to(target_position))

        if SHOW_DEBUG: print("Dist: "+ str(dist))

        if dist < self.error_margin:
            # self.robot.move(0,0)
            if SHOW_DEBUG: print("FinisehedMove")
            self.finished_moving = True


        else:
            self.finished_moving = False

            angle_to_target = self.current_position.get_angle_to(target_position)
            angle_diff = self.current_angle - angle_to_target
            absolute_angle_diff = self.current_angle.get_absolute_distance_to(angle_to_target)

            #print("diff:", absolute_angle_diff)
            if absolute_angle_diff < self.angle_error_margin:
                self.right_wheel.move(self.velocity)
                self.left_wheel.move(self.velocity)


            elif absolute_angle_diff > self.strong_rotation_start:
                if 180 > angle_diff.degrees > 0 or angle_diff.degrees < -180:
                    self.right_wheel.move(self.velocity)
                    self.left_wheel.move(self.velocity * -1)
                else:
                    self.right_wheel.move(self.velocity * -1)
                    self.left_wheel.move(self.velocity)

            else:
                distance_speed = dist * -self.distance_weight
                angle_speed = absolute_angle_diff.radians * self.angle_weight

                speed = angle_speed * self.turning_speed_multiplier

                if 180 > angle_diff.degrees > 0 or angle_diff.degrees < -180:
                    self.right_wheel.move(speed)
                    self.left_wheel.move(speed * distance_speed)
                else:
                    self.right_wheel.move(speed * distance_speed)
                    self.left_wheel.move(speed)
                
                
    




#######################
# FILE: robot/robot.py
#######################




# Devices



import cv2 as cv


class Robot:
    """
    Abstraction layer for the webots robot. In charge of low level movement and sensing.
    """
    def __init__(self, time_step):
        self.time_step = time_step
        self.time = 0

        self.diameter = 0.074 # Robot diameter in meters
        
        self.robot = WebotsRobot() # Robot object provided by webots

        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        
        self.pose_manager = PoseManager(self.gps, self.gyroscope) # This manages position and orientation

        # LIDAR
        lidar_interval = 6
        self.lidar = Lidar(webots_device = self.robot.getDevice("lidar"), 
                           time_step = self.time_step * lidar_interval, 
                           step_counter = StepCounter(lidar_interval),
                           layers_used=(2,))
        
        # Cameras
        self.camera_distance_from_center = 0.0295
        camera_interval = 3
        self.center_camera = Camera(webots_device = self.robot.getDevice("camera1"),
                                    time_step = self.time_step * camera_interval,
                                    step_counter = StepCounter(camera_interval),
                                    orientation=Angle(0, Angle.DEGREES),
                                    distance_from_center=self.camera_distance_from_center)
        
        self.right_camera = Camera(webots_device = self.robot.getDevice("camera2"),
                                   time_step = self.time_step * camera_interval,
                                   step_counter = StepCounter(camera_interval),
                                   orientation=Angle(270, Angle.DEGREES),
                                   distance_from_center=self.camera_distance_from_center)
        
        self.left_camera = Camera(webots_device = self.robot.getDevice("camera3"), 
                                  time_step = self.time_step * camera_interval, 
                                  step_counter = StepCounter(camera_interval),
                                  orientation=Angle(90, Angle.DEGREES),
                                  distance_from_center=self.camera_distance_from_center,
                                  rotate180=True)
        
        # Comunicator (Emmiter and reciever)
        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        
        # Low level movement
        max_wheel_speed = 6.28
        self.drive_base = DriveBase(left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), max_wheel_speed), 
                                    right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), max_wheel_speed),
                                    max_wheel_velocity = max_wheel_speed)

    def update(self):
        """Must run every TimeStep"""
        # Update current time
        self.time = self.robot.getTime()

        # Update pose manager (Position and rotation)
        self.pose_manager.update(wheel_direction=self.drive_base.get_wheel_direction())

        # Update drive base
        self.drive_base.orientation = self.orientation
        self.drive_base.position = self.position

        # Lidar update
        self.lidar.set_orientation(self.orientation)
        self.lidar.update()

        # Camera update
        self.right_camera.update(self.orientation)
        self.left_camera.update(self.orientation)
        self.center_camera.update(self.orientation)

    def do_loop(self):
        """Advances the simulation by one step and returns True if the simulation is running."""
        return self.robot.step(self.time_step) != -1

    # Wrappers for DriveBase
    @property
    def max_wheel_speed(self):
        return self.drive_base.max_wheel_velocity

    def move_wheels(self, left_ratio, right_ratio):
        self.drive_base.move_wheels(left_ratio, right_ratio)

    def rotate_to_angle(self, angle, direction=Criteria.CLOSEST):
        return self.drive_base.rotate_to_angle(Angle(angle, Angle.DEGREES), direction)

    def move_to_coords(self, targetPos):
        return self.drive_base.move_to_position(Position2D(targetPos[0], targetPos[1]))
    
    # Wrappers for lidar
    @property
    def point_is_close(self) -> bool:
        return self.lidar.is_point_close

    def get_point_cloud(self):
        return self.lidar.get_point_cloud()

    def get_out_of_bounds_point_cloud(self):
        return self.lidar.get_out_of_bounds_point_cloud()
    

    def get_lidar_detections(self):
        return self.lidar.get_detections()
    
    # Wrapper for cameras
    def get_camera_images(self):
        if self.center_camera.step_counter.check():
            return [self.right_camera.get_image(), 
                    self.center_camera.get_image(), 
                    self.left_camera.get_image()]
        
    # Wrappers for pose
    @property
    def position(self):
        return self.pose_manager.position
    
    @property
    def previous_position(self):
        return self.pose_manager.previous_position
    
    @property
    def position_offsets(self):
        return self.pose_manager.position_offsets
    
    @position_offsets.setter
    def position_offsets(self, value):
        self.pose_manager.position_offsets = value
    
    @property
    def orientation(self):
        return self.pose_manager.orientation
    
    @property
    def previous_orientation(self):
        return self.pose_manager.previous_orientation
    
    @property
    def auto_decide_orientation_sensor(self):
        return self.pose_manager.automatically_decide_orientation_sensor
    
    @auto_decide_orientation_sensor.setter
    def auto_decide_orientation_sensor(self, value):
        self.pose_manager.automatically_decide_orientation_sensor = value

    @property
    def orientation_sensor(self):
        return self.pose_manager.orientation_sensor
    
    @orientation_sensor.setter
    def orientation_sensor(self, value):
        self.pose_manager.orientation_sensor = value
    
    @property
    def GPS(self):
        return PoseManager.GPS
    
    @property
    def GYROSCOPE(self):
        return PoseManager.GYROSCOPE
    


#######################
# FILE: data_structures/compound_pixel_grid.py
#######################

import numpy as np


class CompoundExpandablePixelGrid:
    def __init__(self, initial_shape, pixel_per_m, robot_radius_m):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2
        self.resolution = pixel_per_m # resolution of the grid with regards to the coordinate system of the gps / the world

        self.arrays = {
            "detected_points": np.zeros(self.array_shape, np.uint8), # Number of points detected in position
            "walls": np.zeros(self.array_shape, np.bool_),
            "occupied": np.zeros(self.array_shape, np.bool_), # Confirmed occupied point
            "traversable": np.zeros(self.array_shape, np.bool_), # Is or not traversable by the robot, assuming that the robot center is there. True means not traversable.
            "navigation_preference": np.zeros(self.array_shape, np.float32), # The preference for navigation for each pixel. More means less preferred to navigate through.
            "traversed": np.zeros(self.array_shape, np.bool_), # Robot has already gone through there
            "seen_by_camera": np.zeros(self.array_shape, np.bool_), # Has been seen by any of the cameras
            "seen_by_lidar": np.zeros(self.array_shape, np.bool_), # Has been seen by the lidar (Though not necessarily detected as occupied)
            "walls_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "walls_not_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "discovered": np.zeros(self.array_shape, np.bool_),
            "floor_color": np.zeros((self.array_shape[0], self.array_shape[1], 3), np.uint8),
            "floor_color_detection_distance": np.zeros(self.array_shape, np.uint8),
            "average_floor_color": np.zeros((self.array_shape[0], self.array_shape[1], 3), np.uint8),
            "holes": np.zeros(self.array_shape, np.bool_),
            "victims": np.zeros(self.array_shape, np.bool_),
            "victim_angles": np.zeros(self.array_shape, np.float32),
        }

    @property
    def grid_index_max(self):
        return self.array_shape - self.offsets # Maximum grid index
    
    @property
    def grid_index_min(self):
        return self.offsets * -1 # Minimum grid index

    # Index conversion
    def coordinates_to_grid_index(self, coordinates: np.ndarray) -> np.ndarray:
        coords = (coordinates * self.resolution).astype(int)
        return np.flip(coords)

    def grid_index_to_coordinates(self, grid_index: np.ndarray) -> np.ndarray:
        index = (grid_index.astype(float) / self.resolution)
        return np.flip(index)

    def array_index_to_grid_index(self, array_index: np.ndarray) -> np.ndarray:
        return array_index - self.offsets
    
    def grid_index_to_array_index(self, grid_index: np.ndarray) -> np.ndarray:
        return grid_index + self.offsets
    
    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)
    
    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # Grid expansion
    def expand_to_grid_index(self, grid_index: np.ndarray):
        """
        Expands all arrays to the specified index. 
        Note that all array_idexes should be recalculated after this operation.
        """

        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)

        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)

        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)
    
    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        
        for key in self.arrays:
            self.arrays[key] = self.__add_end_row_to_grid(self.arrays[key], size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_row_to_grid(self.arrays[key], size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_end_column_to_grid(self.arrays[key], size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_column_to_grid(self.arrays[key], size)

    def __add_end_row_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = size
        shape[1] = self.array_shape[1]
        grid = np.vstack((grid, np.zeros(shape, dtype=grid.dtype)))
        return grid
    
    def __add_begining_row_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = size
        shape[1] = self.array_shape[1]
        grid = np.vstack((np.zeros(shape, dtype=grid.dtype), grid))
        return grid
    
    def __add_end_column_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = self.array_shape[0]
        shape[1] = size
        grid = np.hstack((grid, np.zeros(shape, dtype=grid.dtype)))
        return grid

    def __add_begining_column_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = self.array_shape[0]
        shape[1] = size
        grid = np.hstack((np.zeros(shape, dtype=grid.dtype), grid))
        return grid

    # Debug
    def get_colored_grid(self):
        """
        Get graphical representation of the grid for debug.
        """
        #color_grid = np.zeros((self.array_shape[0], self.array_shape[1], 3), dtype=np.float32)
        color_grid = self.arrays["floor_color"].astype(np.float32) / 255
        #color_grid[self.arrays["traversed"]] = (.5, 0., .5)
        #color_grid[:, :, 1] = self.arrays["navigation_preference"][:, :] / 200
        color_grid[self.arrays["traversable"]] = (1, 0, 0)
        
        #color_grid[self.arrays["discovered"]] = (0, 1, 1)
        #color_grid[self.arrays["seen_by_lidar"]] += (0.5, 0, 0)

        color_grid[self.arrays["occupied"]] = (1, 1, 1)

        color_grid *= 0.5

        color_grid[self.arrays["victims"]] = (0, 1, 0)

        return color_grid


#######################
# FILE: data_structures/tile_color_grid.py
#######################

import numpy as np


class TileColorExpandableGrid:
    def __init__(self, initial_shape, tile_size):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets # Maximum grid index
        self.grid_index_min = self.offsets * -1 # Minimum grid index

        self.array = np.zeros(self.array_shape, np.bool_)

        self.resolution = 1 / tile_size # resolution of the grid with regards to the coordinate system of the gps / the world
    
    # Index conversion
    def coordinates_to_grid_index(self, coordinates: np.ndarray) -> np.ndarray:
        coords = (coordinates * self.resolution).astype(int)
        return np.flip(coords)

    def grid_index_to_coordinates(self, grid_index: np.ndarray) -> np.ndarray:
        index = (grid_index.astype(float) / self.resolution)
        return np.flip(index)

    def array_index_to_grid_index(self, array_index: np.ndarray) -> np.ndarray:
        return array_index - self.offsets
    
    def grid_index_to_array_index(self, grid_index: np.ndarray) -> np.ndarray:
        return grid_index + self.offsets
    
    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)
    
    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # Grid expansion
    def expand_to_grid_index(self, grid_index: np.ndarray):
        """
        Expands all arrays to the specified index. 
        Note that all array_idexes should be recalculated after this operation.
        """

        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)

        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)

        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)
    
    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        
        self.array = self.__add_end_row_to_array(self.array, size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        self.array = self.__add_begining_row_to_array(self.array, size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        self.array = self.__add_end_column_to_array(self.array, size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        self.array = self.__add_begining_column_to_array(self.array, size)

    def __add_end_row_to_array(self, array, size):
        array = np.vstack((array, np.zeros((size, self.array_shape[1]), dtype=array.dtype)))
        return array
    
    def __add_begining_row_to_array(self, array, size):
        array = np.vstack((np.zeros((size, self.array_shape[1]), dtype=array.dtype), array))
        return array
    
    def __add_end_column_to_array(self, array, size):
        array = np.hstack((array, np.zeros((self.array_shape[0], size), dtype=array.dtype)))
        return array

    def __add_begining_column_to_array(self, array, size):
        array = np.hstack((np.zeros((self.array_shape[0], size), dtype=array.dtype), array))
        return array
    
    # Debug
    def get_colored_grid(self):
       pass
    


#######################
# FILE: mapping/wall_mapper.py
#######################

import numpy as np
import cv2 as cv


class WallMapper:
    def __init__(self, compound_grid: CompoundExpandablePixelGrid, robot_diameter: float) -> None:
        self.grid = compound_grid

        self.robot_diameter = int(robot_diameter * self.grid.resolution)
        self.robot_radius = int(robot_diameter / 2 * self.grid.resolution)

        self.to_boolean_threshold = 3
        self.delete_threshold = 1
        
        # Circle with the radius of the robot
        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template, (self.robot_radius, self.robot_radius), self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)

        # A template to calculated the preference of each pixel for navigation taking into account the distance from the wall
        self.preference_template = self.__generate_quadratic_circle_gradient(self.robot_radius, self.robot_radius * 2)


    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        Loads into the corresponding arrays what has been seen by the lidar, what the lidar has detected, and
        what walls the lidar has detected but the camera hasn't seen.
        Calculates the travesable areas and the preference of each point for navigation.
        """
        
        
        robot_position_as_array = np.array(robot_position, dtype=float)
        
        self.__reset_seen_by_lidar()

        self.load_in_bounds_point_cloud(in_bounds_point_cloud, robot_position_as_array)
        self.load_out_of_bounds_point_cloud(out_of_bounds_point_cloud, robot_position_as_array)


    def load_in_bounds_point_cloud(self, point_cloud, robot_position):
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)

            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.occupy_point(point_array_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)
            
        self.filter_out_noise()

        self.generate_navigation_margins()

    def load_out_of_bounds_point_cloud(self, point_cloud, robot_position):
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)

        self.calculate_seen_walls()

    def calculate_seen_walls(self):
        self.grid.arrays["walls_seen_by_camera"] = self.grid.arrays["seen_by_camera"] * self.grid.arrays["walls"]
        self.grid.arrays["walls_not_seen_by_camera"] =  np.logical_xor(self.grid.arrays["walls"], self.grid.arrays["walls_seen_by_camera"])

    def generate_navigation_margins(self):
        # Areas traversable by the robot
        occupied_as_int = self.grid.arrays["occupied"].astype(np.uint8)
        diameter_template_as_int = self.robot_diameter_template.astype(np.uint8)

        self.grid.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, diameter_template_as_int)
        self.grid.arrays["traversable"] = self.grid.arrays["traversable"].astype(np.bool_)

        # Areas that the robot prefers to navigate through
        self.grid.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)

    def filter_out_noise(self):
        """
        Filters out noise from the 'detected_points' array.
        """
        self.grid.arrays["detected_points"] = self.grid.arrays["detected_points"] * (self.grid.arrays["detected_points"] > self.delete_threshold)


    # Initialization methods
    def __generate_quadratic_circle_gradient(self, min_radius, max_radius):
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            template = cv.circle(template, (max_radius, max_radius), i, max_radius ** 2 - i ** 2, -1)
        
        return template * 0.1
    
    def __generate_linear_circle_gradient(self, min_radius, max_radius):
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            print("i:", i)
            template = cv.circle(template, (max_radius, max_radius), i, max_radius - i, -1)
        
        return template * 0.5
    
    def occupy_point(self, point_array_index):        
        if not self.grid.arrays["walls"][point_array_index[0], point_array_index[1]]:
            self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] += 1
            
            if self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] > self.to_boolean_threshold:
                if not self.grid.arrays["traversed"][point_array_index[0], point_array_index[1]]:
                    self.grid.arrays["walls"][point_array_index[0], point_array_index[1]] = True
                    self.grid.arrays["occupied"][point_array_index[0], point_array_index[1]] = True
                    

    def mark_point_as_seen_by_lidar(self, robot_array_index, point_array_index):
        self.grid.arrays["seen_by_lidar"] = self.__draw_bool_line(self.grid.arrays["seen_by_lidar"], point_array_index, robot_array_index)
    
    def __draw_bool_line(self, array, point1, point2):
        array = cv.line(array.astype(np.uint8), (point1[1], point1[0]), (point2[1], point2[0]), 255, thickness=1, lineType=cv.LINE_8)
        return array.astype(np.bool_)
    
    def __reset_seen_by_lidar(self):
        self.grid.arrays["seen_by_lidar"] = np.zeros_like(self.grid.arrays["seen_by_lidar"])


#######################
# FILE: mapping/floor_mapper.py
#######################

import numpy as np
import cv2 as cv
import imutils

class ColorFilter:
    def __init__(self, lower_hsv, upper_hsv):
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)
    
    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask

class FloorMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, tile_resolution, tile_size, camera_distance_from_center) -> None:
        self.pixel_grid = pixel_grid
        self.tile_resolution = tile_resolution
        self.tile_size = tile_size
        self.pixel_per_m = tile_resolution / tile_size
        self.pov_distance_from_center = round(0.064 * self.pixel_per_m) 
        self.hole_color_filter = ColorFilter((0, 0, 10), (0, 0, 30))

        tiles_up = 0
        tiles_down = 1
        tiles_sides = 1

        min_x = self.tile_resolution * tiles_sides
        max_x = self.tile_resolution * (tiles_sides + 1)
        min_y = self.tile_resolution * tiles_down
        max_y = self.tile_resolution * (tiles_down + 1)

        self.center_tile_points_in_final_image = np.array(((min_x, min_y),
                                                           (max_x, min_y),
                                                           (max_x, max_y),
                                                           (min_x, max_y),), dtype=np.float32)
        
        self.center_tile_points_in_input_image = np.array(([0, 24],  [39, 24], [32, 16], [7, 16]), dtype=np.float32)

        self.flattened_image_shape = (self.tile_resolution * (tiles_sides * 2 + 1),
                                      self.tile_resolution * (tiles_up + tiles_down + 1))
        
        self.final_povs_shape = (120, 120)
        self.distance_to_center_gradient = self.__get_distance_to_center_gradient(self.final_povs_shape)

    def flatten_camera_pov(self, camera_pov: np.ndarray):
        ipm_matrix = cv.getPerspectiveTransform(self.center_tile_points_in_input_image, 
                                                self.center_tile_points_in_final_image, 
                                                solveMethod=cv.DECOMP_SVD)
        
        ipm = cv.warpPerspective(camera_pov, ipm_matrix, self.flattened_image_shape, flags=cv.INTER_NEAREST)

        ipm = cv.resize(ipm, self.flattened_image_shape, interpolation=cv.INTER_CUBIC)

        blank_space = np.zeros((self.pov_distance_from_center, self.flattened_image_shape[0], 4), dtype=np.uint8)
        ipm = np.vstack((blank_space, ipm))

        return ipm
    
    def set_in_background(self, pov: np.ndarray, background=None):
        #cv.imshow('pov', pov)
        max_dim = max(pov.shape)
        if background  is None: background = np.zeros((max_dim * 2, max_dim * 2, 4), dtype=np.uint8)

        start = (max_dim, max_dim - round(pov.shape[1] / 2))
        end =  (start[0] + pov.shape[0], start[1] + pov.shape[1])
        
        background[start[0]:end[0], start[1]:end[1], :] = pov[:,:,:]

        #cv.imshow("pov in background", background)

        return background
    

    def get_global_camera_orientations(self, robot_orientation: Angle):
        global_camera_orientations = []
        for camera_orientation in self.pixel_grid.camera_orientations:
            o = camera_orientation + robot_orientation
            o.normalize()
            global_camera_orientations.append(o)
        
        return global_camera_orientations
    
    def rotate_image_to_angle(self, image: np.ndarray, angle: Angle):
        return imutils.rotate(image, angle.degrees, (image.shape[0] // 2, image.shape[1] // 2))
    

    def get_unified_povs(self, camera_images: List[CameraImage]):
        povs_list = []
        for camera_image in camera_images:
            pov = self.flatten_camera_pov(np.rot90(copy(camera_image.image), k=3))
            pov = np.flip(pov, 1)
            pov = self.set_in_background(pov)
            pov = self.rotate_image_to_angle(pov, camera_image.data.horizontal_orientation)
            povs_list.append(pov)

        return sum(povs_list)
    
    def map_floor(self, camera_images, robot_grid_index):
        povs = self.get_unified_povs(camera_images)

        #cv.imshow("final_pov", povs[:, :, 3])

        self.load_povs_to_grid(robot_grid_index, povs)

    def load_povs_to_grid(self, robot_grid_index, povs):
        
        start = np.array((robot_grid_index[0] - (povs.shape[0] // 2), robot_grid_index[1] - (povs.shape[1] // 2)))
        end = np.array((robot_grid_index[0] + (povs.shape[0] // 2), robot_grid_index[1] + (povs.shape[1] // 2)))

        self.pixel_grid.expand_to_grid_index(start)
        self.pixel_grid.expand_to_grid_index(end)

        start = self.pixel_grid.grid_index_to_array_index(start)
        end = self.pixel_grid.grid_index_to_array_index(end)

        mask = povs[:,:,3] > 254

        gradient = self.__get_distance_to_center_gradient(povs.shape[:2])

        povs_gradient = np.zeros_like(self.distance_to_center_gradient)
        povs_gradient[mask] = self.distance_to_center_gradient[mask]

        #cv.imshow("gradient", povs_gradient)

        detection_distance_mask = self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]] < povs_gradient

        seen_by_camera_mask = self.pixel_grid.arrays["seen_by_camera"][start[0]:end[0], start[1]:end[1]]

        final_mask = seen_by_camera_mask * detection_distance_mask

        self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]][final_mask] = povs_gradient[final_mask]


        self.pixel_grid.arrays["floor_color"][start[0]:end[0], start[1]:end[1]][final_mask] = povs[:,:,:3][final_mask]

        self.detect_holes()

        #self.load_average_tile_color()
        
    
    def __get_distance_to_center_gradient(self, shape):
        gradient = np.zeros(shape, dtype=np.float32)
        for x in range(shape[0]):
            for y in range(shape[1]):
                gradient[x, y] = (x - shape[0] // 2) ** 2 + (y - shape[1] // 2) ** 2
        
        gradient = 1 - gradient / gradient.max()

        return (gradient * 255).astype(np.uint8)
    
    def __get_offsets(self, tile_size):
        x_offset = int(self.pixel_grid.offsets[0] % tile_size + tile_size / 2) 
        y_offset = int(self.pixel_grid.offsets[1] % tile_size + tile_size / 2)

        return (x_offset, y_offset)
    
    def offset_array(self, array, offsets):
        return array[offsets[0]:, offsets[1]:]
    
    def get_color_average_kernel(self):
        tile_size = round(self.tile_size * self.pixel_per_m)
        square_propotion = 0.8
        square_size = round(tile_size * square_propotion)

        kernel = np.ones((square_size, square_size), dtype=np.float32)

        kernel = kernel / kernel.sum()

        return kernel
    
    def detect_holes(self):
        tile_size = self.tile_size * self.pixel_per_m
        offsets = self.__get_offsets(tile_size)
        floor_color = deepcopy(self.pixel_grid.arrays["floor_color"])

        self.pixel_grid.arrays["holes"] = self.hole_color_filter.filter(self.pixel_grid.arrays["floor_color"])

        self.pixel_grid.arrays["occupied"] += self.pixel_grid.arrays["holes"].astype(np.bool_)

        """
        for x in range(round(offsets[0] + tile_size / 2), floor_color.shape[0], round(tile_size)):
            row = []
            for y in range(round(offsets[1] + tile_size / 2), floor_color.shape[1], round(tile_size)):
                row.append(floor_color[x, y, :])
            image.append(row)

        image = np.array(image, dtype=np.uint8)
        """

    def load_average_tile_color(self):
        tile_size = self.tile_size * self.pixel_per_m
        offsets = self.__get_offsets(tile_size)
        floor_color = deepcopy(self.pixel_grid.arrays["floor_color"])

        kernel = self.get_color_average_kernel()

        floor_color = cv.filter2D(floor_color, -1, kernel)
        #print("offsets", offsets)
        image = []

        for x in range(round(offsets[0] + tile_size / 2), floor_color.shape[0], round(tile_size)):
            row = []
            for y in range(round(offsets[1] + tile_size / 2), floor_color.shape[1], round(tile_size)):
                row.append(floor_color[x, y, :])
            image.append(row)

        image = np.array(image, dtype=np.uint8)

        
        image = cv.resize(image, (0, 0), fx=tile_size, fy=tile_size, interpolation=cv.INTER_NEAREST)

                    
        final_x = image.shape[0] if image.shape[0] + offsets[0] < self.pixel_grid.array_shape[0] else self.pixel_grid.array_shape[0] - offsets[0]
        final_y = image.shape[1] if image.shape[1] + offsets[1] < self.pixel_grid.array_shape[1] else self.pixel_grid.array_shape[1] - offsets[1]

        #self.pixel_grid.arrays["average_floor_color"] = np.zeros((final_x, final_y, 3), dtype=np.uint8)

        self.pixel_grid.arrays["average_floor_color"][offsets[0]:offsets[0] + final_x:, offsets[1]:offsets[1] + final_y, :] = image[:final_x,:final_y, :]
        


#######################
# FILE: mapping/robot_mapper.py
#######################

import numpy as np
import cv2 as cv
import math

class RobotMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, robot_diameter, pixels_per_m) -> None:
        self.pixel_grid = pixel_grid
        self.robot_radius = round(robot_diameter / 2 * pixels_per_m)
        
        # True indexes inside the circle
        self.__robot_diameter_indexes = self.__get_circle_template_indexes(self.robot_radius)


        self.__camera_pov_amplitude = Angle(30, Angle.DEGREES) # Horizontal amplitued of the fostrum of each camera
        self.__camera_pov_lenght = int(0.12 * 2 * pixels_per_m) # Range of each camera
        self.__camera_orientations = (Angle(0, Angle.DEGREES), Angle(270, Angle.DEGREES), Angle(90, Angle.DEGREES)) # Orientation of the cameras
        
        self.__discovery_pov_amplitude =  Angle(170, Angle.DEGREES)
        self.__discovery_pov_lenght = self.__camera_pov_lenght
        self.__discovery_pov_orientation = Angle(0, Angle.DEGREES)

    def map_traversed_by_robot(self, robot_grid_index):
        circle = np.zeros_like(self.__robot_diameter_indexes)
        circle[0] = self.__robot_diameter_indexes[0] + np.array(robot_grid_index)[0]
        circle[1] = self.__robot_diameter_indexes[1] + np.array(robot_grid_index)[1]

        self.pixel_grid.expand_to_grid_index((np.max(circle[0]), np.max(circle[1])))
        self.pixel_grid.expand_to_grid_index((np.min(circle[0]), np.min(circle[1])))

        robot_array_index =  self.pixel_grid.grid_index_to_array_index(robot_grid_index)[:]

        circle[0] = self.__robot_diameter_indexes[0] + robot_array_index[0]
        circle[1] = self.__robot_diameter_indexes[1] + robot_array_index[1]

        self.pixel_grid.arrays["traversed"][circle[0], circle[1]] = True

    def map_seen_by_camera(self, robot_grid_index, robot_rotation: Angle):
        global_camera_orientations = []

        for o in self.__camera_orientations:
            o1 = o + robot_rotation
            o1.normalize()
            global_camera_orientations.append(o1)

        camera_povs = self.__get_camera_povs_template_indexes(global_camera_orientations, robot_grid_index)

        self.pixel_grid.expand_to_grid_index(np.array((np.max(camera_povs[0]), np.max(camera_povs[1]))))
        self.pixel_grid.expand_to_grid_index(np.array((np.min(camera_povs[0]), np.min(camera_povs[1]))))


        camera_povs[0] += self.pixel_grid.offsets[0]
        camera_povs[1] += self.pixel_grid.offsets[1]

        self.pixel_grid.arrays["seen_by_camera"][camera_povs[0], camera_povs[1]] += self.pixel_grid.arrays["seen_by_lidar"][camera_povs[0], camera_povs[1]]

    def map_discovered_by_robot(self, robot_grid_index, robot_rotation: Angle):
        global_discovered_orientation = self.__discovery_pov_orientation + robot_rotation
        global_discovered_orientation.normalize()
        
        discovered_template = self.__get_cone_template(self.__discovery_pov_lenght, 
                                                       global_discovered_orientation, 
                                                       self.__discovery_pov_amplitude)
        
        disc_povs = self.__get_indexes_from_template(discovered_template, robot_grid_index - np.array((self.__discovery_pov_lenght, self.__discovery_pov_lenght)))
        
        self.pixel_grid.expand_to_grid_index(np.array((np.max(disc_povs[0]), np.max(disc_povs[1]))))
        self.pixel_grid.expand_to_grid_index(np.array((np.min(disc_povs[0]), np.min(disc_povs[1]))))
        
        disc_povs[0] += self.pixel_grid.offsets[0]
        disc_povs[1] += self.pixel_grid.offsets[1]

        self.pixel_grid.arrays["discovered"][disc_povs[0], disc_povs[1]] += self.pixel_grid.arrays["seen_by_lidar"][disc_povs[0], disc_povs[1]]

    def __get_cone_template(self, lenght, orientation: Angle, amplitude: Angle):
        matrix_size = math.ceil(lenght) * 2
        int_lenght = math.ceil(lenght)

        matrix = np.zeros((matrix_size + 1, matrix_size + 1), np.uint8)

        circle_matrix = cv.circle(np.zeros_like(matrix), (int_lenght,  int_lenght), int_lenght, 1, -1)
        
        center_position = Position2D(int_lenght, int_lenght)
        
        start_angle = orientation - (amplitude / 2)
        start_angle.normalize()
        start_vector = Vector2D(start_angle, lenght * 2)
        start_position = start_vector.to_position()
        start_position += center_position
        start_position = (math.ceil(start_position.x), math.ceil(start_position.y))

        center_angle = orientation
        center_angle.normalize()
        center_vector = Vector2D(center_angle, lenght * 2)
        center_up_position = center_vector.to_position()
        center_up_position += center_position
        center_up_position = center_up_position.astype(int)

        end_angle = orientation + (amplitude / 2)
        end_angle.normalize()
        end_vector = Vector2D(end_angle, lenght * 2)
        end_position = end_vector.to_position()
        end_position += center_position
        end_position = (math.ceil(end_position.x), math.ceil(end_position.y))

        triangle_matrix = cv.fillPoly(np.zeros_like(matrix), 
                                      [np.array([start_position, center_up_position, end_position, np.array(center_position)])],
                                      1)
        
        final_matrix = triangle_matrix * circle_matrix

        #cv.imshow("cone template", final_matrix * 100)

        return final_matrix
    
    def __get_camera_povs_template_indexes(self,  camera_orientations, robot_index):
        final_template = None
        for orientation in camera_orientations:
            cone_template = self.__get_cone_template(self.__camera_pov_lenght, orientation, self.__camera_pov_amplitude)
            if final_template is None:
                final_template = cone_template
            else:
                final_template += cone_template

        povs_indexes = self.__get_indexes_from_template(final_template, (-self.__camera_pov_lenght + robot_index[0], -self.__camera_pov_lenght + robot_index[1]))

        return povs_indexes

    # Camera fostrum template generation
    def __get_circle_template_indexes(self, radius):
        diameter = int(radius * 2 + 1)

        diameter_template = np.zeros((diameter, diameter), dtype=np.uint8)
        diameter_template = cv.circle(diameter_template, (radius, radius), radius, 255, -1)
        diameter_template = diameter_template.astype(np.bool_)

        return self.__get_indexes_from_template(diameter_template, (-radius, -radius))

    def __get_indexes_from_template(self, template: np.ndarray, offsets=(0, 0)):
        indexes = []
        indexes = template.nonzero()
        indexes = np.array(indexes)
        offsets = np.array(offsets)
        indexes[0] += offsets[0]
        indexes[1] += offsets[1]
        return indexes


#######################
# FILE: mapping/data_extractor.py
#######################

import numpy as np
import cv2 as cv
import copy

import utilities


class FloorColorExtractor:
    def __init__(self, tile_resolution) -> None:
        self.tile_resolution = tile_resolution
        self.floor_color_ranges = {
                    "normal":
                        {   
                            "range":   ((0, 0, 37), (0, 0, 192)), 
                            "threshold":0.2},

                    "nothing":
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},
                    
                    "checkpoint":
                        {
                            "range":((95, 0, 65), (128, 122, 198)),
                            "threshold":0.2},
                    "hole":
                        {
                            "range":((0, 0, 10), (0, 0, 30)),
                            "threshold":0.2},
                    
                    "swamp":
                        {
                            "range":((19, 112, 32), (19, 141, 166)),
                            "threshold":0.2},

                    "connection1-2":
                        {
                            "range":((120, 182, 49), (120, 204, 232)),
                            "threshold":0.2},

                    "connection1-3":
                        {
                            "range":((132, 156, 36), (133, 192, 185)),
                            "threshold":0.2},

                    "connection2-3":
                        {
                            "range":((0, 182, 49), (0, 204, 232)),
                            "threshold":0.2},
                    }
        self.final_image = np.zeros((700, 700, 3), np.uint8)
        
    def get_square_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        square = cv.cvtColor(square, cv.COLOR_BGR2HSV)
        if np.count_nonzero(square) == 0:
            return "nothing"
        color_counts = {}
        for color_key, color_range in self.floor_color_ranges.items():
            colour_count = np.count_nonzero(cv.inRange(square, color_range["range"][0], color_range["range"][1]))
            if colour_count > color_range["threshold"] * square.shape[0] * square.shape[1]:
                color_counts[color_key] = colour_count
        
        if len(color_counts) == 0:
            return "nothing"
        else:
            return max(color_counts, key=color_counts.get)
    
    def get_sq_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        # remove pixels with value 0, 0, 0
        white_count = np.count_nonzero(cv.inRange(square, (180, 180, 180), (255, 255, 255)))
        black_count = np.count_nonzero(cv.inRange(square, (20, 20, 20), (180, 180, 180)))

        if white_count > black_count and white_count > square.shape[0] * square.shape[1] / 8:
            return (255, 255, 255)
        else:
            return (100, 100, 100)

    def get_floor_colors(self, floor_image, robot_position):

        grid_offsets = [(((p + 0) % 0.06) / 0.06) * 50 for p in robot_position]
        
        grid_offsets = [int(o) for o in grid_offsets]

        offsets = [((((p + 0.03) % 0.06) - 0.03) / 0.06) * 50 for p in robot_position]
        
        offsets = [int(o) for o in offsets]

        
        utilities.save_image(floor_image, "floor_image.png")

        squares_grid = utilities.get_squares(floor_image, self.tile_resolution, offsets)

        color_tiles = []
        for row in squares_grid:
            for square in row:
                color_key = self.get_square_color(floor_image, square)
                if color_key == "normal":
                    color = (255, 255, 255)
                elif color_key == "checkpoint":
                    color = (100, 100, 100)
                else:
                    color = (0, 0, 0)
                #if color != (0, 0, 0):
                #cv.rectangle(self.final_image, [square[2], square[0]], [square[3], square[1]], color, -1)

                tile = [square[2], square[0]]
                tile = utilities.substractLists(tile, (350 - offsets[0], 350 - offsets[1]))
                tile = utilities.divideLists(tile, [self.tile_resolution, self.tile_resolution])
                tile = [int(t) for t in tile]
                if color_key != "nothing":
                    if SHOW_DEBUG:
                        print(tile, color_key)
                    color_tiles.append((tile, color_key))

        if SHOW_DEBUG:
            drawing_image = floor_image.copy() #self.final_image.copy()
            utilities.draw_grid(drawing_image, self.tile_resolution, offset=grid_offsets)
            cv.circle(drawing_image, (350 - offsets[0], 350 - offsets[1]), 10, (255, 0, 0), -1)
            cv.imshow("final_floor_image", utilities.resize_image_to_fixed_size(drawing_image, (600, 600)))        
        return color_tiles


        
        

class PointCloudExtarctor:
    def __init__(self, resolution):
        self.threshold = 8
        self.resolution = resolution
        self.straight_template = np.zeros((self.resolution + 1, self.resolution + 1), dtype=int)
        self.straight_template[:][0:2] = 1
        #self.straight_template[3:-3][0:2] = 2
        self.straight_template[0][0:2] = 0
        self.straight_template[-1][0:2] = 0

        straight = [
            [0, 1, 2, 2, 2, 1, 0],
            [0, 1, 2, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.straight_template = np.array(straight)

        curved = [
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 0],
            [0, 0, 3, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.curved_template = np.array(curved)


        self.templates = {}

        for i, name in enumerate([("u",), ("l",), ("d",), ("r",)]):
            self.templates[name] = np.rot90(self.straight_template, i)
        
        for i, name in enumerate([("u", "l"), ("d", "l"), ("d", "r"),  ("u", "r")]):
            self.templates[name] = np.rot90(self.curved_template, i)

    def get_tile_status(self, min_x, min_y, max_x, max_y, point_cloud):
        counts = {name: 0 for name in self.templates}
        square = point_cloud[min_x:max_x+1, min_y:max_y+1]
        if square.shape != (self.resolution+1, self.resolution+1):
            return []

        non_zero_indices = np.where(square != 0)
        for name, template in self.templates.items():
            counts[name] = np.sum(template[non_zero_indices])

        names = [name for name, count in counts.items() if count >= self.threshold]

        return [i for sub in names for i in sub]

    def transform_to_grid(self, point_cloud):
        offsets = point_cloud.offsets
        offsets = [o % self.resolution for o in offsets]
        offsets.reverse()
        grid = []
        bool_array_copy = point_cloud.get_bool_array()
        if SHOW_DEBUG:
            bool_array_copy = bool_array_copy.astype(np.uint8) * 100
        for x in range(offsets[0], bool_array_copy.shape[0] - self.resolution, self.resolution):
            row = []
            for y in range(offsets[1], bool_array_copy.shape[1] - self.resolution, self.resolution):
                min_x = x
                min_y = y
                max_x = x + self.resolution
                max_y = y + self.resolution
                #print(min_x, min_y, max_x, max_y)

                if SHOW_DEBUG:
                    bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
                
                val = self.get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())
                
                row.append(list(val))
            grid.append(row)
        factor = 10

        if SHOW_DEBUG:
            cv.imshow("point_cloud_with_squares", utilities.resize_image_to_fixed_size(bool_array_copy, (600, 600)))
        offsets = point_cloud.offsets
        return grid, [o // self.resolution for o in offsets]


#######################
# FILE: fixture_detection/color_filter.py
#######################

import cv2 as cv
import numpy as np

class ColorFilter:
    def __init__(self, lower_hsv, upper_hsv):
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)
    
    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask


#######################
# FILE: fixture_detection/fixture_detection.py
#######################

import skimage

import copy

import math

import numpy as np
import cv2 as cv


class FixtureDetector:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid) -> None:
        self.pixel_grid = pixel_grid

        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 0)),
            "white": ColorFilter(lower_hsv=(0, 0, 207), upper_hsv=(0, 0, 207)),
            "yellow": ColorFilter(lower_hsv=(25, 157, 82), upper_hsv=(30, 255, 255)),
            "red": ColorFilter(lower_hsv=(160, 170, 127), upper_hsv=(170, 255, 255))
        }

        self.max_detection_distance = 0.12 * 5

    def get_fixture_positions_and_angles(self, robot_position: Position2D, camera_image: CameraImage) -> list:
        positions_in_image = self.get_fixture_positions_in_image(np.flip(camera_image.image, axis=1))

        #debug = self.pixel_grid.get_colored_grid()

        fixture_positions = []
        fixture_angles = []
        for position in positions_in_image:
            relative_horizontal_angle = Angle(position[1] * (camera_image.data.horizontal_fov.radians / camera_image.data.width))

            fixture_horizontal_angle = (relative_horizontal_angle - camera_image.data.horizontal_fov / 2) + camera_image.data.horizontal_orientation 

            fixture_horizontal_angle.normalize()

            camera_vector = Vector2D(camera_image.data.horizontal_orientation, camera_image.data.distance_from_center)
            camera_pos = camera_vector.to_position()
            camera_pos += robot_position

            detection_vector = Vector2D(fixture_horizontal_angle, self.max_detection_distance)
            detection_pos = detection_vector.to_position()

            detection_pos += camera_pos

            camera_array_index = self.pixel_grid.coordinates_to_array_index(camera_pos)
            detection_array_index = self.pixel_grid.coordinates_to_array_index(detection_pos)

            line_xx, line_yy = skimage.draw.line(camera_array_index[0], camera_array_index[1], detection_array_index[0], detection_array_index[1])

            for x, y in zip(line_xx, line_yy):
                if x >= 0 and y >= 0 and x < self.pixel_grid.array_shape[0] and y < self.pixel_grid.array_shape[1]:
                    #debug[x, y] = (0, 255, 0)
                    if self.pixel_grid.arrays["walls"][x, y]:
                        fixture_positions.append(self.pixel_grid.array_index_to_coordinates(np.array([x, y])))
                        fixture_angles.append(copy.deepcopy(fixture_horizontal_angle))
                        break

        #cv.imshow("fixture_detection_debug", debug)

        return fixture_positions, fixture_angles
    
    def get_fixture_positions_in_image(self, image: np.ndarray) -> List[Position2D]:
        image_sum = np.zeros(image.shape[:2], dtype=np.bool_)
        for filter in self.color_filters.values():
            image_sum += filter.filter(image) > 0

        image_sum = image_sum.astype(np.uint8) * 255

        cv.imshow("fixtures", image_sum)
        
        contours, _ = cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


        debug = copy.deepcopy(image)
        
        final_victims = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append(Position2D((x + x + w) / 2, (y + y + h) / 2))

        for f in final_victims:
            debug = cv.circle(debug, np.array(f, dtype=int), 3, (255, 0, 0), -1)
        
        cv.imshow("victim_pos_debug", debug)

        return final_victims



#######################
# FILE: mapping/mapper.py
#######################


import numpy as np
import cv2 as cv







class Mapper:
    def __init__(self, tile_size, robot_diameter, camera_distance_from_center):
        self.tile_size = tile_size
        self.quarter_tile_size = tile_size / 2
        self.robot_diameter = robot_diameter

        self.robot_position = None
        self.robot_orientation = None
        self.start_position = None

        self.robot_grid_index = None

        # Data structures
        pixels_per_tile = 10
        self.pixel_grid = CompoundExpandablePixelGrid(initial_shape=np.array([1, 1]), 
                                                      pixel_per_m=pixels_per_tile / self.quarter_tile_size, 
                                                      robot_radius_m=(self.robot_diameter / 2) -0.008)
        
        self.tile_color_grid = TileColorExpandableGrid(initial_shape=np.array((1, 1)),
                                                       tile_size=self.tile_size)

        #Data processors
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)
        self.floor_mapper = FloorMapper(pixel_grid=self.pixel_grid, 
                                        tile_resolution=pixels_per_tile * 2,
                                        tile_size=self.tile_size,
                                        camera_distance_from_center=camera_distance_from_center)
        
        self.robot_mapper = RobotMapper(pixel_grid=self.pixel_grid,
                                        robot_diameter=self.robot_diameter,
                                        pixels_per_m=pixels_per_tile / self.quarter_tile_size)
        

        # Data extractors
        self.point_cloud_extractor = PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = FloorColorExtractor(tile_resolution=50)

        self.fixture_detector = FixtureDetector(self.pixel_grid)

    def update(self, in_bounds_point_cloud: list = None, 
               out_of_bounds_point_cloud: list = None,
               lidar_detections: list = None,
               camera_images: list = None, 
               robot_position: Position2D = None, 
               robot_orientation: Angle = None):
        
        if robot_position is None or robot_orientation is None:
            return
        
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation

        self.robot_grid_index = self.pixel_grid.coordinates_to_grid_index(self.robot_position)

        # Load walls and obstacles (Lidar detections)
        if in_bounds_point_cloud is not None and out_of_bounds_point_cloud is not None:
            self.wall_mapper.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)
        
        self.robot_mapper.map_traversed_by_robot(self.robot_grid_index)
        self.robot_mapper.map_seen_by_camera(self.robot_grid_index, self.robot_orientation)
        self.robot_mapper.map_discovered_by_robot(self.robot_grid_index, self.robot_orientation)

        # Load floor colors
        if camera_images is not None:
            self.floor_mapper.map_floor(camera_images, self.pixel_grid.coordinates_to_grid_index(self.robot_position))


        
        if camera_images is not None and lidar_detections is not None:
            #debug_grid = self.pixel_grid.get_colored_grid()
            for i in camera_images:
                positions, angles = self.fixture_detector.get_fixture_positions_and_angles(self.robot_position, i)
                for pos, angle in zip(positions, angles):
                    index = self.pixel_grid.coordinates_to_array_index(pos)
                    self.pixel_grid.arrays["victims"][index[0], index[1]] = True
                    self.pixel_grid.arrays["victim_angles"][index[0], index[1]] = angle.radians
                    #debug_grid = cv.circle(debug_grid, (index[1], index[0]), 3, (0, 255, 0), -1)

            #robot_array_index = self.pixel_grid.coordinates_to_array_index(self.robot_position)
            #debug_grid = cv.circle(debug_grid, (robot_array_index[1], robot_array_index[0]), 5, (255, 0, 255), -1)

            #cv.imshow("fixture_debug_grid", debug_grid)
        
        """
        if lidar_detections is not None:
            debug_grid = self.pixel_grid.get_colored_grid()
            for l in lidar_detections:
                l.direction.normalize()
                pos = l.to_position()
                pos += self.robot_position
                index = self.pixel_grid.coordinates_to_array_index(pos)
                debug_grid = cv.circle(debug_grid, (index[1], index[0]), 1, (0, 255, 0), -1)

            cv.imshow("lidar_debug_grid", debug_grid)
        """

        #DEBUG
        if DO_WAIT_KEY:
            cv.waitKey(1)

    
    def register_start(self, robot_position):
        self.start_position = deepcopy(robot_position)
        print("registered start position:", self.start_position)

    
    # Grids
    def get_grid_for_bonus(self):
        """
        final_grid = []
        for row in self.get_node_grid().grid:
            final_row = []
            for node in row:
                final_row.append(node.get_representation())
            final_grid.append(final_row)
        return np.array(final_grid)
        """
        pass # TODO
    

    def __lidar_to_node_grid(self):
        """
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
                #print(value)
                for direction in value:
                    self.node_grid.load_straight_wall((xx, yy),  direction)
        """


#######################
# FILE: agents/agent.py
#######################

import random

class Agent(ABC):
    def __init__(self, mapper) -> None:
        self.mapper = mapper

    @abstractmethod
    def update(self) -> None:
        pass
    
    @abstractmethod
    def get_target_position(self) -> Position2D:
        pass

    @abstractmethod
    def do_end(self) -> bool:
        pass

    @abstractmethod
    def do_report_victim(self) -> bool:
        pass


#######################
# FILE: algorithms/np_bool_array/efficient_a_star.py
#######################

import numpy as np
import cv2 as cv
import math


class aStarNode:
    def __init__(self, location):
        self.location = location
        self.parent = None
        self.g = float('inf')
        self.p = 0
        self.f = 0

    def __gt__(self, other):  # make nodes comparable
        return self.f > other.f

    def __repr__(self):
        return str(self.location)


class aStarAlgorithm:
    def __init__(self):
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]#[1, 1], [1, -1], [-1, -1], [-1, 1]]
        self.preference_weight = 5
    
    @staticmethod
    def reconstructpath(node):
        path = []
        while node is not None:
            path.append(node.location)
            node = node.parent
        path.reverse()
        return path

    @staticmethod
    def heuristic(start, target):
        # optimistic score, assuming all cells are friendly
        dy = abs(start[0] - target[0])
        dx = abs(start[1] - target[1])
        return min(dx, dy) * 15 + abs(dx - dy) * 10

    @staticmethod
    def get_preference(preference_grid, position):
        if preference_grid is None:
            return 0
        elif not (position[0] >= preference_grid.shape[0] or position[1] >= preference_grid.shape[1] or position[0] < 0 or position[1] < 0):
            return preference_grid[position[0], position[1]]
        else:
            return 0
    
    @staticmethod
    def is_traversable(grid, position):
        if not (position[0] >= grid.shape[0] or position[1] >= grid.shape[1] or position[0] < 0 or position[1] < 0):
            return not grid[position[0], position[1]]
        else:
            return True


        
    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def a_star(self, grid: np.ndarray, start, end, preference_grid=None, search_limit=float('inf')):
        debug_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # Create start and end node
        start_node = aStarNode(tuple(start))
        start_node.g = 0
        
        if not self.is_traversable(grid, start):
            print("WARNING: Start position is not traversable")

        end_node = aStarNode(tuple(end))

        if not self.is_traversable(grid, end):
            print("WARNING: End position is not traversable")
            return []

        end_node.g = end_node.h = end_node.f = 0
        # Initialize open and closed list
        openList = [start_node]
        best_cost_for_node_lookup = {tuple(start_node.location): start_node.g}
        closed = set()

        loop_n = 0
        # Loop until end
        while openList:            
            # Get the current node
            node = heappop(openList)
            if node.location in closed:
                continue

            closed.add(node.location)
            # If found the goal
            if node.location == end_node.location:
                #print(f"Finished Astar. Took {loop_n} loops.")
                return self.reconstructpath(node)
            
            # Generate children
            for adj in self.adjacents:  # Adjacent squares
                # Get node position
                child_location = (node.location[0] + adj[0], node.location[1] + adj[1])
                # Make sure walkable terrain
                if not self.is_traversable(grid, child_location):
                    continue
                # Create new node
                new_child = aStarNode(child_location)
                new_child.parent = node

                new_child.g = node.g + 1
                new_child.h =  self.heuristic(new_child.location, end_node.location)
                
                new_child.p = self.get_preference(preference_grid, new_child.location) * self.preference_weight
                
                new_child.f = new_child.g + new_child.h + new_child.p

                if child_location in best_cost_for_node_lookup.keys():
                    if new_child.g + new_child.p < best_cost_for_node_lookup[child_location]:
                        best_cost_for_node_lookup[child_location] = new_child.g + new_child.p
                        heappush(openList, new_child)
                        
                else:
                    best_cost_for_node_lookup[child_location] = new_child.g + new_child.p
                    heappush(openList, new_child)

            loop_n += 1
            if loop_n > search_limit:
                break
            
            """
            for o in openList:
                debug_grid[o.location[0], o.location[1]] = [0, 0, 255]

            cv.imshow("debug", debug_grid)

            cv.waitKey(1)
            """
            
        return []


#######################
# FILE: algorithms/np_bool_array/bfs.py
#######################

import numpy as np

class BFSAlgorithm:
    def __init__(self, found_function) -> None:
        self.found_function = found_function
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield [node[0] + a[0], node[1] + a[1]]
    
    def bfs(self, array, start_node):
        open_list = []
        open_list.append(start_node)

        while len(open_list) > 0:
            node = open_list.pop(0)

            value = array[node[0], node[1]]

            if self.found_function(value):
                return node

            for n in self.get_neighbours(node):
                if not n in open_list:
                    open_list.append(n)


class NavigatingBFSAlgorithm:
    def __init__(self, found_function, traversable_function, max_result_number=1) -> None:
        self.found_function = found_function
        self.traversable_function = traversable_function
        self.adjacents = ((0, 1), (0, -1), (-1, 0), (1, 0))
        self.max_result_number = max_result_number

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield (node[0] + a[0], node[1] + a[1])
    
    def bfs(self, found_array, traversable_array, start_node):
        open_list = []
        open_list.append(tuple(start_node))

        closed_set = set()
        closed_set.add(tuple(start_node))

        results = []

        while len(open_list) > 0:
            node = open_list.pop(0)

            if traversable_array[node[0], node[1]]:
                continue

            value = found_array[node[0], node[1]]

            if not value:
                results.append(node)
                if len(results) >= self.max_result_number:
                    return results

            for n in self.get_neighbours(node):
                if n not in closed_set:
                    open_list.append(n)
                    closed_set.add(n)    
        
        return results




#######################
# FILE: algorithms/np_bool_array/a_star.py
#######################

import numpy as np
import cv2 as cv
import math

# aStarNode class for A* pathfinding 
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.p = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
        return str(self.position)

class aStarAlgorithm:
    def __init__(self):
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]#[1, 1], [1, -1], [-1, -1], [-1, 1]]
        self.preference_weight = 50

    def get_preference(self, preference_grid, position):
        if preference_grid is None:
            return 0
        elif not (position[0] >= preference_grid.shape[0] or position[1] >= preference_grid.shape[1] or position[0] < 0 or position[1] < 0):
            return preference_grid[position[0], position[1]]
        else:
            return 0
        
    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def a_star(self, grid: np.ndarray, start, end, preference_grid=None):
        debug_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # Create start and end node
        startNode = aStarNode(None, list(start))
        startNode.g = startNode.h = startNode.f = 0
        
        if grid[start[0], start[1]]:
            print("WARNING: Start position is not traversable")

        endNode = aStarNode(None, list(end))

        if grid[end[0], end[1]]:
            print("WARNING: End position is not traversable")
            return []

        endNode.g = endNode.h = endNode.f = 0
        # Initialize open and closed list
        openList = []
        closedList = []

        # Add the start node
        openList.append(startNode)

        
        # Loop until end
        while len(openList) > 0:            
            # Get the current node
            currentNode = openList[0]
            currentIndex = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIndex = index
            # Pop current off open list, add to closed list
            openList.pop(currentIndex)
            
            closedList.append(currentNode)
            # If found the goal
            if currentNode == endNode:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path
            
            # Generate children
            children = []
            for adj in self.adjacents:  # Adjacent squares
                # Get node position
                nodePosition = [currentNode.position[0] + adj[0], currentNode.position[1] + adj[1]]
                # Make sure walkable terrain
                if not (nodePosition[0] >= grid.shape[0] or nodePosition[1] >= grid.shape[1] or nodePosition[0] < 0 or nodePosition[1] < 0):
                    #print("OUT OF BOUNDS")
                    if grid[nodePosition[0], nodePosition[1]]:
                        #print("NOT TRAVERSABLE")
                        continue
                # Create new node
                newNode = aStarNode(currentNode, nodePosition)
                # Append
                children.append(newNode)
            
            # Loop through children
            for child in children:
                continueLoop = False
                # Child is on the closed list
                for closedChild in closedList:
                    if child == closedChild:
                        continueLoop = True
                        break
                # Create the f, g, and h values
                child.g = currentNode.g + 1
                child.h =  ((child.position[0] - endNode.position[0]) ** 2) + (
                           (child.position[1] - endNode.position[1]) ** 2)
                
                child.p = self.get_preference(preference_grid, child.position) * self.preference_weight
                
                child.f = child.g + child.h + child.p
                # Child is already in the open list
                for index, openNode in enumerate(openList):
                    if child == openNode:
                        if child.p + child.g > openNode.p + openNode.g:
                            continueLoop = True
                            break



                if continueLoop:
                    continue
                # Add the child to the open list
                openList.append(child)
            
            
            for o in openList:
                debug_grid[o.position[0], o.position[1]] = [0, 0, 255]

            cv.imshow("debug", debug_grid)

            cv.waitKey(1)
            
        return []


#######################
# FILE: agents/granular_navigation_agent/path_smoothing.py
#######################

class PathSmoother:
    def __init__(self, strenght) -> None:
        self.strenght = strenght

    def smooth(self, path):
        new_path = []
        for index, node in enumerate(path):
            prior = path[max(index - 1, 0)]
            next = path[min(index + 1, len(path) - 1)]

            avg_x = (node[0] + prior[0] * self.strenght + next[0] * self.strenght) / (1 + self.strenght * 2)
            avg_y = (node[1] + prior[1] * self.strenght + next[1] * self.strenght) / (1 + self.strenght * 2)

            new_path.append([avg_x, avg_y])
        
        return new_path


#######################
# FILE: agents/granular_navigation_agent/path_finder.py
#######################

import numpy as np
import cv2 as cv





class PathFinder():
    def __init__(self, mapper: Mapper):
        self.a_star = aStarAlgorithm()
        self.closest_free_point_finder = BFSAlgorithm(lambda x : x == 0)

        self.a_star_path_smoother = PathSmoother(1)

        self.robot_grid_index = np.array([0, 0])
        self.target_position = np.array([0, 0])

        self.a_star_path = []
        self.smooth_astar_path = []
        self.a_star_index = 0

        self.mapper = mapper

        self.path_not_found = False
    
    def update(self, target_position: np.ndarray = None) -> None:
        if target_position is not None:
            self.target_position = target_position

        self.robot_grid_index = self.mapper.pixel_grid.coordinates_to_grid_index(self.mapper.robot_position) # Get robot position grid index
        self.mapper.pixel_grid.expand_to_grid_index(self.robot_grid_index) # Expand grid to robot position

        if SHOW_PATHFINDING_DEBUG: 
            if self.is_path_finished(): print("FINISHED PATH")
            if self.is_path_obstructed(): print("PATH OBSTRUCTED")

        if self.is_path_finished() or self.is_path_obstructed():
            self.calculate_path()
            
        self.calculate_path_index()

        #DEBUG
        if SHOW_GRANULAR_NAVIGATION_GRID:
            debug_grid = self.mapper.pixel_grid.get_colored_grid()
            for node in self.a_star_path:
                n = np.array(self.mapper.pixel_grid.grid_index_to_array_index(node))
                try:
                    debug_grid[n[0], n[1]] = [0, 0, 255]
                except IndexError:
                    pass

            cv.imshow("granular_grid", debug_grid)
        


    def calculate_path(self):
        # Get start array index (if robot index occupied, get closest unoccupied point)
        start_array_index = self.mapper.pixel_grid.coordinates_to_array_index(self.mapper.robot_position)
        start_array_index = self.get_closest_traversable_array_index(start_array_index)

        # Expand grid to target index
        target_grid_index = self.mapper.pixel_grid.coordinates_to_grid_index(self.target_position)
        self.mapper.pixel_grid.expand_to_grid_index(target_grid_index)

        # Get target array index (if target index occupied, get closest unoccupied point)
        target_array_index = self.mapper.pixel_grid.coordinates_to_array_index(self.target_position)
        target_array_index = self.get_closest_traversable_array_index(target_array_index)

        # Calculate path
        best_path = self.a_star.a_star(self.mapper.pixel_grid.arrays["traversable"], 
                                        start_array_index,
                                        target_array_index,
                                        self.mapper.pixel_grid.arrays["navigation_preference"])

        # If path was successfully calculated, transform all indexes to grid indexes
        if len(best_path) > 1:
            self.a_star_path = []
            for array_index in best_path:
                self.a_star_path.append(self.mapper.pixel_grid.array_index_to_grid_index(array_index))

            self.a_star_path = self.a_star_path[1:]
            self.a_star_index = 0
            self.path_not_found = False
        else:
            if SHOW_PATHFINDING_DEBUG: print("PATH NOT FOUND")
            self.path_not_found = True
        
        self.a_star_path = self.dither_path(self.a_star_path) # Remove every second positon of the path
        self.smooth_astar_path = self.a_star_path_smoother.smooth(self.a_star_path) # Smooth the path

    def calculate_path_index(self):
        self.a_star_index = min(self.a_star_index, len(self.a_star_path) - 1)   
        if len(self.a_star_path) > 0:
            next_node = self.a_star_path[self.a_star_index]
            next_node = Position2D(next_node)

            current_grid_index = self.mapper.pixel_grid.coordinates_to_grid_index(self.mapper.robot_position)
            current_node = Position2D(current_grid_index[0], current_grid_index[1])

            if abs(current_node.get_distance_to(next_node)) < 3:
                self.a_star_index += 1

    def dither_path(self, path):
        final_path = []
        dither_interval = 2
        for index, value in enumerate(path):
            if index % dither_interval == 0:
                final_path.append(value)
        if len(final_path):
            return final_path
        else:
            return path
    
    def get_next_position(self) -> Position2D:
        self.a_star_index = min(self.a_star_index, len(self.a_star_path) -1)
        if len(self.smooth_astar_path):
            pos = self.mapper.pixel_grid.grid_index_to_coordinates(np.array(self.smooth_astar_path[self.a_star_index]))
            pos = Position2D(pos[0], pos[1])
            return pos
        
        else:
            return self.mapper.robot_position
    
    def is_path_obstructed(self):
        """
        Is current Astar path obstructed?
        """
        array_index_path = []
        for n in self.a_star_path:
            array_index_path.append(self.mapper.pixel_grid.grid_index_to_array_index(n))
            
        for position in array_index_path:
            if position[0] >= self.mapper.pixel_grid.arrays["traversable"].shape[0] or \
               position[1] >=  self.mapper.pixel_grid.arrays["traversable"].shape[1]:
                continue

            if position[0] < 0 or position[1] < 0:
                continue

            if self.mapper.pixel_grid.arrays["traversable"][position[0], position[1]]:
                return True
            
        return False
    
    def is_path_finished(self):
        return len(self.a_star_path) - 1 <= self.a_star_index
    

    def get_closest_traversable_array_index(self, array_index):
        if self.mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.closest_free_point_finder.bfs(array=self.mapper.pixel_grid.arrays["traversable"],
                                                       start_node=array_index)
        else:
            return array_index


#######################
# FILE: agents/granular_navigation_agent/best_position_finder.py
#######################


import numpy as np
import cv2 as cv
import math
import skimage

class BestPositionFinder:
    """
    Finds the best position for the robot to go to, with the objective of exploring the maze.
    """
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.closest_unseen_finder = NavigatingBFSAlgorithm(found_function=lambda x: x == False, 
                                                            traversable_function=lambda x: x == False,
                                                            max_result_number=1)
        
        self.closest_free_point_finder = BFSAlgorithm(lambda x : x == 0)
        
        self.closest_unseen_grid_index = None
        

    def calculate_best_position(self, finished_path):
        """
        Calculate closest unseen position only when the robot has reached the previous one or if the objective
        is no longer traversable.
        """
        if self.is_objective_untraversable() or finished_path:
            self.closest_unseen_grid_index = self.get_closest_unseen_grid_index()

        # DEBUG
        if SHOW_BEST_POSITION_FINDER_DEBUG:
            debug_grid = self.mapper.pixel_grid.get_colored_grid()  
            if self.closest_unseen_grid_index is not None:
                closest_unseen_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.closest_unseen_grid_index)
            else:
                closest_unseen_array_index = self.mapper.pixel_grid.coordinates_to_array_index(self.mapper.start_position)
            cv.circle(debug_grid, (closest_unseen_array_index[1], closest_unseen_array_index[0]), 4, (0, 255, 100), -1)
            cv.imshow("closest_position_finder_debug", debug_grid)

    def is_objective_untraversable(self):
        if self.closest_unseen_grid_index is None: return False

        closest_unseen_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.closest_unseen_grid_index)
        return self.mapper.pixel_grid.arrays["traversable"][closest_unseen_array_index[0], closest_unseen_array_index[1]]
    
    def get_closest_unseen_grid_index(self):
        robot_array_index = self.mapper.pixel_grid.coordinates_to_array_index(self.mapper.robot_position)

        start_node = self.get_closest_traversable_array_index(robot_array_index)

        closest_unseen_array_indexes = self.closest_unseen_finder.bfs(found_array=self.mapper.pixel_grid.arrays["discovered"],
                                                                      traversable_array=self.mapper.pixel_grid.arrays["traversable"],
                                                                      start_node=start_node)
        if len(closest_unseen_array_indexes):
            return self.mapper.pixel_grid.array_index_to_grid_index(closest_unseen_array_indexes[0])
        else:
            return None

    def get_best_position(self):
        if self.closest_unseen_grid_index is None:
            return self.mapper.start_position
        else:
            coords = self.mapper.pixel_grid.grid_index_to_coordinates(self.closest_unseen_grid_index)
            return Position2D(coords)
    

    def __get_line(self, point1, point2):
        xx, yy = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        indexes = [[x, y] for x, y in zip(xx, yy)]

    def __has_line_of_sight(self, point1, point2, matrix):
        xx, yy = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        for x, y in zip(xx[1:-1], yy[1:-1]):
            if matrix[x, y]:
                return False
        return True
    
    def __get_seen_circle(self, radius, center_point, matrix):
        xx, yy = skimage.draw.circle(center_point, radius)
        indexes = [[x, y] for x, y in zip(xx, yy)]

        farthest_points = deepcopy(indexes)

        for index, current_farthest_point in enumerate(indexes):
            for possible_farthest_point in self.get_line(current_farthest_point, center_point):
                if self.__has_line_of_sight(possible_farthest_point, center_point, matrix):
                    farthest_points[index] = possible_farthest_point
                    break
        
        return farthest_points
    


    def get_closest_traversable_array_index(self, array_index):
        if self.mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.closest_free_point_finder.bfs(array=self.mapper.pixel_grid.arrays["traversable"],
                                                       start_node=array_index)
        else:
            return array_index


        


    



#######################
# FILE: agents/granular_navigation_agent/victim_position_finder.py
#######################

import numpy as np
import cv2 as cv


class VictimPositionFinder:
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.fixture_normal_template = self.get_fixture_normal_template(radius=3)

    def update(self):
        pass
        
        debug = self.mapper.pixel_grid.get_colored_grid()
        fixture_indexes = np.nonzero(self.mapper.pixel_grid.arrays["victims"])
        for fixture_x, fixture_y in zip(fixture_indexes[0], fixture_indexes[1]):
            pos1 = Position2D(fixture_x, fixture_y)

            angle = self.mapper.pixel_grid.arrays["victim_angles"][fixture_x, fixture_y]
            angle = Angle(angle)
            angle = Angle(180, unit=Angle.DEGREES) + angle
            angle.normalize()

            fixture_square = self.get_fixture_normal_detection_square(np.array(pos1))
            

            vec = Vector2D(Angle(angle), 10)
            pos2 = vec.to_position()
            pos2 = Position2D(pos2.y, pos2.x) + pos1
            pos2 = pos2.astype(int)

            debug = cv.line(debug, (pos1[1], pos1[0]), (pos2[1], pos2[0]), (0, 255, 0), 1)
        
        cv.imshow("victim_normal_debug", debug)
        


    def find_victim_position(self):
        pass

    def get_victim_position(self):
        pass

    def is_there_victims(self):
        pass


    def get_fixture_normal_detection_square(self, victim_array_index: np.ndarray):
        vns_min_x = victim_array_index[0] - (self.fixture_normal_template.shape[0] // 2)
        vns_min_y = victim_array_index[1] - (self.fixture_normal_template.shape[1] // 2)
        vns_max_x = vns_min_x + self.fixture_normal_template.shape[0]
        vns_max_y = vns_min_y + self.fixture_normal_template.shape[1]

        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_min_x, vns_min_y)))
        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_max_x, vns_max_y)))
        return self.mapper.pixel_grid.arrays["walls"][vns_min_x:vns_max_x, vns_min_y:vns_max_y]


    def get_fixture_normal_angle(self, victim_array_index: np.ndarray):
        vns_min_x = victim_array_index[0] - (self.fixture_normal_template.shape[0] // 2)
        vns_min_y = victim_array_index[1] - (self.fixture_normal_template.shape[1] // 2)
        vns_max_x = vns_min_x + self.fixture_normal_template.shape[0]
        vns_max_y = vns_min_y + self.fixture_normal_template.shape[1]

        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_min_x, vns_min_y)))
        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_max_x, vns_max_y)))

        victim_normal_square = np.zeros_like(self.fixture_normal_template)
        victim_normal_square[self.fixture_normal_template] = self.mapper.pixel_grid.arrays["walls"][vns_min_x:vns_max_x, vns_min_y:vns_max_y][self.fixture_normal_template]

        cv.imshow("normal_sqaure", victim_normal_square.astype(np.uint8) * 255)

        moments = cv.moments(victim_normal_square.astype(np.uint8)) # calculate moments of binary image
        centroid = Position2D(moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]) # calculate x,y coordinate of center
        normal_pos =  Position2D(victim_normal_square.shape) + centroid * -1

        print("normal_pos", centroid)

        center = Position2D(victim_normal_square.shape)
        center = center / 2

        return center.get_angle_to(centroid)

    def get_fixture_normal_template(self, radius):
        diameter = radius * 2 + 1

        template = np.zeros((diameter, diameter), dtype=np.uint8)

        template = cv.circle(template, (radius, radius), radius, 255, -1)

        return template.astype(np.bool_)




#######################
# FILE: agents/granular_navigation_agent/granular_navigation_agent.py
#######################

import numpy as np
import cv2 as cv




class GranularNavigationAgent(Agent):
    """
    Navigates the map without any concept of 'tiles'.
    """
    def __init__(self, mapper: Mapper):
        self.path_finder = PathFinder(mapper)
        self.best_position_finder = BestPositionFinder(mapper)
        self.victim_position_finder = VictimPositionFinder(mapper)
        self.best_position = None
        self.mapper = mapper
        self.__end = False
    
    def update(self) -> None:
        self.best_position_finder.calculate_best_position(finished_path=self.path_finder.is_path_finished() or self.path_finder.path_not_found)

        self.best_position = self.best_position_finder.get_best_position()
        self.path_finder.update(target_position=self.best_position)#np.array(Position2D(-0.08884384679907074, -0.01975882018000104)))

        if self.path_finder.is_path_finished() and \
           self.best_position == self.mapper.start_position and \
           self.best_position.get_distance_to(self.mapper.robot_position) < 0.04:
            self.__end = True

        self.victim_position_finder.update()

    def get_target_position(self) -> Position2D:
        return self.path_finder.get_next_position()

    def do_end(self) -> bool:
        return self.__end
    
    def do_report_victim(self) -> bool:
        return False
        


#######################
# FILE: fixture_detection/victim_clasification.py
#######################

import cv2 as cv
import numpy as np
import random



class VictimClassifier:
    def __init__(self):
        self.white = 255

        self.victim_letter_filter = ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(5, 255, 100))

        self.top_image_reduction = 1
        self.horizontal_image_reduction = 1

        
        self.area_width = 10#20
        self.area_height = 30
        self.min_count_in_area = int(self.area_height * self.area_width * 0.3)

        """
        self.areas = {
            "top": ((0, self.area_height),(50 - self.area_width // 2, 50 + self.area_width // 2)),
            "middle": ((50 - self.area_height // 2, 50 + self.area_height // 2), (50 - self.area_width // 2, 50 + self.area_width // 2)),
            "bottom": ((100 - self.area_height, 100), (50 - self.area_width // 2, 50 + self.area_width // 2 ))
            }
        """

        self.areas = {
            "top": ((0, self.area_height),                                       (self.area_width // -2, self.area_width // 2)),
            "middle": ((50 - self.area_height // 2, 50 + self.area_height // 2), (self.area_width // -2, self.area_width // 2)),
            "bottom": ((100 - self.area_height, 100),                            (self.area_width // -2, self.area_width // 2 ))
            }
        
        self.letters = {
            "H":{'top': False, 'middle': True, 'bottom': False},
            "S":{'top': True, 'middle': True, 'bottom': True},
            "U":{'top': False, 'middle': False, 'bottom': True}
            }

    def crop_white(self, binaryImg):
        white = 255
        rows, cols = np.where(binaryImg == white)
        if len(rows) == 0 or len(cols) == 0:
            # no white pixels found
            return binaryImg
        else:
            minY, maxY = np.min(rows), np.max(rows)
            minX, maxX = np.min(cols), np.max(cols)
            return binaryImg[minY:maxY+1, minX:maxX+1]
    
    def isolate_victim(self, image):
        binary = self.victim_letter_filter.filter(image)
        letter = self.crop_white(binary)

        letter = letter[self.top_image_reduction:, self.horizontal_image_reduction:letter.shape[1] - self.horizontal_image_reduction]
        letter = self.crop_white(letter)
        
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("thresh", binary)

        return letter

    def classify_victim(self, victim):
        letter = self.isolate_victim(victim["image"])

        letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)

        # Calculat centroid of letter and reverse it
        moments = cv.moments(letter)
        center = int(letter.shape[1] - moments["m10"] / moments["m00"])
      
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("letra", letter)

        letter_color = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
        
        images = {
            "top":    letter[self.areas["top"][0][0]   :self.areas["top"][0][1],    self.areas["top"][1][0]    + center:self.areas["top"][1][1]    + center],
            "middle": letter[self.areas["middle"][0][0]:self.areas["middle"][0][1], self.areas["middle"][1][0] + center:self.areas["middle"][1][1] + center],
            "bottom": letter[self.areas["bottom"][0][0]:self.areas["bottom"][0][1], self.areas["bottom"][1][0] + center:self.areas["bottom"][1][1] + center]
            }
        
        if SHOW_FIXTURE_DEBUG:
            cv.rectangle(letter_color,(self.areas["top"][1][0] + center, self.areas["top"][0][0]),        (self.areas["top"][1][1] + center, self.areas["top"][0][1]     ), (0, 255, 0), 1)
            cv.rectangle(letter_color, (self.areas["middle"][1][0] + center, self.areas["middle"][0][0]), (self.areas["middle"][1][1]+ center, self.areas["middle"][0][1]), (0, 0, 255), 1)
            cv.rectangle(letter_color,(self.areas["bottom"][1][0] + center , self.areas["bottom"][0][0]),  (self.areas["bottom"][1][1]+ center, self.areas["bottom"][0][1]), (225, 0, 255), 1)
            cv.imshow("letter_color", letter_color)

        counts = {}
        for key in images.keys():
            count = 0
            for row in images[key]:
                for pixel in row:
                    if pixel == self.white:
                        count += 1

            counts[key] = count > self.min_count_in_area


        final_letter = random.choice(list(self.letters.keys()))
        for letter_key in self.letters.keys():
            if counts == self.letters[letter_key]:
                final_letter = letter_key
                break
        
        return final_letter


#######################
# FILE: fixture_detection/fixture_clasification.py
#######################

import math
import random

import numpy as np
import cv2 as cv


    
class FixtureType:
    def __init__(self, fixture_type, default_letter, ranges=None):
        self.fixture_type = fixture_type
        self.default_letter = default_letter
        self.ranges = ranges
    
    def is_fixture(self, colour_counts: dict):
        for color in self.ranges:
            if not self.ranges[color][0] <= colour_counts[color] <= self.ranges[color][1]:
                return False
        return True
            
class FixtureDetector:
    def __init__(self):
        # Victim classification
        self.victim_classifier = VictimClassifier()

        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 0)),
            "white": ColorFilter(lower_hsv=(0, 0, 207), upper_hsv=(0, 0, 207)),
            "yellow": ColorFilter(lower_hsv=(25, 157, 82), upper_hsv=(30, 255, 255)),
            "red": ColorFilter(lower_hsv=(160, 170, 127), upper_hsv=(170, 255, 255))
        }

        # Fixture filtering
        self.min_fixture_height = 23
        self.min_fixture_width = 19
    
        # Fixture classification
        self.possible_fixture_letters = ["P", "O", "F", "C", "S", "H", "U"]

        # In order of priority
        self.fixture_types = (
            FixtureType("already_detected", "",  {"white": (1,    math.inf), 
                                                  "black": (0,    0),
                                                  "red":   (0,    0), 
                                                  "yellow":(0,    0),}),

            FixtureType("flammable", "F",        {"white": (1,    math.inf), 
                                                  "red":   (1,    math.inf),}),

            FixtureType("organic_peroxide", "O", {"red":   (1,    math.inf), 
                                                  "yellow":(1,    math.inf),}),

            FixtureType("victim",    "H",        {"white": (4000, math.inf), 
                                                  "black": (100,  4000),}),

            FixtureType("corrosive", "C",        {"white": (700,  2500), 
                                                  "black": (1000, 2500),}),

            FixtureType("poison",    "P",        {"white": (700,  4000), 
                                                  "black": (0,    600),}),
        )                    


        # For tuning color filters
        self.do_color_filter_tuning = False
        self.filter_for_tuning = self.color_filters["white"]                       

        if self.do_color_filter_tuning:
            cv.namedWindow("trackbars")

            cv.createTrackbar("min_h", "trackbars", self.filter_for_tuning.lower[0], 255, lambda x: None)
            cv.createTrackbar("max_h", "trackbars", self.filter_for_tuning.upper[0], 255, lambda x: None)

            cv.createTrackbar("min_s", "trackbars", self.filter_for_tuning.lower[1], 255, lambda x: None)
            cv.createTrackbar("max_s", "trackbars", self.filter_for_tuning.upper[1], 255, lambda x: None)

            cv.createTrackbar("min_v", "trackbars", self.filter_for_tuning.lower[2], 255, lambda x: None)
            cv.createTrackbar("max_v", "trackbars", self.filter_for_tuning.upper[2], 255, lambda x: None)
        
    def tune_filter(self, image):
        min_h = cv.getTrackbarPos("min_h", "trackbars")
        max_h = cv.getTrackbarPos("max_h", "trackbars")
        min_s = cv.getTrackbarPos("min_s", "trackbars")
        max_s = cv.getTrackbarPos("max_s", "trackbars")
        min_v = cv.getTrackbarPos("min_v", "trackbars")
        max_v = cv.getTrackbarPos("max_v", "trackbars")
        self.filter_for_tuning = ColorFilter((min_h, min_s, min_v), (max_h, max_s, max_v))
        print(self.filter_for_tuning.lower, self.filter_for_tuning.upper)
        cv.imshow("tunedImage", self.filter_for_tuning.filter(image))


    def sum_images(self, images):
        final_img = images[0]
        for index, image in enumerate(images):
            final_img += image
            #cv.imshow(str(index), image)
        final_img[final_img > 255] = 255
        return final_img

    def filter_fixtures(self, victims) -> list:
        final_victims = []
        for vic in victims:
            if SHOW_FIXTURE_DEBUG:
                print("victim:", vic["position"], vic["image"].shape)

            if vic["image"].shape[0] > self.min_fixture_height and vic["image"].shape[1] > self.min_fixture_width:
                final_victims.append(vic)

        return final_victims

    def find_fixtures(self, image) -> list:
        """
        Finds fixtures in the image.
        Returns a list of dictionaries containing fixture positions and images.
        """
        binary_images = []
        for f in self.color_filters.values():
            binary_images.append(f.filter(image))

        binary_image = self.sum_images(binary_images)
        #print(binary_image)
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("binaryImage", binary_image)
        
        # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
        # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
        for c0 in contours:
            x, y, w, h = cv.boundingRect(c0)
            cv.rectangle(binary_image, (x, y), (x + w, y + h), (225, 255, 255), -1)
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
        # Tambien anade la posicion de cada recuadro en la imagen original
        final_victims = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append({"image":image[y:y + h, x:x + w], "position":(x, y)})
        return self.filter_fixtures(final_victims)
            
    def count_colors(self, image) -> dict:
        color_point_counts = {}

        for name, filter in self.color_filters.items():
            # Filter image to get specific color
            color_image = filter.filter(image)

            # Count where the mask is true
            color_point_counts[name] = np.count_nonzero(color_image)

        return color_point_counts

    def classify_fixture(self, fixture) -> str:
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)

        color_point_counts = self.count_colors(image)
        
        if SHOW_FIXTURE_DEBUG:
            print(color_point_counts)

        # Check all filters. Find first color counts that fit.
        final_fixture_filter = None
        for filter in self.fixture_types:
            if filter.is_fixture(color_point_counts):
                final_fixture_filter = filter
                break
        
        # If nothing matches return random letter
        if final_fixture_filter is None:
            letter = random.choice(self.possible_fixture_letters)

        # If it's a victim classify it
        elif final_fixture_filter.fixture_type == "victim":
            letter = self.victim_classifier.classify_victim(fixture)

        # If already detected then it shouldn't be reported
        elif final_fixture_filter.fixture_type == "already_detected":
            letter = None
        
        # If it's any other type then the letter defined for it can be returned
        else:
            letter = final_fixture_filter.default_letter

        if SHOW_FIXTURE_DEBUG:
            print("FIXTURE: ", letter)

        return letter


#######################
# FILE: executor/executor.py
#######################









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
        self.state_machine.create_state("explore", self.state_explore, {"end",}) # This state follows the position returned by the agent
        self.state_machine.create_state("end", self.state_end)

        self.sequencer = Sequencer(reset_function=self.delay_manager.reset_delay) # Allows for asynchronous programming

        self.fixture_detector = FixtureDetector()
        
        # Flags
        self.mapping_enabled = False
        self.victim_reporting_enabled = False

        # Sequential functions used frequently
        self.seq_print =           self.sequencer.make_simple_event( print)
        self.seq_move_wheels =     self.sequencer.make_simple_event( self.robot.move_wheels)

        self.seq_rotate_to_angle = self.sequencer.make_complex_event(self.robot.rotate_to_angle)
        self.seq_move_to_coords =  self.sequencer.make_complex_event(self.robot.move_to_coords)
        self.seq_delay_seconds =   self.sequencer.make_complex_event(self.delay_manager.delay_seconds)

    def run(self):
        """Advances the simulation, updates all components and executes the state machine."""
        
        while self.robot.do_loop():
            self.robot.update() # Updates robot position and rotation, sensor positions and values, etc.

            self.delay_manager.update(self.robot.time)
            self.stuck_detector.update(self.robot.position,
                                       self.robot.previous_position,
                                       self.robot.drive_base.get_wheel_direction())
            
            self.do_mapping()

            self.state_machine.run()

            time.sleep(0.032)
            
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

    def state_explore(self, change_state_function):
        """Follows the instructions of the agent."""

        self.sequencer.start_sequence() # Starts the sequence

        self.agent.update()

        self.seq_move_to_coords(self.agent.get_target_position())

        self.sequencer.seq_reset_sequence() # Resets the sequence but doesn't change state, so it starts all over again.

        if SHOW_DEBUG:
            print("rotation:", self.robot.orientation)
            print("position:", self.robot.position)
        
        if self.agent.do_end():
            self.state_machine.change_state("end")
    
    def state_end(self, change_state_function):
        self.robot.comunicator.send_end_of_play()

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
            





    





#######################
# FILE: main.py
#######################


def main():
    robot = Robot(time_step=32)
    mapper = Mapper(tile_size=0.12, robot_diameter=robot.diameter, camera_distance_from_center=robot.diameter / 2)
    agent = GranularNavigationAgent(mapper=mapper)

    executor = Executor(agent, mapper, robot)

    executor.run()


main()