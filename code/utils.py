import dataclasses
import math
from numbers import Number
from typing import Type, SupportsFloat, TypeVar, Literal, Union, Iterable
from data_structures.vectors import Position2D


class InstanceSubclass:
    """
    Using this class as a first parent class will avoid creating new instance of the
    child class, instead it will reuse the existing instance of the parent class.

    This enabled extending classes which are returned by library/API calls and their
    creation cannot be modified.

    Example:
    >>> # In some example library module:
    ... class A:
    ...     pass
    ...
    ... def api_call() -> A:
    ...     # This represents example API call which returns instance of A(), of which creation we cannot modify.
    ...     return A()
    ...
    ... # User code
    ... class B(InstanceSubclass, A):
    ...     def __init__(self, a: A, *args, **kwargs) -> None:
    ...         super().__init__(self)  # Not required, but to satisfy the linter.
    ...         assert self is a  # Just to showcase that the instance is reused.
    ...
    ... aa = api_call()
    ... assert isinstance(aa, A), "aa is instance of A"
    ... assert not isinstance(aa, B), "aa is not instance of B"
    ... bb = B(aa)
    ... assert aa is bb, "No new instance of B was created, instead the instance of A was reused"
    ... assert isinstance(bb, A), "bb is instance of parent class A"
    ... assert isinstance(bb, B), "bb is instance of B"
    ... assert isinstance(aa, B), "As aa is the same object, it is now also instance of B"

    :note: Do not forget to specify the same instance as a first argument of the __init__ method.
    :note: super().__init__() method does not need to be called in the child class.
    """

    def __new__(cls, instance: Type, *args, **kwargs) -> 'InstanceSubclass':
        instance.__class__ = cls
        # Type Checker cannot detect that the provided instance had just had the __class__ (type) changed.
        # noinspection PyTypeChecker
        return instance

    def __init__(self, instance: 'InstanceSubclass', *args, **kwargs) -> None:
        """This method does not need to be called in the child class"""
        # This __init__ method is implemented to ensure that the __init__ method of the second parent
        # class will not be called, as it was already called when the instance was created.
        assert instance is self


# TypeVar is used in function typing instead of just the base-class to enable static type checking of the return type
# of function, based on the arguments or argument type. This is just a generic TypeVar to conform the original float
# definition below.
# https://medium.com/@steveYeah/using-generics-in-python-99010e5056eb
# https://stackoverflow.com/questions/69184577/return-type-based-on-type-argument
# https://stackoverflow.com/questions/68650071/type-annotations-for-base-and-inherited-classes-is-generic-and-typevar-the-rig
_T = TypeVar("_T")


class Angle(float):
    """
    This class is a float number designed to hold an angle value in radians

    In Webots, angles are generally in radians and derived units (e.g. angular velocity in radians per second),
    but many times using degrees is clearer for user logic and debugging. This class behaves as a float number
    representing an angle in radians, so it can be directly used in Webots API calls. However, it can also be
    initialized with degrees, which are internally converted to radians - all internal operations are done
    using radians, only repr() method adds value converted to degrees.

    Tests:
    >>> assert Angle(0, normalize=None).deg == 0
    >>> assert Angle(math.pi / 2, normalize=None).deg == 90
    >>> assert Angle(2 * math.pi, normalize=None).deg == 360
    >>> assert Angle(3 * math.pi, normalize=None).deg == 540
    >>> assert Angle(-3 * math.pi, normalize=None).deg == -540
    >>> assert Angle(-2 * math.pi, normalize=None).deg == -360
    >>> assert Angle(-math.pi / 2, normalize=None).deg == -90

    >>> assert Angle(0, normalize='2pi').deg == 0
    >>> assert Angle(math.pi / 2, normalize='2pi').deg == 90
    >>> assert Angle(2 * math.pi, normalize='2pi').deg == 0
    >>> assert Angle(3 * math.pi, normalize='2pi').deg == 180
    >>> assert Angle(-3 * math.pi, normalize='2pi').deg == 180
    >>> assert Angle(-2 * math.pi, normalize='2pi').deg == 0
    >>> assert Angle(-math.pi / 2, normalize='2pi').deg == 270

    >>> assert Angle(0, normalize='-pi').deg == 0
    >>> assert Angle(math.pi / 2, normalize='-pi').deg == 90
    >>> assert Angle(2 * math.pi, normalize='-pi').deg == 0
    >>> assert Angle(3 * math.pi, normalize='-pi').deg == 180
    >>> assert Angle(-3 * math.pi, normalize='-pi').deg == 180
    >>> assert Angle(-2 * math.pi, normalize='-pi').deg == 0
    >>> assert Angle(-math.pi / 2, normalize='-pi').deg == -90

    >>> assert Angle(deg=0) == 0
    >>> assert Angle(deg=90) == math.pi / 2
    >>> assert Angle(deg=180) == math.pi
    >>> assert Angle(deg=-180) == math.pi
    >>> assert Angle(deg=360) == 0
    >>> assert Angle(deg=360, normalize=None) == 2 * math.pi
    >>> assert Angle(deg=-190, normalize='2pi') - Angle(deg=170, normalize=None) < 1e-10  # float calculation error
    >>> assert Angle(deg=-190, normalize='-pi') - Angle(deg=170, normalize=None) < 1e-10
    >>> assert Angle(deg=-170, normalize='2pi') - Angle(deg=190, normalize=None) < 1e-10
    >>> assert Angle(deg=-170, normalize='-pi') - Angle(deg=-170, normalize=None) < 1e-10

    >>> assert Angle(3 * math.pi) == Angle(deg=180)
    >>> assert Angle(3 * math.pi) == Angle(deg=-180)
    >>> assert Angle(3 * math.pi) == Angle(deg=540)
    >>> assert Angle(-3 * math.pi) == Angle(deg=180)
    >>> assert Angle(3 * math.pi, normalize=None) == Angle(deg=540, normalize=None)

    >>> assert Angle(math.pi / 2) + Angle(deg=90) == Angle(deg=180)
    >>> assert Angle(math.pi / 2) + Angle(deg=90) == math.pi
    >>> assert Angle(2 * math.pi) + Angle(2 * math.pi) == 0

    >>> def assert_rotation(from_deg: float, to_deg: float, result_deg: float):
    ...     diff = Angle(deg=from_deg).rotation_to(Angle(deg=to_deg)).deg
    ...     assert abs(diff - result_deg) < 1e-10, f"Angle({from_deg}°).rotation_to({to_deg}°)={diff}° != {result_deg}°"
    ...     # Also test the reverse rotation - the result should be opposite, except for 180° which is always positive
    ...     if result_deg != 180:
    ...         result_deg = -result_deg
    ...     diff = Angle(deg=to_deg).rotation_to(Angle(deg=from_deg)).deg
    ...     assert abs(diff - result_deg) < 1e-10, f"Angle({to_deg}°).rotation_to({from_deg}°)={diff}° != {result_deg}°"

    >>> assert_rotation(0, 0, 0)
    >>> assert_rotation(0, 170, 170)
    >>> assert_rotation(0, 180, 180)
    >>> assert_rotation(0, 190, -170)
    >>> assert_rotation(0, 350, -10)
    >>> assert_rotation(0, 360, 0)
    >>> assert_rotation(0, 370, 10)
    >>> assert_rotation(-10, 10, 20)
    >>> assert_rotation(540, -720, 180)
    >>> assert_rotation(-720, 730, 10)
    """

    def __new__(cls: Type[_T], rad: SupportsFloat = None, deg: SupportsFloat = None,
                normalize: Literal[None, '2pi', '-pi'] = '2pi') -> _T:
        """
        Create a new Angle object from given angle in radians or degrees.

        :param rad:       Angle in radians.
        :param deg:       Angle in degrees, if `rad` is not provided. Converted to radians internally.
        :param normalize: If `None`, no angle normalization is performed.
                          If '2pi', angle is reduced to the "fundamental range", which is [0, 2π).
                          If '-pi', angle is reduced to the "symmetric range", which is [-π, π).
        """
        if rad is None:
            rad = math.radians(deg)
        else:
            assert deg is None, "Only one of radians or degrees can be provided"

        if normalize is not None:
            assert normalize in ('2pi', '-pi'), f"normalize can be None|'2pi'|'-pi', not '{normalize}'"

            # Normalize the angle to [0, 2π), also called reducing to the "fundamental range"
            rad %= 2 * math.pi
            # Optionally, shift the angle to (-π, π], also called reducing it to the "symmetric/centered range"
            # Note: doing `((rad + math.pi) % (2 * math.pi)) - math.pi` reduces to [-π, π) instead of (-π, π]!
            if normalize == '-pi' and rad > math.pi:
                rad -= 2 * math.pi  # Reduce the angle to (-π, π]

        return super().__new__(cls, rad)

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self:f}={self.deg:f}°)"

    @property
    def deg(self) -> float:
        """Get this angle in degrees"""
        return math.degrees(self)

    @property
    def radians(self) -> float:
        return self

    def rotation_to(self, other: float) -> 'Angle':
        """
        Get the shortest rotation to other angle (maximum 180 degrees)

        :param other: Target angle to which we want to rotate, in radians.

        :return: Angle in (-π, π] radians, where positive angle means counter-clockwise rotation
                 and negative angle means clockwise rotation (math convention).
        """
        # If the difference is more than 180 degrees, then the shortest rotation is the other way around
        return Angle(other - self, normalize='-pi')


# https://towardsdatascience.com/battle-of-the-data-containers-which-python-typed-structure-is-the-best-6d28fde824e
@dataclasses.dataclass(slots=True)
class Point:
    """
    A point in 2D space

    This class uses RUB (Right-Up-Back) and NUE (North-Up-East) coordinate systems, however the points
    are named x,y instead of x,z for less confusion - we are used to ENU (East-North-Up).
    https://gitlab.com/rcj-rescue-tc/erebus/erebus/-/blob/master/CHANGELOG.md#release-v2200-2022-09-07
    https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a
    """
    x: float  # x in both NUE and ENU
    y: float  # z in NUE, -y in ENU

    @classmethod
    def from_xyz(cls, x_or_xyz: Union[Number, Iterable[Number]], y: Number = None, z: Number = None) -> 'Point':
        """
        Create a new Point object from given 3D coordinates

        :param x_or_xyz:  X coordinate or 3-element array containing the X,Y,Z coordinates.
        :param y:         Y coordinate, if `x` is a number.
        :param z:         Z coordinate, if `x` is a number.
        """
        if y is None:
            x_or_xyz, y, z = x_or_xyz
        return cls(x_or_xyz, z)  # NUE coordinate system orientation

    @property
    def distance(self) -> float:
        """Get the distance from the origin"""
        # The same as math.sqrt(self.x**2 + self.y**2)
        return math.hypot(self.x, self.y)

    def distance_to(self, other: 'Point') -> float:
        """Get the distance to another point"""
        # The same as math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
        return math.dist((self.x, self.y), (other.x, other.y))

    @property
    def position2d(self) -> Position2D:
        return Position2D(self.x, self.y)


def static_vars(**kwargs):
    """
    Decorator to add static variables ("local" variables which store their state) to a function or any other object

    See https://stackoverflow.com/a/279586/5616255

    >>> @static_vars(a=1, b=10)
    ... def foo():
    ...     foo.a += 1
    ...     foo.b -= 1
    ...     return foo.a, foo.b
    >>> assert foo() == (2, 9)
    >>> assert foo() == (3, 8)
    """

    # https://stackoverflow.com/a/279586/5616255

    def wrapper(func):
        for k, v in kwargs.items():
            assert not hasattr(func, k), f"'{func}' already has attribute '{k}'"
            setattr(func, k, v)
        return func

    return wrapper


def divide_into_chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def cords_from_rads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


def deg_to_rad(deg):
    return deg * math.pi / 180
