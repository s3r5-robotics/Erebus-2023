from typing import Type


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
