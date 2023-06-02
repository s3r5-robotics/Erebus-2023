from typing import Callable, Dict, Set, NamedTuple

import debug

_StateName = str  # Type alias if we want to change to e.g. enum later
_StateFunction = Callable[[], bool]


class State(NamedTuple):
    name: _StateName
    function: _StateFunction
    """Main state function - returns False if the state machine should stop execution"""
    possible_changes: Set[_StateName]

    def __eq__(self, other) -> bool:
        return isinstance(other, self.__class__) and (self.name == other.name)

    @classmethod
    def for_function(cls, function: _StateFunction, *possible_changes: _StateFunction) -> 'State':
        """Create new State instance with the same name as function name and the given functions as possible changes"""
        return cls(function.__name__, function, {f.__name__ for f in possible_changes})


class StateMachine:
    """
    A simple state machine.
    """

    def __init__(self, initial_state: _StateName, on_state_change: Callable[[_StateName, _StateName], None] = None):
        """
        Initialize a new state machine

        :param initial_state:   The initial state of the state machine (can be registered later).
        :param on_state_change: A function (current_state, new_state) that is called when the state changes.
        """
        self._on_state_change = on_state_change
        self._current_state: _StateName = initial_state
        self._states: Dict[_StateName, State] = {}

        if debug.STATES:
            print(f"\n{self.__class__.__name__}: Initial state '{self._current_state}'")

    def __iadd__(self, state: State) -> 'StateMachine':
        """Add a state to the state machine using += operator."""
        assert state.name not in self._states, f"State '{state.name}' already exists: {self._states[state.name]}"

        self._states[state.name] = state

        return self

    def register_state(self, name: _StateName, function: _StateFunction, possible_changes: Set[_StateName]) -> None:
        self.__iadd__(State(name, function, possible_changes))

    @property
    def state(self) -> State:
        return self._states[self._current_state]

    def change_state(self, new_state: _StateName) -> None:
        """Sets the state the specified value."""
        if new_state == self._current_state:
            return

        assert new_state in self._states, \
            f"State '{new_state}' does not exist in {self._states}"
        assert new_state in self.state.possible_changes, \
            f"State '{new_state}' is not in the possible changes for current state '{self.state}'"

        if debug.STATES:
            print(f"\n{self.__class__.__name__}: Changing state from '{self._current_state}' to '{new_state}'")

        if self._on_state_change:
            self._on_state_change(self._current_state, new_state)
        self._current_state = new_state

    def run(self) -> bool:
        return self.state.function()
