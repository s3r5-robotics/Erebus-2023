# Instead of just writing bare `print` statements, use debug bools which can be toggled from here.
# Use separate variables for major debugging sections to prevent cluttering the console.

# Debugging example:
"""
>>> import debug
>>> if debug.POSITION:
>>>     print(robot.getPosition())
"""

DISTANCE: bool = False
""">> {distance left} | {distance front} | {distance right}"""

MOVEMENT: bool = False
""">> {x axis} | {y axis}
>> {yaw} | {pitch} | {roll}
>> {current velocity} | {target velocity} | {movement angle"""

STATES: bool = True
"""Whenever the state changes"""

STEP: bool = False
"""Current step number"""

FIXTURE_DETECTION: bool = True
"""Print the detected color"""

FIXTURE_CLASIFICATION: bool = True
"""Print the classified fixture"""

FIXTURE_FITTING: bool = False
"""Print the chance of each fixture for the detected image"""

ANY = DISTANCE or MOVEMENT or STATES or FIXTURE_DETECTION
