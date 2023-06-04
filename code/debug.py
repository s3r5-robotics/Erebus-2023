# Instead of just writing bare `print` statements, use debug bools which can be toggled from here.
# Use separate variables for major debugging sections to prevent cluttering the console.

# Debugging example:
"""
>>> import debug
>>> if debug.POSITION:
>>>     print(robot.getPosition())
"""

CAMERA: bool = True
DISTANCE: bool = True
MOVEMENT: bool = True
STATES: bool = True
LIDAR: bool = False

ANY = CAMERA or DISTANCE or MOVEMENT or STATES or LIDAR
