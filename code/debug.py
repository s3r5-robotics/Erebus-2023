# Instead of just writing bare `print` statements, use debug bools which can be toggled from here.
# Use separate variables for major debugging sections to prevent cluttering the console.

# Debugging example:
"""
>>> import debug
>>> if debug.POSITION:
>>>     print(robot.getPosition())
"""

CAMERA: bool = False
DISTANCE: bool = False
MOVEMENT: bool = False
STATES: bool = True
STEP: bool = False
LIDAR: bool = False
FIXTURE_DETECTION: bool = True

ANY = CAMERA or DISTANCE or MOVEMENT or STATES or LIDAR or FIXTURE_DETECTION
