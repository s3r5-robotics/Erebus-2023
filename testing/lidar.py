from controller import Robot, Lidar

robot = Robot()
lidar = robot.getLidar("lidar")
timestep = int(robot.getBasicTimeStep())

lidar.enable(timestep)
lidar.enablePointCloud()

while robot.step(timestep) != -1:
    points = lidar.getPointCloud()
    print(points,
          "\nx:", str(points[0].x),
          "\ny:", str(points[0].y),
          "\nz:", str(points[0].z),
          "\nlayer:", str(points[0].layer_id))
