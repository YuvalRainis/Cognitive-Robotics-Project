from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    print("Python is working!")
