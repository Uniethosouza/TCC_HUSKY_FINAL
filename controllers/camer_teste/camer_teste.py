from controller import Robot, Camera
robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = Camera('camera')
camera.enable(100)
while robot.setp(timestep) != -1:
    img = camera.getImage()
    pass