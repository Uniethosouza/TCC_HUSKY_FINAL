from controller import Robot

robot = Robot()


timestep = int(robot.getBasicTimeStep())

# getMotors
motor_fl = robot.getDevice("front_left_wheel")
motor_fr = robot.getDevice("front_right_wheel")
motor_rl = robot.getDevice("rear_left_wheel")
motor_rr = robot.getDevice("rear_right_wheel")

# set for Velocity mode
motor_fl.setPosition(float('inf'))
motor_fr.setPosition(float('inf'))
motor_rl.setPosition(float('inf'))
motor_rr.setPosition(float('inf'))

while robot.step(timestep) != -1:
    motor_fl.setVelocity(4) 
    motor_fr.setVelocity(4)
    motor_rl.setVelocity(4)
    motor_rr.setVelocity(4)
    # wait 2 seconds
    robot.step(2000)
    # stop
    motor_fl.setVelocity(0) 
    motor_fr.setVelocity(0)
    motor_rl.setVelocity(0)
    motor_rr.setVelocity(0)
    # wait 0.8 seconds
    robot.step(800)
    #turn left
    motor_fl.setVelocity(-5.0) 
    motor_fr.setVelocity(5.0)
    motor_rl.setVelocity(-5.0)
    motor_rr.setVelocity(5.0)
    # wait 2 seconds
    robot.step(1000)
    