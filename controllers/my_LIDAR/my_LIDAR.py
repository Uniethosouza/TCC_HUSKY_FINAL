from controller import Robot, Camera

import numpy as np
import pandas as pd

def run_robot(robot):     
    velocidade_maxima = 6.28
    # get the time step of the current world.
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
        
    # stop
    motor_fl.setVelocity(0.0) 
    motor_fr.setVelocity(0.0)
    motor_rl.setVelocity(0.0)
    motor_rr.setVelocity(0.0)
        
    # LIDAR
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    
    # Camera
    #camera = robot.getDevice('camera')
    #camera.enable(timestep)    
        
    while robot.step(timestep) != -1:
        range_image = lidar.getRangeImage()
        #print("{}".format(range_image[254:258]))
        """avgResult = [np.average(range_image[0:102]),
                     np.average(range_image[102:204]),
                     np.average(range_image[204:306]),
                     np.average(range_image[306:408]),
                     np.average(range_image[408:510])]
        print(avgResult)"""
        
        # Condicionais de movimento
        ## Setor C (Ideal)
        ## Setor A
        if((min(range_image[0:101])<1)):
            motor_fl.setVelocity(velocidade_maxima*0.70) 
            motor_fr.setVelocity(-velocidade_maxima*0.05)
            motor_rl.setVelocity(velocidade_maxima*0.70)
            motor_rr.setVelocity(-velocidade_maxima*0.05)
        ## Setor B
        elif((min(range_image[102:204])<2.5)):
            motor_fl.setVelocity(velocidade_maxima*0.40) 
            motor_fr.setVelocity(velocidade_maxima*0.05)
            motor_rl.setVelocity(velocidade_maxima*0.40)
            motor_rr.setVelocity(velocidade_maxima*0.05)
        ## Setor D
        elif((min(range_image[306:407])<2.5)):
            motor_fl.setVelocity(velocidade_maxima*0.05) 
            motor_fr.setVelocity(velocidade_maxima*0.40)
            motor_rl.setVelocity(velocidade_maxima*0.05)
            motor_rr.setVelocity(velocidade_maxima*0.40)
        ## Setor E
        elif((min(range_image[408:510])<1)):
            motor_fl.setVelocity(-velocidade_maxima*0.05) 
            motor_fr.setVelocity(velocidade_maxima*0.70)
            motor_rl.setVelocity(-velocidade_maxima*0.05)
            motor_rr.setVelocity(velocidade_maxima*0.70)
        else:
            motor_fl.setVelocity(velocidade_maxima) 
            motor_fr.setVelocity(velocidade_maxima)
            motor_rl.setVelocity(velocidade_maxima)
            motor_rr.setVelocity(velocidade_maxima)
        pass
        
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)