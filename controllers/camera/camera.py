import cv2
import time
import numpy as np

from controller import Robot, Camera

robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = Camera("camera")
camera.enable(timestep)

 #Carregando as classes
class_names = []
#with open("coco.names", "r") as f:
#    class_names = [cname.strip() for cname in f.readlines()]

#Carregando os pesos  da rede neural
#net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")

#Setando os parâmetros da rede neural
#model = cv2.dnn_DetectionModel(net)
#model.setInputParams(size=(416,416),scale=1/255)
print("camera...")
while robot.step(timestep) != -1:
    #Cores das classes
    COLORS = [(0,255,255), (255,255,0), (0,255,0), (255,0,0)]
    print("cap")
    cap = camera.getImage('camera')
    print(cap)