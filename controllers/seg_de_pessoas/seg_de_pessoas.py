# ----------------------------------------- #
# | Projetista: Leonardo Gomes            |
# | Projetista: Thomas Patrick            |
# | Faculdade: Centro universitário FEI   |
# | Curso: Eng. de Controle e Automação   |
# | Período: Diurno                       |
# | Professor Orientador: Danilo Perico   |
# ----------------------------------------- #

from controller import Robot, Camera

import numpy as np
import cv2
import time
import argparse
import math

def yolo(net):
    # Carregando as classes
    class_names = []
    with open("coco.names", "r") as f:
        class_names = [cname.strip() for cname in f.readlines()]
            
    #Setando os parâmetros da rede neural
    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(size=(416,416),scale=1/255)
    return model

def deteccao(frame_image, model):
    #Retorna classe, confiança, aonde tá na imagem o que o YOLO achou
    classes, scores, boxes = model.detect(frame_image, 0.01, 0.2)
    return classes, scores, boxes

def pessoa(classes):
    count = np.count_nonzero(classes == 0)
    #count = np.mean(classes)
    if count != 0:
        return 1
    else:
        return 0

def dist_media_euclidiana_coord(boxes):
    if len(boxes) != 0:
        xa,xb,ya,yb = boxes[0][0],boxes[0][3],boxes[0][1],boxes[0][2]
        xm = (xb-xa)/2
        loc_pessoa_em_x = xm+xa
        ym = (yb-ya)/2
        loc_pessoa_em_y = ym+ya
        coord = [loc_pessoa_em_x, loc_pessoa_em_y]
        return coord
    else:
        coord = [0,0]
        return coord 
    
def run_robot(robot):
    velocidade_maxima = 10    
    limite_max_C = 3
    limite_min_C = 1
    limite_min_BD = 1
    limite_max_BD = 3
    limite_AE = 1
    #timestep = int(robot.getBasicTimeStep()) # Padrão do sistema
    timestep = 70
     
    # Câmera
    camera = robot.getDevice('camera')
    camera.enable(timestep)
   
    # Lidar
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
  
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

    # Carregando Yolo
    #Carregando os pesos  da rede neural
    net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
    #net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    model = yolo(net)
    
    while robot.step(timestep) != -1:
        #Lidar
        range_image = lidar.getRangeImage()
        
        #Câmera
        cap = camera.getImage()
        Camera.saveImage(camera, 'img.png',1)
        
        frame = np.frombuffer(cap,np.uint8).reshape(camera.getHeight(), camera.getWidth(),4)
        frame_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        #Detecção
        classes, scores, boxes = deteccao(frame_image, model)
               
        localizacao_pessoa = pessoa(classes)
        
        posicao = dist_media_euclidiana_coord(boxes)
        
        # Seção A
        if min(range_image[0:101])<limite_AE:
            letra = 'A'
            distancia = min(range_image[0:101])
            motor_fl.setVelocity(velocidade_maxima*0.70) 
            motor_fr.setVelocity(-velocidade_maxima*0.05)
            motor_rl.setVelocity(velocidade_maxima*0.70)
            motor_rr.setVelocity(-velocidade_maxima*0.05)
        # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
        # Setor B
        elif (localizacao_pessoa == 1) & (limite_min_BD<min(range_image[102:204])) & (min(range_image[102:204])<limite_max_BD):
            motor_fl.setVelocity(-velocidade_maxima*0.05) 
            motor_fr.setVelocity(velocidade_maxima*0.30)
            motor_rl.setVelocity(-velocidade_maxima*0.05)
            motor_rr.setVelocity(velocidade_maxima*0.30)
            letra = 'B'
            distancia = min(range_image[102:204])
            if (128<posicao[0]) & (posicao[0]<168):
                motor_fl.setVelocity(-velocidade_maxima*0.05) 
                motor_fr.setVelocity(velocidade_maxima*0.20)
                motor_rl.setVelocity(-velocidade_maxima*0.05)
                motor_rr.setVelocity(velocidade_maxima*0.20)
        elif min(range_image[102:204])<0.7:
            motor_fl.setVelocity(-velocidade_maxima*0.05) 
            motor_fr.setVelocity(velocidade_maxima*0.40)
            motor_rl.setVelocity(-velocidade_maxima*0.05)
            motor_rr.setVelocity(velocidade_maxima*0.40)
        # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
        # Setor C (Ideal)
        elif (localizacao_pessoa == 1) & (limite_min_C<min(range_image[204:306])) & (min(range_image[204:306])<limite_max_C):
            letra = 'C'
            distancia = min(range_image[204:306])
            motor_fl.setVelocity(velocidade_maxima*0.6) 
            motor_fr.setVelocity(velocidade_maxima*0.6)
            motor_rl.setVelocity(velocidade_maxima*0.6)
            motor_rr.setVelocity(velocidade_maxima*0.6)
            if(168<posicao[0]) & (posicao[0]<248):
                motor_fl.setVelocity(velocidade_maxima) 
                motor_fr.setVelocity(velocidade_maxima)
                motor_rl.setVelocity(velocidade_maxima)
                motor_rr.setVelocity(velocidade_maxima)
            else:
                motor_fl.setVelocity(velocidade_maxima*0.2) 
                motor_fr.setVelocity(velocidade_maxima*0.2)
                motor_rl.setVelocity(velocidade_maxima*0.2)
                motor_rr.setVelocity(velocidade_maxima*0.2)
        # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
        # Setor D
        elif (localizacao_pessoa == 1) & (limite_min_BD<min(range_image[306:407])) & (min(range_image[306:407])<limite_max_BD):
            letra = 'D'
            distancia = min(range_image[306:407])
            motor_fl.setVelocity(velocidade_maxima*0.30) 
            motor_fr.setVelocity(-velocidade_maxima*0.05)
            motor_rl.setVelocity(velocidade_maxima*0.30)
            motor_rr.setVelocity(-velocidade_maxima*0.05)
            if (248<posicao[0]) & (posicao[0]<314):
                motor_fl.setVelocity(velocidade_maxima*0.20) 
                motor_fr.setVelocity(-velocidade_maxima*0.05)
                motor_rl.setVelocity(velocidade_maxima*0.20)
                motor_rr.setVelocity(-velocidade_maxima*0.05)
        elif min(range_image[306:407])<0.7:
            motor_fl.setVelocity(velocidade_maxima*0.40) 
            motor_fr.setVelocity(-velocidade_maxima*0.05)
            motor_rl.setVelocity(velocidade_maxima*0.40)
            motor_rr.setVelocity(-velocidade_maxima*0.05)
        # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
        # Setor E
        elif(min(range_image[408:510])<limite_AE):
            letra = 'E'
            distancia = min(range_image[408:510])
            motor_fl.setVelocity(-velocidade_maxima*0.05) 
            motor_fr.setVelocity(velocidade_maxima*0.70)
            motor_rl.setVelocity(-velocidade_maxima*0.05)
            motor_rr.setVelocity(velocidade_maxima*0.70)
        # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*    
        else:
            motor_fl.setVelocity(0) 
            motor_fr.setVelocity(0)
            motor_rl.setVelocity(0)
            motor_rr.setVelocity(0)
            
        print('----------------------------------------')
        print('Zona:',letra)
        print('Distância:',min(range_image[204:306]))
        print('----------------------------------------')

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)