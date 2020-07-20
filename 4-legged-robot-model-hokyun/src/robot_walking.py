#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 20:31:07 2020

@author: miguel-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
from simple_pid import PID 

import angleToPulse
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from joystick import Joystick
from serial_com import ArduinoSerial
from gaitPlanner import trotGait


############################################################################
##### 시뮬레이션 영역 ###########################################################
############################################################################
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


cubeStartPos = [0,0,0.3]
FixedBase = False
boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=FixedBase)
if (FixedBase == False):
    p.loadURDF("plane.urdf")
jointIds = []
paramIds = [] 
time.sleep(0.5)
for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 3
footFL_index = 7
footBR_index = 11
footBL_index = 15   

pybulletDebug = pybulletDebug()
############################################################################
############################################################################
############################################################################

# 조이스틱 포트 연결
joystick = Joystick('/dev/input/event11') #need to specify the event route
# 아두이노 포트 연결
arduino = ArduinoSerial('/dev/ttyACM0') #need to specify the serial port
# kinematic_model.py의 robotKinematics 클래스의 인스턴스 'robotKinematics' 생성
robotKinematics = robotKinematics()
# gaitPlanner.py의 trotGait 클래스의 인스턴스 'trot' 생성 
trot = trotGait() 


# 안 씀, 'robotKinematics' 생성자에 포함되어 있음, 초기 앵글값 설정
#robot properties
"""initial safe position"""
#angles
targetAngs = np.array([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                        0 , np.pi/4 , -np.pi/2, 0 ,#BL
                        0 , np.pi/4 , -np.pi/2, 0 ,#FL
                        0 , np.pi/4 , -np.pi/2, 0 ])#FR

#FR_0  to FR_4 
#FRcoord = np.matrix([0. , -3.6 , -0.15])
#FLcoord = np.matrix([0. ,  3.6 , -0.15])
#BRcoord = np.matrix([0. , -3.6 , -0.15])
#BLcoord = np.matrix([0. ,  3.6 , -0.15])


# 초기 발 위치 설정
"initial foot position"
#foot separation (0.182 -> tetta=0) and distance to floor
# Ydist = 0.17
# Xdist = 0.25
height = 0.17
#body frame to foot frame vector
# (FR,FL,BR,BL)순 인것 같음. 앞방향 +x, 왼쪽 +y, 위쪽 +z
bodytoFeet0 = np.matrix([[ 0.08 , -0.07 , -height],
                         [ 0.08 ,  0.07 , -height],
                         [-0.11 , -0.07 , -height],
                         [-0.11 ,  0.07 , -height]])

orn = np.array([0. , 0. , 0.])
pos = np.array([0. , 0. , 0.])
lastTime = 0.

# pid 모듈 사용, 샘플링 타임 0.02s(50 Hz)
pidX = PID(-0.0005 , 0.0 , 0.000001 , setpoint=0.)
pidY = PID(0.0005 , 0. , 0. , setpoint=0.)
pidX.sample_time = 0.02  # update every 0.02 seconds
pidY.sample_time = 0.02  


T = 0.4 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
p.setRealTimeSimulation(1) #파이불릿 설정
p.setTimeStep(0.002) #파이불릿 설정
while(True):
    loopTime = time.time() #현재 루프 시작시간 기록
    _ , _ , _ , _ , _ , _ = pybulletDebug.cam_and_robotstates(boxId) #파이불릿
    
    commandCoM , V , angle , Wrot , T  , compliantMode , yaw , pitch  = joystick.read() #조이스틱 입력 읽어옴
    
    # gaitPlanner.py 실행 
    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    # 발 궤적 얻어옴
    bodytoFeet = trot.loop(V , angle , Wrot , T , offset , bodytoFeet0) 
    
    # 아두이노의 현재 상태를 serial message 받음
    arduinoLoopTime , Xacc , Yacc , realRoll , realPitch = arduino.serialRecive()#recive serial message

    pos[0] = pidX(realPitch)
    pos[1] = pidY(realRoll)
    
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    # kinematic_model.py 실행
    # IK로 푼 값들 얻어옴
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos + commandCoM , bodytoFeet)

    # angleToPulse.py 실행
    pulsesCommand = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
    
    # 아두이노 serial command 보냄(pulsesCommand)
    arduino.serialSend(pulsesCommand)#send serial command to arduino
    
    # 파이불릿 시뮬레이션
    #move movable joints
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])
    
    p.stepSimulation()#compute simulation
    
    # 코드 구동 시간 체크
    print(time.time() - loopTime ,T)
p.disconnect()



