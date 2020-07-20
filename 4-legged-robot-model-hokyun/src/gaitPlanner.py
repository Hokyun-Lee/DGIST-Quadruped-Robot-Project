#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:38:15 2020

@author: miguel-asd
"""
import time
import numpy as np

##function neccesary to build a parametrized bezier curve 
# 팩토리얼(재귀함수로 구현)
def f(n,k): #calculates binomial factor (n k)
    return np.math.factorial(n)/(np.math.factorial(k)*np.math.factorial(n-k))

# 베지어 곡선 그리는 함수(정의 참조 https://terms.naver.com/entry.nhn?docId=3473603&cid=58439&categoryId=58439 )
# 10개 포인트로 베지어 곡선 만듦(원리 참조 https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot/log/178481-step-trajectory-and-gait-planner-from-mit-cheetah)
def b(t,k,point):
    n = 9 #10 points bezier curve
    return point*f(n,k)*np.power(t,k)*np.power(1-t,n-k)


#gait planner in order to move all feet
class trotGait:
    def __init__(self):
        self.bodytoFeet = np.zeros([4,3])  
        self.phi = 0.
        self.phiStance = 0.
        self.lastTime = 0.
        self.alpha = 0.
        self.s = False
    
    """This trajectory planning is mostly based on: 
    https://www.researchgate.net/publication/332374021_Leg_Trajectory_Planning_for_Quadruped_Robots_with_High-Speed_Trot_Gait"""
    def calculateStance(self , phi_st , V , angle):#phi_st between [0,1), angle in degrees
        # 조이스틱에서 받아온 각도 : angle, arctan(x/y)로 구해짐. joystick.py -> robot_walking.py -> gaitPlanner.py 의 경로를 거침
        # 회전변환에 쓰임
        # degree로 들어오는걸 radian 으로 변환함
        c = np.cos(np.deg2rad(angle))#cylindrical coordinates
        s = np.sin(np.deg2rad(angle))
        
        A = 0.001
        halfL = 0.05
        #phi_st란 stance 상태를 한 주기로 봤을때 어느 주기에 있는지 나타낸다.
        p_stance=halfL*(1-2*phi_st)
        
        # 회전 변환
        # V 는 조이스틱 (x^2 + y^2)/250 값으로(joystick.py에서 확인가능) 전진하는 크기라고 생각하면 될듯
        stanceX =  c*p_stance*np.abs(V)
        stanceY = -s*p_stance*np.abs(V)
        stanceZ = -A*np.cos(np.pi/(2*halfL)*p_stance) #Z축도 0이 아니라, 베지어 커프 궤적 그림을 보면 알 수 있듯이 0.001 계수를 갖는 cos함수를 그린다.
        
        return stanceX, stanceY , stanceZ
        
        
    def calculateBezier_swing(self , phi_sw , V , angle):#phi between [0,1), angle in degrees
        #curve generator https://www.desmos.com/calculator/xlpbe9bgll
    
         # 조이스틱에서 받아온 각도 : angle, arctan(x/y)로 구해짐. joystick.py -> robot_walking.py -> gaitPlanner.py 의 경로를 거침
        # 회전변환에 쓰임
        # degree로 들어오는걸 radian 으로 변환함
        c = np.cos(np.deg2rad(angle))#cylindrical coordinates
        s = np.sin(np.deg2rad(angle))
#        if (phi >= 0.75 or phi < 0.25):
#            self.s = True
##            print('foot DOWN' , self.s , phi)
#            
#        elif (phi <= 0.75 and phi > 0.25):
#            self.s = False
##            print('foot UP', self.s , phi)
            
        
        X = np.abs(V)*c*np.array([-0.05 ,
                                  -0.06 ,
                                  -0.07 , 
                                  -0.07 ,
                                  0. ,
                                  0. , 
                                  0.07 ,
                                  0.07 ,
                                  0.06 ,
                                  0.05 ])
    
        Y = np.abs(V)*s*np.array([ 0.05 ,
                                   0.06 ,
                                   0.07 , 
                                   0.07 ,
                                   0. ,
                                   -0. , 
                                   -0.07 ,
                                   -0.07 ,
                                   -0.06 ,
                                   -0.05 ])
    
        Z = np.abs(V)*np.array([0. ,
                                0. ,
                                0.05 , 
                                0.05 ,
                                0.05 ,
                                0.06 , 
                                0.06 ,
                                0.06 ,
                                0. ,
                                0. ])
        swingX = 0.
        swingY = 0.
        swingZ = 0.
        for i in range(10): #sum all terms of the curve 
            # 스윙 베지어 커브의 포인트를 구하는 방법으로, 함수 10개의 포인트에 대해 b 함수의 리턴값를 다 더해주는것임.
            swingX = swingX + b(phi_sw,i,X[i]) 
            swingY = swingY + b(phi_sw,i,Y[i])
            swingZ = swingZ + b(phi_sw,i,Z[i])
            
        return swingX, swingY , swingZ


    def stepTrajectory(self , phi , V , angle , Wrot , centerToFoot): #phi belong [0,1), angles in degrees
        # offset 주기를 처리해주기 위한 구문임.
        # robot_walking.py 에서 offset은 4개의 발에 대해 내딛는 타이밍에 대한 offset으로 정의되어있음.
        # offset = np.array([0.5 , 0. , 0. , 0.5]) 라고 되어 있으므로, FR 과 BL이 반주기 오프셋을 가지고 있음
        # 0.7주기 상태에 오프셋 0.5에 의해 1.2 주기로 들어온다고 생각하면, 0.2가 되는 것
        if (phi >= 1):
            phi = phi - 1.
        #step describes a circuference in order to rotate
        # 제한된 원의 반지름, 원의 중심이 body 중심이고 발의 위치를 한 점으로 하는 xy평면상의 원
        r = np.sqrt(centerToFoot[0]**2 + centerToFoot[1]**2) #radius of the ciscunscribed circle
        # body중심에서 발을 바라본 각도
        footAngle = np.arctan2(centerToFoot[1],centerToFoot[0]) 
        
        # Wrot이란 조이스틱 오른쪽 키의 x값(R3[0], joystick.py에서 확인 가능)에서 들어오는 몸통 회전값
        # 원통좌표계 기준이므로 회전각을 양수일때는 90에서 빼주고 음수일때는 270에서 빼줌.
        # alpha는 trotGait 클래스 생성자에 포함된 self 변수로, 값을 계속해서 저장하여 각도 변화량을 구하는데 씀
        if Wrot >= 0.:#As it is defined inside cylindrical coordinates, when Wrot < 0, this is the same as rotate it 180ª
            circleTrayectory = 90. - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrayectory = 270. - np.rad2deg(footAngle - self.alpha)
        
        # phi는 보행의 한 주기를 나타내며, 0부터 1까지의 값을 갖는다. 
        # 0부터 0.5는 발을 stance한 상태에서 뒤로 미는 과정, 0.5부터 1은 베지어 곡선을 따라 스윙하는 과정.
        stepOffset = 0.5
        if phi <= stepOffset: #stance phase
            phiStance = phi/stepOffset #stance하는 총 시간을 한 주기로 보고 새로 phiStance를 정의함. 
            # 스텝 길이 계산
            stepX_long , stepY_long , stepZ_long = self.calculateStance(phiStance , V , angle)#longitudinal step
            # 스텝 회전 계산
            stepX_rot , stepY_rot , stepZ_rot = self.calculateStance(phiStance , Wrot , circleTrayectory)#rotational step
#            print(phi,phiStance, stepX_long)
        else: #swing phase
            phiSwing = (phi-stepOffset)/(1-stepOffset) #마찬가지로 swing하는 총 시간을 한 주기로 보고 새로 phiSwing을 정의함.
            # 스텝 길이 계산
            stepX_long , stepY_long , stepZ_long = self.calculateBezier_swing(phiSwing , V , angle)#longitudinal step
            # 스텝 길이 계산
            stepX_rot , stepY_rot , stepZ_rot = self.calculateBezier_swing(phiSwing , Wrot , circleTrayectory)#rotational step

        # 회전한 각도 저장
        if (centerToFoot[1] > 0):#define the sign for every quadrant 
            if (stepX_rot < 0):
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)   
        else:
            if (stepX_rot < 0):
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)   

        # 발 포인트 생성
        coord = np.empty(3)        
        coord[0] = stepX_long + stepX_rot
        coord[1] = stepY_long + stepY_rot
        coord[2] = stepZ_long + stepZ_rot
        
        return coord 
        
        
    #computes step trajectory for every foot, defining L which is like velocity command, its angle, 
    #offset between each foot, period of time of each step and the initial vector from center of robot to feet.
    def loop(self , V , angle , Wrot , T , offset , bodytoFeet_ ):
        
        if T <= 0.01: 
            T = 0.01
        
        # phi는 보행의 한 주기를 나타내며, 0부터 1까지의 값을 갖는다. 
        # 0부터 0.5는 발을 stance한 상태에서 뒤로 미는 과정, 0.5부터 1은 베지어 곡선을 따라 스윙하는 과정.
        if (self.phi >= 0.99):
            self.lastTime= time.time()
        self.phi = (time.time()-self.lastTime)/T
#        print(self.phi)
        #now it calculates step trajectory for every foot
        step_coord = self.stepTrajectory(self.phi + offset[0] , V , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[0,:]))) #FR
        self.bodytoFeet[0,0] =  bodytoFeet_[0,0] + step_coord[0]
        self.bodytoFeet[0,1] =  bodytoFeet_[0,1] + step_coord[1] 
        self.bodytoFeet[0,2] =  bodytoFeet_[0,2] + step_coord[2]
    
        step_coord = self.stepTrajectory(self.phi + offset[1] , V , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[1,:])))#FL
        self.bodytoFeet[1,0] =  bodytoFeet_[1,0] + step_coord[0]
        self.bodytoFeet[1,1] =  bodytoFeet_[1,1] + step_coord[1]
        self.bodytoFeet[1,2] =  bodytoFeet_[1,2] + step_coord[2]
        
        step_coord = self.stepTrajectory(self.phi + offset[2] , V , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[2,:])))#BR
        self.bodytoFeet[2,0] =  bodytoFeet_[2,0] + step_coord[0]
        self.bodytoFeet[2,1] =  bodytoFeet_[2,1] + step_coord[1]
        self.bodytoFeet[2,2] =  bodytoFeet_[2,2] + step_coord[2]

        step_coord = self.stepTrajectory(self.phi + offset[3] , V , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[3,:])))#BL
        self.bodytoFeet[3,0] =  bodytoFeet_[3,0] + step_coord[0]
        self.bodytoFeet[3,1] =  bodytoFeet_[3,1] + step_coord[1]
        self.bodytoFeet[3,2] =  bodytoFeet_[3,2] + step_coord[2]
#            

        return self.bodytoFeet
