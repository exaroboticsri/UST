import serial
import time 
import threading
import ctypes
import math

import os
from enum import IntEnum
import numpy as np
from cmath import pi
class feedbackId(IntEnum):
    EncoderData = 1
    

class CExaRobot():
    def __init__(self, port = '/dev/robot'):
        self.com = serial.Serial(port, baudrate=115200)
        self.com_lock = threading.Lock()
        self.send_lock = threading.Lock()

        self.thread_read = threading.Thread(target=self.PacketParser)
        self.thread_read.start()


        self.wheelRadius = 0.08255
        self.wheelBase = 0.38 

        self.prevTime=0.0
        self.prevEncoderR=0.0
        self.prevEncoderL=0.0
        self.prevTheta = 0.0
        #odom
        self.theta = 0.0
        self.poseX = 0.0
        self.poseY = 0.0

        self.linearVel = 0.0
        self.angleVel = 0.0
        self.velR = 0.0
        self.velL = 0.0

        self.dt  = 0.0

    def sendPacket(self, payload):
        buf = bytearray()

        buf.append(0x02)
        buf.append(0x5b)
      
        for i in range(0,len(payload)):
            buf.append(payload[i])
        
        buf.append(0x5D)
        buf.append(0x03)
        # print('send', buf.hex())
        # self.com_lock.acquire()
        self.com.write(buf)
        # self.com_lock.release()
    
    def sendEncoderRead(self):
        readTime = 50 #ms
        data = bytearray()
        data.append(0xF5)# mode
        data.append(0)
        data.append(0x01)
        data.append((readTime>>8)&0xFF)
        data.append((readTime>>0)&0xFF)
        self.sendPacket(data)


    def SetMoveControl(self, velCenter, yawRate): # m/s , rad/s
        wL = (velCenter - (self.wheelBase / 2) * yawRate) / self.wheelRadius
        wR = (velCenter + (self.wheelBase / 2) * yawRate) / self.wheelRadius
        rpmL = int((wL / (2 * pi)) * 60)
        rpmR = int((wR / (2 * pi)) * 60)
        
        data = bytearray()
        data.append(0xF3)# mode
        data.append((rpmL>>8)&0xFF)
        data.append((rpmL>>0)&0xFF)
        data.append((rpmR>>8)&0xFF)
        data.append((rpmR>>0)&0xFF)
        self.sendPacket(data)

    def ProcEncoder(self, data):
        encoderL = (data[3]<<24) | (data[4]<<16) | (data[5]<<8) | (data[6]<<0) 
        encoderR = (data[7]<<24) | (data[8]<<16) | (data[9]<<8) | (data[10]<<0) 
        # print(encoderR, encoderL)
        pulse2m = 7905.0
        now = time.time()
        dt = now - self.prevTime
        
        _encoder_R = encoderR - self.prevEncoderR
        _encoder_L = encoderL - self.prevEncoderL 

        if(abs(_encoder_L) > 20000 ):
            _encoder_L = 0

        if(abs(_encoder_R) > 20000 ):
            _encoder_R = 0

        velL = float((_encoder_L / pulse2m)/dt) #m/s
        velR = float((_encoder_R / pulse2m)/dt) #m/s
        velA = float(((velR-velL)/(self.wheelBase))) #rad/s
        

        vel = (velL+velR)/2.
        # print(round(vel,3), round(dt,3))

        # print(round(vel,3), round(velA,3))
        self.theta = self.theta + velA*dt
        self.angleVel = velA 
        self.linearVel = vel
        self.velL = velL
        self.velR = velR
        
        self.prevTime = now
        self.prevEncoderL = encoderL
        self.prevEncoderR = encoderR
        self.dt = dt

        

    def PayloadParser(self, data):
            if(data[2] == 0xF5):
                self.ProcEncoder(data)
                self.CalCoordinate(self.linearVel, self.angleVel, self.theta, self.dt)
            else:
                pass
        
    def PacketParser(self):
        fStep = 0
        readBuf = bytearray()
        while(1):
            # self.com_lock.acquire()
            c = int.from_bytes(self.com.read(), byteorder='big', signed=False)
            
            if (fStep == 0 and c == 0x02):
                readBuf.append(c) #STX
                fStep = 1
            elif (fStep == 1 and c == 0x5B):
                readBuf.append(c) #STX
                fStep = 2
            elif (fStep == 2):
                readBuf.append(c) #DATA, ETX
                if(len(readBuf)> 3):
                    if(readBuf[-1] == 0x03 and readBuf[-2] == 0x5D): # if get ETX
                        self.PayloadParser(readBuf) # paring data
                        readBuf.clear()
                        fStep == 0 
            else:
                time.sleep(0.0005)
                pass
            
            # self.com_lock.release()

    def GetRobotData(self):
        x = self.poseX 
        y = self.poseY 
        vel = self.linearVel
        theta = self.theta
        angleVel = self.angleVel
        return x, y, vel, theta, angleVel
     
    def CalCoordinate(self, velocity, omega, theta, dt):
        method = 1
        
        prevTheta = self.prevTheta
        if method==0:
            deltaX = (velocity * math.cos(theta))*dt
            deltaY = (velocity * math.sin(theta))*dt

            self.poseX += round(deltaX,10)*-1.0
            self.poseY += round(deltaY,10)
        elif method==1:
            if omega == 0 and velocity != 0:
                deltaX = velocity * dt * math.cos(theta + ((omega*dt)/2))
                deltaY = velocity * dt * math.sin(theta + ((omega*dt)/2))
           
                self.poseX += round(deltaX,10)
                self.poseY += round(deltaY,10)
            elif omega != 0 and velocity != 0:
                deltaX = velocity/omega * (math.sin(theta)-math.sin(prevTheta)) 
                deltaY = velocity/omega * (math.cos(theta)-math.cos(prevTheta))*-1.
   
                self.poseX += round(deltaX,10)
                self.poseY += round(deltaY,10)
             
            else:
                pass
         
            self.prevTheta = theta
        # print(round(self.poseX,3), round(self.poseY,3), round(self.theta, 3))
if __name__ == '__main__':
    robot = CExaRobot()
    robot.sendEncoderRead()
    while(1):
        time.sleep(1)
        robot.SetMoveControl(0.00, 0.0)
      
