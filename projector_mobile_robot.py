# -*- coding: utf-8 -*-
"""
Created on Thu Dec 21 10:09:57 2017

@author: 牛帥
"""
import RPi.GPIO as GPIO
import math
import threading
import time
import C

GPIO.setwarnings(False)       #warning
GPIO.setmode(GPIO.BOARD)

def init():
    GPIO.setup(C.IN1_R, GPIO.OUT)
    GPIO.setup(C.IN2_R, GPIO.OUT)
    GPIO.setup(C.PWM_R, GPIO.OUT)  #right motor init
    GPIO.setup(C.IN3_L, GPIO.OUT)
    GPIO.setup(C.IN4_L, GPIO.OUT)
    GPIO.setup(C.PWM_L, GPIO.OUT)  #left motor


def forward():
    pwmR.ChangeDutyCycle(100)
    GPIO.output(C.IN1_R, GPIO.HIGH)
    GPIO.output(C.IN2_R, GPIO.LOW)
    pwmL.ChangeDutyCycle(100)
    GPIO.output(C.IN3_L, GPIO.HIGH)
    GPIO.output(C.IN4_L, GPIO.LOW)


def turnRight():
    global DC_L, DC_R
    pwmR.ChangeDutyCycle(DC_R)
    GPIO.output(C.IN1_R, GPIO.LOW)
    GPIO.output(C.IN2_R, GPIO.HIGH)
    pwmL.ChangeDutyCycle(DC_L)
    GPIO.output(C.IN3_L, GPIO.HIGH)
    GPIO.output(C.IN4_L, GPIO.LOW)


def carStop():
    pwmR.stop()
    pwmL.stop()
    time.sleep(100)

def leftWheel(channel):
    global COUNT_L
    if channel == 13:
        COUNT_L += 1


def rightWheel(channel):
    global COUNT_R
    if channel == 11:
        COUNT_R += 1


def readCount():
    global timer
    global COUNT_L, COUNT_R
    timer = threading.Timer(C.t, readCount)
    timer.start()
    global W_L, W_R
    W_L = ((2*(math.pi)/20)*COUNT_L)/C.t
    W_R = ((2*(math.pi)/20)*COUNT_R)/C.t
    COUNT_L = 0
    COUNT_R = 0


def turnDegree(degree):
    global robotDeg_now
    robotDeg_now = (robotDeg_now*(math.pi))/180
    robotDeg_target = (degree*(math.pi))/180
    while robotDeg_target > robotDeg_now:
        turnRight()
        time.sleep(1)
        global W_L, W_R
        wheel_vl = -W_L*C.wheel_r
        wheel_vr = W_R*C.wheel_r
        C.robot_v = (wheel_vr + wheel_vl)/2
        robot_w = (wheel_vr - wheel_vl)/(2*C.robot_r)
        robot_w_target = C.Kp*(robotDeg_target - robotDeg_now)
        robot_v_target = robot_w_target*C.robot_r
        wheel_vr_target = wheel_vl_target = robot_v_target     #Have problem, why use robot W calculate wheel velocity?
        global DC_L, DC_R
        DC_L = C.Kp*(wheel_vl_target -wheel_vl) + DC_L
        DC_R = C.Kp*(wheel_vr_target -wheel_vr) + DC_R
        robotDeg_now = robotDeg_now +robot_w*C.t
        #print("robot degree now"),
        #print(robotDeg_now)
        if robotDeg_now >= robotDeg_target:
            C.COUNT_L = 0
            C.COUNT_R = 0
            carStop()
            break


GPIO.setup(13, GPIO.IN)
GPIO.setup(11, GPIO.IN)
GPIO.add_event_detect(13, GPIO.RISING, callback=leftWheel, bouncetime=1)
GPIO.add_event_detect(11, GPIO.RISING, callback=rightWheel, bouncetime=1)

try:
    while True:
        init()
        pwmR = GPIO.PWM(C.PWM_R, 50)
        pwmL = GPIO.PWM(C.PWM_L, 50)
        pwmR.start(0)
        pwmL.start(0)
        timer = threading.Timer(C.t, readCount)
        timer.start()
#        global robot_w, robot_v
#        robot_th = robot_th + robot_w*t
#        x = x + robot_v*math.cos(robot_th)
#        y = y + robot_v*math.sin(robot_th)
        turnDegree(180)
        forward()
        time.sleep(1)



except KeyboardInterrupt:
    pass

GPIO.remove_event_detect(13)
GPIO.remove_event_detect(11)
timer.cancel()
GPIO.cleanup()





