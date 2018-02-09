# -*- coding: utf-8 -*-
"""
Created on Thu Jan 11 14:58:53 2018

@author: 牛帥
"""

#import numpy as np
#import matplotlib.pyplot as plt
#from turtle import *

if __name__ == '__main__':
    IN1_R = 8
    IN2_R = 10
    IN3_L = 16
    IN4_L = 18

    PWM_R = 12
    PWM_L = 22

    COUNT_L = 0
    COUNT_R = 0

    robot_r = 7.5
    robot_w = 0.0
    robot_v = 0.0
    wheel_r = 4.25
    wheelDeg_L = 0
    wheelDeg_R = 0
    robotDeg_target = 0
    robotDeg_now = 0
    Kp = 0.23
    t = 2
    W_L = 0
    W_R = 0
    DC_L = 44
    DC_R = 44

    x = 0.0
    y = 0.0
    robot_th = 0.0     #theta is the angle from x





