#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

ev3 = EV3Brick()

class MotorPair:
    def __init__(self, port1, port2):
        self.motor1 = Motor(port1)
        self.motor2 = Motor(port2)
        self.timer = StopWatch()
    def move_angle(self,amount,speed1,speed2, timeout = 1000):
        self.motor1.reset_angle(0)
        self.motor2.reset_angle(0)
        self.timer.reset()
        while abs(self.motor1.angle()) < amount or self.timer.time() < timeout:
            while abs(self.motor2.angle()) < amount or self.timer.time() < timeout:
                self.motor1.run(speed1)
                self.motor2.run(speed2)
        self.motor1.stop()
        self.motor2.stop()
        return "succeded"
    def move_tank(self,amount, speed1, speed2):    
        self.motor1.run(speed1)
        self.motor2.run(speed2)
        wait(amount)
        self.motor1.stop()
        self.motor2.stop()
    def start_tank(self, speed1, speed2):
        self.motor1.run(speed1)
        self.motor2.run(speed2)           
    def stop_tank(self):
        self.motor1.hold()
        self.motor2.hold()