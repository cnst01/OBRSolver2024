#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math

class Claw:
    def __init__(self,Arm,Hand):
        self.arm = Motor(Arm)
        self.hand = Motor(Hand)
    def up(self, speed = 500):
        self.arm.run(speed)
        wait(2000)
        self.arm.stop()
        print("up")
    def down(self, speed = -500):
        self.arm.run(speed)
        wait(2000)
        self.arm.stop()
        print("down")
    def slowdown(self, speed = -250):
        self.arm.run(speed)
        wait(2000)
        self.arm.stop()
        print("slowdown")
    def open(self, speed = -250):
        self.hand.run(speed)
        wait(1000)
        self.hand.stop()
        print("open")
    def close(self, speed = 250):
        self.hand.run(speed)
        wait(1000)
        self.hand.stop()
        print("close")
    def pickUp(self):
        self.down()
        wait(1500)
        self.close()
        self.up()
    def release(self):
        self.open()
        self.slowdown()
        wait(1000)
        self.up()
