#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
from modules.motorpair import MotorPair
from modules.robot import Robot

ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()







if __name__ == "__main__":
    # defining motors
    motors = MotorPair(Port.D,Port.A)    
    robot = Robot(motors)
    robot.pointTo(-45)
    robot.back_goTo(20,20)