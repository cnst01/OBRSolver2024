#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from modules.intersection import Intersection
from modules.finishline import FinishLine

ev3 = EV3Brick()
if __name__ == "__main__":
    g_values = [[[-14.54, 5.885000000000002, 5.414999999999999], [25.46, 45.88500000000001, 45.415]], [[-15.24, 5.695, 1.414999999999999], [24.76, 45.69500000000001, 41.415]]]
    i = Intersection(ColorSensor(Port.S1),ColorSensor(Port.S2), g_values)
    g_values = [i.getGreenValues('left'), i.getGreenValues('right')]
    print(g_values)
    while True:
        if i.checkGreen(g_values) != [False, False]:
            ev3.speaker.beep()    
            wait(1000)
