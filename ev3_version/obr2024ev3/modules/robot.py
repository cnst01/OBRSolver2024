#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math

ev3 = EV3Brick()

class myMap:
    def __init__ (self,start_pos):
        self.start_pos = start_pos
        pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
        self.points = [pos]

    def addPoint(self,point):
        pos = [point[0], point[1], point[2]]
        if not pos in self.points:
            self.points += [pos]

class Robot:
    def __init__ (self,motors,force_sensor_port = None,position = [0,0,0]):
        self.position = position
        self.map = myMap(self.position)
        self.motors = motors
        self.hub = EV3Brick()
        if force_sensor_port:
            self.force_sensor = TouchSensor(force_sensor_port)
     
    def pointToaPoint(self,x,y):
        x = int(x + (-1*self.position[0]) )
        y = int(y + (-1*self.position[1]) )
        # dist = math.sqrt(x**2 + y**2)
        if x != 0 and y > 0:
            self.pointTo(int(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.pointTo(90 + int(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.pointTo(-90 - int(abs(math.degrees(math.atan(y/x)))))
        return 1

    def pointTo(self, degrees, precision = 5):
        dir = self.position[2]
        # print(range(degrees - precision, degrees + precision))
        if(dir < degrees):
            diff = degrees - dir
            self.motors.move_angle(diff*5,-200,200)
            self.motors.stop_tank()
        else:
            diff = dir - degrees
            self.motors.move_angle(diff*5,200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = degrees
        self.map.points[-1][2] = self.position[2]
    
    def back_pointTo(self, degrees, precision = 4):
        dir = self.position[2]
        if degrees < 0:
            degrees += 180
        else:
            degrees -= 180
        # degrees = degrees * -1 
        # print(range(degrees - precision, degrees + precision))
        if(dir < degrees):
            diff = degrees - dir
            self.motors.move_angle(diff*5,-200,200)
            self.motors.stop_tank()
        else:
            diff = dir - degrees
            self.motors.move_angle(diff*5,200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = degrees
        self.map.points[-1][2] = self.position[2]

    def moveX(self,q):
        d = int(q * 25.66)
        if q:
            if q > 0:
                self.pointTo(90)
            else:
                self.pointTo(-90)
            self.motors.move_angle(abs(d), 200, 200)
            print('moving ' + str(q) + ' on X')
            self.position[0] += q
            self.map.addPoint(self.position)

    def moveY(self,q):
        d = int(q * 25.66)
        if q :
            if q > 0:
                self.pointTo(0)
            else:
                self.pointTo(178)
            self.motors.move_angle(abs(d),200, 200)
            print('moving ' + str(q) + ' on Y')
            self.position[1] += q
            self.map.addPoint(self.position)
    
    def back_goTo(self,x,y):
        x = int( x + (-1*self.position[0]) )
        y = int( y + (-1*self.position[1]) )
        dist = math.sqrt(x**2 + y**2)*25.66
        if x != 0 and y > 0:
            self.back_pointTo(int(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.back_pointTo(90 + int(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.back_pointTo(-90 - int(abs(math.degrees(math.atan(y/x)))))
        elif x == 0 and y != 0:
            if y > 0:
                self.back_pointTo(0)
            else:
                self.back_pointTo(180)
        elif x != 0 and y == 0:
            if x > 0:
                self.back_pointTo(90)
            else:
                self.back_pointTo(-90)
        else:
            return 0
        self.motors.move_angle(dist,-200, -200)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

    def goTo(self,x,y): 
        x = int( x + (-1*self.position[0]))
        y = int( y + (-1*self.position[1]))
        print(str(x) + "," + str(y))
        dist = math.sqrt(x**2 + y**2)*25.66
        if x != 0 and y > 0:
            self.pointTo(int(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.pointTo(90 + int(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.pointTo(-90 - int(abs(math.degrees(math.atan(y/x)))))
        elif x == 0 and y != 0:
            self.moveY(y)
            return 1
        elif x != 0 and y == 0:
            self.moveX(x)
            return 1
        else:
            return 0
        self.motors.move_angle(dist,200, 200)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

    def doRoute(self, pointlist, goandback = False, back = False):
        for point in pointlist:
            if back:
                self.back_goTo(point[0],point[1])    
            else:
                self.goTo(point[0],point[1])
        if goandback == True:
            lista = self.map.points.copy()
            lista.reverse()
            self.doRoute(lista, False, back)