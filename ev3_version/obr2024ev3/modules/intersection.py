from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from modules.motorpair import MotorPair
from modules.finishline import FinishLine
from modules.robot import Robot
#creating intersection object
class Intersection:
    def __init__(self, se, sd, green_values):
        self.se = se
        self.sd = sd
    def intersectionSolver(self, valores, set_point_1, set_point_2):
        se = self.se
        sd = self.sd
        last_values = valores
        name = 'intersectionSolver'
        move_side = ''
        while valores[0] != False or valores[1] != False:
            if valores[0] == True and valores[1] == True:
                last_values = valores
                break
            motors.start_tank(100,100)
            wait(200)
            last_values = valores
            valores = self.checkGreen(green_values)
        valores = last_values
        motors.stop_tank()
        if valores[0] == True and valores[1] == True:
            print('dar voltinha')
            if se.reflection() > set_point_1 or sd.reflection() > set_point_1 :
                print('fake double')
                return [name,'','Failed']
            motors.move_tank(1000, 350, 350)
            while se.reflection() > set_point_2 and sd.reflection() > set_point_2 :
                motors.start_tank(-350, 350)
            motors.stop_tank()
            wait(1000)
        else:
            if valores[0] == True:
                if se.reflection() > set_point_1:
                    print('fake left')
                    return [name,'','Failed']
                print('esquerdinha')
                motors.stop_tank()
                wait(1000)
                motors.start_tank(300,0)
                wait(1000)
                motors.stop_tank()
                move_side = 'left'
            else:
                if sd.reflection() > set_point_1 :
                    print('fake right')
                    return [name,'','Failed']
                print('direitinha')
                motors.stop_tank()
                motors.start_tank(0,300)
                wait(1000)
                while se.reflection() > set_point_2 and sd.reflection() > set_point_2 :
                    motors.start_tank(0,300)
                motors.stop_tank()
                wait(1000)
                move_side = 'right'
        name = 'intersectionSolver'
        log = 'succeded'
        ev3.speaker.beep()
        return [name,move_side,log]
    def getGreenValues(self,side):
        hsv_min = [0,0,0]
        hsv_max = [0,0,0]
        hsv_med = [0,0,0]
        if side == 'left':
            sensor = self.se
        elif side == "right":
            sensor = self.sd
        for x in range(200):
            wait(10)
            hsv_obj = sensor.rgb()
            hsv_med[0] += hsv_obj[0]
            hsv_med[1] += hsv_obj[1]
            hsv_med[2] += hsv_obj[2]
            print(hsv_med)
        for i in range(3):
            hsv_med[i] = hsv_med[i]/200
            hsv_min[i] = hsv_med[i] - 5
            hsv_max[i] = hsv_med[i] + 5  
            # if hsv_obj.h < hsv_min[0] or hsv_min[0] == 0 :
            #     hsv_min[0] = hsv_obj.h
            # if hsv_obj.s < hsv_min[1] or hsv_min[1] == 0 :
            #     hsv_min[1] = hsv_obj.s
            # if hsv_obj.v < hsv_min[2] or hsv_min[2] == 0 :
            #     hsv_min[2] = hsv_obj.v
            # if hsv_obj.h > hsv_max[0]:
            #     hsv_max[0] = hsv_obj.h
            # if hsv_obj.s > hsv_max[1]:
            #     hsv_max[1] = hsv_obj.s
            # if hsv_obj.v > hsv_max[2]:
            #     hsv_max[2] = hsv_obj.v
            
            wait(50)
        hsv_values = [hsv_min, hsv_max]
        print(hsv_values)
        return hsv_values
    def checkGreen(self, valores):
        valuesE = valores[0]
        valuesD = valores[1]
        sensor_d = self.sd.rgb()
        sensor_e = self.se.rgb()
        direita = False
        esquerda = False
        # print(sensor)
        # if sensor.h < values[0][0] or sensor.h > values[1][0]:
        #     return False
        # if sensor.s < values[0][1] or sensor.s > values[1][1]:
        #     return False
        # if sensor.v < values[0][2] or sensor.v > values[1][2]:
        #     return False
        if sensor_d[0] > valuesD[0][0] and sensor_d[0] < valuesD[1][0]:
            if sensor_d[1] > valuesD[0][1] and sensor_d[1] < valuesD[1][1]:
                if sensor_d[2] > valuesD[0][2] and sensor_d[2] < valuesD[1][2]:
                    direita = True
        if sensor_e[0] > valuesE[0][0] and sensor_e[0] < valuesE[1][0]:
            if sensor_e[1] > valuesE[0][1] and sensor_e[1] < valuesE[1][1]:
                if sensor_e[2] > valuesE[0][2] and sensor_e[2] < valuesE[1][2]:
                    esquerda = True
        return [esquerda, direita]
