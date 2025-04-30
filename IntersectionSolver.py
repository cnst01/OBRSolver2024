from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay

hub = PrimeHub()

class Intersection:
    def __init__(self, se, sd, green_values, motors):
        self.se = se
        self.sd = sd
        self.motors = motors
    def intersectionSolver(self, valores, set_point_1, set_point_2):
        se = self.se
        sd = self.sd
        motors = self.motors
        last_values = valores
        intersectionSolverDisplay()
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
                wait(1500)
                motors.stop_tank()
                move_side = 'left'
            else:
                if sd.reflection() > set_point_1 :
                    print('fake right')
                    return [name,'','Failed']
                print('direitinha')
                motors.stop_tank()
                wait(1000)
                motors.start_tank(0,300)
                wait(1500)
                motors.stop_tank()
                move_side = 'right'
        name = 'intersectionSolver'
        log = 'succeded'
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
            if side == "right":
                calibrateRightDisplay(int((x+1)/2))
            if side == "left":
                calibrateLeftDisplay(int((x+1)/2))
            hsv_obj = sensor.hsv()
            hsv_med[0] += hsv_obj.h
            hsv_med[1] += hsv_obj.s
            hsv_med[2] += hsv_obj.v
            print(hsv_med)
        for i in range(3):
            hsv_med[i] = hsv_med[i]/200
            hsv_min[i] = hsv_med[i] - 20
            hsv_max[i] = hsv_med[i] + 20  
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
        sensor_d = self.sd.hsv()
        sensor_e = self.se.hsv()
        direita = False
        esquerda = False
        # print(sensor)
        # if sensor.h < values[0][0] or sensor.h > values[1][0]:
        #     return False
        # if sensor.s < values[0][1] or sensor.s > values[1][1]:
        #     return False
        # if sensor.v < values[0][2] or sensor.v > values[1][2]:
        #     return False
        if sensor_d.h > valuesD[0][0] and sensor_d.h < valuesD[1][0]:
            if sensor_d.s > valuesD[0][1] and sensor_d.s < valuesD[1][1]:
                if sensor_d.v > valuesD[0][2] and sensor_d.v < valuesD[1][2]:
                    direita = True
        if sensor_e.h > valuesE[0][0] and sensor_e.h < valuesE[1][0]:
            if sensor_e.s > valuesE[0][1] and sensor_e.s < valuesE[1][1]:
                if sensor_e.v > valuesE[0][2] and sensor_e.v < valuesE[1][2]:
                    esquerda = True
        return [esquerda, direita]
