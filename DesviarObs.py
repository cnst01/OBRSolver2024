from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay

hub = PrimeHub()

def desviarObs(lado = 'left'):
    desviarObsDisplay()
    if lado == 'right':
        print("here")
        if name == 'axis correction **Corner**':
            motors.move_tank(1000, -400, 400)
            motors.stop_tank()  
            motors.move_tank(1600, 500, 200)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1400, 500, 130)
            motors.move_tank(300, 100, 100)
        else:
            motors.move_tank(1000, -400, 400)
            motors.stop_tank()  
            motors.move_tank(1850, 500, 250)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1850, 500, 180)
        while se.reflection() > 90 and sd.reflection() > 90 :
            motors.start_tank(225, 90)
        motors.stop_tank()
        wait(1000)
        return [name, lado, 'succeded']
    elif lado == 'left':
        print("here")
        if name == 'axis correction **Corner**':
            motors.move_tank(1000, 400, -400)
            motors.stop_tank()  
            motors.move_tank(1600, 200, 500)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1400, 130, 500)
            motors.move_tank(300, 100, 100)
        else:
            motors.move_tank(1000, 400, -400)
            motors.stop_tank()  
            motors.move_tank(1850, 250, 500)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1850, 180, 500)
        while se.reflection() > 90 and sd.reflection() > 90 :
            motors.start_tank(90, 225)
        motors.stop_tank()
        wait(1000)
        return [name, lado, 'succeded']
    return [name, lado, 'failed']   
