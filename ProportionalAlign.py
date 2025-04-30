from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay

hub = PrimeHub()

def proportionalAlign(se, sd, kP,set_point, robo, motors):
    robo.hub.imu.reset_heading(0)
    proportionalAlignDisplay()
    name ='proportional align'
    move_side = ''
    log='failed'
    errorE = se - set_point
    errorD = sd - set_point
    diff = errorD - errorE  
    leftMotorSpd = 150 + (errorD + diff)*kP
    rightMotorSpd = 150 + (errorE - diff)*kP
    motors.start_tank(leftMotorSpd,rightMotorSpd)
    diff_l_r = leftMotorSpd - rightMotorSpd
    if diff_l_r > 0:
        move_side = 'right'
    else:
        move_side = 'left'
    log = 'succeded'
    return [name, move_side, log]
