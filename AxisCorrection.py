from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay

hub = PrimeHub()

def axis_correction(last_move, move_side, corner_log ,set_point_c ,set_point_s, timeout_s, timeout_c, max_corner, sd, se, motors):
    timer = StopWatch()
    axisCorrectionDisplay()
    name = ''
    corner = corner_log
    print(corner)
    log = ''
    if last_move != "axis correction **Corner**" and last_move != "axis correction **Suave**":
        corner = 0
    if corner >= max_corner:
        corner = 0 
        motors.stop_tank()
        if sd.reflection() < set_point_s:
            timer.reset()
            while sd.reflection() < set_point_s:
                motors.start_tank(-200,27)
                if timer.time() >= timeout_s:
                    motors.stop_tank()
                    return ["axis correction **Suave**", 'right', 'failed']
            motors.stop_tank()
            move_side = 'right'
        elif se.reflection() < set_point_s : 
            timer.reset()
            while se.reflection() < set_point_s:
                motors.start_tank(27,-200)
                if timer.time() >= timeout_s:
                    motors.stop_tank()
                    return ["axis correction **Suave**",'left','failed']
            motors.stop_tank()
            move_side = 'left'
        if corner == 6:
            corner == 0
        name = "axis correction **Suave**"
        log = 'succeded'
    else:
        if sd.reflection() > se.reflection():
            timer.reset()
            while sd.reflection() > set_point_c:
                motors.start_tank(275,-25)
                move_side = 'right'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            corner += 1
        else:
            timer.reset()
            while se.reflection() > set_point_c:
                motors.start_tank(-25,275)
                move_side = 'left'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    return [name, move_side, log, corner]
