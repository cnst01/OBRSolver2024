from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay

hub = PrimeHub()

def recoveryTask(set_point):
    global logs
    timer = StopWatch()
    global time_recovery
    timeout = 1600 * time_recovery
    last_task = logs[-1]
    recoveryTaskDisplay() #displaying an "R" to the hub screen
    ltName = last_task[0] #defining a variable for the last task name
    ltMoveSide = last_task[1] #defining a variable for the move side of the last task
    isMoveSide = ''
    if ltMoveSide == 'right':
        isMoveSide = 'left'
    if ltMoveSide == 'left':
        isMoveSide = 'right'
    name = 'recovery task'
    move_side = ltMoveSide
    log = 'failed'
    if ltName == "axis correction **Corner**" or ltName == "axis correction **Suave**" or ltName == 'recovery task': #if last task was axis correction, then:
        if isMoveSide == "left": #if last task side was right, then:
            timer.reset()
            while se.reflection() > set_point:
                motors.start_tank(-300,325)
                if timer.time() >= timeout:
                    motors.stop_tank()
                    time_recovery += 0.9
                    return [name, "left", "failed"]
            move_side = "left"
            log = "succeded"
            time_recovery = 1
            motors.stop_tank()
        elif isMoveSide == "right": #if last task side was left, then:
            timer.reset()
            while sd.reflection() > set_point:
                motors.start_tank(325,-300)
                if timer.time() >= timeout:
                    motors.stop_tank()
                    time_recovery += 1
                    return [name,"right","failed"]
            move_side = "right"
            time_recovery = 1
            log = "succeded"
            motors.stop_tank()
        else:
            motors.move_tank(500,-200,-200)
    if ltName == "gap":
        motors.move_tank(1800,-200,-200)
    if ltName == "intersectionSolver": #if last task was intersection solver, then:
        if ltMoveSide == "right": #if last task side was right, then:
            motors.move_tank(1000,200,-200)
        if ltMoveSide == "left": #if last task side was left, then:
            motors.move_tank(1000,-200,200)
    if ltName == "proportional align": #if last task was proportional align, then:
            print("")
    return [name, move_side, log]
