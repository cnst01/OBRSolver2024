#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from modules.motorpair import MotorPair
from modules.intersection import Intersection
from modules.finishline import FinishLine
from modules.robot import Robot
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

def updateLog(log):
    global logs
    if len(log) != 3:
        return False
    if log != logs[-1]:
        logs.append(log)
        return True
#creating the axis correction function
def axis_correction(last_move, set_point_c , set_point_s, timeout_s, timeout_c, max_corner):
    timer = StopWatch()
    #axisCorrectionDisplay()
    global corner
    global logs 
    name = ''
    move_side = logs[-1][1]
    log = ''
    if last_move != "axis correction **Corner**" and last_move != "axis correction **Suave**":
        corner = 0
    if corner >= max_corner:
        corner += 1
        motors.stop_tank()
        if sd.reflection() < set_point_s:
            timer.reset()
            while sd.reflection() < set_point_s:
                motors.start_tank(-150,0)
                if timer.time() >= timeout_s:
                    motors.stop_tank()
                    return ["axis correction **Suave**", 'right', 'failed']
            move_side = 'right'
        elif se.reflection() < set_point_s : 
            timer.reset()
            while se.reflection() < set_point_s:
                motors.start_tank(0,-150)
                if timer.time() >= timeout_s:
                    motors.stop_tank()
                    return ["axis correction **Suave**",'left','failed']
            move_side = 'left'
        if corner == 5:
            corner == 0
        name = "axis correction **Suave**"
        log = 'succeded'
    else:
        if sd.reflection() > se.reflection():
            timer.reset()
            while sd.reflection() > set_point_c:
                motors.start_tank(200,-50)
                move_side = 'right'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            motors.stop_tank()
            corner += 1
        else:
            timer.reset()
            while se.reflection() > set_point_c:
                motors.start_tank(-50,200)
                move_side = 'left'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            motors.stop_tank()
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    return [name, move_side, log]

#creating the proportional align function
def proportionalAlign(se, sd, kP,set_point):
    name ='proportional align'
    move_side = ''
    log='failed'
    errorE = se.reflection() - set_point
    errorD = sd.reflection() - set_point
    leftMotorSpd = 50 + (errorD * kP)
    rightMotorSpd = 50 + (errorE * kP)
    motors.start_tank(leftMotorSpd,rightMotorSpd)
    diff_l_r = leftMotorSpd - rightMotorSpd
    if diff_l_r > 0:
        move_side = 'right'
    else:
        move_side = 'left'
    log = 'succeded'
    return [name, move_side, log]

#creating recovery task function
def recoveryTask(set_point):
    print("recovery task")
    global logs
    timer = StopWatch()
    global time_recovery
    timeout = 1600 * time_recovery
    last_task = logs[-1]
    ltName = last_task[0] #defining a variable for the last task name
    ltMoveSide = last_task[1] #defining a variable for the move side of the last task
    isMoveSide = ''
    print("ltName: " + str(ltName))
    if ltMoveSide == 'right':
        isMoveSide = 'left'
    if ltMoveSide == 'left':
        isMoveSide = 'right'
    name = 'recovery task'
    move_side = ltMoveSide
    log = 'failed'
    if ltName == "axis correction **Corner**" or ltName == "axis correction **Suave**" or ltName == 'recovery task': #if last task was axis correction, then:
        print(isMoveSide)
        if isMoveSide == "left": #if last task side was right, then:
            timer.reset()
            while se.reflection() > set_point:
                motors.start_tank(-300,300)
                if timer.time() >= timeout:
                    motors.stop_tank()
                    time_recovery += 1
                    return [name, "left", "failed"]
            move_side = "left"
            log = "succeded"
            time_recovery = 1
            motors.stop_tank()
        elif isMoveSide == "right": #if last task side was left, then:
            timer.reset()
            while sd.reflection() > set_point:
                motors.start_tank(300,-300)
                if timer.time() >= timeout*2:
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
        motors.move_tank(2000,-200,-200)
    if ltName == "intersectionSolver": #if last task was intersection solver, then:
        if ltMoveSide == "right": #if last task side was right, then:
            motors.move_tank(1000,200,-200)
        if ltMoveSide == "left": #if last task side was left, then:
            motors.move_tank(1000,-200,200)
    if ltName == "proportional align": #if last task was proportional align, then:
        print(ltName)
    return [name, move_side, log]
#creating a function to avoid the obstacle            
def desviarObs(lado = 'left'):
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
      
def FindSafe(areas):
    pos_areas = areas
    if [PontoInicial[0],PontoInicial[1]] in pos_areas:
        print("estou aqui")
        pos_areas.pop(pos_areas.index([PontoInicial[0],PontoInicial[1]]))
    if [out[0],out[1]] in pos_areas:
        pos_areas.pop(pos_areas.index([out[0],out[1]]))
    for area in pos_areas:
        robot.pointToaPoint(area[0], area[1])
        wait(500)
        u_value = u2.distance()
        if u_value > 50 and u_value < 350:
            ev3.speaker.beep()
            print('Safe on:' + str(area))
            return area
    return False

def resgate():
    robot.pointTo(PontoInicial[2])
    robot.motors.move_tank(3000,250,250)
    robot.back_goTo(45,35)
    robot.back_goTo(Center[0],Center[1])
    safe = FindSafe(AreaResgate)
    if not safe:
        robot.goTo(out[0], out[1])
        robot.pointTo(out[2])
    else:
        robot.back_goTo(safe[0], safe[1])
        robot.goTo(Center[0],Center[1])
        robot.goTo(45,75)
        robot.goTo(out[0], out[1])
        robot.pointTo(out[2])
        robot.motors.move_tank(1000,250,250)
    print(robot.map.points)

#creating a function to detect if the robot is in the rescue zone
def checarResgate(u_value):
    r = False
    if u_value > 300 and u_value < 500:
        motors.move_tank(500,-250,250)
        if u2.distance() < 500 and u2.distance() > 300:
            motors.move_tank(700,250,-250)
            r = True
        else:
            motors.move_tank(1000,250,-250)
            if u2.distance() < 500:
                r = True
            motors.move_tank(500,-250,250)
        if r:
            motors.stop_tank()
            ev3.speaker.beep(20)
            # resgate()    
            return True
    elif u_value < 100:
        ev3.speaker.beep(100)
        motors.stop_tank()
        move_side = 'right'
        desviarObs()
    return False

#defining the general comparation value to the sensors and creating the darkest variable to use it later
darkest = ""

# defining motors
motors = MotorPair(Port.D,Port.A)

# defining sensors
green_values = [[[0, 24.1, 17.4], [8.7, 34.1, 27.4]], [[0, 27.5, 18.3], [8.8, 37.5, 28.3]]]
u2 = UltrasonicSensor(Port.S4)
sc = ColorSensor(Port.S3)
sd = ColorSensor(Port.S2)
se = ColorSensor(Port.S1)
i = Intersection(se, sd, green_values)
red = FinishLine(se, sd)

#creating the log list and the corner variable
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
logs = [name,move_side,log]
corner = 0

#creating the mode variable to use it later to choose the robot mode between calibrate mode and execution mode 
mode = "execution"

#Saídas = [[385,0],[1155,0],[1925,0],[385,2310][1155,2310],[1925,2310],[0,385],[0,1155],[0,1925],[2310,385],[2310,1155],[2310,1925]]
PontoInicial = [45,10,0]
Center = [45,45]
AreaResgate = [[20,20],[20,70],[70,20],[70,70]]
out = [75,85,90]
safe = None
robot = Robot(motors, None, [PontoInicial[0],PontoInicial[1], 0])
set_point_c = 40
set_point_s = 45
timeout_s = 1200
timeout_c = 1500
max_corner = 3
kP = 4
set_point_i1 = 20
set_point_i2 = 20
set_point_r = 20
set_point_p = 15
set_point_gap = 50
safe = None


time_recovery = 1
#main loop
if __name__ == "__main__":
    while True:
        if mode == "calibrate": #if the actual mode is calibrate, then:
            print("------calibrando------") #debug
            leftValues = i.getGreenValues("left") #set the variable leftValues with the function getGreenValues(Correct placement of the robot is necessary to get correct values for the left sensor)
            rightValues = i.getGreenValues("right") #set the variable rightValues with the function getGreenValues(Correct placement of the robot is necessary to get correct values for the right sensor)
            green_values = [leftValues, rightValues] #update the green_values array to the new values got with the intersection object 
            print(green_values)#debug
            mode = ""#set the mode to blank after the calibrate is done
        if mode == "execution": #if the actual mode is execution, then:
            u_value = u2.distance() # constantly get the distance value
            while checarResgate(u_value) == False: #while the robot isn't in rescue zone, then:
                if red.checkRed():
                    motors.stop_tank()
                    ev3.speaker.beep(10)
                    mode = ''
                    break
                print(u_value)
                print(logs[-1],corner) #debug for showing the logs every second 
                sensor_values = str(se.reflection()) + ',' + str(sc.reflection()) + ',' + str(sd.reflection()) #sets a variable to show the updated sensor values
                print(sensor_values) #debug for showing the values of the sensor every second
                u_value = u2.distance() #constantly get the distance value
                se_value = se.reflection() #constantly get the left sensor value
                sd_value = sd.reflection() #constantly get the right sensor value 
                sc_value = sc.reflection() #constantly get the middle sensor value
                if se.reflection() > 25 and sd.reflection() > 25 and sc.reflection() < 30: #if right-left sensors values are bigger then 50(if they are seeing white), and middle value is smaller then 55(if its seeing black), then(if the robot is in line):
                    updateLog(proportionalAlign(se, sd, kP,set_point_p)) #do proportional align to correct little route errors
                else: #else(if the robot isn't in line), then:
                    valores_verdes = i.checkGreen(green_values) #constantly use the checkGreen function from the Intersection object to return if any of the right-left sensors are seeig green
                    if valores_verdes[0] != False or valores_verdes[1] != False: #if any of the right-left sensors is seeing green, then:
                        updateLog(i.intersectionSolver(valores_verdes,set_point_i1,set_point_i2))# do intersection solver
                    if se.reflection() > set_point_gap and sd.reflection() > set_point_gap and sc.reflection() > set_point_gap: #if every sensor is seeing white, then:
                        if logs[-1][0] == 'proportional align': #if the last task was proportional align(if the robot were in line before seeing all white), then:
                            motors.move_tank(1300,200,200)
                            updateLog(["gap", 'None', "succeded"]) #it's a gap(uptade the log to a gap case)
                        else: #if the last task wasn't proportional align(something is wrong), then:
                            print("estou aqui")
                            updateLog(recoveryTask(set_point_r)) #shit, lets try recovery task
                    else: #else, if the robot isn't in line and isn't seeing everything white, then:
                        motors.stop_tank() #stop the motors from moving
                        if se_value < 30 and sd_value < 30: #if both right-left sensors are seeing black, then:
                            motors.move_tank(1000, 200, 200) #move tank during 2000 milliseconds
                            se_value = se.reflection() #update the left sensor value
                            sd_value = sd.reflection() #update the right sensor value
                            sc_value = sc.reflection() #update the middle sensor value
                            sensor_values = str(se_value) + ',' + str(sc_value) + ',' + str(sd_value) #sets a variable to show the updated sensor values
                            print(sensor_values) #debug for showing the values of the sensor every second
                            if se.reflection() > 25 and sd.reflection() > 25 and sc.reflection() < 30: #if the robot is in line, then:
                                updateLog(proportionalAlign(se,sd,kP,set_point_p)) #do proportional align 
                            else: #if the robot isn't in line, then:
                                print('back until see black') #debug
                                motors.move_tank(1000, -200, -200) #go back until see black
                                if se.reflection() > 40 and sd.reflection() > 40 and sc.reflection() > 40:
                                    motors.move_tank(1000, -200, -200)
                                corner = 0
                                updateLog(axis_correction(logs[-1][0],set_point_c,set_point_s,timeout_s,timeout_c,max_corner)) # do axis correction after it returns
                        else: #else, if both left-right are seeing a value higher then 30, then:
                            if se_value > 30 and sd_value > 30:
                                print('axis correction no branco') #debug
                                updateLog(["Axis Correction no branco",move_side,log])
                            updateLog(axis_correction(logs[-1][0],set_point_c,set_point_s,timeout_s,timeout_c,max_corner)) #do axis correction
