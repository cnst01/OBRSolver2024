from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath as math
from displays import executionDisplay, desviarObsDisplay, recoveryTaskDisplay, intersectionSolverDisplay, axisCorrectionDisplay, calibrateRightDisplay, calibrateLeftDisplay, proportionalAlignDisplay
from MotorPair import MotorPair
from AxisCorrection import axis_correction
from ProportionalAlign import proportionalAlign
from IntersectionSolver import Intersection
from RecoveryTask import recoveryTask
from DesviarObs import desviarObs

hub = PrimeHub(broadcast_channel=1)   
display = hub.display#defining the display object
yaw = hub.imu#defining the angle object of the hub

class myMap:
    def __init__ (self,start_pos):
        self.start_pos = start_pos
        pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
        self.points = [pos]

    def addPoint(self,point):
        pos = [point[0], point[1], point[2]]
        if not pos in self.points:
            self.points += [pos]

class Robot: #resgate antigo(a ser alterado)
    def __init__ (self,motors,force_sensor_port = None,position = [0,0,0]):
        self.position = position
        self.map = myMap(self.position)
        self.motors = motors
        self.hub = PrimeHub(broadcast_channel=1, observe_channels=[2])
        self.hub.imu.reset_heading(0)
        if force_sensor_port:
            self.force_sensor = ForceSensor(force_sensor_port)
     
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

    def pointTo(self, degrees, precision = 4):
        dir = self.position[2]
        # print(range(degrees - precision, degrees + precision))
        if(dir < degrees):
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                print(self.hub.imu.heading())
                self.motors.start_tank(-200,200)
            self.motors.stop_tank()
        else:
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                self.motors.start_tank(200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = self.hub.imu.heading()
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
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                print(self.hub.imu.heading())
                self.motors.start_tank(-200,200)
            self.motors.stop_tank()
        else:
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                self.motors.start_tank(200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = self.hub.imu.heading()
        self.map.points[-1][2] = self.position[2]


    def calibrateDirTo(self, goal, precision = 1):
        dir = self.position[2]
        if dir < goal:
            while not self.hub.imu.heading() == goal:
                real_value = self.hub.imu.heading()
                diff = goal - real_value
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        else:
            while not self.hub.imu.heading() == goal:
                real_value = self.imu.heading()
                diff = real_value - goal
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        real_value = self.hub.imu.heading()
        print('dir calibrated to ' + str(real_value) + ' degrees, the goal was ' + str(goal))
        self.position[2] = self.hub.imu.heading()
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

def FindSafe(areas): #resgate antigo(a ser alterado)
    pos_areas = areas
    if [PontoInicial[0],PontoInicial[1]] in pos_areas:
        print("estou aqui")
        pos_areas.pop(pos_areas.index([PontoInicial[0],PontoInicial[1]]))
    if [out[0],out[1]] in pos_areas:
        pos_areas.pop(pos_areas.index([out[0],out[1]]))
    for area in pos_areas:
        robo.pointToaPoint(area[0], area[1])
        wait(500)
        u_value = u2.distance()
        if u_value > 50 and u_value < 350:
            hub.speaker.beep
            print('Safe on:' + str(area))
            return area
    return False

def resgate(): #resgate antigo(a ser alterado)
    real_angle = robo.hub.imu.heading()
    robo.position[2] = real_angle
    robo.pointTo(PontoInicial[2])
    robo.motors.move_tank(2000,250,250)
    robo.goTo(30,40)
    robo.back_goTo(60,55)
    hub.ble.broadcast(0) #claw pickup
    wait(1000)
    hub.ble.broadcast(2) #claw reset
    wait(2000)
    robo.back_goTo(Center[0],Center[1])
    safe = FindSafe(AreaResgate)
    if not safe:
        robo.goTo(Center[0],Center[1])
        robo.goTo(out2[0], out2[1])
        robo.goTo(out[0], out[1])
        robo.pointTo(out[2])
    else:
        robo.back_goTo(safe[0], safe[1])
        wait(1000)
        hub.ble.broadcast(1) #claw release
        wait(1000)
        hub.ble.broadcast(2) #claw reset
        wait(2000)
        robo.goTo(Center[0],Center[1])
        robo.goTo(out2[0], out2[1])
        robo.goTo(out[0], out[1])
        robo.pointTo(out[2])
        robo.motors.move_tank(1250,250,250)
    print(robo.map.points)

def updateLog(log):
    global logs
    if len(log) != 4:
        return False
    if log != logs[-1]:
        logs.append(log)
        return True

class FinishLine: #um objeto inteiro só pra ver o vermelho(meio desnecessário, a ser alterado)
    def __init__(self, se, sd):
        self.sd = sd
        self.se = se
        self.values = [[[330, 40, 62], [370, 110, 102]], [[329, 40, 63], [369, 98, 103]]]
    def getRedValues(self,side):
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
        self.values = hsv_values
        return hsv_values
    def checkRed(self):
        valuesE = self.values[0]
        valuesD = self.values[1]
        sensor_d = self.sd.hsv()
        sensor_e = self.se.hsv()
        if sensor_d.h > valuesD[0][0] and sensor_d.h < valuesD[1][0]:
            if sensor_d.s > valuesD[0][1] and sensor_d.s < valuesD[1][1]:
                if sensor_d.v > valuesD[0][2] and sensor_d.v < valuesD[1][2]:
                    return True
        if sensor_e.h > valuesE[0][0] and sensor_e.h < valuesE[1][0]:
            if sensor_e.s > valuesE[0][1] and sensor_e.s < valuesE[1][1]:
                if sensor_e.v > valuesE[0][2] and sensor_e.v < valuesE[1][2]:
                    return True
        return False   

def checarResgate(u_value):
    return False
    r = False
    if u_value > 700 and u_value < 0:
        motors.move_tank(500,-250,250)
        if u2.distance() < 1000:
            motors.move_tank(700,250,-250)
            r = True
        else:
            motors.move_tank(1000,250,-250)
            if u2.distance() < 1000:
                r = True
            motors.move_tank(500,-250,250)
        if r:
            motors.stop_tank()
            hub.speaker.beep()
            resgate()    
            return True
    elif u_value < 100:
        # hub.speaker.beep()
        # motors.stop_tank()
        # move_side = 'right'
        # desviarObs()
        print("obs")
    return False

# defining motors
motors = MotorPair(Port.A,Port.B)

# defining sensors
u2 = UltrasonicSensor(Port.E)
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)

#creating the log list and the corner variable
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
corner = 0
logs = [[name,move_side,log, corner]]


#creating the mode variable to use it later to choose the robot mode between calibrate mode and execution mode 
mode = ""

#defining values 
green_values = [[[144.18, 40, 30], [190, 100, 92.26]], [[146.77, 40, 30], [190, 100, 93.105]]]
PontoInicial = [20,30,0]
Center = [60,65]
AreaResgate = [[20,30],[20,100],[105,30],[115,80]]
out = [115,100,0]
out2 = [110,65]
safe = None
set_point_c = 28
set_point_s = 60
timeout_s = 800
timeout_c = 900
max_corner = 3
kP = 3
set_point_i1 = 50 
set_point_i2 = 80
set_point_r = 10
set_point_p = 25
set_point_gap = 75

#creating objects
i = Intersection(se, sd, green_values, motors)
red = FinishLine(se, sd)
robo = Robot(motors, None, [PontoInicial[0],PontoInicial[1], 0])

time_recovery = 1

#main loop
if __name__ == "__main__":
    while True:
        if hub.buttons.pressed() == {Button.LEFT} : #if the left button were pressed, start the execution mode
            mode = "execution"
        if hub.buttons.pressed() == {Button.RIGHT} : #if the right button were pressed, start the calibrate mode
            mode = "calibrate"
        if mode == "calibrate": #if the actual mode is calibrate, then:
            print("------calibrando------") #debug
            leftValues = i.getGreenValues("left") #set the variable leftValues with the function getGreenValues(Correct placement of the robot is necessary to get correct values for the left sensor)
            rightValues = i.getGreenValues("right") #set the variable rightValues with the function getGreenValues(Correct placement of the robot is necessary to get correct values for the right sensor)
            green_values = [leftValues, rightValues] #update the green_values array to the new values got with the intersection object 
            print(green_values)#debug
            display.off()#turn off the display to show that the mode has restarted
            mode = ""#set the mode to blank after the calibrate is done
        if mode == "execution": #if the actual mode is execution, then:
            executionDisplay() #set the display to show an "E"w
            u_value = u2.distance() # constantly get the distance value
            while checarResgate(u_value) == False: #while the robot isn't in rescue zone, then:
                if red.checkRed():
                    motors.stop_tank()
                    hub.speaker.play_notes(["E4/4", "G4/4", "A4/4"])
                    wait(120)
                    hub.speaker.play_notes(["E4/4", "G4/4"])
                    hub.speaker.play_notes(["Bb4/4","A4/4"], 240)
                    mode = ''
                    break
                print(u_value)
                print(logs[-1]) #debug for showing the logs every second 
                sensor_values = str(se.reflection()) + ',' + str(sc.reflection()) + ',' + str(sd.reflection()) #sets a variable to show the updated sensor values
                print(sensor_values) #debug for showing the values of the sensor every second
                u_value = u2.distance() #constantly get the distance value
                se_value = se.reflection() #constantly get the left sensor value
                sd_value = sd.reflection() #constantly get the right sensor value 
                sc_value = sc.reflection() #constantly get the middle sensor value
                if se.reflection() > 28 and sd.reflection() > 28 and sc.reflection() < 15: #if right-left sensors values are bigger then 50(if they are seeing white), and middle value is smaller then 55(if its seeing black), then(if the robot is in line):
                    updateLog(proportionalAlign(se.reflection(), sd.reflection(), kP,set_point_p, robo, motors)) #do proportional align to correct little route errors
                else: #else(if the robot isn't in line), then:
                    valores_verdes = i.checkGreen(green_values) #constantly use the checkGreen function from the Intersection object to return if any of the right-left sensors are seeig green
                    if valores_verdes[0] != False or valores_verdes[1] != False: #if any of the right-left sensors is seeing green, then:
                        updateLog(i.intersectionSolver(valores_verdes,set_point_i1,set_point_i2))# do intersection solver
                    if se.reflection() > set_point_gap and sd.reflection() > set_point_gap and sc.reflection() > set_point_gap: #if every sensor is seeing white, then:
                        if logs[-1][0] == 'proportional align': #if the last task was proportional align(if the robot were in line before seeing all white), then:
                            motors.move_tank(1600,200,200)
                            updateLog(["gap", 'None', "succeded",logs[-1][3]]) #it's a gap(uptade the log to a gap case)
                        else: #if the last task wasn't proportional align(something is wrong), then:
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
                            if se.reflection() > 45 and sd.reflection() > 45 and sc.reflection() < 66: #if the robot is in line, then:
                                updateLog(proportionalAlign(se,sd,kP,set_point_p)) #do proportional align 
                            else: #if the robot isn't in line, then:
                                print('back until see black') #debug
                                motors.move_tank(1000, -200, -200) #go back until see black
                                if se.reflection() > 60 and sd.reflection() > 60 and sc.reflection() > 60:
                                    motors.move_tank(1000, -200, -200)
                                uptadeLog(logs[-1][0],logs[-1][1],logs[-1][2],0)
                                updateLog(axis_correction(logs[-1][0],logs[-1][1], logs[-1][3],set_point_c,set_point_s,timeout_s,timeout_c,max_corner, sd, se, motors )) # do axis correction after it returns
                        else: #else, if both left-right are seeing a value higher then 30, then:
                            if se_value > 50 and sd_value > 50:
                                print('axis correction no branco') #debug
                                updateLog(["Axis Correction no branco",move_side,log])
                            updateLog(axis_correction(logs[-1][0],logs[-1][1],logs[-1][3],set_point_c,set_point_s,timeout_s,timeout_c,max_corner, sd, se, motors)) #do axis correction
