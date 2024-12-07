from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath as math

hub = PrimeHub(broadcast_channel=1)
display = hub.display#defining the display object
yaw = hub.imu#defining the angle object of the hub

#defining display's
def executionDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(4,1)
    display.pixel(4,2)
    display.pixel(4,3)
def calibrateLeftDisplay(brightness):
    display.pixel(0,0,brightness)
    display.pixel(0,1,brightness)
    display.pixel(1,0,brightness)
    display.pixel(1,1,brightness)
    display.pixel(2,0,brightness)
    display.pixel(2,1,brightness)
    display.pixel(3,0,brightness)
    display.pixel(3,1,brightness)
    display.pixel(4,0,brightness)
    display.pixel(4,1,brightness)
def calibrateRightDisplay(brightness):
    display.pixel(0,3,brightness)
    display.pixel(0,4,brightness)
    display.pixel(1,3,brightness)
    display.pixel(1,4,brightness)
    display.pixel(2,3,brightness)
    display.pixel(2,4,brightness)
    display.pixel(3,3,brightness)
    display.pixel(3,4,brightness)
    display.pixel(4,3,brightness)
    display.pixel(4,4,brightness)
def proportionalAlignDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(4,1)
def axisCorrectionDisplay():
    display.off()
    display.pixel(0,2)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,3)
def intersectionSolverDisplay():
    display.off()
    display.pixel(0,2)
    display.pixel(2,2)
    display.pixel(3,2)
    display.pixel(4,2)
def recoveryTaskDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,3)
def desviarObsDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,2)

#creating motor pair object
class MotorPair:
    def __init__(self, port1, port2):
        self.motor1 = Motor(port1)
        self.motor2 = Motor(port2)
        self.timer = StopWatch()
    def move_angle(self,amount,speed1,speed2, timeout = 1000):
        self.motor1.reset_angle(0)
        self.motor2.reset_angle(0)
        self.timer.reset()
        while abs(self.motor1.angle()) < amount or self.timer.time() < timeout:
            while abs(self.motor2.angle()) < amount or self.timer.time() < timeout:
                self.motor1.run(speed1)
                self.motor2.run(-(speed2))
        self.motor1.stop()
        self.motor2.stop()
        return "succeded"
    def move_tank(self,amount, speed1, speed2):    
        self.motor1.run(speed1)
        self.motor2.run(-(speed2))
        wait(amount)
        self.motor1.stop()
        self.motor2.stop()
    def start_tank(self, speed1, speed2):
        self.motor1.run(speed1)
        self.motor2.run(-(speed2))           
    def stop_tank(self):
        self.motor1.hold()
        self.motor2.hold()

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

def FindSafe(areas):
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

def resgate():
    real_angle = robo.hub.imu.heading()
    robo.position[2] = real_angle
    robo.pointTo(PontoInicial[2])
    robo.motors.move_tank(3000,250,250)
    robo.back_goTo(45,35)
    hub.ble.broadcast(0) #claw pickup
    wait(1000)
    hub.ble.broadcast(2) #claw reset
    wait(2000)
    robo.back_goTo(Center[0],Center[1])
    safe = FindSafe(AreaResgate)
    if not safe:
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
        robo.goTo(out[0], out[1])
        robo.pointTo(out[2])
        robo.motors.move_tank(1000,250,250)
    print(robo.map.points)

#creating update log function
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
    axisCorrectionDisplay()
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
                motors.start_tank(-100,50)
                if timer.time() >= timeout_s:
                    motors.stop_tank()
                    return ["axis correction **Suave**", 'right', 'failed']
            move_side = 'right'
        elif se.reflection() < set_point_s : 
            timer.reset()
            while se.reflection() < set_point_s:
                motors.start_tank(50,-100)
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
                motors.start_tank(200,20)
                move_side = 'right'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            corner += 1
        else:
            timer.reset()
            while se.reflection() > set_point_c:
                motors.start_tank(20,200)
                move_side = 'left'
                if timer.time() >= timeout_c:
                    corner += 1
                    return ["axis correction **Corner**", move_side, "failed"]
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    return [name, move_side, log]

#creating the proportional align function
def proportionalAlign(se, sd, kP,set_point):
    proportionalAlignDisplay()
    name ='proportional align'
    move_side = ''
    log='failed'
    errorE = se.reflection() - set_point
    errorD = sd.reflection() - set_point
    diff = errorE - errorD
    leftMotorSpd = 50 + (errorD + diff)*kP
    rightMotorSpd = 50 + (errorE - diff)*kP
    motors.start_tank(leftMotorSpd,rightMotorSpd)
    diff_l_r = leftMotorSpd - rightMotorSpd
    if diff_l_r > 0:
        move_side = 'right'
    else:
        move_side = 'left'
    log = 'succeded'
    return [name, move_side, log]

#creating intersection object
class Intersection:
    def __init__(self, se, sd, green_values):
        self.se = se
        self.sd = sd
    def intersectionSolver(self, valores, set_point_1, set_point_2):
        se = self.se
        sd = self.sd
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

class FinishLine:
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

#creating recovery task function
def recoveryTask(set_point):
    print("recovery task")
    global logs
    timer = StopWatch()
    global time_recovery
    timeout = 1600 * time_recovery
    last_task = logs[-1]
    recoveryTaskDisplay() #displaying an "R" to the hub screen
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

#creating a function to detect if the robot is in the rescue zone
def checarResgate(u_value):
    r = False
    if u_value > 700 and u_value < 930:
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
        hub.speaker.beep()
        motors.stop_tank()
        move_side = 'right'
        desviarObs()
    return False

#defining the general comparation value to the sensors and creating the darkest variable to use it later
darkest = ""

# defining motors
motors = MotorPair(Port.A,Port.B)

# defining sensors
green_values = [[[144.18, 59.945, 52.26], [184.18, 99.945, 92.26]], [[146.77, 56.265, 53.105], [186.77, 96.265, 93.105]]]
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)
i = Intersection(se, sd, green_values)
red = FinishLine(se, sd)

#creating the log list and the corner variable
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
logs = [name,move_side,log]
corner = 0

#creating the mode variable to use it later to choose the robot mode between calibrate mode and execution mode 
mode = ""

PontoInicial = [45,10,0]
Center = [45,45]
AreaResgate = [[20,20],[20,70],[70,20],[70,70]]
out = [75,45,90]
safe = None
set_point_c = 28
set_point_s = 65
timeout_s = 800
timeout_c = 900
max_corner = 3
kP = 4
set_point_i1 = 50 
set_point_i2 = 80
set_point_r = 40
set_point_p = 77
set_point_gap = 65
safe = None
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
                print(logs[-1],corner) #debug for showing the logs every second 
                sensor_values = str(se.reflection()) + ',' + str(sc.reflection()) + ',' + str(sd.reflection()) #sets a variable to show the updated sensor values
                print(sensor_values) #debug for showing the values of the sensor every second
                u_value = u2.distance() #constantly get the distance value
                se_value = se.reflection() #constantly get the left sensor value
                sd_value = sd.reflection() #constantly get the right sensor value 
                sc_value = sc.reflection() #constantly get the middle sensor value
                if se.reflection() > 45 and sd.reflection() > 45 and sc.reflection() < 45: #if right-left sensors values are bigger then 50(if they are seeing white), and middle value is smaller then 55(if its seeing black), then(if the robot is in line):
                    updateLog(proportionalAlign(se, sd, kP,set_point_p)) #do proportional align to correct little route errors
                else: #else(if the robot isn't in line), then:
                    valores_verdes = i.checkGreen(green_values) #constantly use the checkGreen function from the Intersection object to return if any of the right-left sensors are seeig green
                    if valores_verdes[0] != False or valores_verdes[1] != False: #if any of the right-left sensors is seeing green, then:
                        updateLog(i.intersectionSolver(valores_verdes,set_point_i1,set_point_i2))# do intersection solver
                    if se.reflection() > set_point_gap and sd.reflection() > set_point_gap and sc.reflection() > set_point_gap: #if every sensor is seeing white, then:
                        if logs[-1][0] == 'proportional align': #if the last task was proportional align(if the robot were in line before seeing all white), then:
                            motors.move_tank(1800,200,200)
                            updateLog(["gap", 'None', "succeded"]) #it's a gap(uptade the log to a gap case)
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
                            if se.reflection() > 45 and sd.reflection() > 45 and sc.reflection() < 45: #if the robot is in line, then:
                                updateLog(proportionalAlign(se,sd,kP,set_point_p)) #do proportional align 
                            else: #if the robot isn't in line, then:
                                print('back until see black') #debug
                                motors.move_tank(1000, -200, -200) #go back until see black
                                if se.reflection() > 60 and sd.reflection() > 60 and sc.reflection() > 60:
                                    motors.move_tank(1000, -200, -200)
                                corner = 0
                                updateLog(axis_correction(logs[-1][0],set_point_c,set_point_s,timeout_s,timeout_c,max_corner)) # do axis correction after it returns
                        else: #else, if both left-right are seeing a value higher then 30, then:
                            if se_value > 50 and sd_value > 50:
                                print('axis correction no branco') #debug
                                updateLog(["Axis Correction no branco",move_side,log])
                            updateLog(axis_correction(logs[-1][0],set_point_c,set_point_s,timeout_s,timeout_c,max_corner)) #do axis correction
