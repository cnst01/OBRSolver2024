from pybricks.hubs import PrimeHub
from spike_pybricks_motorpair import MotorPair
from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, UltrasonicSensor

hub = PrimeHub()
display = hub.display

motors = MotorPair(Port.A,Port.B)

# defining sensors
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)

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

def axis_correction(last_move, corner = 0, set_point_c = 40, set_point_s = 75):
    axisCorrectionDisplay()
    name = ''
    move_side = ''
    log = ''
    if last_move != "axis correction **Corner**" or last_move != "axis correction **Suave**":
        corner = 0
    if corner > 3:
        motors.stop_tank()
        if sd.reflection() < set_point_s:
            while sd.reflection() < set_point_s:
                motors.start_tank(-100,0)
                move_side = 'right'
        if se.reflection() < set_point_s : 
            while se.reflection() < set_point_s:
                motors.start_tank(0,-100)
                move_side = 'left'
        if corner == 5:
            corner == 0
        name = "axis correction **Suave**"
        log = 'succeded'
    else:
        if sd.reflection() > se.reflection():
            while sd.reflection() > set_point_c:
                motors.start_tank(250,-40)
                move_side = 'right'
            corner += 1
        else:
            while se.reflection() > set_point_c:
                motors.start_tank(-40,250)
                move_side = 'left'
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    return [name, move_side, log]
