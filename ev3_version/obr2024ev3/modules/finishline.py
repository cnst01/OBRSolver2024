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
        sensor_d = self.sd.rgb()
        sensor_e = self.se.rgb()
        if sensor_d[0] > valuesD[0][0] and sensor_d[0] < valuesD[1][0]:
            if sensor_d[1] > valuesD[0][1] and sensor_d[1] < valuesD[1][1]:
                if sensor_d[2] > valuesD[0][2] and sensor_d[2] < valuesD[1][2]:
                    return True
        if sensor_e[0] > valuesE[0][0] and sensor_e[0] < valuesE[1][0]:
            if sensor_e[1] > valuesE[0][1] and sensor_e[1] < valuesE[1][1]:
                if sensor_e[2] > valuesE[0][2] and sensor_e[2] < valuesE[1][2]:
                    return True
        return False