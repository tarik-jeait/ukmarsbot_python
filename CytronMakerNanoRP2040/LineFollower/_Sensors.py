import time
import math
import ujson as json

from _Config import Config

class Sensors:
    def __init__(self,board):
        self.board = board
        # Sensor Raw Data
        self.leftside = 0
        self.leftfront = 0
        self.rightfront = 0
        self.rightside = 0
        self.calibrated_data = None
        # Sensor Calibrated Data
        self.leftside_min = 10000
        self.leftside_max = 0
        
        self.leftfront_min = 10000
        self.leftfront_max = 0
        
        self.rightfront_min = 10000
        self.rightfront_max = 0
        
        self.rightside_min = 10000
        self.rightside_max = 0
        #Normalized values
        self.leftside_normalized = 0
        self.leftfront_normalized = 0
        self.rightfront_normalized = 0
        self.rightside_normalized = 0
    
    def readSensors(self):
        #global leftside,leftfront,rightfront,rightside        
        self.board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(0.001)        
        leftside_dark = self.board.LEFT_SENSOR.read_u16()
        leftfront_dark = self.board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_dark = self.board.RIGHT_SENSOR.read_u16()
        rightside_dark = self.board.RIGHT_SENSOR.read_u16()
        
        self.board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(0.001)
            
        leftside_light = self.board.LEFT_SENSOR.read_u16()
        leftfront_light = self.board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_light = self.board.RIGHT_FRONT_SENSOR.read_u16()
        rightside_light = self.board.RIGHT_SENSOR.read_u16()

        self.leftside = math.fabs(leftside_light - leftside_dark)
        self.leftfront = math.fabs(leftfront_light - leftfront_dark)
        self.rightfront = math.fabs(rightfront_light - rightfront_dark)
        self.rightside = math.fabs(rightside_light - rightside_dark)

        return self
    
    def calibrateSensors(self):
            
        count = 1000
        index = count
        
        leftside_min = 10000
        leftside_max = 0    
        leftfront_min = 10000
        leftfront_max = 0    
        rightfront_min = 10000
        rightfront_max = 0
        rightside_min = 10000
        rightside_max = 0

        basespeed = Config.BASE_SPEED
        adjustment = 0
        self.board.RMOTOR_DIR.value(1^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(0^Config.MOTOR_L_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
            
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         

        while index>0 :
            self.readSensors()        
            leftside_min = min(leftside_min,self.leftside)
            leftside_max = max(leftside_max,self.leftside)        
            leftfront_min = min(leftfront_min,self.leftfront)
            leftfront_max = max(leftfront_max,self.leftfront)
            rightfront_min = min(rightfront_min,self.rightfront)
            rightfront_max = max(rightfront_max,self.rightfront)        
            rightside_min = min(rightside_min,self.rightside)
            rightside_max = max(rightside_max,self.rightside)        
            index = index - 1
        
        #STOP MOTORS AFTER CALIBRATION
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0)         
        
        print("leftside: Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))    
        print("leftfront: Min:(%d) - Max:(%d)"% (leftfront_min,leftfront_max))    
        print("rightfront: Min:(%d) - Max:(%d)"% (rightfront_min,rightfront_max))    
        print("rightside: Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
        
        jsonData = {"leftside": {"min":leftside_min,"max":leftside_max},"leftfront":{"min":leftfront_min,"max":leftfront_max},"rightfront":{"min":rightfront_min,"max":rightfront_max},"rightside":{"min":rightside_min,"max":rightside_max}}    
        self.save_calibrated_data(jsonData)

    def save_calibrated_data(self,jsonData):
        try:
            with open('calibrated_data.json', 'w') as f:
                json.dump(jsonData, f)
        except:
            print("Could not save data.")
        
    def read_calibrated_data(self):
        
        if self.calibrated_data is None:
            try:
                with open('calibrated_data.json', 'r') as f:
                    self.calibrated_data = json.load(f)        
            except:
                print("Could not read calibrated data")

    def readSensorsNormalized(self):
        
        self.read_calibrated_data()
        leftside_min = self.calibrated_data["leftside"]["min"]
        leftside_max = self.calibrated_data["leftside"]["max"]
        #print("leftside:Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))
        
        leftfront_min = self.calibrated_data["leftfront"]["min"]
        leftfront_max = self.calibrated_data["leftfront"]["max"]
        #print("leftfront:Min:(%d) - Max:(%d)"% (leftfront_min,leftfront_max))
        
        rightfront_min = self.calibrated_data["rightfront"]["min"]
        rightfront_max = self.calibrated_data["rightfront"]["max"]
        #print("rightfront:Min:(%d) - Max:(%d)"% (rightfront_min,rightfront_max))
        
        rightside_min = self.calibrated_data["rightside"]["min"]
        rightside_max = self.calibrated_data["rightside"]["max"]
        #print("rightside:Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
        
        self.readSensors()
        
        #print("leftside:%.2f%%"% ((self.leftside-leftside_min)/(leftside_max-leftside_min)*100))
        #print("leftfront:%.2f%%"% ((self.leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100))
        #print("rightfront:%.2f%%"% ((self.rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100))
        #print("rightside:%.2f%%"% ((self.rightside-rightside_min)/(rightside_max-rightside_min)*100))
        
        self.leftside_normalized = (self.leftside-leftside_min)/(leftside_max-leftside_min)*100
        self.leftfront_normalized = (self.leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100
        self.rightfront_normalized = (self.rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100
        self.rightside_normalized = (self.rightside-rightside_min)/(rightside_max-rightside_min)*100
        
        return self
        sensors = Sensors()
        
        sensors.leftside = leftside
        sensors.leftfront = leftfront
        sensors.rightfront = rightfront
        sensors.rightside = rightside
        
        
        
        return sensors    

    def testReadSensorsNormalized(self):
        global updateTime
        LOOP = True
        while LOOP:
            s = self.readSensorsNormalized()
            gSensorDifference = s.rightfront - s.leftfront
            if(time.time() > updateTime):
                updateTime = updateTime + updateInterval
                print("-----------------------------------------------")
                print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-diff:(%d)"% (s.leftside,s.leftfront,s.rightfront,s.rightside,gSensorDifference))
                print("left n:(%d)-left front n:(%d)-right front n:(%d)-right n:(%d)-diff:(%d)"% (s.leftside_normalized,s.leftfront_normalized,s.rightfront_normalized,s.rightside_normalized,gSensorDifference))
                
            time.sleep(.5) # white around 4,500, black around 55,000


    
    def isCrossRoads(self):
        if(self.leftside_normalized<50 and self.rightside_normalized<50):
            return False
        else:
            return True
    
    def isRouteLeft(self):
        if(self.leftside_normalized >= 50):
            return True
        else:
            return False
        
    def isRouteRight(self):
        if(self.rightside_normalized >= 50):
            return True
        else:
            return False
    
    def isTheEnd(self):
        if(self.rightside_normalized >= 50 and self.leftside_normalized >= 50 and self.leftfront_normalized>=50 and self.rightfront_normalized>=50):
            return True
        else:
            return False

    def isRouteStraigth(self):
        if(self.rightside_normalized < 20 and self.leftside_normalized < 20 and self.leftfront_normalized>=50 and self.rightfront_normalized>=50):
            return True
        else:
            return False
    def isBlackSurface(self):
        if(self.leftfront_normalized<20 and self.rightfront_normalized<20):
            return True
        else:
            return False


