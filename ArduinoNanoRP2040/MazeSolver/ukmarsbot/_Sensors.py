import time
import math
import ujson as json

from _Config import Config
from Logger import Logger


class Sensors:
    def __init__(self,board):
        self.board = board
        # Sensor Raw Data
        self.front = 0        
        self.leftside = 0                
        self.rightside = 0

        self.calibrated_data = None
        # Sensor Calibrated Data
        self.leftside_min = 10000
        self.leftside_max = 0
        
        self.front_min = 10000
        self.front_max = 0
                
        self.rightside_min = 10000
        self.rightside_max = 0
        
        #Normalized values
        self.leftside_normalized = 0
        self.front_normalized = 0
        self.rightside_normalized = 0
    
    def readSensorsWallFollower(self):
                
        self.board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(0.001)        

        leftside_dark = self.board.WALL_LEFT_SENSOR.read_u16()
        front_dark = self.board.WALL_FRONT_SENSOR.read_u16()
        rightside_dark = self.board.WALL_RIGHT_SENSOR.read_u16()
        
        self.board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(0.001)
            
        leftside_light = self.board.WALL_LEFT_SENSOR.read_u16()
        front_light = self.board.WALL_FRONT_SENSOR.read_u16()
        rightside_light = self.board.WALL_RIGHT_SENSOR.read_u16()      

        self.leftside = math.fabs(leftside_light - leftside_dark)
        self.front = math.fabs(front_light - front_dark)
        self.rightside = math.fabs(rightside_light - rightside_dark)

        return self

    def testReadSensorsWallFollower(self):
        updateTime = 0
        updateInterval = 2  # in milliseconds
        LOOP = True
        while LOOP:
            s = self.readSensorsWallFollower()            
            if(time.time() > updateTime):
                updateTime = updateTime + updateInterval
                print("-----------------------------------------------")
                print("left raw:(%d) - front raw:(%d) - right raw:(%d)"% (s.leftside,s.front,s.rightside))
                print("left n:(%d)(W:%s) - front n:(%d)(W:%s)-right n:(%d)(W:%s)"% (s.leftside_normalized,s.isWallLeft(),s.front_normalized,s.isWallFront(),s.rightside_normalized,s.isWallRight()))
                
            time.sleep(.5) # white around 4,500, black around 55,000


    def calibrateSensorsWallFollower(self):
        Logger.log("sensors calibration for Wall Follower ........")          
        count = 1000
        index = count
        
        leftside_min = 10000
        leftside_max = 0
        
        front_min = 10000
        front_max = 0
        
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
            self.readSensorsWallFollower()
            
            leftside_min = min(leftside_min,self.leftside)
            leftside_max = max(leftside_max,self.leftside)
            
            front_min = min(front_min,self.front)
            front_max = max(front_max,self.front)
            
            rightside_min = min(rightside_min,self.rightside)
            rightside_max = max(rightside_max,self.rightside)
            
            index = index - 1
        
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0) 
        Logger.log("leftside: Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))    
        Logger.log("front: Min:(%d) - Max:(%d)"% (front_min,front_max))    
        Logger.log("rightside: Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
        
        jsonData = {"leftside": {"min":leftside_min,"max":leftside_max},"front":{"min":front_min,"max":front_max},"rightside":{"min":rightside_min,"max":rightside_max}}
        
        self.save_calibrated_data(jsonData)

    def save_calibrated_data(self,jsonData):
        self.calibrated_data = jsonData        
        try:
            with open('calibrated_data.json', 'w') as f:
                json.dump(jsonData, f)
        except:
            Logger.log("Could not save data.")
        
    def read_calibrated_data(self):        
        if self.calibrated_data is None:
            #print("calibrated_data is None")
            try:
                with open('calibrated_data.json', 'r') as f:
                    self.calibrated_data = json.load(f)        
            except:
                Logger.log("Could not read calibrated data")
        else:
            #print("calibrated_data is not None")
            pass
            
    def printSensorsNormalized(self):
        Logger.log("left raw:(%d) - front raw:(%d) - right raw:(%d)"% (self.leftside,self.front,self.rightside))
        Logger.log("left n:(%d)(W:%s) - front n:(%d)(W:%s)-right n:(%d)(W:%s)"% (self.leftside_normalized,self.isWallLeft(),self.front_normalized,self.isWallFront(),self.rightside_normalized,self.isWallRight()))
        
    def readSensorsNormalized(self):
        
        self.read_calibrated_data()
        leftside_min = self.calibrated_data["leftside"]["min"]
        leftside_max = self.calibrated_data["leftside"]["max"]
        #Logger.log("leftside:Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))
        
        front_min = self.calibrated_data["front"]["min"]
        front_max = self.calibrated_data["front"]["max"]
        #Logger.log("front:Min:(%d) - Max:(%d)"% (front_min,front_max))
                
        rightside_min = self.calibrated_data["rightside"]["min"]
        rightside_max = self.calibrated_data["rightside"]["max"]
        #Logger.log("rightside:Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
        
        self.readSensorsWallFollower()
        
        #Logger.log("leftside:%.2f%%"% ((self.leftside-leftside_min)/(leftside_max-leftside_min)*100))
        #Logger.log("leftfront:%.2f%%"% ((self.leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100))
        #Logger.log("rightfront:%.2f%%"% ((self.rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100))
        #Logger.log("rightside:%.2f%%"% ((self.rightside-rightside_min)/(rightside_max-rightside_min)*100))
        
        self.leftside_normalized = (self.leftside-leftside_min)/(leftside_max-leftside_min)*100
        self.front_normalized = (self.front-front_min)/(front_max-front_min)*100
        self.rightside_normalized = (self.rightside-rightside_min)/(rightside_max-rightside_min)*100
 
 
        return self
    
        sensors = Sensors()
        
        sensors.leftside = leftside
        sensors.leftfront = leftfront
        sensors.rightfront = rightfront
        sensors.rightside = rightside
        
        
        
        return sensors    

    def testReadSensorsNormalized(self):
        updateTime = 0
        updateInterval = 2  # in milliseconds
        LOOP = True
        while LOOP:
            s = self.readSensorsNormalized()            
            if(time.time() > updateTime):
                updateTime = updateTime + updateInterval
                print("-----------------------------------------------")
                print("left raw:(%d) - front raw:(%d) - right raw:(%d)"% (s.leftside,s.front,s.rightside))
                print("left n:(%d)(W:%s) - front n:(%d)(W:%s)-right n:(%d)(W:%s)"% (s.leftside_normalized,s.isWallLeft(),s.front_normalized,s.isWallFront(),s.rightside_normalized,s.isWallRight()))
                
            time.sleep(.5) # white around 4,500, black around 55,000


    def isWallFront(self):
        if(self.front_normalized>Config.WALL_SENSOR_FRONT_THRESHOLD):
            return True
        else:
            return False
    
    def isWallLeft(self):
        if(self.leftside_normalized > Config.WALL_SENSOR_LEFT_THRESHOLD):
            return True
        else:
            return False
        
    def isWallRight(self):
        if(self.rightside_normalized > Config.WALL_SENSOR_RIGHT_THRESHOLD):
            return True
        else:
            return False