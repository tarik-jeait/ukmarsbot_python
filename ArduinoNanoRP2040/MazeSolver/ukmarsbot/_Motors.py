from _Config import Config
import time
from Logger import Logger

class Motors:
    def __init__(self,board,encoders,sensors):
        self.board = board
        self.encoders = encoders
        self.sensors = sensors

    ###################################################################################
    ######## Motors             #######################################################
    ###################################################################################

    def stopMotors(self):
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0)         

    def MoveForward(self,adjustment):
        self.board.RMOTOR_DIR.value(0^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(0^Config.MOTOR_L_DIRECTION)  # set the left motor direction   
        #Move Forward until a road crossing
        basespeed = Config.BASE_SPEED
        #adjustment = 0    
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         

    def MoveForwardDistance(self,distance):
        Logger.log("MoveForwardDistance(board,distance)")
        self.board.RMOTOR_DIR.value(0^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(0^Config.MOTOR_L_DIRECTION)  # set the left motor direction   
        basespeed = Config.BASE_SPEED
        adjustment = 0
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        current_distance = 0
        current_angle = 0
        self.encoders.init()
        self.encoders.print()
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)
        gSensorDifference = 0
        gSensorDifferencePrevious = 0
        Kp = 0.15
        Kd = Kp/2.0
        while (distance-current_distance) > 0.2:
            encoderSum = self.encoders.m_left_counter + self.encoders.m_right_counter
            encoderDifference = self.encoders.m_right_counter - self.encoders.m_left_counter
            current_distance = Config.MM_PER_COUNT * encoderSum;
            current_angle = Config.DEG_PER_COUNT * encoderDifference            
            # CTE
            self.sensors.readSensorsNormalized()
            #Logger.log("gSensorDifference : %f"%(gSensorDifference))
            
            if self.sensors.isWallLeft() and self.sensors.isWallRight():
                gSensorDifference = self.sensors.leftside_normalized - self.sensors.rightside_normalized;
                #if(gSensorDifference > 20):
                #    adjustment = 3000    
                #if(gSensorDifference < -20):
                #    adjustment = -3000                
                
            elif self.sensors.isWallLeft():
                
                gSensorDifference = 0
                
            elif self.sensors.isWallRight():
                gSensorDifference = 0
            else:
                gSensorDifference = 0
                
            adjustment = (gSensorDifference * Kp + gSensorDifferencePrevious*Kd)*50.0
            
            leftspeed = int(basespeed + adjustment)
            rightspeed = int(basespeed - adjustment)
            self.board.LMOTOR_PWM.duty_u16(leftspeed)
            self.board.RMOTOR_PWM.duty_u16(rightspeed)
            gSensorDifferencePrevious = gSensorDifference
            time.sleep(0.01)    
        
        Logger.log("distance : %f - angle:%f"%(current_distance,current_angle))
        self.stopMotors()    

    def turnLeft(self):
        basespeed = Config.BASE_SPEED
        adjustment = 0
        # Move Forward for 5 cm before turn left    
        # Turn Left
        self.board.RMOTOR_DIR.value(0^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(1^Config.MOTOR_L_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed+adjustment)
        rightspeed = int(basespeed - adjustment)

        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        
        time.sleep(0.5)        
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0)         

    def turnLeftAngle(self,angle):
        basespeed = Config.BASE_SPEED
        adjustment = 0
        # Move Forward for 5 cm before turn left    
        # Turn Left
        self.board.RMOTOR_DIR.value(0^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(1^Config.MOTOR_R_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed+adjustment)
        rightspeed = int(basespeed - adjustment)

        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        current_distance = 0
        current_angle = 0
        self.encoders.init()
        self.encoders.print()
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        while (angle-current_angle) > 0.5:        
            encoderSum = self.encoders.m_left_counter + self.encoders.m_right_counter
            encoderDifference = self.encoders.m_right_counter - self.encoders.m_left_counter
            current_distance = Config.MM_PER_COUNT * encoderSum;
            current_angle = Config.DEG_PER_COUNT * encoderDifference
            time.sleep(0.01)    
        print("distance : %f - angle:%f"%(current_distance,current_angle))
        self.stopMotors()

    def turnRight(self):
        basespeed = Config.BASE_SPEED
        adjustment = 0
        self.board.RMOTOR_DIR.value(1^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(0^Config.MOTOR_L_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed+adjustment)
        rightspeed = int(basespeed - adjustment)
            
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        time.sleep(0.5)        
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0)         

    def turnRightAngle(self,angle):
        basespeed = Config.BASE_SPEED
        adjustment = 0
        # Move Forward for 5 cm before turn left    
        # Turn Left
        self.board.RMOTOR_DIR.value(1^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(0^Config.MOTOR_L_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed+adjustment)
        rightspeed = int(basespeed - adjustment)

        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        current_distance = 0
        current_angle = 0
        self.encoders.init()
        self.encoders.print()
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        while (angle-current_angle) > 0.5:        
            encoderSum = self.encoders.m_left_counter + self.encoders.m_right_counter
            encoderDifference = -self.encoders.m_right_counter + self.encoders.m_left_counter
            current_distance = Config.MM_PER_COUNT * encoderSum;
            current_angle = Config.DEG_PER_COUNT * encoderDifference
            time.sleep(0.01)    
        print("distance : %f - angle:%f"%(current_distance,current_angle))
        self.stopMotors()  

    def turnBackLeft(self):
        basespeed = Config.BASE_SPEED
        adjustment = 0
        self.board.RMOTOR_DIR.value(0^Config.MOTOR_R_DIRECTION)  # set the right motor direction
        self.board.LMOTOR_DIR.value(1^Config.MOTOR_R_DIRECTION)  # set the left motor direction
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
            
        self.board.LMOTOR_PWM.duty_u16(leftspeed)
        self.board.RMOTOR_PWM.duty_u16(rightspeed)         
        time.sleep(1.0)        
        self.board.LMOTOR_PWM.duty_u16(0)
        self.board.RMOTOR_PWM.duty_u16(0)
