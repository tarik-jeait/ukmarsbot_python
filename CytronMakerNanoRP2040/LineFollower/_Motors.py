from _Config import Config
import time

class Motors:
    def __init__(self,board,encoders):
        self.board = board
        self.encoders = encoders

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
        print("MoveForwardDistance(board,distance)")
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
        while (distance-current_distance) > 0.2:        
            encoderSum = self.encoders.m_left_counter + self.encoders.m_right_counter
            encoderDifference = self.encoders.m_right_counter - self.encoders.m_left_counter
            current_distance = Config.MM_PER_COUNT * encoderSum;
            current_angle = Config.DEG_PER_COUNT * encoderDifference
            time.sleep(0.01)    
        print("distance : %f - angle:%f"%(current_distance,current_angle))
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
