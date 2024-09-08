from machine import Pin, ADC, PWM
from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time
import math 

# Global variable definitions
global cytron_maker_rp2040_board

global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
left_oldA = 0
left_oldB = 0
right_oldA = 0
right_oldB = 0

# encoder polarity is either 1 or -1 and is used to account for reversal of the encoder phases
ENCODER_LEFT_POLARITY = 1
ENCODER_RIGHT_POLARITY = 1

m_left_counter = 0
m_right_counter = 0

updateTime=0
updateInterval = 2;  # in milliseconds

BASE_SPEED = 10000

global COUNTS_PER_ROTATION, GEAR_RATIO, WHEEL_DIAMETER, WHEEL_SEPARATION,MM_PER_COUNT,DEG_PER_COUNT,encoderSum,encoderDifference

COUNTS_PER_ROTATION = 12
GEAR_RATIO = 51.45
#GEAR_RATIO = 50
WHEEL_DIAMETER = 33.0
WHEEL_SEPARATION = 92.5

MM_PER_COUNT = (math.pi * WHEEL_DIAMETER) / (2 * COUNTS_PER_ROTATION * GEAR_RATIO);
DEG_PER_COUNT = (360.0 * MM_PER_COUNT) / (math.pi * WHEEL_SEPARATION);

def playTone(board):    
    # Play tone
    PIEZO_PWM = machine.PWM(board.PIEZO_PIN)
    PIEZO_PWM.freq(440)
    PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
    time.sleep(0.5)
    PIEZO_PWM.duty_u16(0)
    
# MOTORS
def MoveForward(board,adjustment):
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction   
    basespeed = BASE_SPEED
    adjustment = 0    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
    
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         

def MoveBack(board,adjustment):
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction   
    basespeed = BASE_SPEED
    adjustment = 0    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
    
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed) 
def turnLeft(board):
    basespeed = BASE_SPEED
    adjustment = 0
    # Move Forward for 5 cm before turn left    
    # Turn Left
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    
    leftspeed = int(basespeed+adjustment)
    rightspeed = int(basespeed - adjustment)

    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    
    time.sleep(0.5)        
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         
    
def turnRight(board):
    basespeed = BASE_SPEED
    adjustment = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction
    
    leftspeed = int(basespeed+adjustment)
    rightspeed = int(basespeed - adjustment)
        
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    time.sleep(0.5)       
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         

def turnBackLeft(board):
    basespeed = BASE_SPEED
    adjustment = 0
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
        
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    time.sleep(1)        
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)
def stopMotors(board):
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)
    

#ENCODERS
def encoders_enable_interrupts():    
    global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
    global cytron_maker_rp2040_board
    
    cytron_maker_rp2040_board.ENCODER_LEFT_CLK.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=left_input_change)
    cytron_maker_rp2040_board.ENCODER_RIGHT_CLK.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=right_input_change)
    
def left_input_change(pin):
    global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
    global cytron_maker_rp2040_board
    newB = cytron_maker_rp2040_board.ENCODER_LEFT_B.value()
    newA = cytron_maker_rp2040_board.ENCODER_LEFT_CLK.value() ^ newB
    delta = ENCODER_LEFT_POLARITY * ((left_oldA ^ newB) - (newA ^ left_oldB))
    m_left_counter += delta
    left_oldA = newA
    left_oldB = newB
    #encoders_print()

def right_input_change(pin):
    global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
    global cytron_maker_rp2040_board
    newB = cytron_maker_rp2040_board.ENCODER_RIGHT_B.value()
    newA = cytron_maker_rp2040_board.ENCODER_RIGHT_CLK.value() ^ newB
    delta = ENCODER_RIGHT_POLARITY * ((right_oldA ^ newB) - (newA ^ right_oldB))
    m_right_counter += delta
    right_oldA = newA
    right_oldB = newB
    #encoders_print()

def encoders_print():
    global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
    print("m_left_counter:%d - m_right_counter:%d"% (m_left_counter,m_right_counter))

def encoders_init():
    global left_oldA,left_oldB,right_oldA,right_oldB,ENCODER_LEFT_POLARITY,ENCODER_RIGHT_POLARITY,m_left_counter,m_right_counter
    m_left_counter = 0
    m_right_counter = 0

# BOARD
cytron_maker_rp2040_board = cytron_board = CytronMakerNanoRP2040()
#cytron_maker_rp2040_board = CytronMakerNanoRP2040()

# ENCODERS TEST
l1count = l2count = r1count = r2count = 0 #reset the encoder counts
left1 = left2 = right1 = right2 = True
prevleft1 = prevleft2 = prevright1 = prevright2 = True # reset the previous encoder values


#motorsBasicTest(cytron_board,0)
#motorsBasicTest(cytron_board,180)

playTone(cytron_board)

#encoders_print()
#MoveForward(cytron_board,0)
#time.sleep(1)
#stopMotors(cytron_board)
#encoders_print()

print("###############################################")
print("Testing encoders ..........")
print("###############################################")
encoders_enable_interrupts()
print("encoders init")
encoders_init()
encoders_print()

print("###############################################")
print("Testing motors ..........")
print("Press ukmarsbot back switch to start........")
print("###############################################")

switch_pressed = 0
while switch_pressed == 0:
    # value = cytron_board.SWITCH.value()
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        #switch_pressed = 1
        time.sleep(2)
        playTone(cytron_board)
        print("switch pressed:%d"%switch_pressed)
        
        print("MoveForward")
        MoveForward(cytron_board,0)
        time.sleep(0.5)
        stopMotors(cytron_board)
        encoders_print()
        encoderSum = m_left_counter + m_right_counter
        encoderDifference = m_left_counter - m_right_counter
        distance = MM_PER_COUNT * encoderSum;
        angle = DEG_PER_COUNT * encoderDifference

        print("distance : %f - angle:%f"%(distance,angle))
        time.sleep(2)
        
        encoders_init()
        encoders_print()        
        turnRight(cytron_board)
        stopMotors(cytron_board)
        encoders_print()
        encoderSum = m_left_counter + m_right_counter
        encoderDifference = m_left_counter - m_right_counter
        distance = MM_PER_COUNT * encoderSum;
        angle = DEG_PER_COUNT * encoderDifference
        print("distance : %f - angle:%f"%(distance,angle))
        switch_pressed = 1
    time.sleep(0.1)


#Moved distance


# ENCODERS
# 2 counters :
# m_left_counter  -> for left motor
# m_right_counter -> for right motor
# when a motor goes forward -> the counter is incremented
# when a motor goes back -> the counter is decremented
