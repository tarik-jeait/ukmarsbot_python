# UKMARSbot running python on maker nano RP2040 board using MicroPython
# 5V cpu board socket pin removed and 5v and 3.3v lines connected on back of the sensor connector
# R7 10K resistor removed to stop battery voltage feed to CPU
# In Thonny on windows PC  - From Run, Select Interpreter then MicroPython (Raspberry Pi Pico)
# 10:1 gear motors with encoders
# uses 2 LEDs on sensor board plus the mainboard red LED to select programs no 0 to 7
# press PCB switch to advance th eprogram number from 0 through to 7
# - program number is dislayed as 3 bits of binary
# prog 0 = wide line follow, 1 = narrow line follow, 2 = drag race, 3 = phototest, 4 = encodertest
#
# to use the press button set all the 4 switches to on. When button pressed switchvalue goes from 0 to 1
# if leftmost switch is set to off, switch value will always read 1
from machine import Pin, ADC, PWM
import neopixel
import time

# Global variable definitions
global leftside, leftfront, rightfront, rightside
global l1count, l2count, r1count, r2count, prevleft1, prevleft2, prevright1, prevright2
global bit0, bit1, bit2, bit3
# pin definitions
# Set pins for digital outputs on Maker Pi RP2040
LED_PIN = Pin(18, Pin.OUT) # ext LED on GP18

SENSOR1_PIN = Pin(19, Pin.OUT) # 1st Sensor LED on line follow sensor board
SENSOR2_PIN = Pin(6, Pin.OUT) # 2nd Sensor LED on line follow sensor board
TRIGGER_PIN = Pin(16, Pin.OUT) # Trigger for LEDs on line follow sensor board

RMOTOR_DIR = Pin(8, Pin.OUT)
LMOTOR_DIR = Pin(7, Pin.OUT) 

PIEZO_PIN = Pin(22, Pin.OUT) # Pin connected to piezo buzzer 

#define PWM motor speed pins
#LMOTOR = machine.Pin(9) # left motor is pin 9
#MOTOR = machine.Pin(17) # right motor is pin 17
LMOTOR_PWM = machine.PWM(machine.Pin(9))
RMOTOR_PWM = machine.PWM(machine.Pin(17))
LMOTOR_PWM.freq(2000)
RMOTOR_PWM.freq(2000)

LMOTOR_PWM.duty_u16(0) # left motor speed - range is 0 to 65535
RMOTOR_PWM.duty_u16(0) # right motor speed - range is 0 to 65535

# Define analogue inputs used on sensor board
Lsidesense = ADC(Pin(29))   # A3
Lfrontsense = ADC(Pin(28))  # A2
Rfrontsense = ADC(Pin(27))  # A1
Rsidesense = ADC(Pin(26))   # A0

# Define RP2040 on-board button with pull up so it goes false when pressed
btn1 = machine.Pin(20, Pin.IN, Pin.PULL_UP)
# define press button switch on UKMARS board next to 4 way switch which should be set to all on
Switch = Pin(14, Pin.IN) # button / switch pin

Leftenc1 = Pin(2, Pin.IN) # encoder pins
Leftenc2 = Pin(4, Pin.IN)
Rightenc1 = Pin(3, Pin.IN)
Rightenc2 = Pin(5, Pin.IN)



# Initialize Neopixel RGB LEDs
#pixels = neopixel.NeoPixel(11,Pin.OUT, 2)
#pixels.fill(0)

def photoread():
    global leftside, leftfront, rightfront, rightside
    TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
    time.sleep(0.02)
    leftside = Lsidesense.read_u16()
    leftfront = Lfrontsense.read_u16()
    rightfront = Rfrontsense.read_u16()
    rightside = Rsidesense.read_u16()
    TRIGGER_PIN.value(0)
    return 
def phototest():
    while True:
        global leftside, leftfront, rightfront, rightside
        photoread()    
        print((leftside),(leftfront),(rightfront),(rightside),)
        time.sleep(.5) # white around 4,500, black around 55,000
    return
def switchread():
    global swvalue
    swvalue = Switch.value() # = 0, or 1 if button pressed or 1st switch set to off
    
def switchtest():
    while True:
        global swvalue
        switchread()    
        print(("switch"),(swvalue))
        time.sleep(.5) # white around 4,500, black around 55,000
def progdisp():
    global swvalue, progno
    switchread()
    if(swvalue == 1):
        progno = progno + 1
        time.sleep(.5) 
    p1count = progno
    progno = p1count - (int(p1count / 8) * 8) # this gets it in range 0 to 7
    bit2 = p1count - (int(p1count / 2) * 2) # this extracts bit 3
    p1count = int(p1count / 2)
    bit1 = p1count - (int(p1count / 2) * 2) # this extracts bit 2
    p1count = int(p1count / 2)
    bit0 = p1count - (int(p1count / 2) * 2) # this extracts bit 1
    p1count = int(p1count / 2)
    if (bit0 == 1):
        SENSOR1_PIN.value(True)
    else:
        SENSOR1_PIN.value(False)
    if (bit1 == 1):
        SENSOR2_PIN.value(True)
    else:
        SENSOR2_PIN.value(False)
    if (bit2 == 1):
        LED_PIN.value(True)
    else:
        LED_PIN.value(False) 


def encoderread():
    global lefte1, lefte2, righte1, righte2
    lefte1 = Leftenc1.value
    lefte2 = Leftenc2.value
    righte1 = Rightenc1.value
    righte2 = Rightenc2.value
    
def encodertest():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    while True:
        encoderread()
        print((l1count), (l2count), (r1count), (r2count),)
        time.sleep(.5)
    return
def leftcount():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    l1count = l1count+1
    l2count = l2count+1
    return
def rightcount():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    r1count = r1count+1
    r2count = r2count+1
    return
def encountdisp(): 
    global l1count, l2count, r1count, r2count
    global bit0, bit1, bit2, progno
    p1count = int(l1count / 8)
    progno = p1count - (int(p1count / 8) * 8) # this gets it in range 0 to 7
    bit2 = p1count - (int(p1count / 2) * 2) # this extracts bit 3
    p1count = int(p1count / 2)
    bit1 = p1count - (int(p1count / 2) * 2) # this extracts bit 2
    p1count = int(p1count / 2)
    bit0 = p1count - (int(p1count / 2) * 2) # this extracts bit 1
    p1count = int(p1count / 2)
    if (bit0 == 1):
        SENSOR1_PIN.value(True)
    else:
        SENSOR1_PIN.value(False)
    if (bit1 == 1):
        SENSOR2_PIN.value(True)
    else:
        SENSOR2_PIN.value(False)
    if (bit2 == 1):
        LED_PIN.value(True)
    else:
        LED_PIN.value(False) 

  #  print (l1count, progno, bit0, bit1, bit2,)

# Line follower code
def widelinefollow():
    global leftfront, rightfront, leftspeed, rightspeed
    basespeed = 8000 - slowdownby
    side = 0
    adjustment = 0
    prevdiff = 0
    while True:
        photoread()
        if (leftfront < 10000):            # put LED indicators on if white line seen
            SENSOR1_PIN.value(True)
        else :
            SENSOR1_PIN.value(False)
        if (rightfront < 10000):
            SENSOR2_PIN.value(True)
        else :
            SENSOR2_PIN.value(False)       
        
        sensordiff = (leftfront - rightfront) / 6
        dterm = (sensordiff - prevdiff) * 3
        if (sensordiff > 40000):
            sensordiff = 40000
        if (sensordiff < -40000):
            sensordiff = - -40000
            
        if ((leftfront > 20000) and (rightfront > 20000)):  # gone off the line
            if (side == 1):
                adjustment = 5000
            if (side == 2):
                adjustment = -5000
        else:                             # proportional control
            adjustment = sensordiff
            if (leftfront > rightfront):
                side = 1
            if (rightfront > leftfront):
                side = 2
            adjustment = adjustment + dterm                     
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        checkspeed()
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        prevdiff = sensordiff
        #print(sensordiff, adjustment, leftspeed, rightspeed,)    
        

def thinlinefollow():
    global leftfront, rightfront, leftspeed, rightspeed
    basespeed = 6500 - slowdownby
    side = 0
    adjustment = 0
    prevdiff = 0
    while True:
        photoread()
        if (leftfront < 10000):            # put LED indicators on if white line seen
            SENSOR1_PIN.value(True)
        else :
            SENSOR1_PIN.value(False)
        if (rightfront < 10000):
            SENSOR2_PIN.value(True)
        else :
            SENSOR2_PIN.value(False)       
        
        sensordiff = (leftfront - rightfront) / 6
        dterm = (sensordiff - prevdiff) * 3
        if (sensordiff > 40000):
            sensordiff = 40000
        if (sensordiff < -40000):
            sensordiff = - -40000
            
        if ((leftfront > 20000) and (rightfront > 20000)):  # gone off the line
            if (side == 1):
                adjustment = 5000
            if (side == 2):
                adjustment = -5000
        else:                             # proportional control
            adjustment = sensordiff
            if (leftfront > rightfront):
                side = 1
            if (rightfront > leftfront):
                side = 2
            adjustment = adjustment + dterm                     
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        checkspeed()
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        prevdiff = sensordiff
        #print(sensordiff, adjustment, leftspeed, rightspeed,)    
    
def dragrace():
    global leftfront, rightfront, leftspeed, rightspeed
    basespeed = 8000
    side = 0
    adjustment = 0
    starttime = int(time.time())
    while True:
        photoread()
        currtime = int(time.time())
        #print (currtime, starttime)
        if ((currtime - starttime) > 1):
            checkending() # after a second check for line across the end
        if (leftfront < 10000):
            SENSOR1_PIN.value(True)
        else :
            SENSOR1_PIN.value(False)
        if (rightfront < 10000):
            SENSOR2_PIN.value(True)
        else :
            SENSOR2_PIN.value(False)       
        
        sensordiff = (leftfront - rightfront) / 20 
        if (sensordiff > 40000):
            sensordiff = 40000
        if (sensordiff < -40000):
            sensordiff = - -40000
            
        if ((leftfront > 10000) and (rightfront > 10000)):  # gone off the line
            if (side == 1):
                adjustment = 4500
            if (side == 2):
                adjustment = -4500
        else:                             # proportional control
  
            if (leftfront > rightfront):
                adjustment = sensordiff
                side = 1
            if (leftfront < rightfront):
                adjustment = sensordiff
                side = 2
        
            
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        checkspeed()
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        #print(sensordiff, adjustment, leftspeed, rightspeed,)    

    
def checkending():
    global leftside, leftfront, rightfront, rightside
   # print ("checkending", leftside, rightside)
    if ((leftside < 10000) and (rightside < 10000)): #seeing white line on both sides
        LMOTOR_PWM.duty_u16(3000)  # slow both motors
        RMOTOR_PWM.duty_u16(3000)  # slow both motors
        time.sleep(.5)
        LMOTOR_PWM.duty_u16(0)  # stop both motors
        RMOTOR_PWM.duty_u16(0)  # stop both motors
        while True:                 # wai for ever
            LED_PIN.value(True)    # while flashing the onboard LED
            time.sleep(0.1)
            LED_PIN.value(False)
            time.sleep(0.1)
    return

def checkspeed():
    global leftspeed, rightspeed
    if leftspeed > 50000:
        leftspeed = 50000
    if rightspeed > 50000:
        rightspeed = 50000
    if leftspeed < 10:
        leftspeed = 10
    if rightspeed < 10:
        rightspeed = 10
    return
    
    
    
# ------- End of definitions -----------------------------------------------------
#---------------------------------------------------------------------------------
# Melody
MELODY_NOTE = [659, 659, 0, 659, 0, 523, 659, 0, 784]
MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]

#-------------------------------------------------

LED_PIN.value(1)     # switch on LED
SENSOR1_PIN.value(1) # switch on sensor1 LED on line folllower board
SENSOR2_PIN.value(1) # switch on sensor2 LED on line folllower board


RMOTOR_DIR.value(1)  # set the right motor direction
LMOTOR_DIR.value(1)  # set the left motor direction

#color = 0
#state = 0

# Play tone
PIEZO_PWM = machine.PWM(machine.Pin(22))
PIEZO_PWM.freq(440)
PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
time.sleep(0.5)
PIEZO_PWM.duty_u16(0)


l1count = l2count = r1count = r2count = 0 #reset the encoder counts
left1 = left2 = right1 = right2 = True
prevleft1 = prevleft2 = prevright1 = prevright2 = True # reset the previous encoder values
progno = 0
swvalue = 0


# configure irq callback
Leftenc1.irq(lambda p:leftcount())
Rightenc1.irq(lambda p:rightcount())

# -------------------------------------------------
# wait for onboard button to be pressed
setting = 1
while (setting == True): # Check button 1 (GP20)
    setting = btn1.value()
    #print ("progno", progno)
    progdisp()
    #encountdisp()     
    time.sleep(0.1)
    slowdownby = 0
    
# run selected program
if (progno == 0):
    widelinefollow()
if (progno == 1):
    thinlinefollow()
if (progno == 2):
    dragrace()
if (progno == 3):
    phototest()
if (progno == 4):
    encodertest()
if (progno == 5):
    slowdownby = 500
    thinlinefollow()
if (progno == 6):
    switchtest() 
        
SENSOR1_PIN.value(0) # switch off sensor1 LED on line folllower board
SENSOR2_PIN.value(0) # switch off sensor2 LED on line folllower board
LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
RMOTOR_PWM.duty_u16(0)

