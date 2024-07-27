# ukmarsbot.org version 1.3 upgraded to 3.3V
# Pins definition for Cytron Maker Nano RP2040
# 

from machine import Pin, ADC, PWM

class ArduinoNanoRP2040:
    def __init__(self):
        # Set pins for digital outputs on Arduino Nano RP2040
        self.LED_PIN = Pin(6, Pin.OUT) #
        # SENSORS
        # Arduino D11 : LEFT LED - LED5
        self.LEFT_LED = Pin(7, Pin.OUT) # 1st Sensor LED on line follow sensor board
        # Arduino D6 : RIGHT LED - LED4
        self.RIGHT_LED = Pin(18, Pin.OUT) # 2nd Sensor LED on line follow sensor board
        #Arduino D12 : Sensor EMITTER
        self.TRIGGER_PIN = Pin(4, Pin.OUT) # Trigger for LEDs on line follow sensor board
        # Define analogue inputs used on sensor board
        # T1 / RIGHT SENSOR
        self.LEFT_SENSOR = ADC(Pin(29))   # A3
        # T2 / LEFT FRONT ANALOG SENSOR
        self.LEFT_FRONT_SENSOR = ADC(Pin(28))  # A2
        # T3 / RIGHT FRONT ANALOG SENSOR
        self.RIGHT_FRONT_SENSOR = ADC(Pin(27))  # A1
        # T4 / RIGHT ANALOG SENSOR
        self.RIGHT_SENSOR = ADC(Pin(26))   # A0

        ###################################################################################
        ######## BUTTONS ##################################################################
        # Define RP2040 on-board button with pull up so it goes false when pressed
        # define press button switch on UKMARS board next to 4 way switch which should be set to all on
        # Arduino A6 : FUNCTION_SELECT SWITCH
        #self.SWITCH = Pin(14, Pin.IN) # button / switch pin

        ###################################################################################
        ######## MOTORS  ##################################################################
        self.RMOTOR_DIR = Pin(20, Pin.OUT)
        self.LMOTOR_DIR = Pin(19, Pin.OUT) 

        self.LMOTOR_PWM = PWM(Pin(21))
        self.RMOTOR_PWM = PWM(Pin(5))
        
        self.LMOTOR_PWM.freq(2000)
        self.RMOTOR_PWM.freq(2000)

        self.LMOTOR_PWM.duty_u16(0) # left motor speed - range is 0 to 65535
        self.RMOTOR_PWM.duty_u16(0) # right motor speed - range is 0 to 65535

        self.Leftenc1 = Pin(25, Pin.IN) # encoder pins
        self.Leftenc2 = Pin(16, Pin.IN)
        self.Rightenc1 = Pin(15, Pin.IN)
        self.Rightenc2 = Pin(17, Pin.IN)