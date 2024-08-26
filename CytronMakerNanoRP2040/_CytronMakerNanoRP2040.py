# ukmarsbot.org version 1.3 upgraded to 3.3V
# Pins definition for Cytron Maker Nano RP2040
# 

from machine import Pin, ADC, PWM

class CytronMakerNanoRP2040:
    def __init__(self):
        # Set pins for digital outputs on Maker Pi RP2040
        self.LED_PIN = Pin(18, Pin.OUT) # ext LED on GP18
        # SENSORS
        # Arduino D11 : LEFT LED - LED5
        self.LEFT_LED = Pin(19, Pin.OUT) # 1st Sensor LED on line follow sensor board
        # Arduino D6 : RIGHT LED - LED4
        self.RIGHT_LED = Pin(6, Pin.OUT) # 2nd Sensor LED on line follow sensor board
        #Arduino D12 : Sensor EMITTER
        self.TRIGGER_PIN = Pin(16, Pin.OUT) # Trigger for LEDs on line follow sensor board
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
        # USER BUTTON marked as in the cytron maker card : GP20
        self.btn1 = Pin(20, Pin.IN, Pin.PULL_UP)
        # define press button switch on UKMARS board next to 4 way switch which should be set to all on
        # Arduino A6 : FUNCTION_SELECT SWITCH
        self.SWITCH = Pin(14, Pin.IN) # button / switch pin

        ###################################################################################
        ######## SOUND   ##################################################################

        self.PIEZO_PIN = Pin(22, Pin.OUT) # Pin connected to piezo buzzer 

        ###################################################################################
        ######## MOTORS  ##################################################################
        self.RMOTOR_DIR = Pin(8, Pin.OUT)
        self.LMOTOR_DIR = Pin(7, Pin.OUT) 

        self.LMOTOR_PWM = PWM(Pin(9))
        self.RMOTOR_PWM = PWM(Pin(17))
        
        self.LMOTOR_PWM.freq(2000)
        self.RMOTOR_PWM.freq(2000)

        self.LMOTOR_PWM.duty_u16(0) # left motor speed - range is 0 to 65535
        self.RMOTOR_PWM.duty_u16(0) # right motor speed - range is 0 to 65535

        self.ENCODER_LEFT_CLK = Pin(2, Pin.IN) # encoder pins
        self.ENCODER_LEFT_B = Pin(4, Pin.IN)
        self.ENCODER_RIGHT_CLK = Pin(3, Pin.IN)
        self.ENCODER_RIGHT_B = Pin(5, Pin.IN)
        
        