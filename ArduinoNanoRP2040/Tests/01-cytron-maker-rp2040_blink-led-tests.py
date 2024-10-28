from machine import Pin
import time


# SPECIFIC TESTS FOR Cytron Maker Nano RP2040
INTERNAL_LED = Pin(18, Pin.OUT)

def blinkLed():
    INTERNAL_LED.value(1)
    #print("ON")
    time.sleep(0.5)
    INTERNAL_LED.value(0)
    #print("OFF")
    time.sleep(0.5)



while True:
    blinkLed()
