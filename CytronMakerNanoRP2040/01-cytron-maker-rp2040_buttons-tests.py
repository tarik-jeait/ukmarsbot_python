from machine import Pin
import time


# SPECIFIC TESTS FOR Cytron Maker Nano RP2040

def detectButtonPress():
    # Define RP2040 on-board button with pull up so it goes false when pressed
    # USER BUTTON marked as in the cytron maker card : GP20
    btn1 = Pin(20, Pin.IN, Pin.PULL_UP)
    button_pressed = 0
    while button_pressed == 0:
        value = btn1.value()
        if(value == 0):
            button_pressed = 1
            print("button pressed:%d"%button_pressed)
            return 1
        time.sleep(0.1)

detectButtonPress()