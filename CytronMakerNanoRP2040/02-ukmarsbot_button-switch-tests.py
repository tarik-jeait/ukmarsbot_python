from machine import Pin, ADC, PWM
from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time


# Simple function to detect if the switch is pressed on time
def detectSwitchPress(board):
    # make sure that that all 4 pins in dip switch are in ON position
    switch_pressed = 0
    while switch_pressed == 0:
        value = board.SWITCH.value()
        print("value:%d"% value)
        if(value == 1):
            switch_pressed = 1
            print("switch pressed:%d"%switch_pressed)
            return 1
        time.sleep(0.1)

# Simple function to detect how many times the switch is pressed
# Use of cytron maker rp2040 user button to stop the count
def getSwitchPressCount(board):
    # make sure that that all 4 pins in dip switch are in ON position
    switch_press_count = 0    
    user_button_pressed = 0
    
    while user_button_pressed == 0:
        value_btn1 = board.btn1.value()
        if(value_btn1 == 0):
            user_button_pressed = 1
            print("user button pressed:%d"%user_button_pressed)
        else:
            value_switch = board.SWITCH.value()
            if(value_switch == 1):
                switch_press_count = switch_press_count + 1
                print("switch pressed -> count:%d"%switch_press_count)            
        time.sleep(0.2)
    
    return switch_press_count

cytron_board = CytronMakerNanoRP2040()

detectSwitchPress(cytron_board)
time.sleep(1)
getSwitchPressCount(cytron_board)