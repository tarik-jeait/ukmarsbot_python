from machine import Pin
import time


# SPECIFIC TESTS FOR Cytron Maker Nano RP2040

def playTone():
    # Play tone
    PIEZO_PWM = machine.PWM(machine.Pin(22))

    # Melody
    MELODY_NOTE = [659, 659, 0, 659, 0, 523, 659, 0, 784]
    MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]
    for i in range(9):
        print(i)
        if(MELODY_NOTE[i] != 0):
            PIEZO_PWM.freq(MELODY_NOTE[i])
            PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
        time.sleep(MELODY_DURATION[i])
        PIEZO_PWM.duty_u16(0)



while True:
    playTone()
    time.sleep(1)
    
