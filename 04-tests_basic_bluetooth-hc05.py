from machine import Pin, UART
from CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time
import math 

board = CytronMakerNanoRP2040()
#uart = UART(0, 9600)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

board.LEFT_LED.value(1)

while True:
  if uart.any() > 0:
    data = uart.read()
    print(data)
    if "on" in data:
      board.RIGHT_LED.value(1)
      print('LED on \n')
      uart.write('LED on \n')
    elif "off" in data:
      board.RIGHT_LED.value(0)
      print('LED off \n')
      uart.write('LED off \n')
  uart.write('LED test\n')
  time.sleep(10)