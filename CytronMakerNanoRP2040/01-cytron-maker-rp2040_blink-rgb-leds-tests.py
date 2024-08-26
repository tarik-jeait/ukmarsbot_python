from machine import Pin
import time
import neopixel

# SPECIFIC TESTS FOR Cytron Maker Nano RP2040

def blinkRGBLeds():
    # Initialize Neopixel RGB LEDs
    pixels_pin = machine.Pin(11)
    num_pixels = 2
    pixels = neopixel.NeoPixel(pixels_pin, num_pixels)
    # pixels = neopixel.NeoPixel(pixels_pin, 2)
    pixels[0] = (255, 0, 0) # Red in an RGBY Setup
    pixels[1] = (0, 0, 0)

    pixels.write()    
    time.sleep(0.5)
    pixels[0] = (0, 0, 0) # Orange in an RGBY Setup
    pixels[1] = (0, 0, 255) # Yellow-green in an RGBY Setup
    pixels.write()    
    time.sleep(0.5)


while True:

    blinkRGBLeds()