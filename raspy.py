# for the raspberry pi
import RPi.GPIO as GPIO
import time

# Set up the GPIO mode to BCM (Broadcom pin-numbering)
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin where the switch is connected (e.g., GPIO17)
SWITCH_PIN = 18

# Set the GPIO pin as an input with an internal pull-down resistor
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

i = 0
try:
    while True:
        # Read the state of the switch
        switch_state = GPIO.input(SWITCH_PIN)
        
        # Print the state of the switch (1 means pressed, 0 means not pressed)
        if switch_state == GPIO.HIGH:
            print("Switch is Pressed {}".format(i))
        else:
            print("Switch is Released {}".format(i))
        i += 1
        # Small delay to prevent overwhelming the console with print statements
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped")
    
finally:
    # Clean up the GPIO settings before exit
    GPIO.cleanup()