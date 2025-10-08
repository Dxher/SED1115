from machine import Pin
import time

led1 = Pin(18, Pin.OUT)
sw5 = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Coding challenge 1: Turn on the LED when the button is not pressed
"""
while True:
	if sw5.value() == 0:
		led1.on()
	else:
		led1.off()
"""

#Coding challenge 2: Utilizing the other 4 buttons. 
"""
led2 = Pin(17, Pin.OUT)
led3 = Pin(20, Pin.OUT)
led4 = Pin(16, Pin.OUT)
pico_led = Pin("LED", Pin.OUT)
sw1 = Pin(10, Pin.IN, Pin.PULL_DOWN)
sw2 = Pin(11, Pin.IN, Pin.PULL_DOWN)
sw3 = Pin(12, Pin.IN, Pin.PULL_DOWN)
sw4 = Pin(13, Pin.IN, Pin.PULL_DOWN)

while True:
	if sw5.value():
		led1.value(1)
	elif sw4.value():
		led2.value(1)
	elif sw3.value():
		led3.value(1)
	elif sw2.value():
		led4.value(1)
	elif sw1.value():
		pico_led.value(1)
	else:
		led1.value(0)
		led2.value(0)
		led3.value(0)
		led4.value(0)
		pico_led.value(0)
"""
# Coding challenge 3: Toggle the LED state with each button press

# Method 1: Using a state variable to prevent multiple toggles while the button is held down
"""
prev_state = 1
led1.value(0)

while True:
    current_state = sw5.value()
    
    if prev_state == 0 and current_state == 1:
        led1.toggle()
    
    prev_state = current_state
    time.sleep_ms(10)
"""

# Method 2: Toggling within an if statement checking if it's pressed, to prevent multiple toggles
#"""
while True:
	# In the event the button is held down, this if will prevent multiple toggles
    if sw5.value() == 0:
        time.sleep_ms(40)
        # Toggles as soon as the button is pressed
        if sw5.value():
            led1.toggle()
#"""