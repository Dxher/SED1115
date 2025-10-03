from machine import Pin

led1 = Pin(18, Pin.OUT)
sw5 = Pin(22, Pin.IN, Pin.PULL_DOWN)

while True:
	if sw5.value():
		led1.value(1)
	else:
		led1.value(0)
