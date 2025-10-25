from machine import Pin, I2C
import time

# Button
sw2 = Pin(11, Pin.IN, Pin.PULL_DOWN)

#I2C for DS3231
i2c = I2C(1, sda=Pin(14), scl=Pin(15))

def read_seconds():
    # [0] removes bytes object, &0x7F to ignore the CH bit
    binaryCount = i2c.readfrom_mem(0x68, 0x00, 1)[0] & 0x7F
    # Upper 4 bits (dizaine) and lower 4 bits (unite) making the seconds value
    decimalCount = (binaryCount >> 4) * 10 + (binaryCount & 0x0F)
    return decimalCount

def wait_for_press(pin):
    # not pressed
    while not pin.value():
        time.sleep_ms(5)
    #wait
    time.sleep_ms(10)
    # pressed
    while pin.value():
        time.sleep_ms(5)

print("Press SW2 to start counting 15 seconds")

log = open("log.txt", "a")
try:
    while True:
        # Start
        wait_for_press(sw2)
        start_sec = read_seconds()
        print("Started! Count 15 seconds in your head...")

        # Stop
        wait_for_press(sw2)
        end_sec = read_seconds()

        # Elapsed time
        elapsed = (end_sec - start_sec) % 60
        print(f"Elapsed: {elapsed} s")
        log.write(f"{elapsed}\n")
        log.flush()

        print("Press SW2 to play again (Ctrl+C to quit).")
except KeyboardInterrupt:
    print("Exiting, log saved.")
finally:
    log.close()