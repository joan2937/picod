"""
pwm_read.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

# Assumes a wire is connected between GPIO 16 and 17.

python pwm_read.py
"""
import time
import picod

OUT_GPIO = 14
IN_GPIO = 13

FREQ=1250000
DUTY=25.0

pico = picod.pico()
if not pico.connected:
   exit()

pico.reset() # free all GPIO and hardware

pico.tx_pwm(OUT_GPIO, FREQ, DUTY)

print("Setting frequency={}, dutycycle={}".format(FREQ, DUTY))

# Read PwM frequency
for i in range(10):
   status, frequency = pico.pwm_read_frequency(IN_GPIO)
   print("frequency={}".format(frequency))
   time.sleep(0.3)

# Read PwM dutycycle
for i in range(10):
   status, dutycycle = pico.pwm_read_dutycycle(IN_GPIO)
   print("dutycycle={:.2f}".format(dutycycle))
   time.sleep(0.3)

# Read high edges
for i in range(10):
   status, count, seconds = pico.pwm_read_high_edges(IN_GPIO)
   print("count={} seconds={:.2f}".format(count, seconds))
   time.sleep(0.3)

pico.tx_close(OUT_GPIO)

