"""
flash.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

python flash.py
"""
import time
import picod

LED=25

pico = picod.pico()
if not pico.connected:
   exit()

pico.reset()

pico.tx_pwm(LED, 7.5, 5)

time.sleep(5)

pico.tx_close(LED)
