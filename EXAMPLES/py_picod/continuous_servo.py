"""
continuous_servo.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

python continuous_servo.py
"""
import time
import picod

GPIO = 5

def servo(gpio, speed_percent):
   
   if speed_percent > 100:
      speed_percent = 100;
   elif speed_percent < -100:
      speed_percent = -100;

   pw = 1500 + (speed_percent * 5)

   pico.tx_servo(gpio, pw)

pico = picod.pico()
if not pico.connected:
   exit()

servo(GPIO, 50) # half clockwise
time.sleep(1)
servo(GPIO, 0) # stop
time.sleep(1)
servo(GPIO, -50) # half anticlockwise
time.sleep(1)

pico.tx_servo(GPIO, 0) # servo off
pico.close()
