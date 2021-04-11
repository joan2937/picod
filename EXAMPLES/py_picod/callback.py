"""
callback.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

python callback.py
"""
import time
import picod

GPIO=17

lastFalling = None
lastRising = None
lastBoth = None

def cbfFalling(gpio, level, tick, levels):
   global lastFalling
   if lastFalling is not None:
      print("Falling: GPIO={} L={} {:.3f}".format(
         gpio, level, (tick-lastFalling)/1e6))
   lastFalling = tick

def cbfRising(gpio, level, tick, levels):
   global lastRising
   if lastRising is not None:
      print(" Rising: GPIO={} L={} {:.3f}".format(
         gpio, level, (tick-lastRising)/1e6))
   lastRising = tick

def cbfBoth(gpio, level, tick, levels):
   global lastBoth
   if lastBoth is not None:
      print("   Both: GPIO={} L={} {:.3f}".format(
         gpio, level, (tick-lastBoth)/1e6))
   lastBoth = tick

pico = picod.pico()
if not pico.connected:
   exit()

pico.gpio_set_alert(GPIO, True)

cb1 = pico.callback(GPIO, picod.EDGE_BOTH, cbfBoth)
cb2 = pico.callback(GPIO, picod.EDGE_FALLING, cbfFalling)
cb3 = pico.callback(GPIO, picod.EDGE_RISING, cbfRising)

pico.tx_pwm(GPIO, 8, 33.33)

for i in range (5):
   time.sleep(1.0)

time.sleep(0.5)
cb1.cancel()

time.sleep(0.5)
cb2.cancel()

time.sleep(0.5)
cb3.cancel()

pico.tx_close(GPIO)

