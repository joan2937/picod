#!/usr/bin/env python
"""
monitor.py
2021-04-15
Public Domain

http://abyz.me.uk/picod/py_picod.html

./monitor.py gpio ...

e.g.

./monitor.py 23 24 25        # monitor 23,24,25
"""

import sys
import time
import picod

last_tick = [None] * 30

def cbf(gpio, level, tick, levels):
   if last_tick[gpio] is not None:
      print("gpio={} level={} ticks={} levels=0x{:x}".format(
         gpio, level,  picod.tick_diff(last_tick[gpio], tick), levels))
   last_tick[gpio] = tick

pico = picod.pico()
if not pico.connected:
   exit()

argc = len(sys.argv)

for i in range(1, argc):

   gpio = int(sys.argv[i])

   pico.callback(gpio, picod.EDGE_BOTH, cbf)


while True:
   time.sleep(1)

