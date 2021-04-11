"""
adc.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

python adc.py
"""
import picod

pico = picod.pico()
if not pico.connected:
   exit()

pico.reset()

for i in range(100):
   ostr = ""
   for channel in range(5):
      status, ch, val = pico.adc_read(channel)
      if status == picod.STATUS_OKAY:
         ostr += "{}={:<4d} ".format(ch, val)
   print(ostr)
