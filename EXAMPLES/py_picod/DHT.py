"""
DHT.py
2021-04-15
Public Domain

http://abyz.me.uk/picod/py_picod.html

python DHT.py
"""
import time
import picod

DHT22=15

last_tick = 0
bits = 0
value = 0

def _validate_DHT11(b1, b2, b3, b4):
   t = b2
   h = b4
   if (b1 == 0) and (b3 == 0) and (t <= 60) and (h >= 9) and (h <= 90):
      valid = True
   else:
      valid = False
   return (valid, t, h)

def _validate_DHTXX(b1, b2, b3, b4):
   if b2 & 128:
      div = -10.0
   else:
      div = 10.0
   t = float(((b2&127)<<8) + b1) / div
   h = float((b4<<8) + b3) / 10.0
   if (h <= 110.0) and (t >= -50.0) and (t <= 135.0):
      valid = True
   else:
      valid = False
   return (valid, t, h)

def _decode_dhtxx(code):
   """
         +-------+-------+
         | DHT11 | DHTXX |
         +-------+-------+
   Temp C| 0-50  |-40-125|
         +-------+-------+
   RH%   | 20-80 | 0-100 |
         +-------+-------+

            0      1      2      3      4
         +------+------+------+------+------+
   DHT11 |check-| 0    | temp |  0   | RH%  |
         |sum   |      |      |      |      |
         +------+------+------+------+------+
   DHT21 |check-| temp | temp | RH%  | RH%  |
   DHT22 |sum   | LSB  | MSB  | LSB  | MSB  |
   DHT33 |      |      |      |      |      |
   DHT44 |      |      |      |      |      |
         +------+------+------+------+------+
   """
   b0 =  code        & 0xff
   b1 = (code >>  8) & 0xff
   b2 = (code >> 16) & 0xff
   b3 = (code >> 24) & 0xff
   b4 = (code >> 32) & 0xff
     
   chksum = (b1 + b2 + b3 + b4) & 0xFF

   if chksum == b0:
      # Try DHTXX first.
      valid, t, h = _validate_DHTXX(b1, b2, b3, b4)
      if not valid:
         # try DHT11.
         valid, t, h = _validate_DHT11(b1, b2, b3, b4)
      if valid:
         return 0, t, h
      else:
         return 1, 0, 0
   else:
      return 2, 0, 0

def cbf(g,l,t,L):

   global last_tick
   global bits
   global value

   # print(g,l,t,L)

   if l == 1:
      d = picod.tick_diff(last_tick, t) # safe way to subtract ticks
      last_tick = t
      if d > 200000:
         bits = 0
         value = 0
      else:
         if (d > 100):
            value = (value << 1) | 1
         else:
            value = (value << 1)
         bits += 1
   elif l == 2: # watchdog
      status, t, h = _decode_dhtxx(value)
      print("t={} h={} status={} bits={} tick={}".format(
         t, h, status, bits, last_tick))

def toggle():

   pico.gpio_set_output(DHT22, 0, flush=False)

   pico.sleep(0.005, flush=False)

   pico.gpio_set_input(DHT22, flush=True)

pico = picod.pico()
if not pico.connected:
   exit()

pico.reset()

pico.gpio_open(DHT22)

pico.gpio_set_pull(DHT22, picod.PULL_UP)

pico.gpio_set_watchdog(DHT22, 0.05) # watchdog after all bits received

cb = pico.callback(DHT22, picod.EDGE_RISING, cbf)

while True:

   toggle()

   time.sleep(2)

