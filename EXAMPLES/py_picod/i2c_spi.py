"""
i2c_spi.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html

python i2c_spi.py
"""
import time
import picod
import picod_mcp3008

pico = picod.pico()
if not pico.connected:
   exit()

pico.reset() # free all GPIO and hardware

adc = picod_mcp3008.MCP3008(pico, 0, 7, 4, 6, 5, 50000) # ch, tx, rx, sck, cs

pico.i2c_open(1, 2, 3, 100000) # ch, sda, scl, speed

dac = 0
inc = 1

end_time = time.time() + 60

while time.time() < end_time:

   pico.i2c_write(1, 0x48, [0x40, dac]) # set the output voltage

   print(adc.read_single_ended(0))

   dac = dac + inc

   if dac > 255:
      dac = 255
      inc = -1
   elif dac < 0:
      dac = 0
      inc = 1

adc.close() # release SPI GPIO

pico.i2c_close(1) # release I2C GPIO

