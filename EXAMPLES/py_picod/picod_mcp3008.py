"""
picod_mcp3008.py
2021-04-01
Public Domain

http://abyz.me.uk/picod/py_picod.html
"""
import picod

class MCP3008:
   """
   MCP3008 8 ch 10-bit ADC

   CH0     1 o o 16 V+
   CH1     2 o o 15 Vref
   CH2     3 o o 14 AGND
   CH3     4 o o 13 SCLK
   CH4     5 o o 12 SDO 
   CH5     6 o o 11 SDI 
   CH6     7 o o 10 CS/SHDN
   CH7     8 o o  9 DGND

   Be aware that SDO will be at the same voltage as V+.
   """
   def __init__(self, pico, channel, tx, rx, sck, cs, speed=1e6):
      """
      """
      self._pico = pico
      self._hw = channel
      self._cs = cs

      status, speed = pico.spi_open(channel, tx, rx, sck, speed)
      if status != picod.STATUS_OKAY:
         raise ValueError

   def read_single_ended(self, channel):
      assert 0 <= channel <= 7

      status, d = self._pico.spi_xfer(
         self._hw, self._cs, [1, 0x80+(channel<<4), 0])

      if status == picod.STATUS_OKAY:
         c1 = d[1] & 0x03
         c2 = d[2]
         val = (c1<<8)+c2
         return val

      return None

   def read_differential_plus(self, channel):
      assert 0 <= channel <= 3

      status, d = self._pico.spi_xfer(
         self._hw, self._cs, [1, channel<<5, 0])

      if status == picod.STATUS_OKAY:
         c1 = d[1] & 0x03
         c2 = d[2]
         val = (c1<<8)+c2
         return val

      return None

   def read_differential_minus(self, channel):
      assert 0 <= channel <= 3

      status, d = self._pico.spi_xfer(
         self._hw, self._cs, [1, (channel<<5)+16, 0])

      if status == picod.STATUS_OKAY:
         c1 = d[1] & 0x03
         c2 = d[2]
         val = (c1<<8)+c2
         return val

      return None

   def close(self):
      self._pico.spi_close(self._hw)

if __name__ == "__main__":

   import time
   import picod
   import picod_mcp3008

   pico = picod.pico()

   pico.pico_reset()

   adc = picod_mcp3008.MCP3008(pico, 0, 7, 4, 6, 5, 50000)

   end_time = time.time() + 60

   while time.time() < end_time:
      print(adc.read_single_ended(0))
      time.sleep(0.1)

   adc.close()

