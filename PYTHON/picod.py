"""
[http://abyz.me.uk/picod/py_picod.html]

picod is a Python module which allows remote control of the GPIO
and other functions of a Pico running the picod daemon.

The picod daemon must be running on the Pico you wish to control.

*Features*

o the picod Python module can run on Windows, Macs, or Linux
o reading and writing GPIO singly and in groups
o hardware timed PWM
o hardware timed servo pulses
o GPIO and event callbacks
o I2C wrapper
o SPI wrapper
o serial link wrapper

*Exceptions*

A fatal exception is raised if you pass an invalid
argument to a picod function.

*Usage*

The picod daemon must be running on the Pico(s) you want to use.

Your Python program must import picod and create one or more instances
of the picod.pico class. Each instance gives access to a specific Pico.

...
import picod

picoA = picod.pico(device='/dev/ttyACM0')
if not picoA.connected:
   exit()

picoB = picod.pico(device='com4')
if not picoB.connected:
   exit()
...

The example code snippets assume that pico is an instance of the
picod.pico class.

OVERVIEW

ESSENTIAL

picod.pico           Opens a connection to the Pico
close                Closes the connection to the Pico

GPIO

GPIO_open            Opens a group of GPIO
GPIO_close           Closes a group of GPIO

GPIO_set_dir         Sets the in/out direction and level of a group of GPIO

GPIO_read            Returns the levels of all GPIO
GPIO_write           Sets the levels of a group of GPIO

GPIO_set_pulls       Sets the pulls of a group of GPIO
GPIO_get_pulls       Returns the pulls of all GPIO

GPIO_set_functions   Sets the functions of a group of GPIO
GPIO_get_functions   Returns the functions of all GPIO

GPIO_set_alerts      Sets alerts on or off for a group of GPIO

gpio_open            Opens a single GPIO
gpio_close           Closes a single GPIO

gpio_set_input       Sets a single GPIO as an input
gpio_set_output      Sets a single GPIO as an output with an initial level

gpio_read            Returns the level of a single GPIO
gpio_write           Sets the level of a single GPIO

gpio_set_pull        Sets the pulls for a single GPIO
gpio_get_pull        Returns the pulls of a single GPIO

gpio_set_function    Sets the function of a single GPIO
gpio_get_function    Returns the function of a single GPIO

gpio_set_alert       Sets alerts on or off for a single GPIO

gpio_set_debounce    Sets the debounce time for a single GPIO
gpio_set_watchdog    Sets the watchdog time for a single GPIO

ADC

adc_read             Returns the value of an ADC channel
adc_close            Closes an ADC channel and frees any associated GPIO

I2C

i2c_open             Opens an I2C channel (master or slave)
i2c_close            Closes an I2C channel and frees associated GPIO
i2c_read             Returns count bytes from an address on an I2C channel
i2c_write            Writes data to an address on an I2C channel
i2c_pop              Returns up to count bytes from an I2C slave channel
i2c_push             Writes data to an I2C slave channel

PWM/SERVO

tx_pwm               Starts hardware PWM pulses on a single GPIO
tx_servo             Starts hardware servo pulses on a single GPIO
tx_close             Stops PWM/Servo pulses and frees the associated GPIO

PWM_READ

pwm_read_dutycycle   Returns the PWM dutycycle of a single GPIO
pwm_read_frequency   Returns the PWM frequency of a single GPIO
pwm_read_high_edges  Returns the high edge count of a single GPIO

SERIAL

serial_open          Opens a serial channel
serial_close         Closes a serial channel and frees associated GPIO

serial_read          Returns count bytes from a serial channel
serial_write         Writes data to a serial channel

SPI

spi_open             Opens a SPI channel (master or slave)
spi_close            Closes a SPI channel and frees associated GPIO

spi_read             Returns count bytes from a SPI channel
spi_write            Writes data to a SPI channel
spi_xfer             Transfers (reads and writes) data to a SPI channel

spi_pop              Returns up to count bytes from a SPI slave channel
spi_push             Writes data to a SPI slave channel

UTILITIES

reset                Resets the Pico (frees GPIO, close I2C, SPI, serial)
sleep                Sleeps the Pico
tick                 Returns the current Pico tick
uid                  Returns the Pico's unique id (64-bit number)
version              Returns the Pico software version (dotted quad)
set_config_value     Sets the value of an internal configuration item
get_config_value     Gets the value of an internal configuration item

CALLBACKS

callback             Starts an alert callback for a single GPIO
event_callback       Start an event callback
reply_callback       Starts a later reply callback for a command

MODULE

modver               Returns the picod Python module version (dotted quad)
status_text          Returns the text associated with a numeric status code
tick_diff            Returns the difference between two ticks
"""
import sys
import os
import time
import struct
import binascii
import threading
import atexit

VERSION = 0x00000600

CLOCK_HZ = 125e6

FUNC_XIP = 0
FUNC_SPI = 1
FUNC_UART = 2
FUNC_I2C = 3
FUNC_PWM = 4
FUNC_SIO = 5
FUNC_GPIO = 5
FUNC_PIO0 = 6
FUNC_PIO1 = 7
FUNC_GPCK = 8
FUNC_USB = 9
FUNC_NULL = 15

UART_TX = ((0, 12, 16, 28, 255), (4, 8, 20, 24, 255))

UART_RX = ((1, 13, 17, 29, 255), (5, 9, 21, 25, 255))

UART_CTS = ((2, 14, 18, 255), (6, 10, 22, 26, 255))

UART_RTS = ((3, 15, 19, 255), (7, 11, 23, 27, 255))

I2C_SDA = ((0, 4, 8, 12, 16, 20, 24, 28), (2, 6, 10, 14, 18, 22, 26))

I2C_SCL = ((1, 5, 9, 13, 17, 21, 25, 29), (3, 7, 11, 15, 19, 23, 27))

SPI_RX =  ((0, 4, 16, 20), ( 8, 12, 24, 28))

SPI_CS =  ((1, 5, 17, 21), ( 9, 13, 25, 29))

SPI_SCK = ((2, 6, 18, 22), (10, 14, 26))

SPI_TX =  ((3, 7, 19, 23), (11, 15, 27))

PARITY_NONE = 0
PARITY_EVEN = 1
PARITY_ODD = 2

PULL_NONE = 0
PULL_DOWN = 1
PULL_UP = 2
PULL_BOTH = 3

GPIO_MIN = 0
GPIO_MAX = 29

EDGE_RISING = 0
EDGE_FALLING = 1
EDGE_BOTH = 2

LEVEL_LOW = 0
LEVEL_HIGH = 1
LEVEL_TIMEOUT = 2

WATCHDOG_BIT = (1<<31)

STATUS_OKAY = 0
STATUS_BAD_CHANNEL = 1
STATUS_CHANNEL_CLOSED = 2
STATUS_BAD_GPIO = 3
STATUS_BAD_PARAM = 4
STATUS_BAD_WRITE = 5
STATUS_BAD_READ = 6
STATUS_NO_REPLY = 7
STATUS_GPIO_IN_USE = 8
STATUS_UNKNOWN_COMMAND = 9
STATUS_TIMED_OUT = 10
STATUS_INVALID_WHEN_MASTER = 11
STATUS_INVALID_WHEN_SLAVE = 12
STATUS_BAD_CONFIG_ITEM = 13

_status = {
   STATUS_OKAY: "okay",
   STATUS_BAD_CHANNEL: "bad channel",
   STATUS_CHANNEL_CLOSED: "channel closed",
   STATUS_BAD_GPIO: "bad GPIO",
   STATUS_BAD_PARAM: "bad parameter",
   STATUS_BAD_WRITE: "bad write",
   STATUS_BAD_READ: "bad read",
   STATUS_NO_REPLY: "no reply expected",
   STATUS_GPIO_IN_USE: "GPIO already in use",
   STATUS_UNKNOWN_COMMAND: "unknown command",
   STATUS_TIMED_OUT: "command timed out",
   STATUS_INVALID_WHEN_MASTER: "invalid command when master",
   STATUS_INVALID_WHEN_SLAVE: "invalid command when slave",
   STATUS_BAD_CONFIG_ITEM: "invalid configuration item",
}

REPLY_NONE = 0
REPLY_NOW = 1
REPLY_LATER = 2

_CMD_GPIO_OPEN = 10
_CMD_GPIO_CLOSE = 11
_CMD_GPIO_SET_IN_OUT = 12
_CMD_GPIO_READ = 13
_CMD_GPIO_WRITE = 14

_CMD_PULLS_SET = 20
_CMD_PULLS_GET = 21

_CMD_FUNCTION_SET = 25
_CMD_FUNCTION_GET = 26

_CMD_ALERT_DEBOUNCE = 30
_CMD_ALERT_WATCHDOG = 31
_CMD_ALERT_SELECT = 32

_CMD_EVT_CONFIG = 35

_CMD_ADC_READ = 40
_CMD_ADC_CLOSE = 41

_CMD_I2C_OPEN = 50
_CMD_I2C_CLOSE = 51
_CMD_I2C_READ = 52
_CMD_I2C_WRITE = 53
_CMD_I2C_PUSH = 54
_CMD_I2C_POP = 55

_CMD_PWM_READ_FREQ = 60
_CMD_PWM_READ_DUTY = 61
_CMD_PWM_READ_EDGE = 62
_CMD_PWM = 63
_CMD_SERVO = 64
_CMD_PWM_CLOSE = 65

_CMD_SPI_OPEN = 70
_CMD_SPI_CLOSE = 71

_CMD_SPI_READ = 75
_CMD_SPI_WRITE = 76
_CMD_SPI_XFER = 77
_CMD_SPI_PUSH = 78
_CMD_SPI_POP = 79

_CMD_UART_OPEN = 85
_CMD_UART_CLOSE = 86
_CMD_UART_READ = 87
_CMD_UART_WRITE = 88

_CMD_UID = 90

_CMD_TICK = 94
_CMD_SLEEP_US = 95
_CMD_RESET_PICO = 96
_CMD_SET_CONFIG_VAL = 97
_CMD_GET_CONFIG_VAL = 98
_CMD_PD_VERSION = 99

MSG_HEADER = 0xff
MSG_HEADER_LEN = 5

MSG_BAD_CHECKSUM = 0xfe
MSG_BAD_LENGTH =  0xfd
MSG_GPIO_LEVELS = 0xfb
MSG_DEBUG = 0xfa
MSG_ERROR = 0xf9
MSG_ASYNC = 0xf8

EVT_UART_0_RX = 0
EVT_UART_1_RX = 1
EVT_I2C_0_RX = 2
EVT_I2C_1_RX = 3
EVT_SPI_0_RX = 4
EVT_SPI_1_RX = 5
EVT_MAX_RX = 5

EVT_I2C_0_TX = 6
EVT_I2C_1_TX = 7
EVT_SPI_0_TX = 8
EVT_SPI_1_TX = 9
EVT_BUFS = 9

EVENT_NONE = 0
EVENT_ACTIVITY = 1
EVENT_COUNT = 2
EVENT_RETURN_COUNT = 3
EVENT_RETURN_COUNT_PLUS = 4

def _tobuf(x):
   l = bytearray()
   if isinstance(x, (bytes, bytearray)):
      return x
   elif isinstance(x, (str)):
      return x.encode('latin-1')
   elif isinstance(x, (list, tuple)):
      return bytearray(x)
   else:
      raise TypeError

def _byte2hex(s):
   return "".join("{:02x} ".format(c) for c in bytearray(s))

class _callback_ADT:
   """
   An ADT class to hold level callback information.
   """

   def __init__(self, gpio, edge, func):
      """
      Initialises a callback ADT.

          gpio:= GPIO number in device.
          edge:= EDGE_BOTH, EDGE_RISING, or EDGE_FALLING.
          func:= a user function taking four arguments
                 (gpio, level, tick, levels).
      """
      self.gpio = gpio
      self.edge = edge
      self.func = func
      self.bit = 1<<gpio

class _reply_ADT:
   """
   An ADT class to hold reply callback information.
   """

   def __init__(self, thread_id, command_id, func):
      """
      Initialises a reply later callback ADT.

          thread_id:= the requesting thread.
          command_id:= the command of interest
          func:= a user function taking three arguments
                 (command_id, status, reply).
      """
      self.thread_id = thread_id
      self.command_id = command_id
      self.func = func

class _event_ADT:
   """
   An ADT class to hold event callback information.
   """

   def __init__(self, event_id, func):
      """
      Initialises an event callback ADT.

          event_id:= the event event of interest
          func:= a user function taking three arguments
                 (event_id, count, reply).
      """
      self.event_id = event_id
      self.func = func

class _callback_thread(threading.Thread):
   """
   A class to encapsulate notification callbacks.
   """

   def __init__(self, pico):
      """
      Initialises notifications.
      """
      threading.Thread.__init__(self)
      self.pico = pico
      self._pico_serial_read = pico._pico_serial_read
      self.daemon = True
      self.monitor = 0
      self.level_callbacks = []
      self.reply_callbacks = []
      self.event_callbacks = []
      self.lastLevel = 0
      self.go = True
      self.start()

   def stop(self):
      """
      Stops notifications.
      """
      if self.go:
         self.go = False

   def append_level_callback(self, callb):
      """
      Adds a level callback to the notification thread.
      """
      self.level_callbacks.append(callb)
      self.monitor = self.monitor | callb.bit
      self.pico.GPIO_set_alerts(0xffffffff, self.monitor)

   def remove_level_callback(self, callb):
      """
      Removes a level callback from the notification thread.
      """
      if callb in self.level_callbacks:
         self.level_callbacks.remove(callb)
         newMonitor = 0
         for c in self.level_callbacks:
            newMonitor |= c.bit
         if newMonitor != self.monitor:
            self.monitor = newMonitor
            self.pico.GPIO_set_alerts(0xffffffff, self.monitor)

   def append_reply_callback(self, callb):
      """
      Adds a reply callback to the notification thread.
      """
      self.reply_callbacks.append(callb)

   def remove_reply_callback(self, callb):
      """
      Removes a reply callback from the notification thread.
      """
      if callb in self.reply_callbacks:
         self.reply_callbacks.remove(callb)

   def append_event_callback(self, callb):
      """
      Adds an event callback to the notification thread.
      """
      for cb in self.event_callbacks:
         if cb.event_id == callb.event_id:
            self.event_callbacks.remove(cb)
      self.event_callbacks.append(callb)

   def remove_event_callback(self, callb):
      """
      Removes an event callback from the notification thread.
      """
      if callb in self.event_callbacks:
         self.event_callbacks.remove(callb)

   def run(self):
      """
      Runs the notification thread.
      """
      lastLevel = self.lastLevel
      buf = bytes()
      in_message = False

      """
      <------------ Length bytes ------------>
      +---+-------+-------+----------+-------+
      |Hdr|Length | CRC1  |Request(s)| CRC2  |
      |xFF|msb|lsb|msb|lsb|          |msb|lsb|
      +---+---+---+---+---+----------+---+---+

      CRC1: Hdr+Length
      CRC2: Hdr+Length+CRC1+Request(s)
      """
      while self.go:
         d = self._pico_serial_read(5000)
         if len(d):
            #print("serial_read", _byte2hex(d))
            if d[0] != 255 and not in_message:
               #print(_byte2hex(d))
               pass
            buf += d
         if len(buf) >= MSG_HEADER_LEN:
            if not in_message:
               while not in_message and len(buf) >= MSG_HEADER_LEN:
                  if buf[0] == MSG_HEADER:
                     msgLen, crc1 = struct.unpack('>HH', buf[1:5])
                     crc = binascii.crc_hqx(buf[:3], 0)
                     if crc == crc1:
                        #print("som")
                        in_message = True
                        break
                  else:
                     buf = buf[1:]
            if in_message:
               if len(buf) >= msgLen:
                  crc2, = struct.unpack('>H', buf[msgLen-2:msgLen])
                  crc = binascii.crc_hqx(buf[:msgLen-2], 0)
                  if crc == crc2:
                     #print("good message")
                     # lose header
                     buf = buf[MSG_HEADER_LEN:]
                     """
                     Request
                     <-------- Length bytes ------->
                     +-------+---+---+-------------+
                     |Length |Flg|Req|Optional data|
                     |msb|lsb|   |   |             |
                     +---+---+---+---+-------------+
                     """
                     length, = struct.unpack('>H', buf[:2])
                     if buf[3] == MSG_GPIO_LEVELS: # level report
                        reports = int((length-4) / 8)
                        #print("# rxd {}".format(reports))
                        for i in range(reports):
                           p = (i*8)+4
                           tick, levels = struct.unpack(">II", buf[p:p+8])
                           if levels & WATCHDOG_BIT:
                              for cb in self.level_callbacks:
                                 if cb.bit & levels:
                                    cb.func(cb.gpio, LEVEL_TIMEOUT, tick,
                                       lastLevel)
                           else:
                              changed = levels ^ lastLevel
                              lastLevel = levels
                              for cb in self.level_callbacks:
                                 if cb.bit & changed:
                                    level = 0
                                    if cb.bit & levels:
                                       level = 1
                                    if (cb.edge ^ level):
                                       cb.func(cb.gpio, level, tick, levels)
                     elif buf[3] == MSG_DEBUG:
                        print(buf[4:length])
                     elif buf[3] == MSG_ERROR:
                        print(buf[4:length])
                     elif buf[3] == MSG_ASYNC:
                        for cb in self.event_callbacks:
                           if (cb.event_id == buf[4]):
                              cb.func(buf[4], buf[5]<<8|buf[6], buf[7:length])
                     else: # sync to correct queue or reply callback
                        queue = buf[2] & 63
                        reply = (buf[2] >> 6) & 3
                        if reply == REPLY_NOW:
                           self.pico._sync[queue].append(buf[3:length])
                        else:
                           for cb in self.reply_callbacks:
                              if (cb.thread_id == queue and
                                 cb.command_id == buf[3]):
                                 cb.func(buf[3], buf[4], buf[5:length])
                     buf = buf[msgLen-MSG_HEADER_LEN:]                     
                  else:
                     print("bad crc {:04x} != {:04x}".format(crc, crc2))
                     buf = buf[msgLen:]
                  #print("out msg", _byte2hex(buf))
                  in_message = False
         time.sleep(0.01)

class _level_callback:
   """
   A class to provide GPIO level change callbacks.
   """

   def __init__(self, notify, gpio, edge=EDGE_RISING, func=None):
      """
      Initialise a callback and adds it to the notification thread.
      """
      self._notify = notify
      self.count=0
      self._reset = False
      if func is None:
         func=self._tally
      self.callb = _callback_ADT(gpio, edge, func)
      self._notify.append_level_callback(self.callb)

   def cancel(self):
      """
      Cancels a callback by removing it from the notification thread.
      """
      self._notify.remove_level_callback(self.callb)

   def _tally(self, gpio, level, tick, levels):
      """
      Increment the callback called count.
      """
      if self._reset:
         self._reset = False
         self.count = 0
      self.count += 1

   def tally(self):
      """
      Provides a count of how many times the default tally
      callback has triggered.

      The count will be zero if the user has supplied their own
      callback function.
      """
      return self.count

   def reset_tally(self):
      """
      Resets the tally count to zero.
      """
      self._reset = True
      self.count = 0

class _reply_callback:
   """
   A class to provide reply callbacks.
   """

   def __init__(self, notify, command_id, func):
      """
      Initialise a reply callback and adds it to the notification thread.
      """
      self._notify = notify
      self.callb = _reply_ADT(self._thread_data.queue, command_id, func)
      self._notify.append_reply_callback(self.callb)

   def cancel(self):
      """
      Cancels a reply callback by removing it from the notification
      thread.
      """
      self._notify.remove_reply_callback(self.callb)

class _event_callback:
   """
   A class to provide event callbacks.
   """

   def __init__(self, notify, event_id, func):
      """
      Initialise an event callback and adds it to the notification thread.
      """
      self._notify = notify
      self.callb = _event_ADT(event_id, func)
      self._notify.append_event_callback(self.callb)

   def cancel(self):
      """
      Cancels an event callback by removing it from the notification
      thread.
      """
      self._notify.remove_event_callback(self.callb)

class pico():

   def _message(self, request=bytearray()):
      """
      <------------ Length bytes ------------>
      +---+-------+-------+----------+-------+
      |Hdr|Length | CRC1  |Request(s)| CRC2  |
      |xFF|msb|lsb|msb|lsb|          |msb|lsb|
      +---+---+---+---+---+----------+---+---+

      CRC1: Hdr+Length
      CRC2: Hdr+Length+CRC1+Request(s)
      """

      length = len(request) + MSG_HEADER_LEN + 2

      msg = struct.pack(">BH", 0xff, length)

      crc1 = binascii.crc_hqx(msg, 0)

      msg += struct.pack(">H", crc1)

      msg += request

      crc2 = binascii.crc_hqx(msg, 0)

      msg += struct.pack(">H", crc2)

      #print("serial_write", _byte2hex(msg))
      self._pico_serial_write(msg)

   def _request(self, req, data=bytearray(), reply=REPLY_NOW, flush=True):
      """
      Request
      <-------- Length bytes ------->
      +-------+---+---+-------------+
      |Length |Flg|Req|Optional data|
      |msb|lsb|   |   |             |
      +---+---+---+---+-------------+
      """

      length = len(data) + 4

      flags = (reply << 6) | self._thread_data.queue

      msg = struct.pack(">HBB", length, flags, req) + data

      self._pending += msg

      if reply == REPLY_NOW or flush:
         self._message(self._pending)
         self._pending = bytearray()

      if reply == REPLY_NOW:
         until = time.time() + 2.0
         while True:
            if len(self._sync[self._thread_data.queue]):
               data = self._sync[self._thread_data.queue].pop(0)
               #print(_byte2hex(data))
               if data[0] == req:
                  return data[1], data[2:]
            if time.time() > until:
               break
            time.sleep(0.01)
         return STATUS_TIMED_OUT, None

      return STATUS_NO_REPLY, None

   # GPIO --------------------------------------------------------------------

   def GPIO_open(self, GPIO, reply=REPLY_NOW, flush=True):
      """
      Opens a group of GPIO.

      GPIO:= the GPIO to be opened.

      if bit x of [#GPIO#] is set then GPIO x is opened.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      pico.GPIO_open(255) # open GPIO 0-7

      if pico.GPIO_open(0x500) == picod.STATUS_OKAY:
         print("GPIO 8 and 10 opened okay")
      else:
         print("GPIO 8 and 10 not opened okay")
      [#GPIO 8 and 10 opened okay#]
      ...
      """

      data = struct.pack(">I", GPIO)

      return self._request(_CMD_GPIO_OPEN, data, reply=reply, flush=flush)[0]

   def gpio_open(self, gpio, reply=REPLY_NOW, flush=True):
      """
      Opens a single GPIO.

      gpio:= the GPIO to be opened.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      pico.gpio_open(25) # open GPIO 25

      if pico.gpio_open(11) == picod.STATUS_OKAY:
         print("GPIO 11 opened okay")
      else:
         print("GPIO 11 not opened okay")
      [#GPIO 11 opened okay#]
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      return self.GPIO_open(1<<gpio, reply=reply, flush=flush)

   def GPIO_close(self, GPIO, reply=REPLY_NONE, flush=True):
      """
      Closes a group of GPIO.

      GPIO:= the GPIO to be closed.

      If bit x of [#GPIO#] is set then GPIO x is closed.

      Nothing is returned.

      ...
      picod.GPIO_close(10) # close GPIO 3 and 1.
      ...
      """

      data = struct.pack(">I", GPIO)

      self._request(_CMD_GPIO_CLOSE, data, reply=reply, flush=flush)

   def gpio_close(self, gpio, reply=REPLY_NONE, flush=True):
      """
      Closes a single GPIO.

      gpio:= the GPIO to be closed.

      Nothing is returned.

      ...
      picod.gpio_close(10) # close GPIO 10.
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      self.GPIO_close(1<<gpio, reply=reply, flush=flush)

   def GPIO_set_dir(self,
      inout_GPIO, out_GPIO, out_LEVEL, reply=REPLY_NONE, flush=True):
      """
      Sets the in/out direction and level of a group of GPIO.

      The level is only set for those GPIO set as outputs.

      inout_GPIO:= the GPIO to be set as inputs or outputs.
        out_GPIO:= the GPIO to be set as outputs.
       out_LEVEL:= the level of the output GPIO.

      If bit x of [#inout_GPIO#] is set then GPIO x is set to be an input or
      an output according to [#out_GPIO#].

      If bit x of [#out_GPIO#] is set then GPIO x is set as an output otherwise
      it is set as an input.  Only GPIO selected by [#inout_GPIO#] are affected.

      If bit x of [#out_LEVEL#] is set then GPIO x is set high otherwise it
      is set low.   Only GPIO selected by [#inout_GPIO#] and [#out_GPIO#]
      are affected.

      Nothing is returned.

      ...
      # set direction of GPIO 0-7.
      # GPIO 0-3 are outputs, GPIO 4-7 are inputs.
      # GPIO 0 and 2 are set high, GPIO 1 and 3 are set low.

      pico.GPIO_set_dir(0xff, 0x0f, 0x05)
      ...
      """

      data = struct.pack(">III", inout_GPIO, out_GPIO, out_LEVEL)

      self._request(_CMD_GPIO_SET_IN_OUT, data, reply=reply, flush=flush)

   def gpio_set_input(self, gpio, reply=REPLY_NONE, flush=True):
      """
      Sets a single GPIO as an input.

      gpio:= the GPIO to be set as an input.

      Nothing is returned.

      ...
      pico.gpio_set_input(15) # Set GPIO 15 as an input.
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      self.GPIO_set_dir(1<<gpio, 0, 0, reply=reply, flush=flush)

   def gpio_set_output(self, gpio, level, reply=REPLY_NONE, flush=True):
      """
      Sets a single GPIO as an output with an initial level.

       gpio:= the GPIO to be set as an output.
      level:= 0 (low) or 1 (high).

      Nothing is returned.

      ...
      pico.gpio_set_output(25, 1) # Set GPIO 25 as a high output.
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert 0 <= level <= 1

      self.GPIO_set_dir(1<<gpio, 1<<gpio, level<<gpio, reply=reply, flush=flush)

   def GPIO_read(self, reply=REPLY_NOW, flush=True):
      """
      Returns the current levels of all GPIO.

      If GPIO x is high bit x of the returned levels will be set.

      If GPIO x is low bit x of the returned levels will be clear.

      A tuple of status and the levels is returned.

      ...
      status, levels = pico.GPIO_read()

      if status == picod.STATUS_OKAY:
         print("GPIO levels are 0x{:x}".format(levels))
      [#GPIO levels are 0x100800e#]

         if levels & (1<<6): # check gpio 6
            print("gpio 6 is high")
         else:
            print("gpio 6 is low")
      [#gpio 6 is low#]
      ...
      """

      status, data = self._request(_CMD_GPIO_READ, reply=reply, flush=flush)

      if status == STATUS_OKAY:
         self._GPIO_levels, = struct.unpack(">I", data)

      return status, self._GPIO_levels

   def gpio_read(self, gpio, reply=REPLY_NOW, flush=True):
      """
      Returns the level of a single GPIO.

      gpio:= the GPIO to be read.

      A tuple of status and the level is returned.

      ...
      status, level = pico.gpio_read(23)

      if status == picod.STATUS_OKAY:
         print("GPIO 23 is {}".format(level))
      [#GPIO 23 is 0#]
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      status, bits = self.GPIO_read(reply=reply, flush=flush)

      level = None

      if status == STATUS_OKAY:
         if bits & (1<<gpio):
            level = 1
         else:
            level = 0

      return status, level

   def GPIO_write(self, out_GPIO, out_LEVEL, reply=REPLY_NONE, flush=True):
      """
      Sets the level of a group of GPIO.

       out_GPIO:= the GPIO to be written.
      out_LEVEL:= the level of the output GPIO.

      If bit x of [#out_LEVEL#] is set then GPIO x is set high otherwise it
      is set low.   Only GPIO selected by [#out_GPIO#] are affected.

      Nothing is returned.

      ...
      pico.GPIO_write(7, 5) # set GPIO 0 high, GPIO 1 low, GPIO 2 high.
      ...
      """

      data = struct.pack(">II", out_GPIO, out_LEVEL)

      self._request(_CMD_GPIO_WRITE, data, reply=reply, flush=flush)

   def gpio_write(self, gpio, level, reply=REPLY_NONE, flush=True):
      """
      Sets the level of a single GPIO.

       gpio:= the GPIO to be written.
      level:= 0 (low) 1 (high).

      Nothing is returned.

      ...
      pico.gpio_write(7, 1) # set GPIO 7 high
      pico.gpio_write(25, 0) # set GPIO 25 low
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert 0 <= level <= 1

      self.GPIO_write(1<<gpio, level<<gpio, reply=reply)

   def GPIO_set_pulls(self, GPIO, PULLS, reply=REPLY_NONE, flush=True):
      """
      Sets the pulls of a group of GPIO.

       GPIO:= the GPIO to act upon.
      PULLS:= the pulls to set.

      If bit x of [#GPIO#] is set then the pulls for GPIO x will be set.

      The pulls for GPIO x are set by bits 2x+1 and 2x of [#PULLS#].

      . .
      00 - PULL_NONE
      01 - PULL_DOWN
      10 - PULL_UP
      11 - PULL_BOTH
      . .

      Only opened GPIO are affected.

      Nothing is returned.

      ...
       # set pull-down on GPIO 15, pull-up on GPIO 6
      pico.GPIO_set_pulls(1<<15|1<<6, picod.PULL_DOWN<<30|picod.PULL_UP<<12)
      ...
      """

      pulls_0_15 = PULLS & 0xffffffff
      pulls_16_31 = (PULLS >> 32) & 0xffffffff

      data = struct.pack(">III", GPIO, pulls_0_15, pulls_16_31)

      self._request(_CMD_PULLS_SET, data, reply=reply, flush=flush)

   def gpio_set_pull(self, gpio, pull, reply=REPLY_NONE, flush=True):
      """
      Sets the pull for a single GPIO.

      gpio:= the GPIO to act upon.
      pull:= the pulls to set.

      pull has one of the following values:

      . .
      0 - PULL_NONE
      1 - PULL_DOWN
      2 - PULL_UP
      3 - PULL_BOTH
      . .

      Only opened GPIO are affected.

      Nothing is returned.

      ...
      pico.gpio_set_pull(5, picod.PULL_UP) # set pull-up on GPIO 5.
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert PULL_NONE <= pull <= PULL_BOTH

      self.GPIO_set_pulls(1<<gpio, pull<<(gpio*2))

   def GPIO_get_pulls(self, reply=REPLY_NOW, flush=True):
      """
      Returns the current pulls of all GPIO.

      The pulls for GPIO x are indicated by bits 2x+1 and 2x of
      the returned value as follows.

      . .
      00 - PULL_NONE
      01 - PULL_DOWN
      10 - PULL_UP
      11 - PULL_BOTH
      . .

      A tuple of status and the pulls is returned.

      ...
      status, pulls = pico.GPIO_get_pulls()

      if status == picod.STATUS_OKAY:
         print("GPIO pulls are 0x{:x}".format(pulls))
      [#GPIO pulls are 0x5555555555555a5#]
      ...
      """

      status, data = self._request(_CMD_PULLS_GET, reply=reply, flush=flush)

      if status == STATUS_OKAY:
         pulls_0_15, pulls_16_31 = struct.unpack(">II", data)
         self._GPIO_pulls = (pulls_16_31 << 32) | pulls_0_15

      return status, self._GPIO_pulls

   def gpio_get_pull(self, gpio, reply=REPLY_NOW, flush=True):
      """
      Returns the pull for a single GPIO.

      gpio:= the GPIO.

      The returned value will be one of:

      . .
      0 - PULL_NONE
      1 - PULL_DOWN
      2 - PULL_UP
      3 - PULL_BOTH
      . .

      A tuple of status and the pull are returned.

      ...
      status, pull = pico.gpio_get_pull(23)

      if status == picod.STATUS_OKAY:
         print("GPIO 23 pull is {}".format(pull))
      [#GPIO 23 pull is 1#]
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      status, PULLS = self.GPIO_get_pulls()

      pull = None

      if status == STATUS_OKAY:
         pull = (PULLS>>(gpio*2))&3

      return status, pull

   def GPIO_set_functions(self, GPIO, FUNCS, reply=REPLY_NONE, flush=True):
      """
      Sets the functions of a group of GPIO.

       GPIO:= the GPIO to act upon.
      FUNCS:= the functions to set.

      If bit x of [#GPIO#] is set then the function of GPIO x will be set.

      The function of GPIO x is set by bits 4x+3, 4x+2, 4x+1, and
      4x of [#FUNCS#].

      . .
      0000 - FUNC_XIP
      0001 - FUNC_SPI
      0010 - FUNC_UART
      0011 - FUNC_I2C
      0100 - FUNC_PWM
      0101 - FUNC_SIO
      0101 - FUNC_GPIO (alias for SIO)
      0110 - FUNC_PIO0
      0111 - FUNC_PIO1
      1000 - FUNC_GPCK
      1001 - FUNC_USB
      1111 - FUNC_NULL
      . .

      Nothing is returned.

      ...
      GPIO_set_functions(3, 0x33) # set GPIO 0 and 1 to function I2C
      ...

      This function should not need to be used.  The GPIO functions
      will be automatically set as needed.
      """

      func_0_7 = FUNCS & 0xffffffff
      func_8_15 = (FUNCS >> 32) & 0xffffffff
      func_16_23 = (FUNCS >> 64) & 0xffffffff
      func_24_31 = (FUNCS >> 96) & 0xffffffff

      data = struct.pack(">IIIII",
         GPIO, func_0_7, func_8_15, func_16_23, func_24_31)

      self._request(_CMD_FUNCTION_SET, data, reply=reply, flush=flush)

   def gpio_set_function(self, gpio, func, reply=REPLY_NONE, flush=True):
      """
      Sets the function of a single GPIO.

      gpio:= the GPIO to act upon.
      func:= the function to set.

      [#func#] should be one of:

      . .
      0 - FUNC_XIP
      1 - FUNC_SPI
      2 - FUNC_UART
      3 - FUNC_I2C
      4 - FUNC_PWM
      5 - FUNC_SIO
      5 - FUNC_GPIO (alias for SIO)
      6 - FUNC_PIO0
      7 - FUNC_PIO1
      8 - FUNC_GPCK
      9 - FUNC_USB
      15 - FUNC_NULL
      . .

      Nothing is returned.

      ...
      pico.gpio_set_function(25, 0) # set GPIO 25 to function XIP
      ...

      This function should not need to be used.  The GPIO functions
      will be automatically set as needed.
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert 0 <= func <= 15

      self.GPIO_set_functions(1<<gpio, func<<(gpio*4))

   def GPIO_get_functions(self, reply=REPLY_NOW, flush=True):
      """
      Returns the current functions of all GPIO.

      The function for GPIO x is indicated by bits 4x+3, 4x+2,
      4x+1 and 4x of the returned value as follows:

      . .
      0000 - FUNC_XIP
      0001 - FUNC_SPI
      0010 - FUNC_UART
      0011 - FUNC_I2C
      0100 - FUNC_PWM
      0101 - FUNC_SIO
      0101 - FUNC_GPIO (alias for SIO)
      0110 - FUNC_PIO0
      0111 - FUNC_PIO1
      1000 - FUNC_GPCK
      1001 - FUNC_USB
      1111 - FUNC_NULL
      . .

      A tuple of status and the functions are returned.

      ...
      status, functions = pico.GPIO_get_functions()

      if status == picod.STATUS_OKAY:
         print("GPIO functions are 0x{:x}".format(functions))
      [#GPIO functions are 0x1f5555ff55555555555555555555522#]
      ...
      """

      status, data = self._request(_CMD_FUNCTION_GET, reply=reply, flush=flush)

      if status == STATUS_OKAY:
         func_0_7, func_8_15, func_16_23, func_24_31 = struct.unpack(
            ">IIII", data)

         self._GPIO_function = ((func_24_31 << 96) |
                                (func_16_23 << 64) |
                                (func_8_15  << 32) |
                                 func_0_7)

      return status, self._GPIO_function

   def gpio_get_function(self, gpio, reply=REPLY_NOW, flush=True):
      """
      Returns the function of a single GPIO.

      The returned value will be one of:

      . .
      0 - FUNC_XIP
      1 - FUNC_SPI
      2 - FUNC_UART
      3 - FUNC_I2C
      4 - FUNC_PWM
      5 - FUNC_SIO
      5 - FUNC_GPIO (alias for SIO)
      6 - FUNC_PIO0
      7 - FUNC_PIO1
      8 - FUNC_GPCK
      9 - FUNC_USB
      15 - FUNC_NULL
      . .

      A tuple of status and the function is returned.

      ...
      status, function = pico.gpio_get_function(20)

      if status == picod.STATUS_OKAY:
         print("GPIO 20 function is {}".format(function))
      [#GPIO 20 function is 5#]
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      status, FUNC = self.GPIO_get_functions()

      func = None

      if status == STATUS_OKAY:
         func = (FUNC>>(gpio*4))&15

      return status, func

   def GPIO_set_alerts(self, GPIO, ALERTS, reply=REPLY_NONE, flush=True):
      """
      Sets alerts on or off for a group of GPIO.

        GPIO:= the GPIO to act upon.
      ALERTS:= the alert settings.

      If bit x of [#GPIO#] is set then alerts for GPIO x will be enabled
      or disabled.

      If bit x of [#ALERTS#] is set the alert for GPIO x is enabled
      otherwise it is disabled.

      Nothing is returned.

      ...
      # enable alerts for GPIO 4 and 6, disable alerts for GPIO 5 and 7.
      GPIO_set_alerts(0xf0, 0x50)
      ...
      """

      data = struct.pack(">II", GPIO, ALERTS)

      self._request(_CMD_ALERT_SELECT, data, reply=reply, flush=flush)

   def gpio_set_alert(self, gpio, enable, reply=REPLY_NONE, flush=True):
      """
      Sets alerts on or off for a single GPIO.

        gpio:= the GPIO.
      enable:= enable if true, disable otherwise.

      Nothing is returned.

      ...
      gpio_set_alert(17, True) # Enable alerts for GPIO 17.
      ...
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX

      if enable:
         v = 1
      else:
         v = 0

      self.GPIO_set_alerts(1<<gpio, v<<gpio)

   def gpio_set_debounce(self, gpio, secs, reply=REPLY_NONE, flush=True):
      """
      Sets the debounce time for a single GPIO.

      gpio:= the GPIO to act upon (0-29).
      secs:= the debounce time in seconds (0 to cancel).

      Nothing is returned.

      ...
      pico.gpio_set_debounce(15, 0.01) # 0.01 second debounce on GPIO 15.
      ...

      A level change for the GPIO will only be reported once the level
      has been stable for at least debounce seconds.
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert 0 <= secs <= 1.0

      data = struct.pack(">BI", gpio, int(secs*1e6))

      self._request(_CMD_ALERT_DEBOUNCE, data, reply=reply, flush=flush)

   def gpio_set_watchdog(self, gpio, secs, reply=REPLY_NONE, flush=True):
      """
      Sets the watchdog time for a single GPIO.

      gpio:= the GPIO to act upon (0-29).
      secs:= the watchdog time in seconds (0 to cancel).

      Nothing is returned.

      ...
      pico.gpio_set_watchdog(10, 0.1) # 0.1 second watchdog on GPIO 10.
      ...

      If there has been no change to the GPIO level for the watchdog period
      a timeout report will be issued.

      Only one watchdog alert will be sent per stream of edge
      alerts. The watchdog is reset by the sending of a new edge alert.

      The level is set to LEVEL_TIMEOUT (2) for a watchdog alert.
      """

      assert GPIO_MIN <= gpio <= GPIO_MAX
      assert 0 <= secs <= 60.0

      data = struct.pack(">BI", gpio, int(secs*1e6))

      self._request(_CMD_ALERT_WATCHDOG, data, reply=reply, flush=flush)

   # ADC ---------------------------------------------------------------------

   def adc_read(self, channel, reply=REPLY_NOW, flush=True):
      """
      Returns the value of an ADC channel.

      channel:= the channel to read (0-4).

      Returns a tuple of status, channel, and reading.

      ...
      status, channel, reading = pico.adc_read(2)

      print(status, channel, reading)
      [#(0, 2, 171)#]
      ...

      The value will be in the range 0 to 4095.  Expect about 8 bits
      of accuracy.

      Channel @ GPIO @ usage
      0       @ 26   @  General input
      1       @ 27   @  General input
      2       @ 28   @  General input
      3       @ 29   @  a voltage divider to measure VSYS
      4       @ --   @  a temperature sensor
      """

      assert 0 <= channel <= 4

      status, data = self._request(_CMD_ADC_READ,
         struct.pack("B", channel), reply=reply, flush=flush)

      ch = None
      val = None

      if status == STATUS_OKAY:
         ch, val = struct.unpack(">BH", data)

      return status, ch, val

   def adc_close(self, channel, reply=REPLY_NONE, flush=True):
      """
      Closes an ADC channel and frees any associated GPIO.

      channel:= the channel to close.

      Nothing is returned.

      ...
      pico.adc_close(2) # close ADC channel 2.
      ...

      Channel 0-3 are connected to GPIO 26-29.  Channel 4 does not have a
      connected GPIO.
      """

      assert 0 <= channel <= 4

      self._request(_CMD_ADC_CLOSE,
         struct.pack("B", channel), reply=reply, flush=flush)

   # I2C ---------------------------------------------------------------------

   def i2c_open(self,
      channel, sda, scl, baud=100000, slave_addr=0,
         reply=REPLY_NOW, flush=True):
      """
      Opens an I2C channel (master or slave).

         channel:= the channel to open (0 or 1).
             sda:= the GPIO to use for the I2C data.
                   channel 0: one of 0, 4, 8, 12, 16, 20, 24, 28.
                   channel 1: one of 2, 6, 10, 14, 18, 22, 26.
             scl:= the GPIO to use for the I2C clock.
                   channel 0: one of 1, 5, 9, 13, 17, 21, 25, 29.
                   channel 1: one of 3, 7, 11, 15, 19, 23, 27.
            baud:= the baud rate in bits per second (default 100000).
      slave_addr:= the slave address if a slave (0 is master).

      Returns a tuple of status and the set baud rate.

      ...
      status, speed = pico.i2c_open(1, 2, 3) # channel, SDA, SCL.

      if status == picod.STATUS_OKAY:
         print("opened okay, speed={}".format(speed))
      else:
         print("open failed with status {}".format(status))
      [#opened okay, speed=100000#]
      ...
      """

      assert 0 <= channel <= 1
      assert sda in I2C_SDA[channel]
      assert scl in I2C_SCL[channel]
      assert 50 <= baud <= 4000000

      status, data = self._request(_CMD_I2C_OPEN,
         struct.pack(">IBBBB", baud, channel, sda, scl, slave_addr),
            reply=reply, flush=flush)

      speed = None

      if status == STATUS_OKAY:
          speed, = struct.unpack(">I", data)

      return status, speed

   def i2c_close(self, channel, reply=REPLY_NONE, flush=True):
      """
      Closes an I2C channel and frees associated GPIO.

      channel:= the channel to close (0 or 1).

      Nothing is returned.

      ...
      pico.i2c_close(1) # close I2C channel 1.
      ...

      The GPIO assigned to the channel may now be reused.
      """

      assert 0 <= channel <= 1

      self._request(_CMD_I2C_CLOSE, struct.pack(">B", channel),
         reply=reply, flush=flush)


   def i2c_read(self, channel, addr, count, nostop=False, timeout=1.0,
      reply=REPLY_NOW, flush=True):
      """
      Returns count bytes from an address on an I2C channel.

      channel:= the channel (0 or 1).
         addr:= the I2C address of the device to read.
        count:= the number of bytes to read.
       nostop:= set to True for no stop condition.
      timeout:= how long to wait in seconds for the call to complete.

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      # read 5 bytes from address 0x48 on I2C channel 1
      status, data = pico.i2c_read(1, 0x48, 5)

      if status == picod.STATUS_OKAY:
         print(data[0], data[1], data[2], data[3], data[4])
      [#(10, 20, 230, 231, 37)#]
      ...
      """

      assert 0 <= channel <= 2
      assert 0 <= addr <= 127
      assert 1 <= count <= 32767
      assert 0 < timeout < 5.0

      if nostop:
         stop = 1
      else:
         stop = 0

      return self._request(_CMD_I2C_READ,
         struct.pack(">IHBBB", int(timeout*1e6), count, channel, addr, stop),
            reply=reply, flush=flush)

   def i2c_write(self, channel, addr, data, nostop=False, timeout=1.0,
      reply=REPLY_NOW, flush=True):
      """
      Writes data to an address on an I2C channel.

      channel:= the channel (0 or 1).
         addr:= the I2C address of the device to write.
         data:= the bytes to write.
       nostop:= set to True for no stop condition.
      timeout:= how long to wait in seconds for the call to complete.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status = pico.i2c_write(1, 0x48, [34, 23, 12])

      if status == picod.STATUS_OKAY:
         print("okay")
      else:
         print("failed")
      [#okay#]
      ...
      """

      assert 0 <= channel <= 1
      assert 0 <= addr <= 127
      assert 1 <= len(data) <= 32767
      assert 0 < timeout < 5.0

      if nostop:
         stop = 1
      else:
         stop = 0

      data = _tobuf(data)

      return self._request(_CMD_I2C_WRITE,
         struct.pack(">IHBBB", int(timeout*1e6),
            len(data), channel, addr, stop)+data, reply=reply, flush=flush)[0]

   def i2c_pop(self, channel, count, reply=REPLY_NOW, flush=True):
      """
      Returns up to count bytes from an I2C slave channel.

      channel:= the channel (0 or 1).
        count:= the maximum number of bytes to read.

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      status, data = pico.i2c_pop(1, 10)

      if status == picod.STATUS_OKAY:
         print("got {} bytes".format(len(data)))
      [#got 0 bytes#]
      ...
      """

      return self._request(_CMD_I2C_POP,
         struct.pack(">BB", channel, count), reply=reply, flush=flush)

   def i2c_push(self, channel, data, reply=REPLY_NOW, flush=True):
      """
      Writes data to an I2C slave channel.

      channel:= the channel (0 or 1).
         data:= the data bytes to write.

      A tuple of status and the number of bytes stored is returned,

      ...
      status, stored = pico.i2c_push(1, "hello")

      if status == picod.STATUS_OKAY:
         print("stored {} bytes".format(stored))
      [#stored 5 bytes#]
      ...
      """

      data = _tobuf(data)

      status, data = self._request(_CMD_I2C_PUSH,
         struct.pack(">BB", channel, len(data)) + data, reply=reply, flush=flush)

      return status, data[0]

   # PWM/SERVO ---------------------------------------------------------------

   def _pwm_raw(self,
      gpioAB, mode, clkdiv, steps, high, reply=REPLY_NONE, flush=True):
      """
      Configures hardware PWM.

     gpioAB:= the GPIO.
       mode:= _CMD_PWM or _CMD_SERVO.
     clkdiv:= 0-255.
      steps:= 0-65535.
       high:= 0-65535.

      If OK returns 0, otherwise returns a non-zero status code.
      """

      assert GPIO_MIN <= gpioAB <= GPIO_MAX
      assert _CMD_PWM <= mode <= _CMD_SERVO
      assert 0 <= clkdiv <= 255
      assert 0 <= steps <= 65535
      assert 0 <= high <= 65535

      return self._request(mode,
         struct.pack(">BBHH", gpioAB, clkdiv, steps, high),
            reply=reply, flush=flush)[0]

   def tx_pwm(self, gpioAB, frequency, dutycycle, reply=REPLY_NOW, flush=True):
      """
      Starts hardware PWM pulses on a single GPIO.

         gpioAB:= the GPIO.
      frequency:= 7.46 Hz to 31250000 Hz.
      dutycycle:= the percentage high time, 0% to 100%

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status = pico.tx_pwm(3, 20, 10)

      if status == picod.STATUS_OKAY:
         print("PWM started okay")
      else:
         print("PWM failed")
      [#PWM started okay#]
      ...
      """

      assert GPIO_MIN <= gpioAB <= GPIO_MAX
      assert 7.46 <= frequency <= 31250000
      assert 0 <= dutycycle <= 100

      norm = CLOCK_HZ / frequency
      div = int(norm / 65536) + 1
      newf = CLOCK_HZ / div
      steps = int(newf / frequency)
      high = int(dutycycle * steps / 100.0)

      #print(dutycycle, frequency, div, steps, high)

      return self._pwm_raw(gpioAB, _CMD_PWM, div&255, steps, high,
         reply=reply, flush=flush)

   def tx_servo(self,
      gpioAB, pulsewidth, frequency=50, reply=REPLY_NOW, flush=True):
      """
      Starts hardware servo pulses on a single GPIO.

          gpioAB:= the GPIO.
      pulsewidth:= 0 (off) or 500 to 2500 microseconds.
       frequency:= 40 Hz to 500 Hz (default 50 Hz).

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status = pico.tx_servo(25, 1500)

      if status == picod.STATUS_OKAY:
         print("Servo pulses started okay")
      else:
         print("Servo pulses failed")
      [#Servo pulses started okay#]
      ...
      """

      assert GPIO_MIN <= gpioAB <= GPIO_MAX
      assert 40 <= frequency <= 500
      assert pulsewidth == 0 or 500 <= pulsewidth <= 2500

      norm = CLOCK_HZ / frequency
      div = int(norm / 65536) + 1
      newf = CLOCK_HZ / div
      steps = int(newf / frequency)
      micros = 1e6 * steps / newf
      high = int(pulsewidth * steps / micros)

      if high > steps:
         high = steps

      #print(pulsewidth, frequency, div, steps, high)

      return self._pwm_raw(gpioAB, _CMD_SERVO, div&255, steps, high,
         reply=reply, flush=flush)

   def tx_close(self, gpioAB, reply=REPLY_NONE, flush=True):
      """
      Stops PWM/Servo pulses and frees the associated GPIO.

      gpioAB:= the GPIO.

      Nothing is returned.

      ...
      pico.tx_close(25)
      ...
      """

      assert GPIO_MIN <= gpioAB <= GPIO_MAX

      return self._request(_CMD_PWM_CLOSE,
         struct.pack(">B", gpioAB), reply=reply, flush=flush)

   # PWM READ ----------------------------------------------------------------

   def _pwm_read_raw(self, gpioB, mode, reply=REPLY_NOW, flush=True):
      """
      Reads a PWM frequency, dutycycle, or high edges.

      gpioB:= the GPIO to monitor (must be an odd numbered GPIO).
      mode:= _CMD_PWM_READ_FREQ to monitor frequency.
             _CMD_PWM_READ_DUTY to monitor dutycycle.
             _CMD_PWM_READ_EDGE to count high edges.

      Returns a tuple of status, count, and time (in seconds).

      ...
      pico._pwm_read_raw(17, picod._CMD_PWM_READ_FREQ)
      [#(0, 0, 1e-06)#]

      pico._pwm_read_raw(17, picod._CMD_PWM_READ_FREQ)
      [#0, 2803, 3.503482)#]
      ...

      When a new mode is received the mode will be initialised and a
      zero reading will be returned.

      For frequency and dutycycle subsequent calls with the same
      mode will return a reading from the data sampled during the
      intervening period.

      The high edges will return the number of high edges and the
      number of seconds since the mode was first set (i.e. the
      totals are cumulative).
      """

      assert GPIO_MIN <= gpioB <= GPIO_MAX
      assert (gpioB%2) == 1
      assert _CMD_PWM_READ_FREQ <= mode <= _CMD_PWM_READ_EDGE

      status, data = self._request(mode,
         struct.pack(">B", gpioB), reply=reply, flush=flush)

      count = None
      secs = None

      if status == STATUS_OKAY:
         countH, countL, microsH, microsL = struct.unpack(">IIII", data)
         count = countL + (countH<<32)
         secs = (microsL + (microsH<<32))/1e6

      return status, count, secs

   def pwm_read_dutycycle(self, gpioB, reply=REPLY_NOW, flush=True):
      """
      Returns the PWM dutycycle of a single GPIO.

      gpioB:= the GPIO to monitor (must be an odd numbered GPIO).

      Returns a tuple of status and dutycycle percentage.

      ...
      status, dutycycle = pico.pwm_read_dutycycle(17)

      if status == picod.STATUS_OKAY:
         print("dutycycle = {:.1f}%".format(dutycycle))
      [#dutycycle = 0.0%#]

      time.sleep(1)

      status, dutycycle = pico.pwm_read_dutycycle(17)

      if status == picod.STATUS_OKAY:
         print("dutycycle = {:.1f}%".format(dutycycle))
      [#dutycycle = 12.8%#]
      ...

      The first call will set dutycycle mode and return a zero reading.

      Subsequent calls will return the dutycycle percentage sampled since
      the last reading.
      """

      assert GPIO_MIN <= gpioB <= GPIO_MAX
      assert (gpioB % 2) == 1

      status, count, secs = self._pwm_read_raw(
         gpioB, _CMD_PWM_READ_DUTY, reply=reply, flush=flush)

      duty = None

      if status == STATUS_OKAY:
         #print("raw count={} micros={}".format(count, micros))
         max_count = CLOCK_HZ * secs
         duty = (count * 100.0) / max_count

      return status, duty

   def pwm_read_frequency(self, gpioB, reply=REPLY_NOW, flush=True):
      """
      Returns the PWM frequency of a single GPIO.

      gpioB:= the GPIO to monitor (must be an odd numbered GPIO).

      Returns a tuple of status and frequency.

      ...
      status, frequency = pico.pwm_read_frequency(17)

      if status == picod.STATUS_OKAY:
         print("frequency = {}".format(frequency))
      [#frequency = 0#]

      time.sleep(1)

      status, frequency = pico.pwm_read_frequency(17)

      if status == picod.STATUS_OKAY:
         print("frequency = {}".format(frequency))
      [#frequency = 799#]
      ...

      The first call will set frequency mode and return a zero reading.

      Subsequent calls will return the frequency sampled since the last
      reading.
      """

      assert GPIO_MIN <= gpioB <= GPIO_MAX
      assert (gpioB % 2) == 1

      status, count, secs = self._pwm_read_raw(
         gpioB, _CMD_PWM_READ_FREQ, reply=reply, flush=flush)

      frequency = None

      if status == STATUS_OKAY:
         #print("raw count={} secs={}".format(count, secs))
         frequency = int(count / secs)

      return status, frequency

   def pwm_read_high_edges(self, gpioB, reply=REPLY_NOW, flush=True):
      """
      Returns the high edge count of a single GPIO.

      gpioB:= the GPIO to monitor (must be an odd numbered GPIO).

      Returns a tuple of status, count, and time (in seconds).

      ...
      status, count, secs = pico.pwm_read_high_edges(17)

      if status == picod.STATUS_OKAY:
         print("count={} time={:.1f}".format(count, secs))
      [#count=0 time=0.0#]

      time.sleep(1)

      status, count, secs = pico.pwm_read_high_edges(17)

      if status == picod.STATUS_OKAY:
         print("count={} time={:.1f}".format(count, secs))
      [#count=812 time=1.0#]

      time.sleep(1)

      status, count, secs = pico.pwm_read_high_edges(17)

      if status == picod.STATUS_OKAY:
         print("count={} time={:.1f}".format(count, secs))
      [#count=1625 time=2.0#]
      ...

      The first call will set read high edge mode and return a zero reading.

      Subsequent calls will return the count of high edges seen and
      the number of seconds since the count started.
      """

      assert GPIO_MIN <= gpioB <= GPIO_MAX
      assert (gpioB % 2) == 1

      return self._pwm_read_raw(
         gpioB, _CMD_PWM_READ_EDGE, reply=reply, flush=flush)

   # SERIAL ---------------------------------------------------------------

   def serial_open(self,
      channel, tx, rx, baud, cts=255, rts=255, data_bits=8,
         stop_bits=1, parity=0, reply=REPLY_NOW, flush=True):
      """
      Opens a serial channel.

        channel:= the channel to open (0 or 1).
             tx:= the GPIO to use for transmit.
                  channel 0: one of 0, 12, 16, 28, 255.
                  channel 1: one of 4, 8, 20, 24, 255.
             rx:= the GPIO to use for receive.
                  channel 0: one of 1, 13, 17, 29, 255.
                  channel 1: one of 5, 9, 21, 25, 255.
           baud:= baud rate in bits per second, 120 to 4000000.
            cts:= the GPIO to use for CTS.
                  channel 0: one of 2, 14, 18, 255.
                  channel 1: one of 6, 10, 22, 26, 255.
            rts:= the GPIO to use for RTS.
                  channel 0: one of 3, 15, 19, 255.
                  channel 1: one of 7, 11, 23, 27, 255.
      data_bits:= data bits, 5 to 8.
      stop_bits:= stop bits, 1 or 2.
         parity:= the parity bit
                  0 - PARITY_NONE
                  1 - PARITY_EVEN
                  2 - PARITY_ODD

      Returns a tuple of status and the set baud rate.

      ...
      status, speed = pico.serial_open(0, 0, 1, 19200)

      if status == picod.STATUS_OKAY:
         print("opened okay, baud={}".format(speed))
      else:
         print("open failed with status {}".format(status))
      [#opened okay, baud=19199#]
      ...
      """

      assert 0 <= channel <= 1
      assert tx in UART_TX[channel]
      assert rx in UART_RX[channel]
      assert cts in UART_CTS[channel]
      assert rts in UART_RTS[channel]
      assert 120 <= baud <= 4000000
      assert 5 <= data_bits <= 8
      assert 1 <= stop_bits <= 2
      assert PARITY_NONE <= parity <= PARITY_ODD

      status, data = self._request(_CMD_UART_OPEN,
         struct.pack(">IBBBBBBBB",baud, channel, tx, rx, cts, rts, data_bits,
            stop_bits, parity), reply=reply, flush=flush)

      speed = None

      if status == STATUS_OKAY:
          speed, = struct.unpack(">I", data)

      return status, speed

   def serial_close(self, channel, reply=REPLY_NONE, flush=True):
      """
      Closes a serial channel and frees associated GPIO.

      channel:= the channel to close (0 or 1).

      Nothing is returned.
      """

      assert 0 <= channel <= 1

      self._request(_CMD_UART_CLOSE,
         struct.pack(">B", channel), reply=reply, flush=flush)


   def serial_read(self, channel, count, reply=REPLY_NOW, flush=True):
      """
      Returns up to count bytes from a serial channel.

      channel:= the channel to read (0 or 1).
        count:= the maximum number of bytes to read.

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      status, data = pico.serial_read(0, 10)

      if status == picod.STATUS_OKAY:
         print("serial returned {} bytes".format(len(data)))
      [#serial returned 0 bytes#]
      ...
      """

      assert 0 <= channel <= 1

      status, data = self._request(_CMD_UART_READ,
         struct.pack(">BH", channel, count), reply=reply, flush=flush)

      count = 0
      chars = bytearray()

      if status == STATUS_OKAY:
         chars = data[2:]

      return status, chars

   def serial_write(self, channel, data, reply=REPLY_NOW, flush=True):
      """
      Writes data to a serial channel.

      channel:= the channel to write (0 or 1).
         data:= the data bytes to write.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status = pico.serial_write(0, "How you doing?")

      if status == picod.STATUS_OKAY:
         print("okay")
      else:
         print("failed")
      [#okay#]
      ...
      """

      assert 0 <= channel <= 1

      data = _tobuf(data)

      return self._request(_CMD_UART_WRITE,
         struct.pack(">BH", channel, len(data)) + data,
            reply=reply, flush=flush)[0]

   # SPI ---------------------------------------------------------------------

   def spi_open(self, channel, tx, rx, sck, baud=1000000, spi_mode=0,
      spi_bits=8, slave_cs=0, reply=REPLY_NOW, flush=True):
      """
      Opens a SPI channel (master or slave).

       channel:= the channel to open (0 or 1).
            tx:= the GPIO to use for transmit.
                 channel 0: one of 3, 7, 19, 23.
                 channel 1: one of 11, 15, 27.
            rx:= the GPIO to use for receive.
                 channel 0: one of 0, 4, 16, 20.
                 channel 1: one of 8, 12, 24, 28.
           sck:= the GPIO to use for the SPI clock.
                 channel 0: one of 2, 6, 18, 22.
                 channel 1: one of 10, 14, 26.
          baud:= the bits per second, 1900-16000000, default 1000000.
      spi_mode:= the mode, 0-3, default 0.
      spi_bits:= bits per word, 4-16, default 8.
      slave_cs:= the GPIO to use for the chip select in slave mode.
                 In master mode leave at the default 0.
                 channel 0: one of 1, 5, 17, 21.
                 channel 1: one of 9, 13, 25, 29.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status, speed = pico.spi_open(0, 3, 0, 2)

      if status == picod.STATUS_OKAY:
         print("SPI open okay, speed={}".format(speed))
      else:
         print("SPI open failed with {}".format(status))
      [#SPI open okay, speed=1000000#]
      ...
      """

      assert 0 <= channel <= 1
      assert tx in SPI_TX[channel]
      assert rx in SPI_RX[channel]
      assert sck in SPI_SCK[channel]
      assert 1900 <= baud <= 16000000
      assert slave_cs in SPI_CS[channel] or slave_cs == 0
      assert 0 <= spi_mode <= 3
      assert 4 <= spi_bits <= 16

      status, data = self._request(_CMD_SPI_OPEN,
         struct.pack(">BBBBBBBI",
            channel, tx, rx, sck, spi_mode, spi_bits,slave_cs, baud),
            reply=reply, flush=flush)

      speed = None

      if status == STATUS_OKAY:
         speed, = struct.unpack(">I", data)

      return status, speed

   def spi_close(self, channel, reply=REPLY_NONE, flush=True):
      """
      Close a SPI channel and frees associated GPIO.

      channel:= the channel to close (0 or 1).

      Nothing is returned.

      ...
      pico.spi_close(0)
      ...
      """

      assert 0 <= channel <= 1

      self._request(_CMD_SPI_CLOSE,
         struct.pack(">B", channel), reply=reply, flush=flush)

   def spi_read(self,
      channel, cs, count, spi_dummy=0, reply=REPLY_NOW, flush=True):
      """
      Returns count bytes from a SPI channel.

        channel:= the channel to read (0 or 1).
             cs:= the GPIO to use for the chip select (any spare GPIO).
          count:= the number of bytes to read.
      spi_dummy:= the dummy byte to send during a SPI read (default 0).

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      status, data = pico.spi_read(0, 1, 5)

      if status == picod.STATUS_OKAY:
         print(data[0], data[1], data[2], data[3], data[4])
      [#(0, 1, 20, 1, 7)#]
      ...
      """

      assert 0 <= channel <= 1
      assert GPIO_MIN <= cs <= GPIO_MAX
      assert 0 <= spi_dummy <= 255

      return self._request(_CMD_SPI_READ,
         struct.pack(">BBHB", channel, cs, count, spi_dummy),
            reply=reply, flush=flush)


   def spi_write(self, channel, cs, data, reply=REPLY_NOW, flush=True):
      """
      Writes data to a SPI channel.

      channel:= the channel to write (0 or 1).
           cs:= the GPIO to use for the chip select (any spare GPIO).
         data:= the data bytes to write.

      If OK returns 0, otherwise returns a non-zero status code.

      ...
      status = pico.spi_write(0, 1, [34, 23, 12])

      if status == picod.STATUS_OKAY:
         print("okay")
      else:
         print("failed")
      [#okay#]
      ...
      """

      assert 0 <= channel <= 1
      assert GPIO_MIN <= cs <= GPIO_MAX

      data = _tobuf(data)

      return self._request(_CMD_SPI_WRITE,
         struct.pack(">BBH", channel, cs, len(data)) + data,
            reply=reply, flush=flush)[0]

   def spi_xfer(self, channel, cs, data, reply=REPLY_NOW, flush=True):
      """
      Transfers (reads and writes) data to a SPI channel.

      channel:= the channel to use (0 or 1).
           cs:= the GPIO to use for the chip select (any spare GPIO).
         data:= the data bytes to write.

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      status, data = pico.spi_xfer(0, 1, [10,11, 12])

      if status == picod.STATUS_OKAY:
         print(data[0], data[1], data[2])
      [#(0, 13, 23)#]
      ...
      """

      assert 0 <= channel <= 1
      assert GPIO_MIN <= cs <= GPIO_MAX

      data = _tobuf(data)

      return self._request(_CMD_SPI_XFER,
         struct.pack(">BBH", channel, cs, len(data)) + data,
            reply=reply, flush=flush)

   def spi_pop(self, channel, count, reply=REPLY_NOW, flush=True):
      """
      Returns up to count bytes from a SPI slave channel.

      channel:= the channel to read (0 or 1).
        count:= the maximum number of bytes to read.

      A tuple of status and a bytearray containing the read
      bytes is returned.

      ...
      status, data = pico.spi_pop(0, 5)

      if status == picod.STATUS_OKAY:
         print("got {} bytes".format(len(data)))
      [#got 0 bytes#]
      ...
      """

      assert 0 <= channel <= 1

      return self._request(_CMD_SPI_POP,
         struct.pack(">BB", channel, count), reply=reply, flush=flush)

   def spi_push(self, channel, data, reply=REPLY_NOW, flush=True):
      """
      Writes data to a SPI slave channel.

      channel:= the channel to write (0 or 1).
         data:= the data bytes to write.

      A tuple of status and the number of bytes stored is returned,

      ...
      status, stored = pico.spi_push(1, "hello")

      if status == picod.STATUS_OKAY:
         print("stored {} bytes".format(stored))
      [#stored 5 bytes#]
      ...

      """

      assert 0 <= channel <= 1

      data = _tobuf(data)

      status, data = self._request(_CMD_SPI_PUSH,
         struct.pack(">BB", channel, len(data)) + data,
            reply=reply, flush=flush)

      return status, data[0]

# UTILITIES ---------------------------------------------------------------

   def reset(self, reply=REPLY_NONE, flush=True):
      """
      Resets the Pico (frees GPIO, close I2C, SPI, serial, PWM).

      Nothing is returned.

      ...
      pico.reset() # reset the Pico
      ...

      All the GPIO will be unassigned.

      Any opened UARTS, I2C, SPI, PWM will be closed.
      """

      self._request(_CMD_RESET_PICO, reply=reply, flush=flush)

   def version(self, reply=REPLY_NOW, flush=True):
      """
      Returns the Pico daemon software version (dotted quad).

      Returns a tuple of status and the version string.

      ...
      status, version = pico.version()

      if status == picod.STATUS_OKAY:
         print("Pico daemon version is {}".format(version))
      else:
         print("Pico not talking")
      [#Pico daemon version is 0.1.0.0#]
      ...

      For a version of A.B.C.D

      . .
      A. API major version, changed if breaks previous API
      B. API minor version, changed when new function added
      C. bug fix
      D. documentation change
      . .
      """
      status, data = self._request(_CMD_PD_VERSION, reply=reply, flush=flush)

      vstr = None

      if status == STATUS_OKAY:
         major, minor, bug, doc = struct.unpack(">BBBB", data)
         vstr = "{}.{}.{}.{}".format(major, minor, bug, doc)

      return status, vstr

   def sleep(self, secs, reply=REPLY_NONE, flush=True):
      """
      Sleeps the Pico.

      secs:= the number of seconds to sleep.

      Nothing is returned.

      ...
      pico.sleep(0.01) # sleep for 0.01 seconds
      ...

      Sleeping the Pico may be useful if you are expecting a
      burst of GPIO level changes.  Sleeping will help prevent
      USB activity during the burst (minimising lost GPIO level
      changes).
      """

      assert 0 < secs <= 2.0

      self._request(_CMD_SLEEP_US,
         struct.pack(">I", int(secs*1e6)), reply=reply, flush=flush)

   def tick(self, reply=REPLY_NOW, flush=True):
      """
      Returns the current Pico tick.

      Returns a tuple of status and current tick.

      ...
      status, tick = pico.tick() # get current Pico tick

      if status == picod.STATUS_OKAY:
         print("Pico tick is {}".format(tick))
      [#Pico tick is 3308552317#]

      status, tick = pico.tick() # get current Pico tick

      if status == picod.STATUS_OKAY:
         print("Pico tick is {}".format(tick))
      [#Pico tick is 3308584543#]
      ...

      Tick is the number of microseconds since system boot. It is
      an unsigned 32 bit quantity and wraps around approximately
      every 71.6 minutes.
      """

      status, data = self._request(_CMD_TICK, reply=reply, flush=flush)

      if status == STATUS_OKAY:
         self._GPIO_tick, = struct.unpack(">I", data)

      return status, self._GPIO_tick

   def uid(self, reply=REPLY_NOW, flush=True):
      """
      Returns the Pico's unique id (64-bit number)

      Returns a tuple of status and the uid.

      ...
      status, uid = pico.uid() # get Pico's unique Id

      if status == picod.STATUS_OKAY:
         print("Pico UID is 0x{:x}".format(uid))
      [#Pico UID is 0xe66038b713096330#]
      ...

      The Pico does not have an on-board unique identifier (all
      instances of RP2040 silicon are identical and have no persistent
      state). However, the Pico boots from serial NOR flash devices
      which have a 64-bit unique ID as a standard feature, and there
      is a 1:1 association between the Pico and the flash, so this is
      suitable for use as a unique identifier.
      """

      status, data = self._request(_CMD_UID, reply=reply, flush=flush)

      uid = None

      if status == STATUS_OKAY:
         uid, = struct.unpack(">Q", data)

      return status, uid

   def set_config_value(self, cfg_item, cfg_value, reply=REPLY_NOW, flush=True):
      """
      Sets the value of an internal configuration item.

       cfg_item:= the config item.
      cfg_value:= the value to set.
      """

      data = struct.pack(">II", cfg_item, cfg_value)

      return self._request(_CMD_SET_CONFIG_VAL,
         data, reply=reply, flush=flush)[0]

   def get_config_value(self, cfg_item, reply=REPLY_NOW, flush=True):
      """
      Gets the value of an internal configuration item.

      cfg_item:= the config item.
      """

      data = struct.pack(">I", cfg_item)

      status, data=  self._request(_CMD_GET_CONFIG_VAL,
         data, reply=reply, flush=flush)

      value = None

      if status == STATUS_OKAY:
         value, = struct.unpack(">I", data)

      return status, value


# CALLBACKS ---------------------------------------------------------------

   def callback(self, gpio, edge=EDGE_RISING, func=None):
      """
      Sets an alert callback for a single GPIO.

      gpio:= the GPIO to act upon.
      edge:= EDGE_BOTH, EDGE_RISING (default), or EDGE_FALLING.
      func:= user supplied callback function.

      Returns a callback instance.

      ...
      def cbf(gpio, level, tick, levels):
         print(gpio, level, tick, levels)

      cb1 = callback(22, EDGE_BOTH, cbf)

      cb2 = callback(4, EDGE_FALLING)

      cb3 = callback(17)

      print(cb3.tally())

      cb3.reset_tally()

      cb1.cancel() # To cancel callback cb1.
      ...

      The user supplied callback [#func#] is called whenever the
      specified GPIO edge is detected.  The callback receives four
      parameters: the GPIO, the GPIO's level, the tick, and all
      GPIO levels.

      The reported level will be one of

      0: change to low (a falling edge) 
      1: change to high (a rising edge) 
      2: no level change (a watchdog timeout)

      The tick is the number of microseconds since Pico power-up.
      WARNING: the tick wraps around from 4294967295 to 0 roughly
      every 72 minutes.

      If a [#func#] is not specified a default tally callback is
      provided which simply counts edges.  The count may be retrieved
      by calling the callback instance's tally() method.  The count may
      be reset to zero by calling the callback instance's reset_tally()
      method.

      The callback may be cancelled by calling the callback
      instance's cancel() method.

      A GPIO may have multiple callbacks (although I can't think of
      a reason to do so).

      If you want to track the level of more than one GPIO do so by
      maintaining the state in the callback.  Do not use [*GPIO_read*]
      or [*gpio_read*].  Remember the alert that triggered the callback
      may have happened several milliseconds before and the GPIO may have
      changed level many times since then.
      """

      return _level_callback(self._notify, gpio, edge, func)


   def event_callback(self, event_id, event_mode, count, func):
      """
      Starts a callback for an external event.

        event_id:= Identifies the type of external event.
                   EVT_UART_0_RX = 0
                   EVT_UART_1_RX = 1
                   EVT_I2C_0_RX = 2
                   EVT_I2C_1_RX = 3
                   EVT_SPI_0_RX = 4
                   EVT_SPI_1_RX = 5
                   EVT_I2C_0_TX = 6
                   EVT_I2C_1_TX = 7
                   EVT_SPI_0_TX = 8
                   EVT_SPI_1_TX = 9
      event_mode:= Selects how to respond to event activity.
                   0 EVENT_NONE              no callback
                   1 EVENT_ACTIVITY          callback on activity
                   2 EVENT_COUNT             callback on count or more bytes
                   3 EVENT_RETURN_COUNT      callback return count bytes
                   4 EVENT_RETURN_COUNT_PLUS callback return count or
                                             more bytes
           count:= For RX count bytes or more are available to read.
                   For TX there are count bytes or less ready to send.
            func:= the function to call on specified activity.

      Returns an event callback instance.

      ...
      import time
      import picod

      CHANNEL=1

      def event_callback(id, left, text):
         print("id={} left={} text={}".format(id, left, text))

      pico = picod.pico()
      if not pico.connected:
         exit()

      pico.reset() # Put Pico into a clean state

      ecb = pico.event_callback(
         picod.EVT_UART_1_RX,           # monitor UART 1 RX
         picod.EVENT_RETURN_COUNT_PLUS, # callback on data and return data
         5,                             # need 5 characters or more
         event_callback)                # the callback to call

      status, speed = pico.serial_open(CHANNEL,20,21,115200) # open serial link

      if status == picod.STATUS_OKAY:

         while True:

            pico.serial_write(CHANNEL, time.asctime())
            time.sleep(1)
      ...

      The user supplied callback [#func#] is called whenever the
      activity specified by [#event_mode#] is detected.

      For activity to be detected on UART 0/1 RX the corresponding
      serial link must be open.

      For activity to be detected on I2C 0/1 RX/TX the corresponding
      I2C bus must be open in slave mode.

      For activity to be detected on SPI 0/1 RX/TX the corresponding
      SPI bus must be open in slave mode.

      The callback receives three parameters: the [#event_id#],
      the remaining buffer byte count, and a bytearray. The
      bytearray will be empty unless the event_mode was
      EVENT_RETURN_COUNT or EVENT_RETURN_COUNT_PLUS and the event
      is for a receive buffer.

      The callback may be cancelled by calling the callback instance's
      cancel() method.

      There may only be one callback per [#event_id#].  A new callback
      with the same [#event_id#] replaces any existing one.

      """

      assert 0 <= event_id < EVT_BUFS
      assert 0 <= event_mode <= EVENT_RETURN_COUNT_PLUS
      assert 0 <= count <= 512

      data = struct.pack(">BBH", event_id, event_mode, count)

      self._request(_CMD_EVT_CONFIG, data, reply=REPLY_NONE, flush=True)

      return _event_callback(self._notify, event_id, func)


   def reply_callback(self, command_id, func):
      """
      Starts a reply callback for a command.

      command_id:= the id of the command
            func:= the function to call.

      Returns a reply callback instance.

      The callback receieves three parameters: the [#command_id#],
      the status, and a bytearray containg any returned data.
      """
      return _reply_callback(_notify, command_id, func)

# __init__ ----------------------------------------------------------------

   def __init__(
      self,
      device = os.getenv("PICO_DEVICE", '/dev/ttyACM0'),
      transport = os.getenv("PICO_TRANSPORT", 'serial'),
      baud=230400, host=None, port=None):
      """
      Grants access to a Pico's GPIO.

         device:= the phyical device used by the transport method.
                  The default is '/dev/ttyACM0' unless overridden
                  by the PICO_DEVICE environment variable.
      transport:= the method of communicating with the Pico.
                  serial - use the standard Python serial module.
                  lgpio  - use the lgpio Python module.
                  rgpio  - use the rgpio Python module (remote).
                  pigpio - use the pigpio Python module (remote).
                  The default is serial unless overridden by the
                  PICO_TRANSPORT environment variable.
           baud:= the baud rate used between the Pico and the device.
                  Changing the rate is unlikely to improve performance.
           host:= The remote host (name or dotted quad).
                  Only relevant when transport is rgpio or pigpio.
           port:= The remote host port.
                  Only relevant when transport is rgpio or pigpio.

      Returns a pico instance.

      This establishes a connection to the Pico to be used for sending
      commands and receiving notifications.

      An instance attribute [#connected#] may be used to check the
      success of the connection.  If the connection is established
      successfully [#connected#] will be True, otherwise False.

      ...
      pico = picod.pico() # use defaults
      if not pico.connected:
         exit()
      ...
      """
      try:
         if transport == 'serial':

            import serial

            _pico_serial = serial.Serial(device, baud, timeout=0)

            def _serial_read(count):
               return bytearray(_pico_serial.read(count))

            self._pico_serial_read = _serial_read
            self._pico_serial_write = _pico_serial.write
      
         elif transport == 'lgpio':

            import lgpio as sbc

            _pico_serial = sbc.serial_open(device, baud)

            def _serial_read(count):
               b, d = sbc.serial_read(_pico_serial, count)
               return d

            def _serial_write(data):
               sbc.serial_write(_pico_serial, data)

            self._pico_serial_read = _serial_read
            self._pico_serial_write = _serial_write

         elif transport == 'rgpio':

            import rgpio

            if host is None and port is None:
               sbc = rgpio.sbc()
            elif host is None:
               sbc = rgpio.sbc(port=port)
            elif port is None:
               sbc = rgpio.sbc(host=host)
            else:
               sbc = rgpio.sbc(host=host, port=port)

            _pico_serial = sbc.serial_open(device, baud)

            def _serial_read(count):
               b, d = sbc.serial_read(_pico_serial, count)
               return d

            def _serial_write(data):
               sbc.serial_write(_pico_serial, data)

            self._pico_serial_read = _serial_read
            self._pico_serial_write = _serial_write

         elif transport == 'pigpio':

            import pigpio

            if host is None and port is None:
               sbc = pigpio.pi()
            elif host is None:
               sbc = pigpio.pi(port=port)
            elif port is None:
               sbc = pigpio.pi(host=host)
            else:
               sbc = pigpio.pi(host=host, port=port)

            _pico_serial = sbc.serial_open(device, baud)

            def _serial_read(count):
               b, d = sbc.serial_read(_pico_serial, count)
               return d

            def _serial_write(data):
               sbc.serial_write(_pico_serial, data)

            self._pico_serial_read = _serial_read
            self._pico_serial_write = _serial_write

         elif transport == 'null':
            def _serial_read(count):
               return bytearray()
            def _serial_write(data):
               print(_byte2hex(data))
            self._pico_serial_read = _serial_read
            self._pico_serial_write = _serial_write


         else:
            print("unknown PICO_LINK of {}".format(transport))
            raise ValueError

      except:
         exception = 1
         raise

      else:
         exception = 0
         atexit.register(self.close)

      if exception == 0:
         self.connected = True
      else:
         self.connected = False

      if host is None and port is None:
         hp = ""
      elif host is None:
         hp = " (port={})".format(port)
      elif port is None:
         hp = " (host={})".format(host)
      else:
         hp = " (host={} port={})".format(host, port)
      
      self.repr = "<pico transport={}{} device={} (baud={})>".format(
         transport, hp, device, baud)

      self._pending = bytearray()
      self._thread_data = threading.local()
      self._thread_data.queue = 0 # main thread is 0
      self._sync = [[],[],[]] # support main thread plus two more
      self._GPIO_levels = 0
      self._GPIO_tick = 0
      self._GPIO_pulls = 0
      self._GPIO_function = 0

      self._notify = _callback_thread(self)

   def __repr__(self):
      return self.repr

   def close(self):
      """
      Release Pico resources.

      Nothing is returned.
      """
      self.connected = False

      if self._notify is not None:
         self._notify.stop()
         self._notify = None

def modver():
   """
   Returns the picod Python module version (dotted quad).

   ...
   version = picod.modver()

   print("picod Python version is {}".format(version))
   [#picod Python version is 0.1.0.1#]
   ...

   For a version of A.B.C.D

   . .
   A. API major version, changed if breaks previous API
   B. API minor version, changed when new function added
   C. bug fix
   D. documentation change
   . .
   """

   major = (VERSION>>24)&255
   minor = (VERSION>>16)&255
   bug = (VERSION>>8)&255
   doc = VERSION&255

   return "{}.{}.{}.{}".format(major, minor, bug, doc)

def tick_diff(t1, t2):
   """
   Returns the microsecond difference between two ticks.

   t1:= the earlier tick
   t2:= the later tick

   ...
   print(picod.tick_diff(4294967272, 12))
   [#36#]
   ...

   The correct result is returned even if tick has wrapped around.
   """
   tDiff = t2 - t1

   if tDiff < 0:
      tDiff += (1 << 32)

   return tDiff

def status_text(status):
   """
   Returns the text associated with a numeric status code.

   status:= the numeric status code.

   ...
   print(picod.status_text(3))
   [#bad GPIO#]
   ...
   """
   if status in _status:
      return _status[status]
   else:
      return "unknown status code"

def xref():
   """
   addr: 0x03 - 0x77
   The address of a device on the I2C bus.
   ALERTS:
   A bit mask indicating the GPIO to select.

   If bit x is set then GPIO x is selected.
   baud:
   The speed of a serial link in bits per second.
   cfg_item:
   A number uniquely identifying a configurable value.
   cfg_value:
   The value to give to a configurable item.
   channel:
   A small number identifying an instance of a hardware device.

   Device @ Channels
   ADC    @ 0-4
   I2C    @ 0-1
   SPI    @ 0-1
   UART   @ 0-1
   command_id:
   A command identifier.  It is used when requesting later notification of
   command replies.
   count:
   The number of bytes to read/write.
   cs:
   The GPIO to be used for the SPI chip select signal.
   cts:
   The GPIO to be used for the CTS serial signal.
   data:
   Data bytes to be transmitted.
   data_bits: 5-8
   The number of serial data bits.
   device:
   The system name for the serial link used to connect to the Pico.
   dutycycle: 0.0 - 100.0
   The PWM percentage of high time.
   edge:
   Constant     @ Value @ Meaning
   EDGE_RISING  @ 0     @ level change from low to high
   EDGE_FALLING @ 1     @ level change from high to low
   EDGE_BOTH    @ 2     @ any level change
   enable:
   Set to true to enable, false to disable.
   event_id:
   Identifies the type of external event.
   event_mode:
   Selects how to respond to event activity.
   frequency:
   The PWM frequency in Hertz.
   func:
   A number used to identify a GPIO function

   Constant  @ Value @ Meaning
   FUNC_XIP  @ 0     @
   FUNC_SPI  @ 1     @ SPI
   FUNC_UART @ 2     @ UART
   FUNC_I2C  @ 3     @ I2C
   FUNC_PWM  @ 4     @ PWM
   FUNC_SIO  @ 5     @ GPIO
   FUNC_PIO0 @ 6     @  PIO
   FUNC_PIO1 @ 7     @ PIO
   FUNC_GPCK @ 8     @ Clock
   FUNC_USB  @ 9     @ USB
   FUNC_NULL @ 15    @ Null

   FUNCS:
   The function for GPIO x is indicated by bits 4x+3, 4x+2, 4x+1, and 4x
   of FUNCS.

   Bits @ Constant
   0000 @ FUNC_XIP
   0001 @ FUNC_SPI
   0010 @ FUNC_UART
   0011 @ FUNC_I2C
   0100 @ FUNC_PWM
   0101 @ FUNC_SIO (GPIO)
   0110 @ FUNC_PIO0
   0111 @ FUNC_PIO1
   1000 @ FUNC_GPCK
   1001 @ FUNC_USB
   1111 @ FUNC_NULL

   GPIO:
   A bit mask indicating the GPIO to select.

   If bit x is set then GPIO x is selected.
   gpio: 0-29
   A GPIO.
   gpioAB: 0-29
   A PWM channel A or B GPIO.

   There are 8 independent PWM engines, referred to as slices.

   Each slice has an A and B channel and can output to several GPIO.

   Each slice may have a different frequency. All GPIO on a slice have
   the same frequency.

   The A and B channels may have different dutycycles.  All GPIO on
   a slice/channel combination have the same dutycycle.

   Note that the latest settings made to a slice apply to all
   the slice GPIO.  In particular setting a slice for output will
   stops its use for input and vice versa.

   Slice @ A      @ B
   0     @ 0, 16  @ 1,17
   1     @ 2, 18  @ 3,19
   2     @ 4, 20  @ 5,21
   3     @ 6, 22  @ 7,23
   4     @ 8, 24  @ 9,25
   5     @ 10, 26 @ 11,27
   6     @ 12, 28 @ 13,29
   7     @ 14     @ 15
   gpioB: 1,3,...,29
   A PWM channel B GPIO.

   There are 8 independent PWM engines, referred to as slices.

   Each slice has an A and B channel.  The B channel can read the input
   from several GPIO.

   Note that the latest settings made to a slice apply to all
   the slice GPIO.  In particular setting a slice for input will
   stops its use for output and vice versa.

   Slice @ B
   0     @ 1,17
   1     @ 3,19
   2     @ 5,21
   3     @ 7,23
   4     @ 9,25
   5     @ 11,27
   6     @ 13,29
   7     @ 15
   host:
   The remote host (name or dotted quad).
   inout_GPIO:
   A bit mask indicating the GPIO to select.

   If bit x is set then GPIO x is selected.
   level:
   The reported level of a GPIO.

   Constant      @ Value @ Meaning
   LEVEL_LOW     @ 0     @ Low
   LEVEL_HIGH    @ 1     @ High
   LEVEL_TIMEOUT @ 2     @ Watchdog timeout
   nostop:
   Normally a stop condition is asserted after an I2C transaction.  If
   one is not required nostop is set.
   out_GPIO:
   A bit mask indicating the GPIO to select.

   If bit x is set then GPIO x is selected.
   out_LEVEL:
   A bit mask indicating the GPIO to select.

   If bit x is set then GPIO x is selected.
   parity:
   The parity to be used on a serial link.

   Constant    @ Value @ Meaning
   PARITY_NONE @ 0     @ No parity
   PARITY_EVEN @ 1     @ Even parity
   PARITY_ODD  @ 2     @ Odd parity
   port:
   The remote host port.
   pull:
   A value indicating the desired pulls on a GPIO.

   Constant  @ Value @ Meaning
   PULL_NONE @ 0     @ No pulls
   PULL_DOWN @ 1     @ Pull down to ground
   PULL_UP   @ 2     @ Pull up to 3V3
   PULL_BOTH @ 3     @ Sticky GPIO
   PULLS:
   The pulls for GPIO x are set by bits 2x+1 and 2x of PULLS.

   Bits @ Constant
   00   @ PULL_NONE
   01   @ PULL_DOWN
   10   @ PULL_UP
   11   @ PULL_BOTH
   pulsewidth: 0, 500-2500
   A servo pulsewidth in microseconds.
   rts:
   The GPIO to be used for the RTS serial signal.
   rx:
   The GPIO to be used for the RX serial/SPI signal
   sck:
   The GPIO to be used for the SPI clock signal.
   scl:
   The GPIO to be used for the I2C clock signal.
   sda:
   The GPIO to be used for I2C data signal.
   secs:
   A number of seconds (may be fractional).   
   slave_addr:
   The I2C address to respond to when an I2C slave.
   slave_cs:
   The GPIO to respond to when a SPI slave.
   spi_bits: 4-16
   The number of bits in a SPI data word.
   spi_dummy:
   The dummy byte to send during a SPI read (default 0).
   spi_mode:
   The SPI mode which determines the clock polarity and phase.
   mode@CPOL@CPHA
   0   @ 0  @ 0
   1   @ 0  @ 1
   2   @ 1  @ 0
   3   @ 1  @ 1
   status:
   A numeric status code.
   stop_bits: 1-2
   The number of serial stop bits.
   t1:
   A tick (earlier).
   t2:
   A tick (later).
   timeout:
   A number of seconds (may be fractional).   
   transport:
   The software used to manage the serial link to the Pico.
   tx:
   The GPIO to be used for the TX serial/SPI signal
   """
   pass

