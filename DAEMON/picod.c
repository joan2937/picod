/*
  Top view
     USB
GP0  o  o  VBUS
GP1  o  o  VSYS
GND  o  o  GND
GP2  o  o  3V3_EN
GP3  o  o  3V3
GP4  o  o  ADC_VREF
GP5  o  o  GP28_ADC
GND  o  o  AGND
GP6  o  o  GP27_A1
GP7  o  o  GP26_A0
GP8  o  o  RUN
GP9  o  o  GP22
GND  o  o  GND
GP10 o  o  GP21
GP11 o  o  GP20
GP12 o  o  GP19
GP13 o  o  GP18
GND  o  o  GND
GP14 o  o  GP17
GP15 o  o  GP16
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

/* defines */

#define PD_VERSION 0x00000600

#define CFG_DEBUG_MASK 0

#define PD_LINK_UART 0
#define PD_LINK_USB 1

#define PD_LINK PD_LINK_USB

#define DBG_LEVEL_1                  (1ul<<0)
#define DBG_LEVEL_COMMAND_OPEN_CLOSE (1ul<<1)
#define DBG_LEVEL_WATCHDOG           (1ul<<2)
#define DBG_LEVEL_DEBOUNCE           (1ul<<3)
#define DBG_LEVEL_EVENT              (1ul<<4)

#define EVT_UART_0_RX 0
#define EVT_UART_1_RX 1
#define EVT_I2C_0_RX 2
#define EVT_I2C_1_RX 3
#define EVT_SPI_0_RX 4
#define EVT_SPI_1_RX 5
#define EVT_MAX_RX 5

#define EVT_I2C_0_TX 6
#define EVT_I2C_1_TX 7
#define EVT_SPI_0_TX 8
#define EVT_SPI_1_TX 9
#define EVT_BUFS 10

#define EVENT_NONE 0
#define EVENT_ACTIVITY 1
#define EVENT_COUNT 2
#define EVENT_RETURN_COUNT 3
#define EVENT_RETURN_COUNT_PLUS 4

#define MSG_HEADER_LEN 5

#define MAX_REPORTS 10000
#define MAX_EMITS 200

#define PD_NUM_GPIO 30
#define PD_MASK_USER 0x1e7fffff

#define PD_TIMEOUT 2

#define PD_PULL_NONE 0
#define PD_PULL_DOWN 1
#define PD_PULL_UP 2
#define PD_PULL_BOTH 3

#define REPLY_NONE 0
#define REPLY_NOW 1
#define REPLY_LATER 2

#define WATCHDOG_BIT (1ul<<31)

#define I2C_BUF_SIZE 128
#define SPI_BUF_SIZE 128
#define UART_BUF_SIZE 512

#define PD_CMD_GPIO_OPEN 10
#define PD_CMD_GPIO_CLOSE 11
#define PD_CMD_GPIO_SET_IN_OUT 12
#define PD_CMD_GPIO_READ 13
#define PD_CMD_GPIO_WRITE 14

#define PD_CMD_PULLS_SET 20
#define PD_CMD_PULLS_GET 21

#define PD_CMD_FUNCTION_SET 25
#define PD_CMD_FUNCTION_GET 26

#define PD_CMD_ALERT_DEBOUNCE 30
#define PD_CMD_ALERT_WATCHDOG 31
#define PD_CMD_ALERT_SELECT 32

#define PD_CMD_EVT_CONFIG 35

#define PD_CMD_ADC_READ 40
#define PD_CMD_ADC_CLOSE 41

#define PD_CMD_I2C_OPEN 50
#define PD_CMD_I2C_CLOSE 51
#define PD_CMD_I2C_READ 52
#define PD_CMD_I2C_WRITE 53
#define PD_CMD_I2C_PUSH 54
#define PD_CMD_I2C_POP 55

#define PD_CMD_PWM_READ_FREQ 60
#define PD_CMD_PWM_READ_DUTY 61
#define PD_CMD_PWM_READ_EDGE 62
#define PD_CMD_PWM 63
#define PD_CMD_SERVO 64
#define PD_CMD_PWM_CLOSE 65

#define PD_CMD_SPI_OPEN 70
#define PD_CMD_SPI_CLOSE 71

#define PD_CMD_SPI_READ 75
#define PD_CMD_SPI_WRITE 76
#define PD_CMD_SPI_XFER 77
#define PD_CMD_SPI_PUSH 78
#define PD_CMD_SPI_POP 79

#define PD_CMD_UART_OPEN 85
#define PD_CMD_UART_CLOSE 86
#define PD_CMD_UART_READ 87
#define PD_CMD_UART_WRITE 88

#define PD_CMD_UID 90

#define PD_CMD_TICK 94
#define PD_CMD_SLEEP_US 95
#define PD_CMD_RESET_PICO 96
#define PD_CMD_SET_CONFIG_VAL 97
#define PD_CMD_GET_CONFIG_VAL 98
#define PD_CMD_PD_VERSION 99

#define STATUS_OKAY 0
#define STATUS_BAD_CHANNEL 1
#define STATUS_CHANNEL_CLOSED 2
#define STATUS_BAD_GPIO 3
#define STATUS_BAD_PARAM 4
#define STATUS_BAD_WRITE 5
#define STATUS_BAD_READ 6
#define STATUS_NO_REPLY 7
#define STATUS_GPIO_IN_USE 8
#define STATUS_UNKNOWN_COMMAND 9
#define STATUS_TIMED_OUT 10
#define STATUS_INVALID_WHEN_MASTER 11
#define STATUS_INVALID_WHEN_SLAVE 12
#define STATUS_BAD_CONFIG_ITEM 13


#define GPIO_ADC_BASE 26

#define RES_MAX_LEN 32764
#define MSG_MAX_LEN 32764

#define MSG_HEADER       0xff

#define MSG_BAD_CHECKSUM 0xfe
#define MSG_BAD_LENGTH   0xfd
#define MSG_BAD_COMMAND  0xfc
#define MSG_GPIO_LEVELS  0xfb
#define MSG_DEBUG        0xfa
#define MSG_ERROR        0xf9
#define MSG_ASYNC        0xf8

#define CMD_FLG 2
#define CMD_CMD 3

#define U_NULL 0
#define U_CH0  1
#define U_CH1  2
#define U_RX   4
#define U_TX   8
#define U_CTS 16
#define U_RTS 32

#define I_NULL 0
#define I_CH0  1
#define I_CH1  2
#define I_SDA  4
#define I_SCL  8

#define S_NULL 0
#define S_CH0  1
#define S_CH1  2
#define S_RX   4
#define S_CS   8
#define S_SCK 16
#define S_TX  32

#define PD_FUNC_FREE 0
#define PD_FUNC_GPIO 1
#define PD_FUNC_SPI 2
#define PD_FUNC_UART 3
#define PD_FUNC_I2C 4
#define PD_FUNC_PWM 5
#define PD_FUNC_SERVO 6
#define PD_FUNC_READ_FREQ 7
#define PD_FUNC_READ_DUTY 8
#define PD_FUNC_READ_EDGE 9
#define PD_FUNC_ADC 10
#define PD_FUNC_RESERVED 15
#define PD_FUNC_OVERRIDE 64

/* typedefs */

typedef int (*callbk_t) ();

typedef struct pd_buf_s
{
   uint8_t *buf;
   uint size;
   uint rPos;
   uint wPos;
} pd_buf_t, *pd_buf_p;

typedef struct pd_config_s
{
   uint8_t channel;
   uint8_t func;
   uint8_t subfunc;
} pd_config_t, *pd_config_p;

typedef struct pd_report_s
{
   uint32_t tick;
   uint32_t levels;
} pd_report_t, *pd_report_p;

typedef struct pd_level_info_s
{
   uint32_t last_rpt_ts;
   uint32_t last_rpt_lv;
   uint32_t last_evt_ts;
   uint32_t last_evt_lv;
   uint32_t watchdogd;
   uint32_t watchdog_micros;
   uint32_t debounced;
   uint32_t debounce_micros;
} pd_level_info_t, *pd_level_info_p;

typedef struct pd_event_config_s
{
   uint32_t type;
   uint32_t count;
} pd_event_config_t, *pd_event_config_p;

typedef struct global_s
{
   int adcInited;
   int adcChannelSelected;
   uint32_t pwmReadWrap[8];
   uint64_t pwmReadTime[8];
   int i2cInited[2];
   int spiInited[2];
   int i2cMaster[2];
   int spiMaster[2];
   int i2cSlaveAddr[2];
   int uartInited[2];
   int adcChannelInited[4];
   uint32_t is_GPIO;       // bit x is set if gpio x is mode GPIO
   uint32_t GPIO_alert;    // bit x is set if gpio x has an irq
   uint32_t GPIO_watchdog; // bit x is set if gpio x has a watchdog
   uint32_t GPIO_debounce; // bit x is set if gpio x has a debounce
   uint32_t GPIO_levels;   // current GPIO levels
   uint32_t GPIO_tick;     // current GPIO tick
   uint32_t event_flag;    // flag async event
   uint32_t event_alert;   // report async event
} global_t, *global_p;

/* prototypes */

static void debug(uint8_t *buf);
static void fatal(uint8_t *buf);
static void pd_init();
static void toggle_led();

/* preset constants */

static const uint LED_PIN = 25;

static i2c_inst_t   *I2C[2] = {i2c0,  i2c1};
static uart_inst_t *UART[2] = {uart0, uart1};
static spi_inst_t   *SPI[2] = {spi0,  spi1};

static int UARTIRQ[2] = {UART0_IRQ, UART1_IRQ};

/*
UART

0 TX  0 12 16 28
  RX  1 13 17 29
  CTS 2 14 18 -
  RTS 3 15 19 -

1 TX  4  8 20 24
  RX  5  9 21 25
  CTS 6 10 22 26
  RTS 7 11 23 27

32  16  08 04 02  01
RTS CTS RX TX CH1 CH0
*/

static const uint8_t gpio2uart[PD_NUM_GPIO]={
   U_CH0|U_TX,   U_CH0|U_RX,   U_CH0|U_CTS,   U_CH0|U_RTS,   // 0 - 3
   U_CH1|U_TX,   U_CH1|U_RX,   U_CH1|U_CTS,   U_CH1|U_RTS,   // 4 - 7
   U_CH1|U_TX,   U_CH1|U_RX,   U_CH1|U_CTS,   U_CH1|U_RTS,   // 8 - 11
   U_CH0|U_TX,   U_CH0|U_RX,   U_CH0|U_CTS,   U_CH0|U_RTS,   // 12 - 15
   U_CH0|U_TX,   U_CH0|U_RX,   U_CH0|U_CTS,   U_CH0|U_RTS,   // 16 - 19
   U_CH1|U_TX,   U_CH1|U_RX,   U_CH1|U_CTS,   U_NULL,        // 20 - 23
   U_NULL,       U_NULL,       U_CH1|U_CTS,   U_CH1|U_RTS,   // 24 - 27
   U_CH0|U_TX,   U_NULL                                      // 28 - 29
};

/*
I2C

0 SDA 0 4  8 12 16 20 24 28
  SCL 1 5  9 13 17 21 25 29

1 SDA 2 6 10 14 18 22 26
  SCL 3 7 11 15 19 23 27

08  04   02  01
SCL SDA CH1 CH0
*/

static const uint8_t gpio2i2c[PD_NUM_GPIO]={
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_CH1|I_SCL, // 0 - 3
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_CH1|I_SCL, // 4 - 7
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_CH1|I_SCL, // 8 - 11
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_CH1|I_SCL, // 12 - 15
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_CH1|I_SCL, // 16 - 19
   I_CH0|I_SDA,   I_CH0|I_SCL,   I_CH1|I_SDA,   I_NULL,      // 20 - 23
   I_NULL,        I_NULL,        I_CH1|I_SDA,   I_CH1|I_SCL, // 24 - 27
   I_CH0|I_SDA,   I_NULL                                     // 28 - 29
};

/*
SPI

0 RX   0 4 16 20
  CS   1 5 17 21
  SCK  2 6 18 22
  TX   3 7 19 23

1 RX   8 12 24 28
  CS   9 13 25 29
  SCK 10 14 26 -
  TX  11 15 27 -

32  16  08 04 02  01
TX  SCK CS TX CH1 CH0
*/

static const uint8_t gpio2spi[PD_NUM_GPIO]={
   S_CH0|S_RX,    S_CH0|S_CS,    S_CH0|S_SCK,   S_CH0|S_TX,  // 0 - 3
   S_CH0|S_RX,    S_CH0|S_CS,    S_CH0|S_SCK,   S_CH0|S_TX,  // 4 - 7
   S_CH1|S_RX,    S_CH1|S_CS,    S_CH1|S_SCK,   S_CH1|S_TX,  // 8 - 11
   S_CH1|S_RX,    S_CH1|S_CS,    S_CH1|S_SCK,   S_CH1|S_TX,  // 12 - 15
   S_CH0|S_RX,    S_CH0|S_CS,    S_CH0|S_SCK,   S_CH0|S_TX,  // 16 - 19
   S_CH0|S_RX,    S_CH0|S_CS,    S_CH0|S_SCK,   S_NULL,      // 20 - 23
   S_NULL,        S_NULL,        S_CH1|S_SCK,   S_CH1|S_TX,  // 24 - 27
   S_CH1|S_RX,    S_CH1|S_CS                                 // 28 - 29
};

static const uint16_t crctab[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

/* variables */

/* init once only */

callbk_t _GETCHAR;
callbk_t _PUTCHAR;

int msgLen;
uint16_t msgCrc1;
uint16_t msgCrc2;

/* re-init rPos and wPos when reset */

int rawWritePos;
int rawReadPos;

uint8_t uartRxBuf0[UART_BUF_SIZE];
uint8_t uartRxBuf1[UART_BUF_SIZE];
uint8_t  i2cRxBuf0[I2C_BUF_SIZE];
uint8_t  i2cRxBuf1[I2C_BUF_SIZE];
uint8_t  spiRxBuf0[SPI_BUF_SIZE];
uint8_t  spiRxBuf1[SPI_BUF_SIZE];

uint8_t  i2cTxBuf0[I2C_BUF_SIZE];
uint8_t  i2cTxBuf1[I2C_BUF_SIZE];
uint8_t  spiTxBuf0[SPI_BUF_SIZE];
uint8_t  spiTxBuf1[SPI_BUF_SIZE];

pd_buf_t pd_buf[EVT_BUFS] =
{
   {uartRxBuf0, sizeof(uartRxBuf0), 0, 0},
   {uartRxBuf1, sizeof(uartRxBuf1), 0, 0},
   { i2cRxBuf0, sizeof(i2cRxBuf0),  0, 0},
   { i2cRxBuf1, sizeof(i2cRxBuf1),  0, 0},
   { spiRxBuf0, sizeof(spiRxBuf0),  0, 0},
   { spiRxBuf1, sizeof(spiRxBuf1),  0, 0},
   { i2cTxBuf0, sizeof(i2cTxBuf0),  0, 0},
   { i2cTxBuf1, sizeof(i2cTxBuf1),  0, 0},
   { spiTxBuf0, sizeof(spiTxBuf0),  0, 0},
   { spiTxBuf1, sizeof(spiTxBuf1),  0, 0},
};

/* re-init when reset */

uint32_t cfg_debug_mask = CFG_DEBUG_MASK;

global_t g;
pd_config_t pd_config[PD_NUM_GPIO];
pd_level_info_t pd_level_info[PD_NUM_GPIO];
pd_event_config_t pd_event_config[EVT_BUFS];

/* arrays which don't need initing (pointers inited instead) */

uint8_t resBuf[RES_MAX_LEN+4]; /* room for length plus checksum */
uint8_t cmdBuf[MSG_MAX_LEN+4]; /* room for length plus checksum */

pd_report_t raw_report[MAX_REPORTS];
pd_report_t emit_report[MAX_EMITS];

/* functions */

void toggle_led()
{
   static int on = 0;
   gpio_put(LED_PIN, on++);
   if (on > 1) on = 0;
}

uint16_t crc16(uint8_t *data, int len, uint16_t crc)
{
   crc &= 0xffff;

   while(len-- > 0)
   {
      crc = ((crc<<8)&0xff00) ^ crctab[(crc>>8)^*data++];
   }

   return crc;
}

uint32_t swap32(uint32_t x)
{ return __builtin_bswap32(x);}

uint16_t swap16(uint16_t x)
{ return __builtin_bswap16(x);}

void my_core1_gpio_handler()
{
   static uint32_t reportedLevels = -1;
   uint32_t nowLevels;
   uint32_t nowTimestamp;
   int nextPos;

   nowTimestamp = time_us_32();

   /* clear all GPIO interrupts */
   iobank0_hw->intr[0] = -1ul;
   iobank0_hw->intr[1] = -1ul;
   iobank0_hw->intr[2] = -1ul;
   iobank0_hw->intr[3] = -1ul;

   nowLevels = gpio_get_all();

   if ((nowLevels & g.GPIO_alert) != (reportedLevels & g.GPIO_alert))
   {
      nextPos = (rawWritePos+1) % MAX_REPORTS;

      if (nextPos != rawReadPos)
      {
         raw_report[rawWritePos].tick = nowTimestamp;
         raw_report[rawWritePos].levels = nowLevels;
         rawWritePos = nextPos;
         reportedLevels = nowLevels;
      }
   }
}

void bufIncRpos(int id)
{
   int nextPos;

   nextPos = (pd_buf[id].rPos + 1) % pd_buf[id].size;

   pd_buf[id].rPos = nextPos;
}

int bufUsed(int id)
{
   int entries;

   entries = pd_buf[id].wPos - pd_buf[id].rPos;

   if (entries < 0) entries += pd_buf[id].size;

   return entries;
}

void bufReset(int id)
{
   pd_buf[id].rPos = 0;
   pd_buf[id].wPos = 0;
}

uint32_t bufPopValue(int id, uint32_t default_value)
{
   uint32_t value = default_value;
   int nextPos;

   if (pd_buf[id].rPos != pd_buf[id].wPos)
   {
      value = pd_buf[id].buf[pd_buf[id].rPos];

      nextPos = (pd_buf[id].rPos + 1) % pd_buf[id].size;

      pd_buf[id].rPos = nextPos;
   }
   return value;
}

int bufPop(int id, int count, uint8_t *buf)
{
   int moved = 0;
   int nextPos;

   while ((pd_buf[id].rPos != pd_buf[id].wPos) && (moved < count))
   {
      buf[moved] = pd_buf[id].buf[pd_buf[id].rPos];

      nextPos = (pd_buf[id].rPos + 1) % pd_buf[id].size;

      pd_buf[id].rPos = nextPos;

      moved++;
   }
   return moved;
}

int bufPushValue(int id, uint value)
{
   int nextPos;

   nextPos = (pd_buf[id].wPos + 1) % pd_buf[id].size;

   if (nextPos != pd_buf[id].rPos)
   {
      pd_buf[id].buf[pd_buf[id].wPos] = value;

      pd_buf[id].wPos = nextPos;
   }
}

int bufPush(int id, int count, uint8_t *buf)
{
   int moved = 0;
   int i, nextPos;

   for (i=0; i<count; i++)
   {
      nextPos = (pd_buf[id].wPos + 1) % pd_buf[id].size;

      if (nextPos != pd_buf[id].rPos)
      {
         moved++;

         pd_buf[id].buf[pd_buf[id].wPos] = buf[i];

         pd_buf[id].wPos = nextPos;
      }
   }
   return moved;
}

void my_pwm_wrap_handler()
{
   int irq;
   int slice;

   irq = pwm_get_irq_status_mask();

   for (slice=0; slice<8; slice++)
   {
      if (irq & (1<<slice))
      {
        ++g.pwmReadWrap[slice];
        pwm_clear_irq(slice);
      }
   }
}

void my_uart_handler()
{
   uint32_t i, status, dummy, value, nextPos;

   for (i=0; i<2; i++)
   {
      if (g.uartInited[i])
      {
         while (uart_is_readable(UART[i]))
         {
            g.event_flag |= (1<<(EVT_UART_0_RX+i));

            value = uart_getc(UART[i]);

            bufPushValue(EVT_UART_0_RX+i, value);
         }
      }
   }
}

void my_i2c_handler()
{
   uint32_t i, status, dummy, value, nextPos;

   for (i=0; i<2; i++)
   {
      if (g.i2cInited[i] && (!g.i2cMaster[i])) // inited and slave
      {
         status = I2C[i]->hw->intr_stat;

         if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) // rx
         {
            g.event_flag |= (1<<(EVT_I2C_0_RX+i));

            value = I2C[i]->hw->data_cmd;

            bufPushValue(EVT_I2C_0_RX+i, value);
         }

         if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) // tx
         {
            g.event_flag |= (1<<(EVT_I2C_0_TX+i));

            value = bufPopValue(EVT_I2C_0_TX+i, 0);

            I2C[i]->hw->data_cmd = value;

            dummy = I2C[i]->hw->clr_rd_req;
         }
      }
   }
}

void my_spi_handler()
{
   uint32_t i, status, dummy, value, nextPos;

   for (i=0; i<2; i++)
   {
      if (g.spiInited[i] && (!g.spiMaster[i])) // inited and slave
      {
         status = spi_get_hw(SPI[i])->mis;

         while (status & SPI_SSPMIS_RXMIS_BITS) // rx
         {
            g.event_flag |= (1<<(EVT_SPI_0_RX+i));

            value = spi_get_hw(SPI[i])->dr;

            bufPushValue(EVT_SPI_0_RX+i, value);

            status = spi_get_hw(SPI[i])->mis;
         }

         while (status & SPI_SSPMIS_TXMIS_BITS) // tx
         {
            g.event_flag |= (1<<(EVT_SPI_0_TX+i));

            value = bufPopValue(EVT_SPI_0_TX+i, 0);

            spi_get_hw(SPI[i])->dr = value;

            status = spi_get_hw(SPI[i])->mis;
         }
      }
   }
}

int is_func_okay(uint gpio, uint mode, uint submode, uint channel, uint8_t *fnc)
{
   uint8_t buf[128];

   if (gpio >= PD_NUM_GPIO)
   {
      sprintf(buf, "bad GPIO #%d", gpio);
      fatal(buf);
      return STATUS_BAD_GPIO;
   }

   if (!(PD_MASK_USER & (1<<gpio)))
   {
      sprintf(buf, "reserved GPIO #%d", gpio);
      fatal(buf);
      return STATUS_BAD_GPIO;
   }

   if (pd_config[gpio].func != PD_FUNC_FREE)
   {
      sprintf(buf, "GPIO #%d already in use", gpio);
      fatal(buf);
      return STATUS_GPIO_IN_USE;
   }

   if (channel > 1)
   {
      sprintf(buf, "bad channel #%d", channel);
      fatal(buf);
      return STATUS_BAD_CHANNEL;
   }

   switch (mode)
   {
      case GPIO_FUNC_I2C:
         if (gpio2i2c[gpio] != (submode |(1<<channel)))
         {
            sprintf(buf, "bad GPIO #%d for %s channel %d", gpio, fnc, channel);
            fatal(buf);
            return STATUS_BAD_GPIO;
         }
         break;

      case GPIO_FUNC_SPI:
         if (gpio2spi[gpio] != (submode |(1<<channel)))
         {
            sprintf(buf, "bad GPIO #%d for %s channel %d", gpio, fnc, channel);
            fatal(buf);
            return STATUS_BAD_GPIO;
         }
         break;

      case GPIO_FUNC_UART:
         if (gpio2uart[gpio] != (submode |(1<<channel)))
         {
            sprintf(buf, "bad GPIO #%d for %s channel %d", gpio, fnc, channel);
            fatal(buf);
            return STATUS_BAD_GPIO;
         }
         break;

      default:
         return STATUS_BAD_GPIO;
   }

   return STATUS_OKAY;
}

int getuartchar(int dummy)
{
   int nextPos;
   int ch=PICO_ERROR_TIMEOUT;

   if (pd_buf[EVT_UART_0_RX].rPos !=
       pd_buf[EVT_UART_0_RX].wPos)
   {
      ch = pd_buf[EVT_UART_0_RX].buf[pd_buf[EVT_UART_0_RX].rPos];

      nextPos = (pd_buf[EVT_UART_0_RX].rPos + 1) %
                 pd_buf[EVT_UART_0_RX].size;

      pd_buf[EVT_UART_0_RX].rPos = nextPos;
   }

   return ch;
}

int putuartchar(int ch)
{
   while (!uart_is_writable(UART[0])) sleep_us(5);
   uart_putc_raw(UART[0], ch);
   return 0;
}

uint16_t unpack16(uint8_t *src)
{
   uint16_t temp;

   memcpy(&temp, src, 2);
   return __builtin_bswap16(temp);
}

void pack16(uint8_t *src, uint16_t value)
{
   uint16_t temp;

   temp = __builtin_bswap16(value);
   memcpy(src, &temp, 2);
}

uint32_t unpack32(uint8_t *src)
{
   uint32_t temp;

   memcpy(&temp, src, 4);
   return __builtin_bswap32(temp);
}

void pack32(uint8_t *src, uint32_t value)
{
   uint32_t temp;

   temp = __builtin_bswap32(value);
   memcpy(src, &temp, 4);
}

void _message(int len)
{
   /*
   <------------ Length bytes ------------>
   +---+-------+-------+----------+-------+
   |Hdr|Length | CRC 1 |Request(s)| CRC 2 |
   |xFF|msb|lsb|msb|lsb|          |msb|lsb|
   +---+---+---+---+---+----------+---+---+

   CRC1: Hdr+Length
   CRC2: Hdr+Length+CRC1+Request(s)
   */

   int i;
   uint16_t crc;

   resBuf[0] = MSG_HEADER;
   pack16(resBuf+1, len+MSG_HEADER_LEN+2);
   crc = crc16(resBuf, 3, 0);
   pack16(resBuf+3, crc);

   crc = crc16(resBuf, len+MSG_HEADER_LEN, 0);
   pack16(resBuf+MSG_HEADER_LEN+len, crc);

   for (i=0; i<(len+MSG_HEADER_LEN+2); i++) _PUTCHAR(resBuf[i]);
}

void cmdRespond(uint cmd, int len, uint8_t *buf)
{
   /*
   Request
   <-------- Length bytes ------->
   +-------+---+---+-------------+
   |Length |Flg|Req|Optional data|
   |msb|lsb|   |   |             |
   +---+---+---+---+-------------+
   */

   int i;

   pack16(resBuf+MSG_HEADER_LEN, (len+4));
   pack16(resBuf+MSG_HEADER_LEN+2, cmd);
   for (i=0; i<len; i++) resBuf[MSG_HEADER_LEN+4+i] = buf[i];
   _message(len+4);
}


void debug(uint8_t *buf)
{
   if (cfg_debug_mask) cmdRespond(MSG_DEBUG, strlen(buf), buf);
}

void fatal(uint8_t *buf)
{
   if (cfg_debug_mask) cmdRespond(MSG_ERROR, strlen(buf), buf);
}

void pd_set_mode(uint gpio, uint func, uint subfunc, uint channel)
{
   pd_config[gpio].channel = channel;
   pd_config[gpio].func = func;
   pd_config[gpio].subfunc = subfunc;

   if (func == PD_FUNC_FREE)
   {
      gpio_init(gpio);
      g.is_GPIO &= ~(1ul << gpio);
   }
   else if (func == PD_FUNC_GPIO)
   {
      gpio_init(gpio);
      g.is_GPIO |= (1ul << gpio);
   }
   else
      g.is_GPIO &= ~(1ul << gpio);
}

void pd_free_func(uint func, uint channel)
{
   int i;

   for (i=0; i<PD_NUM_GPIO; i++)
   {
      if ((pd_config[i].func == func) && (pd_config[i].channel == channel))
         pd_set_mode(i, PD_FUNC_FREE, -1, -1);
   }
}

void spiAssertCS(uint gpio, bool set)
{
   if (set)
   {
      gpio_put(gpio, 0);
      sleep_us(10);
   }
   else
   {
      sleep_us(20);
      gpio_put(gpio, 1);
   }
}

int cmdExec(uint8_t *cBuf)
{
   uint8_t RES[32768];
   int cmd;

   uint channel, clkdiv, gpio, tx, rx, rts, cts, bits, stops, parity,
        sda, scl, sck, cs, mode, dummy, slice, nostop, slave_addr, addr;

   uint steps, high_steps, adc_value, count, id, type;

   uint16_t count16; /* be careful with this count, compiler error? */

   uint32_t microsL, microsH, countL, countH;
   uint32_t micros, GPIO_mask, DIR_mask, LEVEL_mask, ALERT_mask, baud,
            timeout_us, set_value;
   uint32_t func_0_7, func_8_15, func_16_23, func_24_31, pud_0_15, pud_16_31;
   uint32_t item, value;

   pwm_config pwmCfg;
   bool initing;
   int i, nextPos;
   int flags;
   int func;
   int pud;
   bool up, down;
   int status;
   int moved;
   uint32_t *mem;
   uint reply;
   int reply_len;
   uint32_t set_bits;
   uint32_t clear_bits;
   uint64_t tdiff;

   uint d1, d2, d3, d4, d5;

   char buf[256];

   //toggle_led();

   cmd = cBuf[CMD_CMD];
   reply = (cBuf[CMD_FLG] >> 6) & 3;
   reply_len = 1;

   status = STATUS_OKAY;

   switch(cmd)
   {
      case PD_CMD_GPIO_OPEN:

         // ">I" GPIO

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & PD_MASK_USER;

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               if (pd_config[i].func == PD_FUNC_FREE)
               {
                  pd_set_mode(i, PD_FUNC_GPIO, -1, -1);
               }

               if (pd_config[i].func != PD_FUNC_GPIO)
                  status = STATUS_GPIO_IN_USE;
            }
         }

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "GPIO_OPEN: GPIO=%x (OPEN=%x)",
               GPIO_mask, g.is_GPIO);
            debug(buf);
         }

         break;

      case PD_CMD_GPIO_CLOSE:

         // ">I" GPIO_mask

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & g.is_GPIO;

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               pd_set_mode(i, PD_FUNC_FREE, -1, -1);
            }
         }

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "GPIO_CLOSE: GPIO=%x (OPEN=%x)",
               GPIO_mask, g.is_GPIO);
            debug(buf);
         }

         break;

      case PD_CMD_GPIO_SET_IN_OUT:

         // ">III" GPIO_mask GPIO_out GPIO_levels

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & PD_MASK_USER;
         DIR_mask = unpack32(cBuf+CMD_CMD+5);
         LEVEL_mask = unpack32(cBuf+CMD_CMD+9);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "GPIO_SET_IN_OUT: GPIO=%x OUT=%x LEVELS=%x",
               GPIO_mask, DIR_mask, LEVEL_mask);
            debug(buf);
         }

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               if (pd_config[i].func == PD_FUNC_FREE)
               {
                  pd_set_mode(i, PD_FUNC_GPIO, -1, -1);
               }
            }
         }

         // mask off if not set to function gpio

         GPIO_mask &= g.is_GPIO;

         gpio_set_dir_masked(GPIO_mask, DIR_mask); // set GPIO dir
         gpio_put_masked(GPIO_mask & DIR_mask, LEVEL_mask);  // write out GPIO

         break;

      case PD_CMD_GPIO_READ:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "GPIO_READ");
            debug(buf);
         }

         pack32(RES+1, g.GPIO_levels);
         reply_len = 5;

         break;

      case PD_CMD_GPIO_WRITE:

         // ">II" GPIO_mask levels

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & PD_MASK_USER;
         LEVEL_mask = unpack32(cBuf+CMD_CMD+5);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "GPIO_WRITE: GPIO=%x LEVEL=%x",
               GPIO_mask, LEVEL_mask);
            debug(buf);
         }

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               if (pd_config[i].func == PD_FUNC_FREE)
               {
                  pd_set_mode(i, PD_FUNC_GPIO, -1, -1);
               }
            }
         }

         // mask off if not set to function gpio

         GPIO_mask &= g.is_GPIO;

         gpio_set_dir_masked(GPIO_mask, GPIO_mask); // set GPIO dir
         gpio_put_masked(GPIO_mask, LEVEL_mask);

         break;

      case PD_CMD_PULLS_SET:

         // ">III" GPIO_mask PULL_0_15 PULL_16_31

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & g.is_GPIO;
         pud_0_15 = unpack32(cBuf+CMD_CMD+5);
         pud_16_31 = unpack32(cBuf+CMD_CMD+9);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "PULLS_SET: GPIO=%x 0-15=%x 16-31=%x",
               GPIO_mask, pud_0_15, pud_16_31);
            debug(buf);
         }

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               if (i < 16) pud = (pud_0_15 >>  (i*2))      & 3;
               else        pud = (pud_16_31 >> ((i-16)*2)) & 3;

               if (pud & PD_PULL_DOWN) down = true; else down = false;

               if (pud & PD_PULL_UP)   up = true;   else up = false;

               gpio_set_pulls(i, up, down);
            }
         }

         break;

      case PD_CMD_PULLS_GET:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "PULLS_GET");
            debug(buf);
         }

         pud_0_15 = 0;
         pud_16_31 = 0;

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            pud = 0;

            if (gpio_is_pulled_down(i)) pud |= PD_PULL_DOWN;

            if (gpio_is_pulled_up(i))   pud |= PD_PULL_UP;

            if (i<16) pud_0_15  |= (pud << (i*2));
            else      pud_16_31 |= (pud << ((i-16)*2));
         }

         pack32(RES+1, pud_0_15);
         pack32(RES+5, pud_16_31);
         reply_len = 9;

         break;

      case PD_CMD_FUNCTION_SET:

         // ">IIIII" GPIO_mask FUNC_0_7 FUNC_8_15 FUNC_16_23 FUNC_24_31

         GPIO_mask =  unpack32(cBuf+CMD_CMD+ 1) & PD_MASK_USER;
         func_0_7 =   unpack32(cBuf+CMD_CMD+ 5);
         func_8_15 =  unpack32(cBuf+CMD_CMD+ 9);
         func_16_23 = unpack32(cBuf+CMD_CMD+13);
         func_24_31 = unpack32(cBuf+CMD_CMD+17);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf,
               "FUNCTION_SET: GPIO=%x 0-7=%x 8-15=%x 16-23=%x 24_31=%x",
               GPIO_mask, func_0_7, func_8_15, func_16_23, func_24_31);
            debug(buf);
         }

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            if (GPIO_mask & (1<<i))
            {
               if      (i <  8) func = (func_0_7   >> ( i    *4)) & 15;
               else if (i < 16) func = (func_8_15  >> ((i- 8)*4)) & 15;
               else if (i < 24) func = (func_16_23 >> ((i-16)*4)) & 15;
               else             func = (func_24_31 >> ((i-24)*4)) & 15;

               pd_set_mode(i, PD_FUNC_OVERRIDE+func, -1, -1);

               gpio_set_function(i, func);
            }
         }

         break;

      case PD_CMD_FUNCTION_GET:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "FUNCTION_GET");
            debug(buf);
         }

         func_0_7 = 0;
         func_8_15 = 0;
         func_16_23 = 0;
         func_24_31 = 0;

         for (i=0; i<PD_NUM_GPIO; i++)
         {
            func = gpio_get_function(i);

            if      (i <  8) func_0_7   |= (func << ( i    *4));
            else if (i < 16) func_8_15  |= (func << ((i- 8)*4));
            else if (i < 24) func_16_23 |= (func << ((i-16)*4));
            else             func_24_31 |= (func << ((i-24)*4));
         }

         pack32(RES+1,  func_0_7);
         pack32(RES+5,  func_8_15);
         pack32(RES+9,  func_16_23);
         pack32(RES+13, func_24_31);
         reply_len = 17;

         break;

      case PD_CMD_ALERT_DEBOUNCE:

         // ">BI" gpio micros

         gpio = cBuf[CMD_CMD+1];
         micros = unpack32(cBuf+CMD_CMD+2);

         if (gpio < PD_NUM_GPIO)
         {
            if (!pd_level_info[gpio].debounce_micros)
               pd_level_info[gpio].debounced = 1;

            pd_level_info[gpio].debounce_micros = micros;

            if (micros) g.GPIO_debounce |= (1ul<<gpio);
            else        g.GPIO_debounce &= ~(1ul<<gpio);
         }

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "ALERT_DEBOUNCE: gpio=%d micros=%d (mask=%x)",
               gpio, micros, g.GPIO_debounce);
            debug(buf);
         }

         break;

      case PD_CMD_ALERT_WATCHDOG:

         // ">BI" gpio micros

         gpio = cBuf[CMD_CMD+1];
         micros = unpack32(cBuf+CMD_CMD+2);

         if (gpio < PD_NUM_GPIO)
         {
            if (!pd_level_info[gpio].watchdog_micros)
               pd_level_info[gpio].watchdogd = 1;

            pd_level_info[gpio].watchdog_micros = micros;

            if (micros) g.GPIO_watchdog |= (1ul<<gpio);
            else        g.GPIO_watchdog &= ~(1ul<<gpio);
         }

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "ALERT_WATCHDOG: gpio=%d micros=%d (mask=%x)",
               gpio, micros, g.GPIO_watchdog);
            debug(buf);
         }

         break;

      case PD_CMD_ALERT_SELECT:

         // ">II" GPIO_mask alerts

         GPIO_mask = unpack32(cBuf+CMD_CMD+1) & PD_MASK_USER;
         ALERT_mask = unpack32(cBuf+CMD_CMD+5);

         set_bits = GPIO_mask & ALERT_mask;
         clear_bits = GPIO_mask & (~ALERT_mask);

         g.GPIO_alert |= set_bits;
         g.GPIO_alert &= ~clear_bits;

         multicore_fifo_push_blocking(GPIO_mask);
         multicore_fifo_push_blocking(ALERT_mask);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "ALERT_SELECT: GPIO=%x ALERTS=%x alert_GPIO=%x",
               GPIO_mask, ALERT_mask, g.GPIO_alert);
            debug(buf);
         }

         break;

      case PD_CMD_EVT_CONFIG:

         // ">BBH" id type count

         id = cBuf[CMD_CMD+1];
         type = cBuf[CMD_CMD+2];
         count = unpack16(cBuf+CMD_CMD+3);

         if (id < EVT_BUFS)
         {
            pd_event_config[id].type = type;
            pd_event_config[id].count = count;
            if (type) g.event_alert |=   (1<<id);
            else      g.event_alert &= (~(1<<id));
         }

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "EVT_CONFIG: id=%d type=%d count=%d alert=%x",
               id, type, count, g.event_alert);
            debug(buf);
         }

         break;

      case PD_CMD_ADC_READ:

         // ">B" channel

         channel = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "GPIO_ADCR: channel=%d", channel);
            debug(buf);
         }

         adc_value = 0;

         if (channel < 4)
         {
            if (!g.adcChannelInited[channel])
            {
               /* channel 3 is gpio 29 reserved for ADC usage */
               if ((pd_config[channel+GPIO_ADC_BASE].func == PD_FUNC_FREE) ||
                   (channel == 3))
               {
                  if (channel != 3)
                     pd_set_mode(channel+GPIO_ADC_BASE, PD_FUNC_ADC, -1, -1);

                  adc_gpio_init(channel+GPIO_ADC_BASE);

                  g.adcChannelInited[channel] = 1;
               }
               else status = STATUS_GPIO_IN_USE;
            }
         }
         else if (channel > 4) status = STATUS_BAD_CHANNEL;

         if (!status)
         {
            if (!g.adcInited)
            {
               adc_init();
               g.adcInited = 1;
            }

            if (g.adcChannelSelected != channel)
            {
               adc_select_input(channel);

               g.adcChannelSelected = channel;
            }

            adc_value = adc_read();
         }

         RES[1] = channel; // return channel and 16 bit reading
         pack16(RES+2, adc_value);
         reply_len = 4;

         break;

      case PD_CMD_ADC_CLOSE:

         // ">B" channel

         channel = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "GPIO_ADCF: channel=%d", channel);
            debug(buf);
         }

         if ((channel<4) && (g.adcChannelInited[channel]))
         {
            if (channel != 3)
               pd_set_mode(channel+GPIO_ADC_BASE, PD_FUNC_FREE, -1, -1);

            g.adcChannelInited[channel] = 0;
         }

         break;

      case PD_CMD_I2C_OPEN:

         // ">IBBBB" baud channel sda scl slave_addr

         baud = unpack32(cBuf+CMD_CMD+1);
         channel = cBuf[CMD_CMD+5];
         sda = cBuf[CMD_CMD+6];
         scl = cBuf[CMD_CMD+7];
         slave_addr = cBuf[CMD_CMD+8];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "I2C: ch=%d sda=%d scl=%d baud=%d slave=%d",
               channel, sda, scl, baud, slave_addr);
            debug(buf);
         }

         if (!status)
            status = is_func_okay(sda, GPIO_FUNC_I2C, I_SDA, channel, "I2C SDA");

         if (!status)
            status = is_func_okay(scl, GPIO_FUNC_I2C, I_SCL, channel, "I2C SCL");

         if (!status)
         {
            if (g.i2cInited[channel]) i2c_deinit(I2C[channel]);

            g.i2cInited[channel] = 1;

            pd_set_mode(sda, PD_FUNC_I2C, I_SDA, channel);
            gpio_set_function(sda, GPIO_FUNC_I2C);
            gpio_set_pulls(sda, true, false);

            pd_set_mode(scl, PD_FUNC_I2C, I_SCL, channel);
            gpio_set_function(scl, GPIO_FUNC_I2C);
            gpio_set_pulls(scl, true, false);

            set_value = i2c_init(I2C[channel], baud);

            if (slave_addr)
            {
               /* reset RX/TX slave buffers */
               bufReset(EVT_I2C_0_RX+channel);
               bufReset(EVT_I2C_0_TX+channel);

               I2C[channel]->hw->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS |
                                             I2C_IC_INTR_MASK_M_RX_FULL_BITS;

               irq_set_exclusive_handler(I2C0_IRQ+channel, my_i2c_handler);

               irq_set_enabled(I2C0_IRQ+channel, true);

               i2c_set_slave_mode(I2C[channel], true, slave_addr);
               g.i2cMaster[channel] = 0;
               g.i2cSlaveAddr[channel] = slave_addr;
            }
            else
            {
               i2c_set_slave_mode(I2C[channel], false, 0x40);  // any addr
               g.i2cMaster[channel] = 1;
               g.i2cSlaveAddr[channel] = 0;
            }
         }

         pack32(RES+1, set_value);
         reply_len = 5;

         break;

      case PD_CMD_I2C_CLOSE:

         // ">B" channel

         channel = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "I2C_CLOSE: channel=%d", channel);
            debug(buf);
         }

         if (channel < 2)
         {
            if (g.i2cInited[channel])
            {
               pd_free_func(PD_FUNC_I2C, channel);

               i2c_deinit(I2C[channel]);
            }

            g.i2cInited[channel] = 0;
         }

         break;


      case PD_CMD_I2C_READ:

         // ">IHBBB" timeout_us count channel addr nostop

         timeout_us = unpack32(cBuf+CMD_CMD+1);
         count = unpack16(cBuf+CMD_CMD+5);
         channel = cBuf[CMD_CMD+7];
         addr = cBuf[CMD_CMD+8];
         nostop = cBuf[CMD_CMD+9];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "I2C(read): ch=%d addr=%d stop=%d to=%d len=%d",
               channel, addr, nostop, timeout_us, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.i2cInited[channel])
            {
               if (g.i2cMaster[channel])
               {
                  moved = i2c_read_timeout_us(
                     I2C[channel], addr, RES+1, count, nostop, timeout_us);

                  if (moved != count)
                  {
                     status = STATUS_BAD_READ;
                     if (moved < 0) moved = 0;
                  }
               }
               else status = STATUS_INVALID_WHEN_SLAVE;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         reply_len = 1 + moved;

         break;

      case PD_CMD_I2C_WRITE:

         // ">IHBBB..." timeout_us count channel addr nostop data

         timeout_us = unpack32(cBuf+CMD_CMD+1);
         count = unpack16(cBuf+CMD_CMD+5);
         channel = cBuf[CMD_CMD+7];
         addr = cBuf[CMD_CMD+8];
         nostop = cBuf[CMD_CMD+9];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "I2C(write): ch=%d addr=%d stop=%d to=%d len=%d",
               channel, addr, nostop, timeout_us, count);
            debug(buf);
         }

         if (channel < 2)
         {
            if (g.i2cInited[channel])
            {
               if (g.i2cMaster[channel])
               {
                  moved = i2c_write_timeout_us(
                     I2C[channel], addr, cBuf+CMD_CMD+10,
                     count, nostop, timeout_us);

                  if (moved != count) status = STATUS_BAD_WRITE;
               }
               else status = STATUS_INVALID_WHEN_SLAVE;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         break;

      case PD_CMD_I2C_PUSH:

         // ">BB..." channel count data

         channel = cBuf[CMD_CMD+1];
         count = cBuf[CMD_CMD+2];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "I2C_PUSH: ch=%d count=%d", channel, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.i2cInited[channel])
            {
               if (!g.i2cMaster[channel])
               {
                  moved =
                     bufPush(EVT_I2C_0_TX+channel, count, cBuf+CMD_CMD+3);
               }
               else status = STATUS_INVALID_WHEN_MASTER;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         RES[1] = moved;
         reply_len = 2;

         break;

      case PD_CMD_I2C_POP:

         // ">BB" channel count

         channel = cBuf[CMD_CMD+1];
         count = cBuf[CMD_CMD+2];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "I2C_POP: ch=%d count=%d", channel, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.i2cInited[channel])
            {
               if (!g.i2cMaster[channel])
               {
                  moved = bufPop(EVT_I2C_0_RX+channel, count, RES+1);
               }
               else status = STATUS_INVALID_WHEN_MASTER;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         reply_len = 1 + moved;

         break;


      case PD_CMD_PWM_READ_FREQ:
      case PD_CMD_PWM_READ_DUTY:
      case PD_CMD_PWM_READ_EDGE:

         // ">B" gpio

         gpio = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "PWM_READ: g=%d cmd=%d", gpio, cmd);
            debug(buf);
         }

         if      (cmd == PD_CMD_PWM_READ_FREQ) i = PD_FUNC_READ_FREQ;
         else if (cmd == PD_CMD_PWM_READ_DUTY) i = PD_FUNC_READ_DUTY;
         else                                  i = PD_FUNC_READ_EDGE;

         initing = false;

         if (pd_config[gpio].func == PD_FUNC_FREE)
         {
            initing = true;

            pd_set_mode(gpio, i, -1, -1);

            gpio_set_function(gpio, GPIO_FUNC_PWM);
         }

         if ((pd_config[gpio].func == PD_FUNC_READ_FREQ) ||
             (pd_config[gpio].func == PD_FUNC_READ_DUTY) ||
             (pd_config[gpio].func == PD_FUNC_READ_EDGE))
         {

            if (pd_config[gpio].func != i)
            {
               pd_config[gpio].func = i;
               initing = true;
            }

            slice = pwm_gpio_to_slice_num(gpio);

            if (initing)
            {
               pwm_set_enabled(slice, false);
               pwm_set_irq_enabled(slice, false);
               pwm_clear_irq(slice);

               pwmCfg = pwm_get_default_config();

               if ((cmd == PD_CMD_PWM_READ_FREQ) ||
                   (cmd == PD_CMD_PWM_READ_EDGE))
                  pwm_config_set_clkdiv_mode(&pwmCfg, PWM_DIV_B_RISING);
               else
                  pwm_config_set_clkdiv_mode(&pwmCfg, PWM_DIV_B_HIGH);

               pwm_init(slice, &pwmCfg, false);

               irq_set_exclusive_handler(PWM_IRQ_WRAP, my_pwm_wrap_handler);
               irq_set_enabled(PWM_IRQ_WRAP, true);

               microsL = 1;
               microsH = 0;
               countL = 0;
               countH = 0;
            }
            else
            {
               if (cmd != PD_CMD_PWM_READ_EDGE)
               {
                  pwm_set_enabled(slice, false);
                  pwm_set_irq_enabled(slice, false);
                  pwm_clear_irq(slice);
               }

               tdiff = time_us_64() - g.pwmReadTime[slice];
               microsL = tdiff & 0xffffffff;
               microsH = (tdiff>>32)&0xffffffff;
               count16 = pwm_get_counter(slice);
               countL = count16 | ((g.pwmReadWrap[slice]&0xffff)<<16);
               countH = (g.pwmReadWrap[slice]>>16)&0xffff;
            }

            pack32(RES+1, countH);
            pack32(RES+5, countL);
            pack32(RES+9, microsH);
            pack32(RES+13, microsL);
            reply_len = 17;

            if ((cmd != PD_CMD_PWM_READ_EDGE) || initing)
            {
               pwm_clear_irq(slice);
               pwm_set_irq_enabled(slice, true);

               g.pwmReadWrap[slice] = 0;
               pwm_set_counter(slice, 0);
               g.pwmReadTime[slice] = time_us_64();

               pwm_set_enabled(slice, true);
            }
         }
         else status = STATUS_GPIO_IN_USE;

         break;

      case PD_CMD_PWM:
      case PD_CMD_SERVO:

         // ">BBBHH" gpio clkdiv steps high

         gpio = cBuf[CMD_CMD+1];
         clkdiv = cBuf[CMD_CMD+2];
         steps = unpack16(cBuf+CMD_CMD+3);
         high_steps = unpack16(cBuf+CMD_CMD+5);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "PWM/SERVO: g=%d cmd=%d clkdiv=%d steps=%d high=%d",
               gpio, cmd, clkdiv, steps, high_steps);
            debug(buf);
         }

         if (cmd == PD_CMD_PWM) i = PD_FUNC_PWM;
         else                   i = PD_FUNC_SERVO;

         if (pd_config[gpio].func == PD_FUNC_FREE)
         {
            pd_set_mode(gpio, i, -1, -1);

            gpio_set_function(gpio, GPIO_FUNC_PWM);
         }

         if ((pd_config[gpio].func == PD_FUNC_PWM) ||
             (pd_config[gpio].func == PD_FUNC_SERVO))
         {
            pd_config[gpio].func = i;

            slice = pwm_gpio_to_slice_num(gpio);

            pwm_set_clkdiv_mode(slice, PWM_DIV_FREE_RUNNING);

            pwm_set_clkdiv_int_frac(slice, clkdiv, 0);

            pwm_set_wrap(slice, steps);

            pwm_set_gpio_level(gpio, high_steps);

            pwm_set_enabled(slice, true);
         }
         else status = STATUS_GPIO_IN_USE;

         break;

      case PD_CMD_PWM_CLOSE:

         // ">B" gpio

         gpio = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "PWM_CLOSE: gpio=%d", gpio);
            debug(buf);
         }

         if ((pd_config[gpio].func == PD_FUNC_PWM)       ||
             (pd_config[gpio].func == PD_FUNC_SERVO)     ||
             (pd_config[gpio].func == PD_FUNC_READ_FREQ) ||
             (pd_config[gpio].func == PD_FUNC_READ_DUTY) ||
             (pd_config[gpio].func == PD_FUNC_READ_EDGE))
         {
            pd_set_mode(gpio, PD_FUNC_FREE, -1, -1);
         }

         break;


      case PD_CMD_SPI_OPEN:

         // ">BBBBBBBI" channel tx rx sck mode bits slave_cs baud

         channel = cBuf[CMD_CMD+1];
         tx = cBuf[CMD_CMD+2];
         rx = cBuf[CMD_CMD+3];
         sck = cBuf[CMD_CMD+4];
         mode = cBuf[CMD_CMD+5];
         bits = cBuf[CMD_CMD+6];
         cs = cBuf[CMD_CMD+7];
         baud = unpack32(cBuf+CMD_CMD+8);

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf,
               "SPI_OPEN: ch=%d tx=%d rx=%d sck=%d mode=%d"
                  " bits=%d slave_cs=%d baud=%d",
               channel, tx, rx, sck, mode, bits, cs, baud);
            debug(buf);
         }

         if (!status)
            status = is_func_okay(tx, GPIO_FUNC_SPI, S_TX, channel, "SPI TX");

         if (!status)
            status = is_func_okay(rx, GPIO_FUNC_SPI, S_RX, channel, "SPI RX");

         if (!status)
            status = is_func_okay(sck, GPIO_FUNC_SPI, S_SCK, channel, "SPI SCK");

         if (!status && cs)
            status = is_func_okay(cs, GPIO_FUNC_SPI, S_CS, channel, "SPI CS");

         if (!status)
         {
            if (g.spiInited[channel]) spi_deinit(SPI[channel]);

            g.spiInited[channel] = 1;

            pd_set_mode(tx, PD_FUNC_SPI, S_TX, channel);
            gpio_set_function(tx, GPIO_FUNC_SPI);

            pd_set_mode(rx, PD_FUNC_SPI, S_RX, channel);
            gpio_set_function(rx, GPIO_FUNC_SPI);

            pd_set_mode(sck, PD_FUNC_SPI, S_SCK, channel);
            gpio_set_function(sck, GPIO_FUNC_SPI);

            if (cs) // slave
            {
               pd_set_mode(cs, PD_FUNC_SPI, S_CS, channel);
               gpio_set_function(cs, GPIO_FUNC_SPI);
            }

            spi_init(SPI[channel], 1000000);

               //            bits   cpol    cpha    order
            spi_set_format(
               SPI[channel], bits,  mode/2, mode&1, SPI_MSB_FIRST);

            set_value = spi_set_baudrate(SPI[channel], baud);

            if (cs == 0) // master
            {
               spi_set_slave(SPI[channel], false);
               g.spiMaster[channel] = 1;

               irq_set_enabled(SPI0_IRQ+channel, false);
            }
            else         // slave
            {
               /* reset RX/TX slave buffers */
               bufReset(EVT_SPI_0_RX+channel);
               bufReset(EVT_SPI_0_TX+channel);

               spi_set_slave(SPI[channel], true);
               g.spiMaster[channel] = 0;

               /* enable auto rx */
               spi_get_hw(SPI[channel])->imsc = SPI_SSPIMSC_RXIM_BITS;

               irq_set_exclusive_handler(SPI0_IRQ+channel, my_spi_handler);
               irq_set_enabled(SPI0_IRQ+channel, true);
            }
         }

         pack32(RES+1, set_value);
         reply_len = 5;

         break;

      case PD_CMD_SPI_CLOSE:

         // ">B" channel

         channel = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "SPI_CLOSE: channel=%d", channel);
            debug(buf);
         }

         if (channel < 2)
         {
            if (g.spiInited[channel])
            {
               spi_deinit(SPI[channel]);

               pd_free_func(PD_FUNC_SPI, channel);
            }

            g.spiInited[channel] = 0;
         }

         break;

      case PD_CMD_SPI_READ:

         // ">BBHB" channel cs count dummy

         channel = cBuf[CMD_CMD+1];
         cs = cBuf[CMD_CMD+2];
         count = unpack16(cBuf+CMD_CMD+3);
         dummy = cBuf[CMD_CMD+5];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "SPI_READ: ch=%d cs=%d count=%d dummy=%d",
               channel, cs, count, dummy);
            debug(buf);
         }

         if (channel < 2)
         {
            if (cs < PD_NUM_GPIO)
            {
               if (g.spiInited[channel])
               {
                  if (g.spiMaster[channel])
                  {
                     /* Allow any GPIO for CS when master */

                     if (pd_config[cs].func != PD_FUNC_SPI)
                     {
                        pd_set_mode(cs, GPIO_FUNC_SPI, S_CS, channel);
                        gpio_set_function(cs, GPIO_FUNC_SIO);
                        gpio_set_dir(cs, true); // set as output
                     }
                  }
                  else status = STATUS_INVALID_WHEN_SLAVE;
               }
               else status = STATUS_CHANNEL_CLOSED;
            }
            else status = STATUS_BAD_GPIO;

            if (status == STATUS_OKAY)
            {
               spiAssertCS(cs, true);

               spi_read_blocking(SPI[channel], dummy, RES+1, count);

               spiAssertCS(cs, false);
            }
         }
         else status = STATUS_BAD_CHANNEL;

         if (status == STATUS_OKAY) reply_len = 1 + count;

         break;

      case PD_CMD_SPI_WRITE:
      case PD_CMD_SPI_XFER:

         // ">BBH..." channel cs count data

         channel = cBuf[CMD_CMD+1];
         cs = cBuf[CMD_CMD+2];
         count = unpack16(cBuf+CMD_CMD+3);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            if (cmd == PD_CMD_SPI_XFER)
               sprintf(buf, "SPI_XFER: ch=%d cs=%d length=%d",
                  channel, cs, count);
            else
               sprintf(buf, "SPI_WRITE: ch=%d cs=%d length=%d",
                  channel, cs, count);

            debug(buf);
         }

         if (channel < 2)
         {
            if (cs < PD_NUM_GPIO)
            {
               if (g.spiInited[channel])
               {
                  if (g.spiMaster[channel])
                  {
                     /* Allow any GPIO for CS when master */

                     if (pd_config[cs].func != PD_FUNC_SPI)
                     {
                        pd_set_mode(cs, PD_FUNC_SPI, S_CS, channel);
                        gpio_set_function(cs, GPIO_FUNC_SIO);
                        gpio_set_dir(cs, true); // set as output
                     }
                  }
                  else status = STATUS_INVALID_WHEN_SLAVE;
               }
               else status = STATUS_CHANNEL_CLOSED;
            }
            else status = STATUS_BAD_GPIO;

            if (status == STATUS_OKAY)
            {
               spiAssertCS(cs, true);

               spi_write_read_blocking(
                  SPI[channel], cBuf+CMD_CMD+5, RES+1, count);

               spiAssertCS(cs, false);
            }
         }
         else status = STATUS_BAD_CHANNEL;

         if ((status == STATUS_OKAY) && (cmd == PD_CMD_SPI_XFER))
            reply_len = 1 + count;

         break;

      case PD_CMD_SPI_PUSH:

         // ">BB..." channel count data

         channel = cBuf[CMD_CMD+1];
         count = cBuf[CMD_CMD+2];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "SPI_PUSH: ch=%d count=%d", channel, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.spiInited[channel])
            {
               if (!g.spiMaster[channel])
               {
                  moved =
                     bufPush(EVT_SPI_0_TX+channel, count, cBuf+CMD_CMD+3);

                  // now okay to fill tx fifo automatically
                  spi_get_hw(SPI[channel])->imsc = 
                     SPI_SSPIMSC_TXIM_BITS | SPI_SSPIMSC_RXIM_BITS;
               }
               else status = STATUS_INVALID_WHEN_MASTER;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         RES[1] = moved;
         reply_len = 2;

         break;

      case PD_CMD_SPI_POP:

         // ">BB" channel count

         channel = cBuf[CMD_CMD+1];
         count = cBuf[CMD_CMD+2];

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "SPI_POP: ch=%d count=%d", channel, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.spiInited[channel])
            {
               if (!g.spiMaster[channel])
               {
                  moved = bufPop(EVT_SPI_0_RX+channel, count, RES+1);
               }
               else status = STATUS_INVALID_WHEN_MASTER;
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         reply_len = 1 + moved;

         break;

      case PD_CMD_UART_OPEN:

         // ">IBBBBBBBB" baud channel tx rx cts rts bits stops parity

         baud = unpack32(cBuf+CMD_CMD+1);
         channel = cBuf[CMD_CMD+5];
         tx = cBuf[CMD_CMD+6];
         rx = cBuf[CMD_CMD+7];
         cts = cBuf[CMD_CMD+8];
         rts = cBuf[CMD_CMD+9];
         bits = cBuf[CMD_CMD+10];
         stops = cBuf[CMD_CMD+11];
         parity = cBuf[CMD_CMD+12];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf,
               "UART: ch=%d tx=%d rx=%d baud=%d cts=%d, rts=%d, "
               "db=%d sb=%d par=%d",
               channel, tx, rx, baud, cts, rts, bits, stops, parity);
            debug(buf);
         }

         if (!status && (tx == 255) && (rx == 255)) status = STATUS_BAD_PARAM;

         if (!status && (tx != 255))
            status = is_func_okay(tx, GPIO_FUNC_UART, U_TX, channel, "UART TX");

         if (!status && (rx != 255))
            status = is_func_okay(rx, GPIO_FUNC_UART, U_RX, channel, "UART RX");

         if (!status && (cts != 255))
            status = is_func_okay(
               cts, GPIO_FUNC_UART, U_CTS, channel, "UART CTS");

         if (!status && (rts != 255))
            status = is_func_okay(
               rts, GPIO_FUNC_UART, U_RTS, channel, "UART RTS");

         if (!status)
         {
            if (g.uartInited[channel]) uart_deinit(UART[channel]);

            g.uartInited[channel] = 1;

            uart_init(UART[channel], 2400);

            if (tx != 255)
            {
               pd_set_mode(tx, PD_FUNC_UART, U_TX, channel);
               gpio_set_function(tx, GPIO_FUNC_UART);
            }

            if (rx != 255)
            {
               pd_set_mode(rx, PD_FUNC_UART, U_RX, channel);
               gpio_set_function(rx, GPIO_FUNC_UART);
            }

            if (cts != 255)
            {
               up = true;
               pd_set_mode(cts, PD_FUNC_UART, U_CTS, channel);
               gpio_set_function(cts, GPIO_FUNC_UART);
            } else up = false;

            if (rts != 255)
            {
               down = true;
               pd_set_mode(rts, PD_FUNC_UART, U_RTS, channel);
               gpio_set_function(rts, GPIO_FUNC_UART);
            } else down = false;

            /* reset RX buffer */
            bufReset(EVT_UART_0_RX+channel);

            set_value = uart_set_baudrate(UART[channel], baud);
            uart_set_translate_crlf(UART[channel], false);
            uart_set_fifo_enabled (UART[channel], false);
            uart_set_format(UART[channel], bits, stops, parity);
            uart_set_hw_flow(UART[channel], up, down);

            irq_set_exclusive_handler(UARTIRQ[channel], my_uart_handler);
            irq_set_enabled(UARTIRQ[channel], true);
            uart_set_irq_enables(UART[channel], true, false);
         }

         pack32(RES+1, set_value);
         reply_len = 5;

         break;

      case PD_CMD_UART_CLOSE:

         // ">B" channel

         channel = cBuf[CMD_CMD+1];

         if (cfg_debug_mask & DBG_LEVEL_COMMAND_OPEN_CLOSE)
         {
            sprintf(buf, "UART_CLOSE: channel=%d", channel);
            debug(buf);
         }

         if (channel < 2)
         {
            if (g.uartInited[channel])
            {
               pd_free_func(PD_FUNC_UART, channel);

               uart_deinit(UART[channel]);
               irq_set_enabled(UARTIRQ[channel], false);
            }

            g.uartInited[channel] = 0;
         }

         break;

      case PD_CMD_UART_READ:

         // ">BH" channel count

         channel = cBuf[CMD_CMD+1];
         count = unpack16(cBuf+CMD_CMD+2);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "UART READ: ch=%d count=%d", channel, count);
            debug(buf);
         }

         moved = 0;

         if (channel < 2)
         {
            if (g.uartInited[channel])
            {
               moved = bufPop(EVT_UART_0_RX+channel, count, RES+3);
            }
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         pack16(RES+1, moved);
         reply_len = 3 + moved;

         break;

      case PD_CMD_UART_WRITE:

         // ">BH..." channel count data

         channel = cBuf[CMD_CMD+1];
         count = unpack16(cBuf+CMD_CMD+2);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "UART write: tx=%d length=%d", channel, count);
            debug(buf);
         }

         if (channel < 2)
         {
            if (g.uartInited[channel])
               uart_write_blocking(UART[channel], cBuf+CMD_CMD+4, count);
            else status = STATUS_CHANNEL_CLOSED;
         }
         else status = STATUS_BAD_CHANNEL;

         break;

      case PD_CMD_TICK:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "TICK");
            debug(buf);
         }

         pack32(RES+1, g.GPIO_tick);
         reply_len = 5;

         break;

      case PD_CMD_UID:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "UID");
            debug(buf);
         }

         pico_get_unique_board_id((pico_unique_board_id_t *)(RES+1));

         reply_len = 9;

         break;

      case PD_CMD_SLEEP_US:

         // ">I" micros

         micros = unpack32(cBuf+CMD_CMD+1);

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "SLEEP_US: us=%d", micros);
            debug(buf);
         }

         sleep_us(micros);

         break;

      case PD_CMD_RESET_PICO:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "RESET_PICO");
            debug(buf);
         }

         pd_init();

         break;

      case PD_CMD_SET_CONFIG_VAL:

         // ">II" item value

         item = unpack32(cBuf+CMD_CMD+1);
         value = unpack32(cBuf+CMD_CMD+5);

         sprintf(buf, "SET_CONFIG_VAL: item=%d value=%d (0x%x)",
            item, value, value);
         debug(buf);

         switch(item)
         {
            case 0:
               cfg_debug_mask = value;
               break;

            default:
               status = STATUS_BAD_CONFIG_ITEM;
         }

         break;

      case PD_CMD_GET_CONFIG_VAL:

         // ">I" item

         item = unpack32(cBuf+CMD_CMD+1);

         sprintf(buf, "GET_CONFIG_VAL: item=%d", item);
         debug(buf);

         value = 0;

         switch(item)
         {
            case 0:
               value = cfg_debug_mask;
               break;

            default:
               status = STATUS_BAD_CONFIG_ITEM;
         }

         pack32(RES+1, value);
         reply_len = 5;

         break;

      case PD_CMD_PD_VERSION:

         // no parameters

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "PD_VERSION");
            debug(buf);
         }

         pack32(RES+1, PD_VERSION);
         reply_len = 5;

         break;

      default:

         status = STATUS_UNKNOWN_COMMAND;
   }

   if (reply)
   {
      RES[0] = status;
      cmdRespond((cBuf[CMD_FLG]<<8)|cBuf[CMD_CMD], reply_len, RES);
   }
}

int msgGet()
{
   /*
   <------------ Length bytes ------------>
   +---+-------+-------+----------+-------+
   |Hdr|Length | CRC 1 |Request(s)| CRC 2 |
   |xFF|msb|lsb|msb|lsb|          |msb|lsb|
   +---+---+---+---+---+----------+---+---+

   CRC1: Hdr+Length
   CRC2: Hdr+Length+CRC1+Request(s)
   */

   static int msgPos = 0;
   int ch;
   int i;
   uint16_t crc;

   while ((ch = _GETCHAR(0)) != PICO_ERROR_TIMEOUT)
   {
      if (msgPos < MSG_HEADER_LEN)
      {
         cmdBuf[msgPos++] = ch;

         if (msgPos == MSG_HEADER_LEN)
         {
            msgLen = unpack16(cmdBuf+1);
            msgCrc1 = unpack16(cmdBuf+3);
            crc = crc16(cmdBuf, MSG_HEADER_LEN-2, 0);

            if  ((cmdBuf[0] != 0xff)    ||
                 (msgLen > MSG_MAX_LEN) ||
                 (crc != msgCrc1))
            {
               /* lost sync, try to re-sync */
               for (i=0; i<MSG_HEADER_LEN-1; i++) cmdBuf[i] = cmdBuf[i+1];
               msgPos = MSG_HEADER_LEN - 1;
            }
         }
      }
      else if (msgPos < msgLen)
      {
         cmdBuf[msgPos++] = ch;
         if (msgPos == msgLen)
         {
            msgCrc2 = unpack16(cmdBuf+msgLen-2);
            crc = crc16(cmdBuf, msgLen-2, 0);

            if (crc == msgCrc2)
            {
               msgPos = 0;
               return 1;
            }
            else
            {
               /* corrupt message, discard */
               msgPos = 0;
               return 0;
            }
         }
      }
   }

   return 0;
}

uint32_t debounce(uint32_t levels, uint32_t tick)
{
   pd_level_info_p p;
   int i;
   int32_t micro_diff;
   uint32_t bitLevel;
   uint8_t buf[256];

   /*
   Only report stable edges.  A stable edge is defined as one
   which has not changed for debounce microseconds.

   Both edges are monitored.  An edge is reported if and only
   no other edge is detected for at least debounce
   microseconds after it occurred and the edge is different
   to the previously reported edge.
   */

   for (i=0; i<PD_NUM_GPIO; i++)
   {
      p = &pd_level_info[i];

      bitLevel = (levels>>i) & 1;

      if (p->debounce_micros)
      {
         if (bitLevel != p->last_evt_lv)
         {
            p->debounced = 0;
            p->last_evt_ts = tick;
            p->last_evt_lv = bitLevel;
         }
         else
         {
            if (!p->debounced)
            {
               micro_diff = tick - p->last_evt_ts;

               if (cfg_debug_mask & DBG_LEVEL_DEBOUNCE) 
               {
                  sprintf(buf, "GPIO_debounce=%x g=%d md=%d deb=%d",
                     g.GPIO_debounce, i, micro_diff,
                     p->debounce_micros);
                  debug(buf);
               }

               if (micro_diff > p->debounce_micros)
               {
                  /* GPIO stable for debounce period */

                  p->debounced = 1;

                  if (bitLevel != p->last_rpt_lv)
                  {
                     p->last_rpt_lv = bitLevel;
                     p->last_rpt_ts = tick;
                     p->watchdogd = 0;
                  }
               }
            }
         }

         if (p->last_rpt_lv) levels |= (1ul<<i);
         else                levels &= ~(1ul<<i);
      }
      else
      {
         if (bitLevel != p->last_rpt_lv)
         {
            p->watchdogd = 0;
            p->last_rpt_ts = tick;
            p->last_rpt_lv = bitLevel;
         }

         if (bitLevel != p->last_evt_lv)
         {
            p->debounced = 0;
            p->last_evt_ts = tick;
            p->last_evt_lv = bitLevel;
         }
      }
   }
   return levels;
}

uint32_t watchdog(uint32_t levels, uint32_t tick)
{
   pd_level_info_p p;
   int i;
   int32_t micro_diff;
   uint32_t wdogd;
   uint8_t buf[256];

   wdogd = WATCHDOG_BIT;

   /*
   Ensure that a watchdog report is sent for a GPIO in the
   absence of edge reports.

   The following rule is applied.

   If no edge report has been issued for the GPIO in
   the last watchdog microseconds then one watchdog report
   is generated.

   Note that only one watchdog report is sent until the
   watchdog is reset by the sending of an edge report.
   */

   for (i=0; i<PD_NUM_GPIO; i++)
   {
      p = &pd_level_info[i];

      if (p->watchdog_micros)
      {
         if (!p->watchdogd)
         {
            micro_diff = tick - p->last_rpt_ts;

            if (micro_diff > 0)
            {
               if (cfg_debug_mask & DBG_LEVEL_WATCHDOG)
               {
                  sprintf(buf, "watcdog_gpio=%x g=%d md=%d wdog=%d",
                     g.GPIO_watchdog, i, micro_diff, p->watchdog_micros);
                  debug(buf);
               }

               if (micro_diff > p->watchdog_micros)
               {
                  wdogd |= (1ul<<i);

                  p->watchdogd = 1;
                  p->last_rpt_ts = tick;
               }
            }
         }
      }
   }

   return wdogd;
}

void emitLevels(int reports)
{
   static uint32_t reportedLevels = -1;
   int count;
   uint32_t levels;
   uint32_t tick;
   int nextPos;
   int i;
   uint32_t wdogd;
   pd_report_t gpiorpt;
   uint8_t buf[256];

   count = 0;

   do
   {
      tick = time_us_32();
      levels = gpio_get_all();

      if (rawReadPos != rawWritePos)
      {
         tick = raw_report[rawReadPos].tick;
         levels = raw_report[rawReadPos].levels;
         nextPos = (rawReadPos + 1) % MAX_REPORTS;
         rawReadPos = nextPos;
      }

      /* anything being debounced? */

      levels = debounce(levels, tick);

      /* have any watchdogs expired? */

      wdogd = 0;

      if (g.GPIO_watchdog) wdogd = watchdog(levels, tick);

      /* only send callbacks for alert GPIO */

      if ((reportedLevels & g.GPIO_alert) != (levels & g.GPIO_alert))
      {
         gpiorpt.tick = swap32(tick);
         gpiorpt.levels = swap32(levels);

         emit_report[count++] = gpiorpt;

         if (cfg_debug_mask & DBG_LEVEL_EVENT)
         {
            sprintf(buf, "levels report: %x->%x", reportedLevels, levels);
            debug(buf);
         }

         reportedLevels = levels;
      }

      /* only send watchdogs for alert GPIO */

      if (wdogd & g.GPIO_alert)
      {
         gpiorpt.levels = swap32((wdogd & g.GPIO_alert) | WATCHDOG_BIT);
         gpiorpt.tick = swap32(tick);

         emit_report[count++] = gpiorpt;

         if (cfg_debug_mask & DBG_LEVEL_WATCHDOG)
         {
            sprintf(buf, "wdog report: GPIO=%x ts=%d",
               swap32(gpiorpt.levels), swap32(gpiorpt.tick));
            debug(buf);
         }
      }
   }
   while ((rawReadPos != rawWritePos) && (count < reports));

   if (count)
   {
      cmdRespond(MSG_GPIO_LEVELS,
         sizeof(pd_report_t)*count, (void*)emit_report);
   }

   g.GPIO_levels = levels;
   g.GPIO_tick = tick;
}

void emitEvents()
{
   int i, used, moved, count;
   uint32_t event_flag;
   uint8_t buf[256];

   event_flag = g.event_flag & g.event_alert; // anything interesting?

   if (event_flag)
   {
      for (i=0; i<EVT_BUFS; i++)
      {
         if (event_flag & (1<<i))
         {
            g.event_flag ^= (1<<i); // clear reported flag

            used = bufUsed(i); // bytes in buf

            switch(pd_event_config[i].type)
            {
               case EVENT_ACTIVITY:
                  buf[0] = i;
                  pack16(buf+1, used);
                  cmdRespond(MSG_ASYNC, 3, buf);
                  break;

               case EVENT_COUNT:
                  if (( (i <= EVT_MAX_RX) &&
                        (used >= pd_event_config[i].count) ) ||
                      ( (i > EVT_MAX_RX) &&
                        (used <= pd_event_config[i].count) ) )
                  {
                     buf[0] = i;
                     pack16(buf+1, used);
                     cmdRespond(MSG_ASYNC, 3, buf);
                  }
                  break;

               case EVENT_RETURN_COUNT:
               case EVENT_RETURN_COUNT_PLUS:
                  if ((i <= EVT_MAX_RX) &&
                      (used >= pd_event_config[i].count))
                  {
                     if (pd_event_config[i].type == EVENT_RETURN_COUNT)
                        count = pd_event_config[i].count;
                     else
                        count = sizeof(buf) - 3;
                     moved = bufPop(i, count, buf+3);
                     buf[0]=i;
                     used = bufUsed(i); // bytes left in buf
                     pack16(buf+1, used);
                     cmdRespond(MSG_ASYNC, 3+moved, buf);
                  }
                  break;
            }
         }
      }
   }
}

void pd_init()
{
   int i;

   /* switch off GPIO IRQ on core1 */

   multicore_fifo_push_blocking(PD_MASK_USER);
   multicore_fifo_push_blocking(0);

   for (i=0; i<2; i++)
   {
      if (g.i2cInited[i]) i2c_deinit(I2C[i]);
      if (g.spiInited[i]) spi_deinit(SPI[i]);
      if (g.uartInited[i]) uart_deinit(UART[i]);
   }

   memset(&g, 0, sizeof(g));

   memset(pd_level_info, 0, sizeof(pd_level_info));

   memset(pd_config, 0, sizeof(pd_config));

   for (i=0; i<EVT_BUFS; i++) bufReset(i);

   rawWritePos = 0;
   rawReadPos = 0;

   for (i=0; i<PD_NUM_GPIO; i++)
   {
      switch(i)
      {
         case 23:
         case 24:
         case 29:
            pd_set_mode(i, PD_FUNC_RESERVED, -1, -1);
            break;

         default:
            pd_set_mode(i, PD_FUNC_FREE, -1, -1);
      }
   }

   cfg_debug_mask = CFG_DEBUG_MASK;
}

void main_core1()
{
   uint32_t ALERT_mask, GPIO_mask;
   int i;

   for (i=0; i<PD_NUM_GPIO; i++)
   {
      switch(i)
      {
         case 23:
         case 24:
         case 29:
            break;

         default:
            gpio_set_irq_enabled(i, 0xff, false);
      }
   }

   irq_set_exclusive_handler(IO_IRQ_BANK0, my_core1_gpio_handler);
   irq_set_enabled(IO_IRQ_BANK0, true);

   /* wait for changes to alert GPIO and apply */

   while (1)
   {
      GPIO_mask = multicore_fifo_pop_blocking();
      ALERT_mask = multicore_fifo_pop_blocking();

      for (i=0; i<PD_NUM_GPIO; i++)
      {
         if (GPIO_mask & (1<<i))
         {
            if (ALERT_mask & (1<<i))
            {
               gpio_set_irq_enabled(
                  i, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
            }
            else
            {
               gpio_set_irq_enabled(i, 0xff, false);
            }
         }
      }
   }
}

int main()
{
   uint16_t cmdPos;
   uint16_t cmdLen;
   uint32_t f1, f2, f3, f4, f5;
   uint8_t buf[256];

   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);

   stdio_init_all();

   if (PD_LINK == PD_LINK_UART)
   {
      uart_init(UART[0], 230400);
      gpio_set_function(0, GPIO_FUNC_UART);
      gpio_set_function(1, GPIO_FUNC_UART);

      uart_set_baudrate(UART[0], 230400);

      uart_set_hw_flow(UART[0], false, false);

      uart_set_format(UART[0], 8, 1, UART_PARITY_NONE);

      uart_set_fifo_enabled(UART[0], true);

      irq_set_exclusive_handler(UART0_IRQ, my_uart_handler);
      irq_set_enabled(UART0_IRQ, true);
      uart_set_irq_enables(UART[0], true, false);
      _PUTCHAR = putuartchar;
      _GETCHAR = getuartchar;
   }
   else
   {
      stdio_set_translate_crlf(&stdio_usb, false);
      _PUTCHAR = putchar;
      _GETCHAR = getchar_timeout_us;
   }


   pd_init();

   multicore_launch_core1(main_core1);

   while (1)
   {
      //toggle_led();

      while (msgGet())
      {
         // we have a legal message

         if (cfg_debug_mask & DBG_LEVEL_1)
         {
            sprintf(buf, "len=%d crc1=%4x crc2=%04x", msgLen, msgCrc1, msgCrc2);
            debug(buf);
         }

         // may be multiple requests in one message

         cmdPos = MSG_HEADER_LEN;

         while (cmdPos < (msgLen-2))
         {
            cmdLen = unpack16(cmdBuf+cmdPos);
            cmdExec(cmdBuf+cmdPos);
            cmdPos += cmdLen;
         }
      }

      emitLevels(MAX_EMITS-1);

      emitEvents();
   }
}

