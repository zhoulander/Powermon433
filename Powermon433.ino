/*
  Powermon433
 
 Monitoring 433MHz power monitor products (both the same internally)
 -- Black and Decker EM100B 
 -- BlueLine PowerCost Monitor
 
 Original OOK interrupt and decoder based on jeelib "ookRelay2" project https://github.com/jcw/jeelib
 Protocol decoding by "redgreen" https://github.com/merbanan/rtl_433
 via Stewart Russell's blog 
 http://scruss.com/blog/2013/12/03/blueline-black-decker-power-monitor-rf-packets/
 Additional work and porting to ATmega by Bryan Mayland
 General messing about and reformatting by Stewart C. Russell, scruss.com
 
 */
#include <util/atomic.h>
#include "rf69_ook.h"
#include "temp_lerp.h"

#define DPIN_RF69_RESET     7
#define DPIN_OOK_RX         8
#define DPIN_LED            9

// The default ID of the transmitter to decode/encode from/to
// #define DEFAULT_TX_ID 0xfdcc
#define DEFAULT_TX_ID 0xfff8

// If defined will dump all the encoded RX data, and partial decode fails
#define DUMP_RX

// for temperature lookup table
#define TEMP_TABSIZE 22

static coord_t temp_tab[TEMP_TABSIZE] = {
  {
    0, -49  }
  ,
  {
    5, -45  }
  ,
  {
    10, -42  }
  ,
  {
    20, -22  }
  ,
  {
    30, -7  }
  ,
  {
    40, 5  }
  ,
  {
    50, 16  }
  ,
  {
    70, 34  }
  ,
  {
    80, 42  }
  ,
  {
    90, 49  }
  ,
  {
    100, 57  }
  ,
  {
    130, 78  }
  ,
  {
    150, 94  }
  ,
  {
    152, 96  }
  ,
  {
    154, 97  }
  ,
  {
    158, 101  }
  ,
  {
    160, 102  }
  ,
  {
    176, 118  }
  ,
  {
    180, 121  }
  ,
  {
    184, 126  }
  ,
  {
    185, 127  }
  ,
  {
    255, 127  }
};


static uint16_t g_TxId;
static uint8_t g_TxCnt;

// Simulated TX values
static uint16_t g_TxTemperature;
static uint8_t g_TxFlags;
static uint8_t g_TxLowBat;
static uint16_t g_TxWatts;
static uint16_t g_TxTotal;

static char g_SerialBuff[40];

static struct tagWireVal
{
  uint8_t hdr;
  union {
    uint8_t raw[2];
    uint16_t val16;
  } 
  data;
  uint8_t crc;
} 
wireval;

static struct tagDecoder
{
  uint8_t state;
  uint8_t pos;
  uint8_t bit;
  uint8_t data[4];
} 
decoder;

#if defined(DPIN_OOK_RX)
static int8_t g_RxTemperature;
static uint8_t g_RxFlags;
static uint16_t g_RxWatts;
static uint16_t g_RxWattHours;

static bool g_RxDirty;
static uint32_t g_RxLast;
static uint8_t g_RxRssi;

#if DPIN_OOK_RX >= 14
#define VECT PCINT1_vect
#elif DPIN_OOK_RX >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif

volatile uint16_t pulse_433;

static void pinChange(void)
{
  static uint16_t last_433;

  uint16_t now = micros();
  uint16_t cnt = now - last_433;
  if (cnt > 10)
    pulse_433 = cnt;

  last_433 = now;
}

ISR(VECT) {
  pinChange();
}

static void setupPinChangeInterrupt ()
{
  pinMode(DPIN_OOK_RX, INPUT);
#if DPIN_OOK_RX >= 14
  bitSet(PCMSK1, DPIN_OOK_RX - 14);
  bitSet(PCICR, PCIE1);
#elif DPIN_OOK_RX >= 8
  bitSet(PCMSK0, DPIN_OOK_RX - 8);
  bitSet(PCICR, PCIE0);
#else
  PCMSK2 = bit(DPIN_OOK_RX);
  bitSet(PCICR, PCIE2);
#endif
}
#endif /* DPIN_OOK_RX */

static void printWireval(void)
{
  Serial.print("Tx ");
  for (uint8_t i=0; i<sizeof(wireval); ++i)
  {
    uint8_t v = ((uint8_t *)&wireval)[i];
    if (v < 0x10)
      Serial.print('0');
    Serial.print(v, HEX);
  }
  Serial.println();
  Serial.flush();
}

// Short burst in uSec
#define OOK_TX_SHORT 500
// Long burst in uSec
#define OOK_TX_LONG  1000
// Inter-packet delay in msec
#define OOK_TX_DELAY 65
// Inter-ID-packet delay in msec
#define OOK_ID_DELAY 225

#define OOK_PACKET_INSTANT 1
#define OOK_PACKET_TEMP    2
#define OOK_PACKET_TOTAL   3

/* crc8 from chromimum project */
__attribute__((noinline)) uint8_t crc8(uint8_t const *data, uint8_t len)
{
  uint16_t crc = 0;
  for (uint8_t j=0; j<len; ++j)
  {
    crc ^= (data[j] << 8);
    for (uint8_t i=8; i>0; --i)
    {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return crc >> 8;
}

static void setRf69Thresh(uint8_t val)
{
  Serial.print(F("OokFixedThresh="));
  Serial.println(val, DEC);
  rf69ook_writeReg(0x1d, val);
}

static void resetRf69(void)
{
#if defined(DPIN_RF69_RESET)
  const uint8_t pin = 7;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(100);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  delay(5);
  Serial.println(F("RFM reset"));
#endif // DPIN_RF69_RESET
}

static void resetDecoder(void)
{
  decoder.pos = 0;
  decoder.bit = 0;
  decoder.state = 0;
}

static void decoderAddBit(uint8_t bit)
{
  decoder.data[decoder.pos] = (decoder.data[decoder.pos] << 1) | bit;
  if (++decoder.bit > 7)
  {
    decoder.bit = 0;
    if (++decoder.pos >= sizeof(decoder.data))
      resetDecoder();
  }
}

static bool decodeRxPulse(uint16_t width)
{
  // 500,1000,1500 usec pulses with 25% tolerance
  if (width > 375 && width < 1875)
  {
    // The only "extra long" long signals the end of the preamble
    if (width > 1200)
    {
      rf69ook_startRssi();
      resetDecoder();
      return false;
    }

    bool isShort = width < 750;
    if (decoder.state == 0)
    {
      // expecting a short to start a bit
      if (isShort)
      {
        decoder.state = 1;
        return false;
      }
    }
    else if (decoder.state == 1)
    {
      decoder.state = 0;
      if (isShort)
        decoderAddBit(1);
      else
        decoderAddBit(0);

      // If we have all 3 bytes, we're done
      if (decoder.pos > 2)
        return true;
      return false;
    }
  }  // if proper width

#if defined(DUMP_RX)
  // Some debug dump of where the decoder went wrong
  if ((decoder.pos * 8 + decoder.bit) > 2)
  {
    Serial.print(width, DEC);
    Serial.print('@');
    Serial.println(decoder.pos * 8 + decoder.bit, DEC);
  }
#endif

  resetDecoder();
  return false;
}

static void decodePowermon(uint16_t val16)
{
  switch (decoder.data[0] & 3)
  {
  case OOK_PACKET_INSTANT:
    // val16 is the number of milliseconds between blinks
    // Each blink is one watt hour consumed
    g_RxWatts = 3600000UL / val16;
    break;

  case OOK_PACKET_TEMP:
    //float f = decoder.data[1] * 0.823 - 28.63;
    g_RxTemperature = (int8_t)(fudged_f_to_c(temp_lerp(temp_tab, decoder.data[1], TEMP_TABSIZE)));
//    g_RxTemperature = (int8_t)(decoder.data[1] * 210U / 256U) - 28;
    g_RxFlags = decoder.data[0];
    break;

  case OOK_PACKET_TOTAL:
    g_RxWattHours = val16;
    break;
  }
}


static void decodeRxPacket(void)
{
#if defined(DUMP_RX)
  for (uint8_t i=0; i<decoder.pos; ++i)
  {
    if (decoder.data[i] < 16)
      Serial.print('0');
    Serial.print(decoder.data[i], HEX);
  }
  Serial.println();
#endif

  uint16_t val16 = *(uint16_t *)decoder.data;
  if (crc8(decoder.data, 3) == 0)
  {
    g_TxId = decoder.data[1] << 8 | decoder.data[0];
    Serial.print(F("# NEW DEVICE id="));
    Serial.println(val16, HEX);
    return;
  }

  val16 -= g_TxId;
  decoder.data[0] = val16 & 0xff;
  decoder.data[1] = val16 >> 8;
  if (crc8(decoder.data, 3) == 0)
  {
    decodePowermon(val16 & 0xfffc);
    g_RxDirty = true;
    g_RxLast = millis();
    digitalWrite(DPIN_LED, HIGH);
  }
  else
  {
    Serial.println(F("# CRC ERR"));
  }
}

static void txSetup(void)
{
#if defined(DPIN_OOK_TX)
  pinMode(DPIN_OOK_TX, OUTPUT);
  pinMode(DPIN_STARTTX_BUTTON, INPUT);
  digitalWrite(DPIN_STARTTX_BUTTON, HIGH);

  wireval.hdr = 0xfe;
  g_TxTemperature = tempFToCnt(116.22);
  g_TxFlags = 0x7c;
  g_TxWatts = 5000;
  g_TxTotal = 60000;
#endif
}

static void rxSetup(void)
{
#if defined(DPIN_OOK_RX)
  setupPinChangeInterrupt();
#endif
}

static void ookTx(void)
{
#if defined(DPIN_OOK_TX)
  static uint32_t g_LastTx;

  if (digitalReadFast(DPIN_STARTTX_BUTTON) == LOW)
  {
    for (uint8_t i=0; i<4; ++i)
    {
      TxIdOnce(g_TxId);
      delay(OOK_ID_DELAY);
    }

    g_LastTx = millis();
  }

  if (g_LastTx != 0 && millis() - g_LastTx > 30500)
  {
    digitalWrite(DPIN_LED, HIGH);
    for (uint8_t i=0; i<3; ++i)
    {
      if (i == 2 || (g_TxCnt >= 0 && g_TxCnt < 4))
        TxInstantOnce(g_TxId, wattsToCnt(g_TxWatts));
      else if (g_TxCnt == 4)
        TxTempOnce(g_TxId, g_TxTemperature, g_TxLowBat);
      else if (g_TxCnt == 5)
        TxTotalOnce(g_TxId, g_TxTotal);
      delay(OOK_TX_DELAY);
    }

    ++g_TxCnt;
    if (g_TxCnt > 5)
      g_TxCnt = 0;

    digitalWrite(DPIN_LED, LOW);
    g_LastTx = millis();
  }
#endif //defined(DPIN_OOK_TX)
}

static void ookRx(void)
{
#if defined(DPIN_OOK_RX)
  uint16_t v;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    v = pulse_433;
    pulse_433 = 0;
  }
  if (v != 0)
  {
    if (decodeRxPulse(v) == 1)
    {
      g_RxRssi = rf69ook_Rssi();
      decodeRxPacket();
      resetDecoder();
    }
  }

  // If it has been more than 250ms since the last receive, dump the data
  else if (g_RxDirty && (millis() - g_RxLast) > 250U)
  {
#if defined(DUMP_RX)
    Serial.print('['); 
    Serial.print(millis(), DEC); 
    Serial.print(F("] "));
#endif
    Serial.print(F("Energy: ")); 
    Serial.print(g_RxWattHours, DEC);
    Serial.print(F(" Wh, Power: ")); 
    Serial.print(g_RxWatts, DEC);
    Serial.print(F(" W, Temp: ")); 
    Serial.print(g_RxTemperature, DEC);
    Serial.println(F(" C"));

    g_RxDirty = false;
  }
  else if (g_RxLast != 0 && (millis() - g_RxLast) > 32000U)
  { 
    Serial.print('['); 
    Serial.print(millis(), DEC); 
    Serial.println(F("] Missed"));
    g_RxLast = millis();
    digitalWrite(DPIN_LED, LOW);
  }
#endif // DPIN_OOK_RX
}

void setup() {
  Serial.begin(38400);
  Serial.println(F("# Powermon433 built "__DATE__" "__TIME__));
  Serial.print(F("# Listening for Sensor ID: 0x"));
  Serial.println(DEFAULT_TX_ID, HEX);

  pinMode(DPIN_LED, OUTPUT);
  if (rf69ook_init())
    Serial.println(F("# RF69 initialized"));

  rxSetup();

  g_TxId = DEFAULT_TX_ID;
}

void loop()
{
  ookRx();
}

