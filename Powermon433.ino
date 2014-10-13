/*

 Powermon433
 
 Monitoring 433MHz power monitor products (both the same internally)
 -- Black and Decker EM100B 
 -- BlueLine PowerCost Monitor
 
 Protocol decoding by "redgreen" 
 https://github.com/merbanan/rtl_433
 
 via comments on Stewart C. Russell's blog 
 http://scruss.com/blog/2013/12/03/blueline-black-decker-power-monitor-rf-packets/
 
 Additional work (aka "most of it" - SCR) 
 and porting to ATmega by Bryan Mayland
 https://github.com/CapnBry/Powermon433
 
 Original OOK interrupt and decoder based on jeelib 
 "ookRelay2" project https://github.com/jcw/jeelib
 
 General messing about and reformatting 
 by Stewart C. Russell, scruss.com
 https://github.com/scruss/Powermon433
 
 */

#include <util/atomic.h>
#include "rf69_ook.h"
#include "temp_lerp.h"

// the pin connected to the receiver output
#define DPIN_OOK_RX         8

/*
 The default ID of the transmitter to decode from
 - it is likely that yours is different.
 see README.md on how to check yours and set it here.
 */
#define DEFAULT_TX_ID 0xfff8

/*
 TX_ID_LOCK - 
 Uncomment this #define if you want to lock the decoder ID.
 This will prevent nearby stations hijacking your power log
 */
//#define TX_ID_LOCK

/*
 TEMPERATURE_F - 
 Uncomment this #define if you believe in the power of
 D. G. Farenheit's armpit, and want your temperature in °F.
 Otherwise, you get °C, like you should.
 [I'd include an option for K, but it doesn't fit into a byte.]
 */
//#define TEMPERATURE_F

static uint16_t g_TxId;

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

static int8_t g_RxTemperature;
static uint8_t g_RxFlags;
static uint16_t g_RxWatts;
static uint16_t g_RxWattHours;

// Watt-hour counter rolls over every 65.536 kWh, 
// or roughly 2½ days of my use.
// This should be good for > 4 GWh; enough for everyone ...
static unsigned long g_TotalRxWattHours;
// need this too
static uint16_t g_PrevRxWattHours;

// better stats on time between reports
// delta is long as packets are appx ½ range of uint16_t
// so we roll if we miss >2
static unsigned long g_PrintTime_ms;
static unsigned long g_PrevPrintTime_ms;
static unsigned long g_PrintTimeDelta_ms;

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
#if defined(TEMPERATURE_F)
    g_RxTemperature = (int8_t)(temp_lerp(decoder.data[1]));
#else
    g_RxTemperature = (int8_t)(fudged_f_to_c(temp_lerp(decoder.data[1])));
#endif /* TEMPERATURE_F */
    g_RxFlags = decoder.data[0];
    break;

  case OOK_PACKET_TOTAL:
    g_PrevRxWattHours = g_RxWattHours;
    g_RxWattHours = val16;
    // prevent rollover through the power of unsigned arithmetic
    g_TotalRxWattHours += (g_RxWattHours - g_PrevRxWattHours);
    break;
  }
}

static void decodeRxPacket(void)
{

  uint16_t val16 = *(uint16_t *)decoder.data;
#ifndef TX_ID_LOCK
  if (crc8(decoder.data, 3) == 0)
  {
    g_TxId = decoder.data[1] << 8 | decoder.data[0];
    Serial.print(F("# New ID: 0x"));
    Serial.println(val16, HEX);
    return;
  }
#endif /* ifndef TX_ID_LOCK */

  val16 -= g_TxId;
  decoder.data[0] = val16 & 0xff;
  decoder.data[1] = val16 >> 8;
  if (crc8(decoder.data, 3) == 0)
  {
    decodePowermon(val16 & 0xfffc);
    g_RxDirty = true;
    g_RxLast = millis();
  }
  else
  {
    Serial.println(F("# CRC ERR"));
  }
}

static void rxSetup(void)
{
  setupPinChangeInterrupt();
}

static void ookRx(void)
{
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

    /*
     track duration since last report
     
     If > ~31.8 s (318nn ms), we have missed a packet, 
     and the instantaneous Power reading 
     isn't continuous. 
     */
    g_PrevPrintTime_ms = g_PrintTime_ms;
    g_PrintTime_ms = millis();
    g_PrintTimeDelta_ms = g_PrintTime_ms - g_PrevPrintTime_ms;

    Serial.print(F("PrintDelta_ms: ")); 
    Serial.print(g_PrintTimeDelta_ms, DEC);
    // this can roll over, so don't print   
    // Serial.print(F(" Energy_Wh: ")); 
    // Serial.print(g_RxWattHours, DEC);  
    Serial.print(F(" Total_Energy_Wh: ")); 
    Serial.print(g_TotalRxWattHours, DEC);
    Serial.print(F(" Power_W: ")); 
    Serial.print(g_RxWatts, DEC);
#if defined(TEMPERATURE_F)
    Serial.print(F(" Temp_F: "));
#else 
    Serial.print(F(" Temp_C: "));
#endif /* TEMPERATURE_F */ 
    Serial.println(g_RxTemperature, DEC);

    g_RxDirty = false;
  }
  else if (g_RxLast != 0 && (millis() - g_RxLast) > 32000U) { 
    Serial.println(F("# Missed Packet"));
    g_RxLast = millis();
  }
}

void setup() {
  Serial.begin(38400);
  Serial.println(F("# Powermon433 built "__DATE__" "__TIME__));
  Serial.print(F("# Listening for Sensor ID: 0x"));
  Serial.println(DEFAULT_TX_ID, HEX);

  if (rf69ook_init())
    Serial.println(F("# RF69 initialized"));

  rxSetup();

  g_TxId = DEFAULT_TX_ID;
  g_TotalRxWattHours = 0;
  g_PrintTimeDelta_ms = 0;
  g_PrintTime_ms = 0;
}

void loop()
{
  ookRx();
}









