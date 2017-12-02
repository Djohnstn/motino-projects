//  WeatherSoftNode20   - super low power data collection
//  J David Johnston  - Sep 2016, Oct 2016, June 2017, July 2017, August 2017

// how about https://github.com/arduino/Arduino/issues/4863

// from http://forum.arduino.cc/index.php?topic=194603.0
// de-dup char F strings...
// http://michael-buschbeck.github.io/arduino/2013/10/22/string-merging-pstr-percent-codes/
//#define PSTR(str) \
//  (__extension__({ \
//    PGM_P ptr;  \
//    asm volatile \
//    ( \
//      ".pushsection .progmem.data, \"SM\", @progbits, 1" "\n\t" \
//      "0: .string " #str                                 "\n\t" \
//      ".popsection"                                      "\n\t" \
//    ); \
//    asm volatile \
//    ( \
//      "ldi %A0, lo8(0b)"                                 "\n\t" \
//      "ldi %B0, hi8(0b)"                                 "\n\t" \
//      : "=d" (ptr) \
//    ); \
//    ptr; \
//  }))


// see if this overrides it - from 1204 bytes down some - jdj 6/2017
#define SERIAL_TX_BUFFER_SIZE 16
#define SERIAL_RX_BUFFER_SIZE 16
// i guess not
// see C:\Users\David\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.19\cores\arduino
// or see C:\opt\arduino-1.8.2\hardware\arduino\avr\cores\arduino\HardwareSerial.h

#include "Arduino.h"  // standard headers

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <Streaming.h> // support serial << stuff; http://arduiniana.org/libraries/streaming/
//#include <LowPower.h>  // https://github.com/LowPowerLab/LowPower
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
//#include <CRC16.h>  // data packet definitions
#include <ShortPacket2.h>  // data packet definitions
#include <BlinkLED.h> // LED Blinker
#include <FTDIAttached.h> // FTDIAttached information about com port being available
#include <db1k.h> // eeprom database
#include <serbuf.h> // serial buffer processing
#include <serconfig.h> // string configuration command line processor
#include <Reading2.h>  // readings
#include <LowPowerTime.h>   // low power and mlliseconds functions - Johnston version
                           //writeup for LowPower here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

// weathershield includes
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src

// end weathershield includes

// use this to get rid of Serial info
#define SLIMDOWN
#ifndef SLIMDOWN
#endif
// text constants
const char* cT = "T:";
const char* cP = " P:";



//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

FTDIAttached comport;     // set up FTDI / serial status
//#define IFECHO if (comport.Attached())
//#define IFNOECHO if (!comport.Attached())

unsigned long LastSend;   // timestamp of last transmission
unsigned long nextPeriodmillis;   // when to send next

bool quiet;               // go quiet - no radio xmit

BlinkLED led (LED);       // set up the LED
db1k db;                  // eeprom database
RFM69 radio;              // radio
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
ShortPacket2 packet;      // radio packet
unsigned long nonce;      // saved nonce from packet
LowPowerTime lp;          // ==== time and power ===
//bool timeKnown;           //  is current time known? // moved to time library
bool WirelessHexEnabled;  // should we listen for an update to the code?
// serial support
#define SERIAL_BAUD   115200
serbuf SB;
serconfig scfg;



// system settings to get from database
unsigned long ENcrytkey[4];// = {737410024, 482208723, 974085755, 543745905} ; // E1xE2xE3xE4x // 16 bytes
unsigned long HMACkey[4];// =   {736724401, 902293479, 296437593, 301201497} ; // H1xH2xH3x // 16 bytes
unsigned long transmitSeconds; // = 32;   // how many seconds per transmit period; // start value
unsigned long transmitperiod; // = 32000UL; // transmit milliseconds 
unsigned long forceSend; // = 0UL;        // how long to wait before forcing send? 0 = off  !FS1#900
unsigned long HardWareconfig; //  = 1UL;  // hrdware config 1=base, 2=BME280 'HW1'
int lastMillisPerSec;                 // what was previous milliseconds per second?
byte DATaset; // = '1'; // data set to process from
byte NETworkid; // = 1; // NET
byte NOdid; // = 0xb1; //NOn node number
byte GATway; // = 1;    // GAT
byte FREquency = RF69_915MHZ; // FRE
uint8_t RadioLevel;  // starts at 8 max is 31, min is 0

unsigned long radiotime;
#define RADIOSTART       {unsigned long radiostart = millis();
#define RADIOSTOP        radiotime += (millis() - radiostart);}
#define RADIOSTOPV       radiotime += (millis() - radiostart); Serial.print("RT:"); Serial.println((millis() - radiostart)); }
//unsigned long LastSuccessfulSend = 0; // last time we successfully sent a radio packet
//unsigned long LastBlink = 0;          // last time we blinked

// ---  memory management
// https://github.com/arduino/Arduino/issues/5289
//unsigned int stackAvailable() {
//    extern int __heap_start, *__brkval; 
//    unsigned int v; 
//    return (unsigned int)&v - (__brkval == 0 ? (unsigned int)&__heap_start : (unsigned int)__brkval); 
//}

//unsigned int heapAvailable()
//{
//    unsigned int total = stackAvailable();
//
//    struct __freelist* current;
//    extern struct __freelist *__flp;
//    for (current = __flp; current; current = current->nx)
//    {
//        total += 2; /* Add two bytes for the memory block's header  */
//        total += (unsigned int) current->sz;
//    }
//
//    return total;
//}
// --------------------

// WeatherShield stuff

// TODO - move CDS to different port number! A3 is battery voltage!
#define LIGHTSENSOR A2 // pin 17 = A3 This pin is used to read the value of the Right Sensor.

int lightRead() {
  int rawLight;
  int intLight;
  pinMode(LIGHTSENSOR, INPUT); // Defines this pin as an input. The Arduino will read values from this pin.
  digitalWrite(LIGHTSENSOR, HIGH); // Enables an internal pullup resistor
  delay(2);                   // wait a bit
  for (byte ix = 0; ix<2; ix++) rawLight = analogRead(LIGHTSENSOR); // read twice to filter out first value 
  intLight = 1023 - rawLight; // flip integer backwards, raw value is towards zero for more light
  digitalWrite(LIGHTSENSOR, LOW); // Disables an internal pullup resistor
  return intLight;  
}

// close off unused pins.
void pinSetup() {
  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;   // might need to fix this...
    if (i == FLASH_SS) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
}

//*********************************************************************************************
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR  A7   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_CYCLES   2    //read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cyclesyou would get ~1 hour intervals
//#define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_FORMULA_INT(reading) reading * 14750 / 31056 // >>> fine tune this parameter to match your voltage when fully charged

//#define BATT_LOW      3.6  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*1  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************
int batteryRead() {
  long readings=0;
  int batteryVolts;
  //enable battery monitor on WeatherShield (via mosfet controlled by A3)
  pinMode(BATT_MONITOR_EN, OUTPUT);
  digitalWrite(BATT_MONITOR_EN, LOW);
  analogRead(BATT_MONITOR);   // discard first read

  for (byte ix=0; ix<5; ix++)  readings += analogRead(BATT_MONITOR); //take several samples, and average

  pinMode(BATT_MONITOR_EN, INPUT); //highZ mode will allow p-mosfet to be pulled high and disconnect the voltage divider on the weather shield
  //long readings2 = readings;  // forces calculations to 32 bit to allow integer
  batteryVolts = BATT_FORMULA_INT(readings) / 5; // / 5.0 does the averaging
  return batteryVolts;
}

BME280 bme280;

Reading2 <int> rdgBMETemp(17, -1, 1);
Reading2 <int> rdgBMEHumid(18, -1, 1);
Reading2 <int> rdgBMEPress(19, -1, 1);
Reading2 <int> rdgBATmv(20, -2, 2); 
Reading2 <int> rdgCDS(21, 0, 2);

void bmeSetup () {
  if (HardWareconfig & 2UL) {
    //initialize weather shield BME280 sensor
    bme280.settings.commInterface = I2C_MODE;
    bme280.settings.I2CAddress = 0x77;
    bme280.settings.runMode = 3; //Normal mode
    bme280.settings.tStandby = 0;
    bme280.settings.filter = 0;
    bme280.settings.tempOverSample = 1;
    bme280.settings.pressOverSample = 1;
    bme280.settings.humidOverSample = 1;
  }
}

boolean bmeRead(unsigned long currSeconds) {
  int value;
  boolean newvalue = false;
  float C,P,H;
  //char* BATstr = "BAT:5.00v";
  //  char Pstr[10];
  //  char Fstr[10];
  //  char Hstr[10];
  //  char buffer[50];
  if (HardWareconfig & 2UL) {
    // init the sensor every time? jdj 8/2017 // possible my lowpower stuff shut it down. // oh, never called it!
    //bmeSetup();
    //read BME sensor
    bme280.begin();
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    C = bme280.readTempC();
    H = bme280.readFloatHumidity();
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
    //Serial << F("P:") << P << F("C:") << C << F("H:") << H << endl;
    //bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
    // round up by adding 0.05 first
    value = (P + 0.05) * 10.0; //read Pa and convert to deci-inHg
    if(rdgBMEPress.Add(value, currSeconds)) newvalue = true;
    //IFECHO rdgBMEPress.Print();

    value = (C + 0.05) * 10.0;                    // tenths of C
    if(rdgBMETemp.Add(value, currSeconds)) newvalue = true;
    IFECHO rdgBMETemp.Print();
    
    value = (H + 0.05) * 10.0;            // tenth of percent humidity
    if(rdgBMEHumid.Add(value, currSeconds)) newvalue = true;
    //IFECHO rdgBMEHumid.Print();

    value = batteryRead();
    if (value) if(rdgBATmv.Add(value, currSeconds)) newvalue = true; // battery millivolts, if not zero
    IFECHO rdgBATmv.Print();
    //dtostrf(value, 3,2, BATstr);

    value = lightRead();
    if(rdgCDS.Add(value, currSeconds)) newvalue = true;   // light 300 - 900
    IFECHO rdgCDS.Print();

//    dtostrf(C, 3,2, Fstr);
//    dtostrf(H, 3,2, Hstr);
//    dtostrf(P, 3,2, Pstr);
//
//    sprintf(buffer, "C:%s H:%s P:%s", Fstr, Hstr, Pstr);
//    Serial.println(buffer);
    //Serial << F(" P:") << P << F(" F:") << F << F(" H:") << H << endl;
  }
  return newvalue;
}


// end weathershield definitions



void printCompileVersion() {
  //IFECHO {Serial << F("#" __FILE__ ", " __DATE__ " " __TIME__ "#") << endl;}
  IFECHO {Serial << F("#" __FILE__ ", " __TIMESTAMP__ "#\n");}
}

void processSerialInput() {
  IFECHO while (Serial.available()) {
    if (SB.readline() > 0)  {
#ifndef SLIMDOWN
        Serial << F(">") << SB.serBuff << F("<") << endl;
#endif
        char input = SB.getCh();  // peek at first letter of input // get command
        switch (input) {
          case '!':
            scfg.settingsCommand(SB, db, comport.Attached());
            break;
          case '?':
#ifndef SLIMDOWN
            Serial << F("!var $hx #nn \"text\" !x (del) Db Time") << endl;
#endif
            //Serial << F("Db Time") << endl;
            //Serial << F("Time") << endl;
            break;
          case 'D':
            db.dumpdb();
            break;
          case 'T':
            Serial << F("? T:"); lp.printSecondsMillis(); Serial << F(" Up:") << lp.myMillis() << F(" Run:") << millis() << F(" Rt:") << radiotime << endl;
            break;
          case 'V':
            printCompileVersion();
            break;
          case 'F':           // force readings in next 100 millis
            nextPeriodmillis = lp.myMillis() + 100;
            LastSend = nextPeriodmillis - transmitperiod;
            break;
          case 'r': //r=dump radio registers
            radio.readAllRegs();
            break;
          case 'w': // wireless enabled
            WirelessHexEnabled = !WirelessHexEnabled;
            IFECHO Serial << F("OTA DL:") << WirelessHexEnabled;
          default:
#ifndef SLIMDOWN
            IFECHO Serial << F("?:") << _HEX(input) << endl;
#endif
            break;
        }
        IFECHO led.Blink(1);          // show serial processing is complete
    }
  }
}

int readVcc() { // decivolts for the Cpu
  long result;
  int iresult = 0;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(100); // Wait for Vref to settle? trying 100 us instead of 1ms, doesn't seem to harm the results by anythng, and much faster!
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  //result = ADCL;
  //result |= ADCH<<8;
  result = ADCW; // ADCW takes care of ADCL and ADCH
  if (result) {
    //result = 1126400L / result; // Back-calculate AVcc in mV
    result = 112640L / result; // Back-calculate AVcc in deciV
    if (result > 32767L) {  // overflow?
    }
    else {
      iresult = result;
    }
  }
  return iresult;
}

int readPowermVolts(){ // reads Power supply voltage 5 times, one for discard, 4 to average
  int iVcc2 = 0;
  char n = 0;
  readVcc();  // discard first read to allow for debounce of ADC
  for (char ix = 0; ix < 6 && n < 3; ix++) {
    int iVcc1 = readVcc(); //  read internal Vcc millivolts
    iVcc2 += iVcc1;
    if (iVcc1) n++; // count non-zero values
  }
  int iVcc = n ? iVcc2 / n : 0;
  return iVcc;
}

int readCpuTemp() {   // CPU temperature
    int result;
    int rawADC;
    // Read temperature sensor against 1.1V reference
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ADCSRA |= _BV(ADEN); // Convert new: http://playground.arduino.cc/Main/InternalTemperatureSensor, not working?
    delay(4); // Wait for Vref to settle // was 20 ms // was 3 // was 10 
    //delayMicroseconds(500);   // moved from 10 ms to 100us delay. see iif it makes any difference , yep, bad idea
    ADCSRA |= _BV(ADSC); // Convert old // ADCRSA = (1<<ADEN) | 0x07; enable adc with 128 prescaling?
    //delay(10); // Wait for Vref to settle // was 20 ms // was 5 // was off
    while (bit_is_set(ADCSRA,ADSC));
    //result = ADCL; old
    //result |= ADCH<<8; // old
    rawADC = ADCW; // ADCW takes care of ADCL and ADCH
    // result =  (rawADC – 324.31) / 1.22
    //result = ((long)result * 2980L / 351L - 2731L ); // + 5660 - 2731 ;// * 100 / 100; // convert K to C, the 26 or 55 is the offset for accuracy? way ouside expected! // but value seems to be degF?
    //result = (((long)rawADC * 100L - 32431L)*10) / 122; // + 5660 - 2731 ;// * 100 / 100; // convert K to C, the 26 or 55 is the offset for accuracy? way ouside expected! // but value seems to be degF?
    result = (((long)rawADC * 100L - 32100L)) / 10; // + 5660 - 2731 ;// * 100 / 100; // convert K to C, the 26 or 55 is the offset for accuracy? way ouside expected! // but value seems to be degF?
    Serial.print(F("t:")); Serial.println(rawADC); Serial.println(result);
    return result; // result is DeciC (29.0 -> 290E-1)
}

int CpuTemp(){ // reads CPU temp , return EMWA or average
  static int sumTemp = 0;    // EMWA average of temps
  static char missed = 99; // missed readings - first time in, get data, varies from 1 to 6 to 99
  //int iTemp2 = 
  readCpuTemp();  // discard first read
  if (0 == sumTemp) sumTemp = readCpuTemp() * 4; // save first reading as four readings if old value is zero
  int hi = sumTemp / 2;  // hi is 200% of last
  int lo = hi / 4;            // lo is  50% of last 
  //if (hi == 0) {hi = 32000; lo = 1;}  // first time in, go for broke, but not for zero - bypassed with 'missed' counter
  int newTemp = 0;
  for (byte ix = 0; ix < 4 && newTemp == 0; ix++) {
    int iTemp1 = readCpuTemp(); //  read internal Vcc millivolts values, twice each, to allow for debounce of ADC
    if ((iTemp1 <= hi && iTemp1 >= lo) || missed > 5) {
      if (!sumTemp) sumTemp = iTemp1 * 4;
      sumTemp += iTemp1 - sumTemp / 4;              // EMWA or running average, no matter
      missed = 0;                                   // if reading, then good
      break;
    }
  }
  missed++; // if no readings, watch out, will be 1 if successful, higher for misses
  newTemp = sumTemp / 4;                      // is this ok? if cant find it, keep same value as before
  return newTemp;
}

int8_t RadioTemperature() { // degrees C -128 - +127
  //radiosleeping = false;
  int8_t temperature = radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
  if (-128 == temperature) temperature = radio.readTemperature(-1); // try again if get -128?
  return temperature;
}

// 1 = 1,2,4 byte time
// 2 = 4 byte time when time is unknown

Reading2 <int> rdgPowerVolts(8, -2, 2);
Reading2 <int> rdgCpuTemp(11, -1, 1);
Reading2 <byte> rdgRadioTemp(10, 0, 1);
Reading2 <signed byte> rdgRadioRSSI(9, 0, 0);

int8_t _rssi; // save rssi after a receive

// set output power: 0 = min, 31 = max

bool initNetworkAdapter() {
  RadioLevel = 8;  // starts at 8 max is 31, min is 0
  // setup data parameters
  DATaset      = db.getByte('d','a','t'); // which is active dataset?
  //Serial << F("byte FREquency = RF69_915MHZ; // FRE ") << FREquency << endl;
  FREquency    = db.getByte('f','r',DATaset); // frequency
  GATway       = db.getByte('G','A',DATaset);   // gateway id
  NETworkid    = db.getByte('n','e',DATaset); // network id
  NOdid        = db.getByte('N','O',DATaset);   // nodeid id
  forceSend    = db.getLong('F','S',DATaset);   // how long to force data sendng, suggest 15 minutes?
  HardWareconfig = db.getLong('H','W',DATaset); // what hardware is onboard? 1=base, 2 = bme280
  Serial << F("#HW:") << HardWareconfig; // << endl;
  int sig = 0;  // super dumb down signature of keys 
  for (int ix = 0; ix <4; ix++) {
    char middle = '0' + ix;
    ENcrytkey[ix] = db.getLong('E', middle, DATaset); // encrypt key n
    HMACkey[ix]   = db.getLong('H', middle, DATaset); // HMAC    key n
    sig *= 757;
    sig += ENcrytkey[ix] ^ HMACkey[ix];
    //Serial << '[' << middle << ']' << HMACkey[ix];
  }
  sig %= 9973;
  IFECHO Serial << F(" S:") << sig << endl;
  // db.dumpdb(); // use this to foce config dump
  //                uint16_t check2 = Fletcher16((uint8_t *)&config, sizeof(configclass)); // better checksum, 
  transmitSeconds = db.getLong('T','P', DATaset);  // transmit period
  transmitperiod = transmitSeconds * lp.MillisPerSec;
  if (transmitperiod < 2000UL) transmitperiod = 15000UL; // set minimum transmit period
  lp.transmitperiod = transmitperiod;     // copy time max  // i know, poor design, to duplicate data
  
  radio.initialize(FREquency, NOdid, NETworkid);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt((char*)ENcrytkey);
  radio.setPowerLevel(RadioLevel); // reduce/increase transmit power level

  lp._radio = &radio;
  lp.sleepRadioAndFlash();
  _rssi = -128;
  return true;
}

bool initFlash() {
  if (flash.initialize())
  {
#ifndef SLIMDOWN
    IFECHO Serial.print(F("SPI Flash OK. MAC:"));
    flash.readUniqueId();
    IFECHO for (byte i=0;i<8;i++)
    {
      Serial << _HEX(flash.UNIQUEID[i]) << F(":") ;
    }
    Serial << endl;
#endif
    return true;
  }
  else {
//#ifndef SLIMDOWN
//    IFECHO Serial << F("SPI Flash FAIL. (No chip?)\n");
//#endif
    return false;
  }
  flash.sleep();
}

int commWatch = 350;                   // 200 millisecond listen time - how long to watch for response messages // tryig 350 ms from 250 as of 8/13/2017

// put your setup code here, to run once:
void setup() {
  lp.Seconds(0);          // set time to zero
  db.begin();             // initialise database
  IFECHO Serial.begin(SERIAL_BAUD);
  printCompileVersion();    // say hello anyway, if you can.
  radiotime = 0;
  initNetworkAdapter();   // set up network
  nextPeriodmillis = 5000;// transmitperiod;         // set up to start send after 5 seconds
  LastSend = nextPeriodmillis - transmitperiod;   // set up to start transmit asap
  IFECHO packet.SpSetVerbose(true);
  IFECHO commWatch = 9000;                        // if running connected, then allow for user input
  quiet = false;                      // enable the radio
  WirelessHexEnabled = false;         // not enabled wireless until trusted packet
  initFlash();
  bmeSetup();
  pinSetup();
  led.Blink(10);          // show Setup() is complete
}

//unsigned long lastSuccessfulSend = 0; // when did we las successfully send data to the host?

bool takeReadings(unsigned long currSeconds) {
    int8_t newdata = 0; // is there new data?
    //bool forcesend = forceSend ? ((long)(currSeconds - LastSuccessfulSend)) > forceSend : false;  // forceSend = 0, no force, else use time out
    newdata += rdgPowerVolts.Add(readPowermVolts(), currSeconds);
    IFECHO rdgPowerVolts.Print();
    newdata += rdgRadioTemp.Add(RadioTemperature(), currSeconds);
//    if (_rssi > -128) newdata += rdgRadioRSSI.Add(_rssi, currSeconds);  // skip if rssi is bad
    newdata += rdgCpuTemp.Add(CpuTemp(), currSeconds); 
    IFECHO rdgCpuTemp.Print();
    newdata += bmeRead(currSeconds);
    return newdata;
}

//void printReadingHeader(unsigned long tm, char tag){
//#ifndef SLIMDOWN
//    IFECHO Serial << cT << tm << cP << _HEX(tag) << F(" ");
//#endif
//}

struct nakpacket {
  uint16_t nak;
  char type;
  char info; 
  uint32_t sec;
  uint16_t ms; 
  uint16_t check;
};

uint16_t Fletcher16( uint8_t *data, int count ) {
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   for(byte index = 0; index < count; ++index ) {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

//bool timeRequestSent = false;
byte sendsanity = 0;
bool sendReadings() {
    long beginMillis = lp.myMillis();
    unsigned long currSeconds = lp.Seconds(); //beginMillis / 1000UL;
    if (quiet) {
      //IFECHO {lp.printFormatted(); Serial << F("q.\n");}
      return false;  // if quiet mode, then send no readings
    }
    // begin packet
    //IFECHO {Serial << F("@"); lp.printFormatted(); Serial << F(" ") << currSeconds << F(" Sending ") << endl;}
#ifndef SLIMDOWN
    IFECHO {lp.printFormatted(); Serial << F("Send ");}
#endif
    packet.Begin(currSeconds);  // puts 2 bytes of data into packet
    boolean sent = false;
    boolean sendNeeded = false;
    {
      if (!lp.OK())     // if we don't know the time, tell the hub about it
      {
        const char tag = 2;   // tagid = 2, length = 4
        packet.Add(currSeconds, tag, currSeconds, 0);
        sendNeeded = true;
      }
      //rdgBATmv.PrintAll();
      //rdgRadioRSSI.PrintAll();
      IFECHO rdgRadioRSSI.Print();

///      bool newsendNeeded = true;
      int8_t newsendNeeded = 1;
      for (char dosend = 0; (dosend++ < 4) && newsendNeeded && !packet.Full(2); )
      {
        newsendNeeded = 0;
        if (HardWareconfig & 1UL) {
          newsendNeeded += (rdgPowerVolts.Send(&packet));
          newsendNeeded += (rdgCpuTemp.Send(&packet));
          newsendNeeded += (rdgRadioTemp.Send(&packet));
          newsendNeeded += (rdgRadioRSSI.Send(&packet));
        }
        if (HardWareconfig & 2UL) {
          newsendNeeded += (rdgBMETemp.Send(&packet));
          newsendNeeded += (rdgBMEHumid.Send(&packet));
          newsendNeeded += (rdgBMEPress.Send(&packet));
          newsendNeeded += (rdgBATmv.Send(&packet));
          newsendNeeded += (rdgCDS.Send(&packet));
        }
        if (newsendNeeded) sendNeeded = true;
      }
      byte datasize = packet.dataSize();  // do we have data? (OK, two bytes is data... but really, just the id byte and low byte of time from Begin call)
      if ((datasize > 2) && sendNeeded) {
        //IFECHO 
        led.Blink(1);          // show sending
        packet.Close((uint8_t*) &HMACkey); // close is expensive, only do this if we really have something to say
        //led.BlinkOff();          // show sending
        nonce = packet.nonce - 1;   // save the packet nonce to see what we get back from hub
        byte sendsize = packet.sendSize();
        RADIOSTART;
        sent = radio.sendWithRetry(GATway, packet.Data(), sendsize, 3, 30);    // 3 tries, 190 ms response time
        //RADIOSTOP;
        RADIOSTOPV;   // print radio online time to see how long it takes, may tweak the retry time
        IFECHO packet.SerialDisplay();  // debug dump
        //Serial << sent << F("L") << radio.DATALEN << endl;
        if (sent) {     // check if we got a NAK
          //Serial << "Ac";
          //if (radio.ACKReceived(GATway))  // not sure what good this call is, it doesn't work.
          if (int8_t datalen = radio.DATALEN) // this finds size of return packet from ack.
          {
            //Serial << datalen;
            if (datalen >= sizeof(nakpacket)) {
              nakpacket nakpak;
              datalen = sizeof(nakpacket);
              memcpy (&nakpak, (const void *) radio.DATA, datalen);
              if (nakpak.nak == 0x1515) { // got NAK NAK
                sent = false;            // data NOT processed, yes it was sent, bt it was refused.
                _rssi = radio.RSSI; // save RSSI if failed sendWithRetry  !!
                if (nakpak.type == '@' && nakpak.info == 0x01 && nakpak.sec != 0) {  // we got the time?
                  uint16_t check2 = Fletcher16((uint8_t *)&nakpak, sizeof(nakpacket) - 2); // better checksum, but will it fit?
                  if (check2 == nakpak.check) {
                    lp.Seconds(nakpak.sec, nakpak.ms); // set the time
#ifndef SLIMDOWN
                    IFECHO {Serial << F("nk TZ:"); lp.printSecondsMillis();} // Serial << F("s");}
#endif
                  }
                }
              }
            }
          }
        }
        if (sent) {
          sendsanity = 0;     // send was ok
          _rssi = radio.RSSI; // save RSSI if successful sendWithRetry  !!
          //LastSuccessfulSend = lp.Seconds();
          if (HardWareconfig & 1UL) {
            rdgPowerVolts.SentOK();
            rdgCpuTemp.SentOK();
            rdgRadioTemp.SentOK();
            rdgRadioRSSI.SentOK();
          }
          if (HardWareconfig & 2UL) {
            rdgBMETemp.SentOK();
            rdgBMEHumid.SentOK();
            rdgBMEPress.SentOK();
            rdgBATmv.SentOK();
            rdgCDS.SentOK();
          }

//#ifndef SLIMDOWN
//          IFECHO Serial << F(" OK.") << endl;
//#endif
          led.Blink(1);          // show Send worked
        }
        else {
          sendsanity++;   // count failures
          if (HardWareconfig & 1UL) {
            rdgPowerVolts.Unsent();
            rdgCpuTemp.Unsent();
            rdgRadioTemp.Unsent();
            rdgRadioRSSI.Unsent();
          }
          if (HardWareconfig & 2UL) {
            rdgBMETemp.Unsent();
            rdgBMEHumid.Unsent();
            rdgBMEPress.Unsent();
            rdgBATmv.Unsent();
            rdgCDS.Unsent();
          }
#ifndef SLIMDOWN
//          IFECHO Serial << F(" Fail.") << endl;
#endif
          for (byte ix = 0; ix++ < 2;) {
            led.Blink(1);          // show Send failed
            delay(10);
          }
          // should probably up the radio power if we haven't sent successfully for more than 20 tries
          // up power if last 20 sends failed, then wait another 20, no need to shout suddenly, also dont get real loud ever
          if ((sendsanity > 60) && (RadioLevel < 20)) {RadioLevel++; radio.setPowerLevel(RadioLevel); sendsanity = 0;}
        }
      }
    }
    return sent;
}

bool savePacket() {
  byte accepted = false;
  if ((radio.DATALEN <= 64) && (radio.DATALEN > 0)) {
    accepted = packet.setPayload((const void *) radio.DATA, radio.DATALEN);
  }
  else {
//#ifndef SLIMDOWN
//    IFECHO Serial << F(" packet XL.");
//#endif
  }
  return accepted;
}

bool processRadioInput() {
  byte theSenderID;
  byte theTargetID;
  byte theDatalen;
  //int16_t theRSSI;
  char theRSSI;
  bool processed = false;   // did we process anything?
  int timeMilliseconds; // what was milliseconds sent down
  if (radio.receiveDone()) {
    theRSSI = radio.RSSI; // very volatile!
    unsigned long currSeconds = lp.Seconds();
    processed = true;     // ok, we processed it, go away...
    if (theRSSI != -128) rdgRadioRSSI.Add(theRSSI, currSeconds - 10);  // TODO: remove the -10
    nonce = packet.nonce - 1; // save the nonce
    bool packetSaved = savePacket();
    // this gets lost on reply/ACK!
    theSenderID = radio.SENDERID;
    theTargetID = radio.TARGETID;
    theDatalen = radio.DATALEN;
    //Serial.println(F("PAK"));
    if (radio.ACKRequested()) {     // When a node requests an ACK, respond to the ACK
      RADIOSTART;
      radio.sendACK();
      RADIOSTOP;
//#ifndef SLIMDOWN
      IFECHO Serial.println(F("AC>"));
//#endif
    }
    IFECHO {
//#ifndef SLIMDOWN
      Serial << endl; lp.printFormatted();
      Serial << F(" From:") << _HEX(theSenderID) << F(" To:") << _HEX(theTargetID) << F(" L:") << _DEC(theDatalen) << F(" RSSI:") << _DEC(theRSSI) << endl;
//#endif
    }
    if (WirelessHexEnabled) {   // enabled to download new code
        CheckForWirelessHEX(radio, flash, true);
    }
    ShortPacket2::status packetAccepted = packet.validPayload(currSeconds, (uint8_t*) &HMACkey); //validatePacket(packetSaved);
    if (!lp.OK()) {   // if we are not sure of time, take what we can get
      if (ShortPacket2::ChecksumFail == packetAccepted) {
        if ((nonce - 1) <= packet.nonce && packet.nonce <= (nonce + 1)) {
          packetAccepted = ShortPacket2::status::OK;
        }
      }
    }
    //IFECHO Serial.println();
#ifndef SLIMDOWN
    IFECHO Serial << F("OK?:") << _HEX(packetAccepted) << F(" Id:") << nonce << F(" pktId:") << packet.nonce << endl;
#endif
    if (ShortPacket2::OK == packetAccepted) { // || (nonce == packet.nonce)) {
          timeMilliseconds = 500; // reset milliseconds - average of 500 milliseconds if not told otherwise
          unsigned long databaseValue = 0;
          for (byte mp = packet.firstPacket(); mp < 0xff ; mp = packet.nextPacket(mp)) {
            byte bminiPacketName = packet.miniPacketName(mp);
            if (bminiPacketName == 0) break;  // no name? done!
            byte bminiPacketLength = packet.miniPacketDataLength(mp);
            byte mp_b = 0;
            char mp_c = 0;
            int mp_int = 0;
            unsigned long mp_ul = 0;
            switch (bminiPacketLength) {
              case 1: mp_b = packet.miniPacketValue_c(mp); mp_c = (char) mp_b; mp_int = mp_b; mp_ul = mp_int; break;
              case 2: mp_int = packet.miniPacketValue_i(mp); mp_ul = mp_int; break;
              case 4: mp_ul = packet.miniPacketValue_L(mp); break;
              default: break;
            }
#ifndef SLIMDOWN
            IFECHO Serial << F("[") << bminiPacketName << F("~") << bminiPacketLength << F("] ");
#endif
            switch (bminiPacketName) {
              case 3:
                    timeMilliseconds = mp_int;    // save the miilliseconds
                    break;
              case 4:
                      if ((mp_ul) && (mp_ul != lp.Seconds())) {
#ifndef SLIMDOWN
                      IFECHO Serial << F("UTC:") << mp_ul << F(".") << timeMilliseconds << endl; // << F("s"); 
#endif
#ifndef SLIMDOWN
                        IFECHO {Serial << F(" From:"); lp.printSecondsMillis();} //Serial << F("s");}
#endif
                        lp.Seconds(mp_ul, timeMilliseconds); // set the time
                        int newmillisPerSec = lp.MillisPerSec;  
#ifndef SLIMDOWN
                        IFECHO Serial << F(" v:") << newmillisPerSec; // << F("ms/s"); 
#endif
                        if (newmillisPerSec != lastMillisPerSec) {      // if we have a new millis, recalculate it
                          lastMillisPerSec = newmillisPerSec;
                          transmitperiod = transmitSeconds * newmillisPerSec;     // recalculate transmit period after reloading time
                          if (transmitperiod < 3000UL) transmitperiod = 60000UL; // set minimum transmit period = 1 minute
                          //LastSend = nextPeriodmillis - transmitperiod;   // reset up to start transmit asap
                          LastSend = lp.myMillis() - timeMilliseconds;      // try this, resync to right now, back up to start of second?
                          nextPeriodmillis = LastSend + transmitperiod;     // recalculate next period from this period
                        }
#ifndef SLIMDOWN
                        IFECHO {Serial << F(" To:"); lp.printSecondsMillis();} // Serial << F("s");}
#endif
                      }
                      break;
              case  7:
                      // get value for database, get value before get data
                      databaseValue = mp_ul;
                      break;
              case  6:
                      // send value for database
                      db.writeKeyValue(mp_int, databaseValue);
                      break;
              case  9: 
//#ifndef SLIMDOWN
//                      IFECHO Serial << F(" RSSI:") << _DEC(mp_c) << F(" Power:") << RadioLevel;
//#endif
                      if (mp_c > -90) {                     // target db level is -92
                        // change power rating down;
                        //if (RadioLevel > 10) RadioLevel--;  // fast power down above 10 // save the program space, do it slow
                        if (RadioLevel > 1) RadioLevel--;   // some power down above 0 - are you sure?
                        radio.setPowerLevel(RadioLevel); // reduce / increase transmit power level
                      }
                      else {                              // if we are really low, bump up the radio level
                        if (mp_c > -128 && mp_c < -95) {
                          // change power rating up;
                          if (RadioLevel < 25) RadioLevel++;    // don't shout
                          radio.setPowerLevel(RadioLevel); // reduce / increase transmit power level
                        }
                      }  
#ifndef SLIMDOWN
                      IFECHO Serial << F(" To:") << RadioLevel;
#endif
                      break;
              case 8: // wireless enable or not
                      WirelessHexEnabled = mp_c;
#ifndef SLIMDOWN
                      IFECHO Serial << F("OTA:") << WirelessHexEnabled;
#endif
                      break;
              default: 
                      // IFECHO Serial << F("Packet ") << _DEC(bminiPacketName); 
                      break;
            }
//#ifndef SLIMDOWN
//            IFECHO Serial << " ";
//#endif
          }
//#ifndef SLIMDOWN
//          IFECHO Serial << endl;
//#endif
      }
      else {
        IFECHO packet.SerialDisplay();
      }
    led.Blink(1);
    packet.nonce = nonce + 1; // put the nonce back!
  }
  return processed;
}

#define VDUMP(x) Serial << #x << " " << x << endl

bool longlisten = false;  // listen after send? false = no, true = yes

// main code here to run forever
void loop() {
  unsigned long currMillis = lp.myMillis();
  if ((currMillis - LastSend) >= transmitperiod) {
    unsigned long currSeconds = lp.Seconds(); //currMillis / 1000UL;
    LastSend = nextPeriodmillis;
    nextPeriodmillis += transmitperiod;
    if (takeReadings(currSeconds)) {                            // take readings, then start a packet if needed
      longlisten = true;  // listen after send
      bool sent = sendReadings();                       // send base set of readings
      if (sent) longlisten = true;
    }
    // update stats if we sent data, else, data is current anyway
    currMillis = lp.myMillis();                 //Serial << "currmillis:" << currMillis << endl;
  }

  // Listen for input? or sleep if we have a chance to sleep...
  if (((currMillis - LastSend) < commWatch) && longlisten) { // || comport.Attached()) {    // if we are wired, and it has been less than a few seconds, handle serial input
      processRadioInput();  // hear anything?
      IFECHO processSerialInput();
//      led.BlinkFree(1, 1024);         // say hi every second?
      delay(1);                       // delay some anyway
  }
  else {
    longlisten = false;     // turn off listen flag if radio took too long
    WirelessHexEnabled = false; // reset OTA  -- when to do this?
    if ((currMillis - LastSend) < transmitperiod) {
      //Serial.println("\nzz"); Serial.flush();
      led.BlinkOff(); 
      long thisDelay = LastSend + transmitperiod - currMillis; //Serial << "thisdelay:" << thisDelay << endl;
      lp.lowPowerDelay(thisDelay);
    }
    else {
//      led.BlinkFree(1, 60000); // say hi // saves 100 bytes pulling this out!
      delay(1); // delay some anyway
    }
  }
}
// END END END END END END END END END END END END END END END END END END END END END END END END END END
