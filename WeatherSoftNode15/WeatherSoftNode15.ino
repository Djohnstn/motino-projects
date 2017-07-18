//  WeatherSoftNode15   - super low power data collection
//  J David Johnston  - Sep 2016, Oct 2016, June 2017

// see if this overrides it - from 1204 bytes down some - jdj 6/2017
#define SERIAL_TX_BUFFER_SIZE 16
#define SERIAL_RX_BUFFER_SIZE 16
// i guess not

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
#include <Reading.h>  // readings
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
#define IFECHO if (comport.Attached())
#define IFNOECHO if (!comport.Attached())

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

// WeatherShield stuff

// TODO - move CDS to different port number! A3 is batttery voltage!
const int LightSensor = A2; // pin 17 = A3 This pin is used to read the value of the Right Sensor.

int lightRead() {
  int intLight;
  pinMode(LightSensor, INPUT); // Defines this pin as an input. The Arduino will read values from this pin.
  digitalWrite(LightSensor, HIGH); // Enables an internal pullup resistor
  delay(2);                   // wait a second
  intLight = map(analogRead(LightSensor),0, 1024, 1024, 0); 
  digitalWrite(LightSensor, LOW); // Disables an internal pullup resistor
  return intLight;  
}

//*********************************************************************************************
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR  A7   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_CYCLES   2    //read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cyclesyou would get ~1 hour intervals
#define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_LOW      3.6  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************
int batteryRead() {
  unsigned int readings=0;
  float batteryVolts;
  //enable battery monitor on WeatherShield (via mosfet controlled by A3)
  pinMode(BATT_MONITOR_EN, OUTPUT);
  digitalWrite(BATT_MONITOR_EN, LOW);
  analogRead(BATT_MONITOR);   // discard first read

  //for (byte i=0; i<4; i++) //take several samples, and average
  //  readings+=analogRead(BATT_MONITOR);
  readings = analogRead(BATT_MONITOR);
  
  //disable battery monitor
  pinMode(BATT_MONITOR_EN, INPUT); //highZ mode will allow p-mosfet to be pulled high and disconnect the voltage divider on the weather shield
    
  batteryVolts = BATT_FORMULA(readings) * 1000;
  return batteryVolts;
}

BME280 bme280;

Reading <int> rdgBMETemp(17);
Reading <int> rdgBMEHumid(18);
Reading <int> rdgBMEPress(19);
Reading <int> rdgBATmv(20);
Reading <int> rdgCDS(21);

void bmeSetup () {
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

boolean bmeRead(unsigned long currSeconds) {
    int value;
    boolean newvalue = false;
      //read BME sensor
    bme280.begin();
    value = bme280.readFloatPressure() * 0.0002953 * 10; //read Pa and convert to deci-inHg
    if(rdgBMEPress.Add(value, currSeconds)) newvalue = true;
    IFECHO Serial.println(value);
    value = bme280.readTempC() * 10;                    // tenths of C
    if(rdgBMETemp.Add(value, currSeconds)) newvalue = true;
    IFECHO Serial.println(value);
    value = bme280.readFloatHumidity() * 10;            // tenth of percent humidity
    if(rdgBMEHumid.Add(value, currSeconds)) newvalue = true;
    IFECHO Serial.println(value);
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280

    value = batteryRead();
    if(rdgBATmv.Add(value, currSeconds)) newvalue = true; // battery millivolts
    IFECHO Serial.println(value);
    value = lightRead();
    if(rdgCDS.Add(value, currSeconds)) newvalue = true;   // light 300 - 900
    IFECHO Serial.println(value);

    return newvalue;
    //dtostrf(F, 3,2, Fstr);
    //dtostrf(H, 3,2, Hstr);
    //dtostrf(P, 3,2, Pstr);

    //sprintf(buffer, "BAT:%sv F:%s H:%s P:%s", BATstr, Fstr, Hstr, Pstr);
    //Serial << F(" P:") << P << F(" F:") << F << F(" H:") << H << endl;
}


// end weathershield definitions


// system settings to get from database
unsigned long ENcrytkey[4];// = {737410024, 482208723, 974085755, 543745905} ; // E1xE2xE3xE4x // 16 bytes
unsigned long HMACkey[4];// =   {736724401, 902293479, 296437593, 301201497} ; // H1xH2xH3x // 16 bytes
unsigned long transmitSeconds = 32;   // how many seconds per transmit period;
unsigned long transmitperiod = 32000UL;
unsigned long forceSend = 0UL;        // how long to wait before forcing send? 0 = off  !FS1#900
int lastMillisPerSec;                 // what was previous milliseconds per second?
byte DATaset = '1'; // data set to process from
byte NETworkid = 1; // NET
byte NOdid = 0xb1; //NOn node number
byte GATway = 1;    // GAT
byte FREquency = RF69_915MHZ; // FRE

unsigned long radiotime;
#define RADIOSTART       unsigned long radiostart = millis();
#define RADIOSTOP        radiotime += (millis() - radiostart);
unsigned long LastSuccessfulSend = 0; // last time se successfully sent a radio packet
unsigned long LastBlink = 0;          // last time we blinked

// ---  memory management
// https://github.com/arduino/Arduino/issues/5289
unsigned int stackAvailable() {
    extern int __heap_start, *__brkval; 
    unsigned int v; 
    return (unsigned int)&v - (__brkval == 0 ? (unsigned int)&__heap_start : (unsigned int)__brkval); 
}

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


void printCompileVersion() {
  IFECHO {Serial << F("#" __FILE__ ", " __DATE__ " " __TIME__ "#") << endl;}
}


void processSerialInput() {
  IFECHO while (Serial.available()) {
    //int rdln;
    if (SB.readline() > 0)  {
#ifndef SLIMDOWN
        Serial << F(">") << SB.serBuff << F("<") << endl;
#endif
        char input = SB.getCh();  // peek at first letter of input // get command
        //Serial << F("Cmd:\"") << input << F("\",");
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
            Serial << F("? T:"); lp.printSecondsMillis(); Serial << F(" Up:") 
                    << lp.myMillis() << F(" Run:") << millis() << F(" Rt:") << radiotime << endl;
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
            IFECHO Serial << F("OTA DL: ") << WirelessHexEnabled;
          default:
#ifndef SLIMDOWN
            IFECHO Serial << F("?:") << _HEX(input) << endl;
#endif
            break;
        }
        IFECHO led.Blink(100);          // show serial processing is complete
    }
  }
}

int readVcc() { // Voltage for the Cpu
  long result;
  int iresult;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(1); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  if (result > 32767L) {  // overflow?
    iresult = 32767;
  }
  else {
    iresult = result;
  }
  return iresult;
}

int readPowermVolts(){ // reads Power supply voltage 5 times, one for discard, 4 to average
  int iVcc2 = 0;
  readVcc();  // discard first read
  for (int ix = 0; ix < 2; ix++) {
    int iVcc1 = readVcc(); //  read internal Vcc millivolts values, twice each, to allow for debounce of ADC
    iVcc2 += iVcc1;
  }
  int iVcc = iVcc2 / 2;
  return iVcc;
}

int readCpuTemp() {   // CPU temperature
    int result;
    // Read temperature sensor against 1.1V reference
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ADCSRA |= _BV(ADEN); // Convert new: http://playground.arduino.cc/Main/InternalTemperatureSensor, not working?
    delay(5); // Wait for Vref to settle // was 20 ms // was 3
    ADCSRA |= _BV(ADSC); // Convert old // ADCRSA = (1<<ADEN) | 0x07; enable adc with 128 prescaling?
    //delay(10); // Wait for Vref to settle // was 20 ms // was 5 // was off
    while (bit_is_set(ADCSRA,ADSC));
    //result = ADCL; old
    //result |= ADCH<<8; // old
    result = ADCW; // ADCW takes care of ADCL and ADCH
    return (result - 320) * 100 / 122; // convert K to C, the 26 or 55 is the offset for accuracy? way ouside expected! // but value seems to be degF?
}

int CpuTemp(){ // reads CPU temp 3 times, one for discard, 2 to average?
  int iTemp2 = 0;
  readCpuTemp();  // discard first read
  for (int ix = 0; ix < 2; ix++) {
    int iTemp1 = readCpuTemp(); //  read internal Vcc millivolts values, twice each, to allow for debounce of ADC
    iTemp2 += iTemp1;
  }
  int iTempx = iTemp2 / 2;
  return iTempx;
}

byte RadioTemperature() { // degrees C -128 - +127
  //radiosleeping = false;
  byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
  if (-128 == temperature) temperature = radio.readTemperature(-1); // try again if get -128?
  return temperature;
}



// 1 = 1,2,4 byte time
// 2 = 4 byte time when time is unknown

Reading <int> rdgPowerVolts(8);
Reading <int> rdgCpuTemp(11);
Reading <byte> rdgRadioTemp(10);
Reading <char> rdgRadioRSSI(9);

char _rssi; // save rssi after a receive

// set output power: 0 = min, 31 = max
uint8_t RadioLevel = 8;  // starts at 8 max is 31, min is 0

bool initNetworkAdapter() {
  // setup data parameters
  DATaset      = db.getByte('d','a','t'); // which is active dataset?
  //Serial << F("byte FREquency = RF69_915MHZ; // FRE ") << FREquency << endl;
  FREquency    = db.getByte('f','r',DATaset); // frequency
  GATway       = db.getByte('G','A',DATaset);   // gateway id
  NETworkid    = db.getByte('n','e',DATaset); // network id
  NOdid        = db.getByte('N','O',DATaset);   // nodeid id
  forceSend    = db.getLong('F','S',DATaset);   // nodeid id
  ENcrytkey[0] = db.getLong('E','0', DATaset); // encrypt key 0
  ENcrytkey[1] = db.getLong('E','1', DATaset); // encrypt key 1
  ENcrytkey[2] = db.getLong('E','2', DATaset); // encrypt key 2
  ENcrytkey[3] = db.getLong('E','3', DATaset); // encrypt key 3
  HMACkey[0]   = db.getLong('H','0', DATaset); // HMAC    key 0
  HMACkey[1]   = db.getLong('H','1', DATaset); // HMAC    key 1
  HMACkey[2]   = db.getLong('H','2', DATaset); // HMAC    key 2
  HMACkey[3]   = db.getLong('H','3', DATaset); // HMAC    key 3
  transmitSeconds = db.getLong('T','P', DATaset);  // transmit period
  transmitperiod = transmitSeconds * lp.MillisPerSec;
  if (transmitperiod < 2000UL) transmitperiod = 5000UL; // set minimum transmit period
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
#ifndef SLIMDOWN
    IFECHO Serial << F("SPI Flash FAIL. (No chip?)\n");
#endif
    return false;
  }
  flash.sleep();
}

int commWatch = 250;                   // 200 millisecond listten time - how long to watch for response messages
//bool radio_xmit = false;                // radio in transmit mode // wish there was a way to be sure radio was in listen only mode

// put your setup code here, to run once:
void setup() {
  lp.Seconds(0);          // set time to zero
  //timeKnown = false;      // no, we don't know the current time - part of lp creation
  db.begin();             // initialise database
  IFECHO Serial.begin(SERIAL_BAUD);
  printCompileVersion();    // say hello anyway, if you can.
  radiotime = 0;
  initNetworkAdapter();   // set up network
  nextPeriodmillis = 5000;// transmitperiod;         // set up to start send after 5 seconds
  LastSend = nextPeriodmillis - transmitperiod;   // set up to start transmit asap
  //IFECHO packet.SpSetVerbose(true);
  IFECHO commWatch = 9000;                        // if running connected, then allow for user input
  quiet = false;                      // enable the radio
  WirelessHexEnabled = false;         // not enabled wireless until trusted packet
  initFlash();
  led.Blink(5);          // show Setup() is complete
}

//unsigned long lastSuccessfulSend = 0; // when did we las successfully send data to the host?

bool takeReadings(unsigned long currSeconds) {
    bool newdata = false; // is there new data?
    bool forcesend = forceSend ? ((long)(currSeconds - LastSuccessfulSend)) > forceSend : false;  // forcSend = 0, no force, else use time out
    if (rdgPowerVolts.Add(readPowermVolts(), currSeconds))  {newdata = true; if (forcesend) rdgPowerVolts.nowNeedSend = true;}
    if (rdgRadioTemp.Add(RadioTemperature(), currSeconds))  {newdata = true; if (forcesend) rdgRadioTemp.nowNeedSend = true;}
    // skip it if rssi is bad
    if (_rssi > -128) if (rdgRadioRSSI.Add(_rssi, currSeconds)) {newdata = true; if (forcesend) rdgRadioRSSI.nowNeedSend = true;}
    if (rdgCpuTemp.Add(CpuTemp(), currSeconds))             {newdata = true; if (forcesend) rdgCpuTemp.nowNeedSend = true;}
    //Serial << F("\nBAT:") << batteryRead();
    if (bmeRead(currSeconds)) newdata = true;
    IFECHO led.Blink(50);          // show serial processing is complete
    return newdata;
}

void printReadingHeader(unsigned long tm, char tag){
#ifndef SLIMDOWN
    IFECHO Serial << cT << tm << cP << _HEX(tag) << F(" ");
#endif
}

//bool timeRequestSent = false;
byte sendsanity = 0;
bool sendReadings() {
    long beginMillis = lp.myMillis();
    unsigned long currSeconds = lp.Seconds(); //beginMillis / 1000UL;
    if (quiet) {
      IFECHO {lp.printFormatted(); Serial << F("q.\n");}
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
      // Serial << F("stackavailable:") << stackAvailable() << endl; //F(" heapavailable:") << heapAvailable() << endl;
      //if (!timeRequestSent && !timeKnown)     // if we don't know the time, tell the hub about it
      if (!lp.OK())     // if we don't know the time, tell the hub about it
      {
        unsigned long tm = currSeconds;  // current seconds readings
        const char tag = 2;   // tagid = 2, length = 4
        //IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(tag) << F(", Time: ") << tm << F("s") << endl;
        //IFECHO Serial << cT << tm << cP << _HEX(tag) << F(", Time: ") << tm << F("s") << endl;
#ifndef SLIMDOWN
        printReadingHeader(tm, tag);  IFECHO Serial << F("Time:") << tm << endl;
#endif
        packet.Add(tm, tag, tm);
//        timeRequestSent = true;
        sendNeeded = true;
      }
//      else {
//        timeRequestSent = false;
//      }
      if (rdgPowerVolts.dataChanged()) 
      {
        unsigned long tm = rdgPowerVolts.GetTime();
        int value = rdgPowerVolts.GetData();
        if (value!= -1) {
          //IFECHO Serial << F("T:") << tm << F(" P:") << _HEX(rdgPowerVolts.Tag()) << F(" V:") << value << F("mV") << endl;
          //IFECHO Serial << cT << tm << cP << _HEX(rdgPowerVolts.Tag()) << F(" V:") << value << endl; //F("mV") << endl;
//#ifndef SLIMDOWN
//          printReadingHeader(tm, rdgPowerVolts.Tag());  IFECHO Serial << F("V:") << value << endl;
//#endif
          packet.Add(tm, rdgPowerVolts.Tag(), value);
          sendNeeded = true;
        }
      }
      if (rdgCpuTemp.dataChanged()) 
      {
        unsigned long tm = rdgCpuTemp.GetTime();
        char temp = rdgCpuTemp.GetData();
        //int tempF = tempC * 9 / 5 + 32;
        if(temp != -1) {
          //IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(rdgCpuTemp.Tag()) << F(", CpuTemp: ") << (int)temp << F("C") << endl;
          //IFECHO Serial << cT << tm << cP << _HEX(rdgCpuTemp.Tag()) << F(", CpuTemp: ") << (int)temp << F("C") << endl;
//#ifndef SLIMDOWN
//          printReadingHeader(tm, rdgCpuTemp.Tag());  IFECHO Serial << F("CpuC:") << (int)temp << endl;
//#endif
          packet.Add(tm, rdgCpuTemp.Tag(), temp);
          sendNeeded = true;
        }
      }
      if (rdgRadioTemp.dataChanged()) 
      {
        unsigned long tm = rdgRadioTemp.GetTime();
        char tempC = rdgRadioTemp.GetData();
        if (tempC != -1) {
          //IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(rdgRadioTemp.Tag()) << F(", RadioTemp: ") << (int)tempC << F("C") << endl;
          //IFECHO Serial << cT << tm << cP << _HEX(rdgRadioTemp.Tag()) << F(", RadioTemp: ") << (int)tempC << F("C") << endl;
//#ifndef SLIMDOWN
//          printReadingHeader(tm, rdgRadioTemp.Tag());  IFECHO Serial << F("RadioC:") << (int)tempC << endl;
//#endif
          packet.Add(tm, rdgRadioTemp.Tag(), tempC);
          sendNeeded = true;
        }
      }
      if (rdgRadioRSSI.dataChanged(1)) // window of 1 vs 0
      {
        unsigned long tm = rdgRadioRSSI.GetTime();
        char value = rdgRadioRSSI.GetData();
        if (value != -128) {
          //IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(rdgRadioRSSI.Tag()) << F(", RSSI: ") << _DEC(value) << F("db") << endl;
          //IFECHO Serial << cT << tm << cP << _HEX(rdgRadioRSSI.Tag()) << F(", RSSI: ") << _DEC(value) << F("db") << endl;
//#ifndef SLIMDOWN
//          printReadingHeader(tm, rdgRadioRSSI.Tag());  IFECHO Serial << F("RSSI:") << _DEC(value) << endl;
//#endif
          packet.Add(tm, rdgRadioRSSI.Tag(), value);
          sendNeeded = true;
        }
      }


// BEGIN BME SECTION

//Reading <int> rdgBMETemp(17);
      if (rdgBMETemp.dataChanged()) {
        unsigned long tm = rdgBMETemp.GetTime();
        int value = rdgBMETemp.GetData();
        if (value!= -1) {packet.Add(tm, rdgBMETemp.Tag(), value); sendNeeded = true;}
      }
//Reading <int> rdgBMEHumid(18);
      if (rdgBMEHumid.dataChanged()) {
        unsigned long tm = rdgBMEHumid.GetTime();
        int value = rdgBMEHumid.GetData();
        if (value!= -1) {packet.Add(tm, rdgBMEHumid.Tag(), value); sendNeeded = true;}
      }
//Reading <int> rdgBMEPress(19);
      if (rdgBMEPress.dataChanged()) {
        unsigned long tm = rdgBMEPress.GetTime();
        int value = rdgBMEPress.GetData();
        if (value!= -1) {packet.Add(tm, rdgBMEPress.Tag(), value); sendNeeded = true;}
      }
//Reading <int> rdgBATmv(20);
      if (rdgBATmv.dataChanged()) {
        unsigned long tm = rdgBATmv.GetTime();
        int value = rdgBATmv.GetData();
        if (value!= -1) {packet.Add(tm, rdgBATmv.Tag(), value); sendNeeded = true;}
      }
//Reading <int> rdgCDS(20);
      if (rdgCDS.dataChanged()) {
        unsigned long tm = rdgCDS.GetTime();
        int value = rdgCDS.GetData();
        if (value!= -1) {packet.Add(tm, rdgCDS.Tag(), value); sendNeeded = true;}
      }

// END BME section
      
      byte datasize = packet.dataSize();  // do we have data? (OK, two bytes is data... but really, just the id byte and low byte of time from Begin call)
      if ((datasize > 2) && sendNeeded) {
        IFECHO led.Blink(30);          // show sending
        //unsigned long closeTime = millis();
        packet.Close((uint8_t*) &HMACkey); // close is expensive, only do this if we really have something to say
        //closeTime = millis() - closeTime;
        nonce = packet.nonce - 1;   // save the packet nonce to see what we get back from hub
        //IFECHO {Serial << F("@") << lp.Seconds() << F("s; closeTime: ") << closeTime << F("ms; Sending "); packet.SerialDisplay();}
        //IFECHO {Serial << F("@") << lp.Seconds() << F("s; Sending "); packet.SerialDisplay();}
//#ifndef SLIMDOWN
//        IFECHO {lp.printFormatted(); Serial << F("Send "); packet.SerialDisplay();}
//#endif
        byte sendsize = packet.sendSize();
        //radio_xmit = true;    // we transmitted
        RADIOSTART;
        sent = radio.sendWithRetry(GATway, packet.Data(), sendsize);
        RADIOSTOP;
        //long packetTime = lp.myMillis() - beginMillis;
        //IFECHO Serial << F("packetTime: ") << packetTime << F("ms") << endl;
        if (sent) {
          sendsanity = 0;     // send was ok
          _rssi = radio.RSSI; // save RSSI if successful sendWithRetry  !!
          LastSuccessfulSend = lp.Seconds();
          rdgPowerVolts.Sent();
          rdgCpuTemp.Sent();
          rdgRadioTemp.Sent();
          rdgRadioRSSI.Sent();
          rdgBMETemp.Sent();
          rdgBMEHumid.Sent();
          rdgBMEPress.Sent();
          rdgBATmv.Sent();

#ifndef SLIMDOWN
          IFECHO Serial << F(" OK.") << endl;
#endif
          led.Blink(3);          // show Send worked
        }
        else {
          sendsanity++;   // count failures
          rdgPowerVolts.Unsent();
          rdgCpuTemp.Unsent();
          rdgRadioTemp.Unsent();
          rdgRadioRSSI.Unsent();
#ifndef SLIMDOWN
          IFECHO Serial << F(" Fail.") << endl;
#endif
          for (byte ix = 0; ix++ < 3;) {
            led.Blink(3);          // show Send failed
            delay(90);
          }
          // should probably up the radio power if we haven't sent successfully for more than 15 minutes
          //if ((lp.Seconds() - lastSuccessfulSend) > 60*15=900) if (radiopower<30) radiopower++; radio.setpower(radiopower)
          // up power if last 20 sends failed, then wait another 20, no need to shout suddenly, also dont get real loud ever
          if ((sendsanity > 20) && (RadioLevel < 20)) {RadioLevel++; radio.setPowerLevel(RadioLevel); sendsanity = 0;}
        }
      }
    }
    return sent;
}

bool savePacket() {
  byte accepted = false;
  if ((radio.DATALEN <= 64) && (radio.DATALEN > 0)) {
    //Serial << "[ getpayload():" << (unsigned long) getPayload() << "]";
    accepted = packet.setPayload((const void *) radio.DATA, radio.DATALEN);
  }
  else {
#ifndef SLIMDOWN
    IFECHO Serial << F(" packet XL.");
#endif
  }
  return accepted;
}

void processRadioInput() {
  byte theSenderID;
  byte theTargetID;
  byte theDatalen;
  //int16_t theRSSI;
  char theRSSI;
  int timeMilliseconds; // what was milliseconds sent down
  if (radio.receiveDone()) {
    theRSSI = radio.RSSI; // very volatile!
    unsigned long currSeconds = lp.Seconds();
    if (theRSSI != -128) rdgRadioRSSI.Add(theRSSI, currSeconds);
    nonce = packet.nonce - 1; // save the nonce
    bool packetSaved = savePacket();
    // this gets lost on reply/ACK!
    theSenderID = radio.SENDERID;
    theTargetID = radio.TARGETID;
    theDatalen = radio.DATALEN;
    if (radio.ACKRequested()) {     // When a node requests an ACK, respond to the ACK
      RADIOSTART;
      radio.sendACK();
      RADIOSTOP;
#ifndef SLIMDOWN
      IFECHO Serial.print(F("ACKd"));
#endif
    }
    IFECHO {
#ifndef SLIMDOWN
      Serial << endl; //<< lp.Seconds();
      lp.printFormatted();
      //Serial << F(" #: ") << (++packetCount) << F("; ");
      Serial << F(" From:") << _HEX(theSenderID); // << F("; ");
      Serial << F(" To:") << _HEX(theTargetID); // << F("; ");
      Serial << F(" L:") << _DEC(theDatalen); // << F("; ");
      Serial << F(" RSSI:") << _DEC(theRSSI); // << F("; ");
#endif
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
      //if (true) {
          //IFECHO packet.SerialDisplay();
          //IFECHO Serial << F("Valid: ") << (packetAccepted ? F("Yes.") : F("No."));
          //IFECHO Serial << endl;
          // TODO: process packet here...
          timeMilliseconds = 500; // reset milliseconds - average of 500 milliseconds if not told otherwise
          for (byte mp = packet.firstPacket(); mp < 0xff ; mp = packet.nextPacket(mp)) {
            byte bminiPacketName = packet.miniPacketName(mp);
            if (bminiPacketName == 0) break;  // no name? done!
            byte bminiPacketLength = packet.miniPacketDataLength(mp);
            byte mp_b = 0;
            char mp_c = 0;
            int mp_int = 0;
            unsigned long mp_ul = 0;
            //IFECHO Serial << F(" [~") << bminiPacketLength << F("]");
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
#ifndef SLIMDOWN
                      IFECHO Serial << F("UTC:") << mp_ul << F(".") << timeMilliseconds; // << F("s"); 
#endif
                      if ((mp_ul) && (mp_ul != lp.Seconds())) {
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
                          LastSend = lp.myMillis();                         // try this, resync to right now
                          nextPeriodmillis = LastSend + transmitperiod;     // recalculate next period from this period
                        }
#ifndef SLIMDOWN
                        IFECHO {Serial << F(" To:"); lp.printSecondsMillis();} // Serial << F("s");}
#endif
                        //timeKnown = true; -> lp.OK()
                      }
                      break;
              case  9: 
#ifndef SLIMDOWN
                      IFECHO Serial << F(" RSSI:") << _DEC(mp_c) << F(" Power:") << RadioLevel;
#endif
                      if (mp_c > -92) {                     // target db level is -92
                        // change power rating down;
                        if (RadioLevel > 10) RadioLevel--;  // fast power down above 10
                        if (RadioLevel > 1) RadioLevel--;   // some power down above 0 - are you sure?
                        radio.setPowerLevel(RadioLevel); // reduce / increase transmit power level
                      }
                      else {                              // if we are really low, bump up the radio level
                        if (mp_c > -126 && mp_c < -98) {
                          // change power rating up;
                          if (RadioLevel < 30) RadioLevel++;
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
#ifndef SLIMDOWN
            IFECHO Serial << " ";
#endif
          }
#ifndef SLIMDOWN
          IFECHO Serial << endl;
#endif
      }
      else {
        IFECHO packet.SerialDisplay();
      }
    led.Blink(3);
    packet.nonce = nonce + 1; // put the nonce back!
  }
}

#define VDUMP(x) Serial << #x << " " << x << endl

// main code here to run forever
void loop() {
  unsigned long currMillis = lp.myMillis();
  long sinceLastSend = currMillis - LastSend;

  // hack - stop sending after 2 hours // DELETE FROM PROD
//  if (currMillis > 7200000UL) {
//    quiet = true;
//  }

  if (sinceLastSend >= transmitperiod) {
    IFECHO Serial << F("\nT") << stackAvailable() << endl;
//    VDUMP(currMillis);
//    VDUMP(LastSend);
//    VDUMP(sinceLastSend);
//    VDUMP(transmitperiod);
    unsigned long currSeconds = lp.Seconds(); //currMillis / 1000UL;
//    VDUMP(currSeconds);
    LastSend = nextPeriodmillis;
//    VDUMP(LastSend);
    nextPeriodmillis += transmitperiod;
//    VDUMP(nextPeriodmillis);
    // next statement is false to prevent successful compile
    //something wrong here - we are looping out when we change transmitperiod
    // take readings, then start a packet if needed
    if (takeReadings(currSeconds)) {
      IFECHO {led.Blink(1); Serial << F("\nR"); lp.printFormatted(); Serial << endl;}          // show send was contemplated, shows still running
      // process readings, old and then new
      bool sent = sendReadings(); // send first set of readings
      if (sent) {
        IFECHO {led.Blink(1); Serial << F("S");}          // show send was contemplated, shows still running
        sent = sendReadings();   // if sent old readings, then try to send send new readings
      }
      if (!sent) {
        //IFECHO Serial << F("@") << currSeconds << F(" Nothing sent.") << endl;
#ifndef SLIMDOWN
        IFECHO {lp.printFormatted(); Serial << F("No Send\n");}
#endif
        IFECHO {led.Blink(1); Serial << F("s");}          // show send was contemplated, shows still running
      }
    }
    else {
#ifndef SLIMDOWN
        //IFECHO Serial << F("@") << currSeconds << F(" No new readings.") << endl;
        IFECHO {lp.printFormatted(); Serial << F("No Data\n");}
#endif
        IFECHO {led.Blink(1); Serial << F("N");}          // show send was contemplated, shows still running
    }
    //radio.SetListen();          // how do we do this?
    //led.Blink(2);          // show send is complete
    // update stats if we sent data, else, data is current anyway
    currMillis = lp.myMillis();                 //Serial << "currmillis:" << currMillis << endl;
    sinceLastSend = currMillis - LastSend;   //Serial << "sincelastsend:" << sinceLastSend << endl;
    //if (timeKnown) timeKnown = lp.OK(); //check if time is still good or if it has timed out - base not available for too long
  }

  // Listen for input? or sleep if we have a chance to sleep...
  if (sinceLastSend < commWatch || comport.Attached()) {    // if we are wired, and it has been less than a few seconds, handle serial input
      //if (radio_xmit) {radio.setMode(RF69_MODE_RX); radio_xmit=false;}  // go to low power // this is a protected function :-(
      processRadioInput();
      IFECHO processSerialInput();
      delay(1);                       // delay some anyway
  }
  else {
    if (sinceLastSend < transmitperiod) {
      long thisDelay =  LastSend + transmitperiod - currMillis; //Serial << "thisdelay:" << thisDelay << endl;
      lp.lowPowerDelay(thisDelay);
    }
    else {
      WirelessHexEnabled = false; // reset OTA  -- when to do this?
      led.BlinkEvery(1, 10000); // say hi
      delay(1); // delay some anyway
    }
  }
}
// END END END END END END END END END END END END END END END END END END END END END END END END END END
