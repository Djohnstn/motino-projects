// MegaGate6   - data hub always on
// J David Johnston  - Oct 2016, April 2017, June 2017
// Gateway - take node info to print
// heavily reworked by JDJohnston from // Library and code by Felix Rusu - felix@lowpowerlab.com

#include "Arduino.h"  // standard headers

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <Streaming.h> // support serial << stuff; http://arduiniana.org/libraries/streaming/
//#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <ShortPacket2.h>  // data packet definitions
#include <BlinkLED.h> // LED Blinker
#include <FTDIAttached.h> // FTDIAttached information about com port being available
#include <db1k.h> // eeprom database
#include <serbuf.h> // serial buffer processing
#include <serconfig.h> // string configuration command line processor
#include <Reading.h>  // readings
#include <LowPowerTime.h>   // low power and mlliseconds functions

#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!

//unsigned int eeprom_size = EEPROM.length();   // 1k uno vs 4k mega


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

boolean flash_attached = true;
#define IFFLASH if (flash_attached)
#define IFNOFLASH if (!flash_attached)


BlinkLED led (LED);       // set up the LED
db1k db;                  // eeprom database
RFM69 radio;              // radio
ShortPacket2 packet;      // radio packet
ShortPacket2 repacket;      // radio response packet
LowPowerTime lp;          // ==== time and power ===

// might need to do something about hangs if this device not attached. jdj 4/2017

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

// radio support
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network
bool listenMode = true; //set to 'false' to quit listening to the network
byte ackCount = 0;
byte powerLevel = 4; // radio power level starts at mid to low level
uint32_t packetCount = 0;

// serial support 115200 has -3.5% off of proper baud, 76800 is +0.2% off of proper baud, http://wormfood.net/avrbaudcalc.php 38400 is also +0.2%
//#define SERIAL_BAUD 115200  
//#define SERIAL_BAUD 76800
#define SERIAL_BAUD 38400
serbuf SB;
serconfig scfg;

// system settings to get from database
unsigned long ENcrytkey[4] = {1, 2, 3, 4} ; // E1xE2xE3xE4x // 16 bytes
unsigned long HMACkey[4] =   {736724401, 902293479, 296437593, 109279389} ; // H1xH2xH3x // 16 bytes
unsigned long transmitperiod = 32000UL;
unsigned long unsureperiod = 15000UL; // if unsure of time, display readings every this many milliseconds until sure of time.
unsigned long lastSyncSeconds = 0UL;  // last time we saw the time updated
byte DATaset = '1'; // data set to process from
byte NETworkid = 1; // NET 
byte NOdid = 0xb1; //NOn node number
byte GATway = 1;    // GAT
byte FREquency = RF69_915MHZ; // FRE
//byte timeKnown = false;         // has time been synced?  // only of last time sync was within 2 seconds of current time (ie: 2 time sets) // moved to lp.OK()LowPowerTime
byte timeSyncReadingNumber = 0; // number of displays since last time read back to system

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

void printTime() {
  IFECHO {Serial << F("[") << NOdid << F("] ZT:"); lp.printSecondsMillis(); Serial << F(" Syncd:") << lp.OK() << F(" RunMs:") << lp.myMillis() << endl;} // << F(" ms") << endl;}
}

void printCompileVersion() {
  IFECHO {Serial << F("?#" __FILE__ ", " __DATE__ ", " __TIME__ "#") << endl;}
}

void processSerialInput() {
  IFECHO while (Serial.available()) {
    unsigned long seconds = 0;
    int millisecondOffset = 0; // current millisecond base
    char millisecondix = 2;   // index for millisecond offset 2 = 100, 1 = 10, 0 = 1, -1 = skip it
    int counter = 0;
    unsigned long jedecid = 0;
    bool digits;
    byte pwr=0;
    if (SB.readline() > 0)  {
        Serial << F(">") << SB.serBuff << F("<") << endl;
        char input = SB.getCh();  // peek at first letter of input // get command
        switch (input) {
          case '!':
            scfg.settingsCommand(SB, db, comport.Attached());
            break;
          case '?':
            Serial << F("? !var $hex #num \"text\" / !x (del) / t newtime / T") << endl;
            Serial << F("? [D]bdump / [V]ersion / [r]adio registers / [p]romiscous mode /d flash dump / [i]d flash /L1-31 radiopower /q Radio Listen On/Off") << endl;
            break;
          case 'D':
            db.dumpdb();
            break;
          case 'L':
            for (char ch = SB.getCh(); ch != 0; ch = SB.getCh()) 
            {
              if (isdigit(ch)) pwr = (pwr * 10) + ch - '0';
            }
            if (pwr) {powerLevel=pwr; radio.setPowerLevel(powerLevel); Serial << F("#Set Radio Power.\n");}    
                // radio power level // ? radiohead: For RF69W, valid values are from -18 to +13
            break;
          case 't':           // set/show time
          case 'T':           // set/show time
            digits = true; // before the decimal
            for (char ch = SB.getCh(); ch != 0; ch = SB.getCh()) 
            {
              //Serial << ch; // debugging to see what we got
              if ('.' == ch) digits = false;
              if (digits) {
                if (isdigit(ch) && digits) seconds = (seconds * 10) + ch - '0';
              }
              else {
                if (isdigit(ch)) {
                  int base = -1;
                  if (2 == millisecondix) {base = 100;}
                  if (1 == millisecondix) {base = 10;}
                  if (0 == millisecondix) {base = 1;}
                  if (millisecondix >= 0) {millisecondOffset += (ch - '0') * base;}
                  millisecondix--;
                }
              }
            }
            if (seconds > 0UL) {
              unsigned long oldtime = lp.Seconds();
              IFECHO Serial << F("# SetTime:");
              if (digits) {
                IFECHO Serial << seconds;
                lp.Seconds(seconds);  // set time // no filter needed - what about after Tue Jan 19 03:14:07 2038 
              }
              else {
                lp.Seconds(seconds, millisecondOffset);  // set time // no filter needed - what about after Tue Jan 19 03:14:07 2038 // using 32 bit time not 31 bit time
                IFECHO lp.printSecondsMillis(); // proper formatted milliseconds
              }
              long deltatime = (long)(seconds - oldtime);
              //timeKnown = abs(deltatime) < 3;
              Serial << F(" mSpS:") << lp.MillisPerSec << F(" Syncd:") << lp.OK() << F(" DiffS:") << deltatime << endl; 
              if (lp.OK()) {
                lastSyncSeconds = seconds;  // save the last time we got the right time.
                if (unsureperiod < 1800000UL) unsureperiod += unsureperiod / 4; // if we got the time, the back off on transmits;
                //if (unsureperiod > 1200000UL) unsureperiod = 1800000UL; // max unsure is about 30 minutes
                timeSyncReadingNumber = 0;  // reset readingnumber display
              }
              else {
                if (unsureperiod > 22000UL) unsureperiod -= unsureperiod / 4; // if we not got the time, the speed up on transmits;
                //if (unsureperiod > 20000UL) unsureperiod = 20000UL; // minimum unsure is about 20 seconds
              }
            }
            printTime();
            break;
          case 'V':
            printCompileVersion();
            break;
          case 'r': //r=dump radio registers
            radio.readAllRegs();
            break;
          case 'p': //promiscous mode
            promiscuousMode = !promiscuousMode;
            radio.promiscuous(promiscuousMode);
            IFECHO Serial << F("# PromiscuousMode:") << (promiscuousMode ? F("on") : F("off"));
            break;
          case 'q': //quiet mode
            listenMode = !listenMode;
            IFECHO Serial << F("# ListenMode:") << (listenMode ? F("on") : F("off"));
            break;
          case 'd': // dump flash area
            Serial.println(F("# Flash content:"));
            IFNOFLASH {
              Serial.print(F("# No Flash;"));
            }
            else {
              while(counter<=256)
              {
                Serial.print(flash.readByte(counter++), HEX);
                Serial.print('.');
              }
              while(flash.busy());
            }
            Serial.println();
            break;
          case 'i': // device id
            Serial.print(F("# DeviceID: "));
            IFNOFLASH {
              Serial.print(F("# No Flash;"));
            }
            else {
              jedecid = flash.readDeviceId();
              IFECHO Serial.println(jedecid, HEX);
              flash.readUniqueId();
              for (byte i=0;i<8;i++)
              {
                IFECHO Serial << _HEX(flash.UNIQUEID[i]) << ((i<7) ? F(":") : F("]\n"));
              }
            }
            break;
          case 'F': // erase flash
            IFFLASH {
              Serial.print(F("# Deleting Flash chip ... or not... "));
              flash.chipErase();
              while(flash.busy());
              Serial.println(F("# DONE"));
            }
            else {
              Serial.println(F("# NO FLASH"));
            }
            break;
          default:
            Serial << endl << F("# Unknown: ") << _HEX(input) << endl;
            break;
        }
    }
    //led.Blink(10);          // show serial processing is complete
  }
}

bool savePacket() {
  byte accepted = false;
  if ((radio.DATALEN <= 64) && (radio.DATALEN > 0)) {
    accepted = packet.setPayload((const void *) radio.DATA, radio.DATALEN);
  }
  else {
    IFECHO Serial << F(" packet len < 1 or > 64.");
  }
  return accepted;
}

//int readVcc() { // Voltage for the Cpu
//  long result;
//  int iresult;
//  // Read 1.1V reference against AVcc
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  delay(1); // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC); // Convert
//  while (bit_is_set(ADCSRA,ADSC));
//  result = ADCL;
//  result |= ADCH<<8;
//  result = 1126400L / result; // Back-calculate AVcc in mV
//  if (result > 32767L) {  // overflow?
//    iresult = 32767;
//  }
//  else {
//    iresult = result;
//  }
//  return iresult;
//}

int readVcc() {   // from jeelabs : http://jeelabs.org/2012/05/04/measuring-vcc-via-the-bandgap/index.html
  analogRead(6);  // set up almost the proper ADC readout
  bitSet(ADMUX, 3); // then fix it to switch to channel 14 - internal vcc reference
  delayMicroseconds(250);  //delay substantially improves accuracy
  bitSet(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;                               // do nothing but go back and check
  word x = ADC;
  return x ? 1125300L / x : 0;     // 1125300 = 1100L * 1023  //i choose zero instead of -1 (jdj) for fault
}

int readPowermVolts(){ // reads Power supply voltage 5 times, one for discard, 4 to average
  int iVcc2 = 0;
  int ic = 0;
  readVcc();  // discard first read
  for (int ix = 0; ic < 2 && ix < 3; ix++) {
    int iVcc1 = readVcc(); //  read internal Vcc millivolts values, twice each, to allow for debounce of ADC
    //iVcc2 += iVcc1;
    if (iVcc1 > 0 && iVcc1 < 15000) {
      iVcc2 += iVcc1;
      ic++;
    }
    else {
      delayMicroseconds(500);  // if it failed, give it another brief rest
    }
  }
  int iVccx = ic ? iVcc2 / ic : 0;    // if nothing, don't average it
  return iVccx;
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

int CpuTemp(){ // reads CPU temp 5 times, one for discard, 4 to average, or 3 times?
  int iTemp2 = 0;
  readCpuTemp();  // discard first read
  int ic = 0;
  for (int ix = 0; ic < 2 && ix < 3; ix++) {
    int iTemp1 = readCpuTemp(); //  read internal Vcc millivolts values, twice each, to allow for debounce of ADC
    if (iTemp1 > 0) {
      iTemp2 += iTemp1;
      ic++;
    }
    else {
      delayMicroseconds(500);  // if it failed, give it a brief rest
    }
  }
  int iTempx = ic? iTemp2 / ic: 0;    // if nothing, dont average it
  return iTempx;
}

byte RadioTemperature() { // degrees C -128 - +127
  //radiosleeping = false;
  byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
  if (-128 == temperature) temperature = radio.readTemperature(-1); // try again if get -128?
  return temperature;
}

unsigned long LastSend;
unsigned long nextPeriodmillis;

// 1 = 4 byte time
// 2 = 2 byte lower time
// 3 = 1 byte lowest time

Reading <unsigned long> rdgTime(4);
//Reading <int> rdgPowerVolts(8);
//Reading <int> rdgCpuTemp(11);
//Reading <byte> rdgRadioTemp(10);
Reading <byte> rdgRadioRSSI(9);
//Reading <int> rdgBMETemp(17);   // tempC
//Reading <int> rdgBMEHumid(18);  // humidity
//Reading <int> rdgBMEPress(19);  // pressure
//Reading <int> rdgBATmv(20); // battery
//Reading <int> rdgCDS(20);   // cadmium cell

char _rssi; // save rssi after a receive
int myRSSI = -127; // save the rssi of remote sent back to base

bool initNetworkAdapter() {
  // setup data parameters
  DATaset = db.getByte('d','a','t'); // which is active dataset?
  //Serial << F("byte FREquency = RF69_915MHZ; // FRE ") << FREquency << endl;
  FREquency = db.getByte('f','r',DATaset); // frequency
  GATway = db.getByte('G','A',DATaset);   // gateway id
  NETworkid = db.getByte('n','e',DATaset); // network id
  NOdid = db.getByte('N','O',DATaset);   // nodeid id
  ENcrytkey[0] = db.getLong('E','0', DATaset); // encrypt key 0
  ENcrytkey[1] = db.getLong('E','1', DATaset); // encrypt key 1
  ENcrytkey[2] = db.getLong('E','2', DATaset); // encrypt key 2
  ENcrytkey[3] = db.getLong('E','3', DATaset); // encrypt key 3
  HMACkey[0]   = db.getLong('H','0', DATaset); // HMAC    key 0
  HMACkey[1]   = db.getLong('H','1', DATaset); // HMAC    key 1
  HMACkey[2]   = db.getLong('H','2', DATaset); // HMAC    key 2
  HMACkey[3]   = db.getLong('H','3', DATaset); // HMAC    key 3
  transmitperiod = db.getLong('T','P', DATaset);  // transmit period
  if (transmitperiod < 2000UL) transmitperiod = 5000UL; // set minimum transmit period
  lp.transmitperiod = transmitperiod;     // copy time max  // i know, poor design, to duplicate data
  radio.initialize(FREquency, NOdid, NETworkid);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt((char*)ENcrytkey);
  radio.setPowerLevel(powerLevel);    // radio power level // ? radiohead: For RF69W, valid values are from -18 to +13
  radio.promiscuous(promiscuousMode);
  lp._radio = &radio;
  lp.sleepRadioAndFlash();
  IFECHO Serial << F("\n# RF:") << (FREquency==RF69_433MHZ ? 433 : FREquency==RF69_868MHZ ? 868 : 915) << F(" Mhz.\n") ;
  _rssi = -128;
  return true;
}

bool initFlash() {

#ifdef __AVR_ATmega1284P__
  // hard code for now
  flash_attached = false;
  return false;
#else
  //Serial << "A"; delay(15);
  flash_attached = flash.initialize();
  IFFLASH {
    //Serial << "B"; delay(15);
    IFECHO Serial.print(F("SPI Flash Init OK. Unique MAC = ["));
    IFFLASH flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      IFECHO Serial << _HEX(flash.UNIQUEID[i]) << ((i<7) ? F(":") : F("]\n"));
    }
    flash.sleep();
    return true;
  }
  else {
      //Serial << "C"; delay(15);
      IFECHO Serial << F("#Fail SPI Flash chip Init.\n");
      return false;
  }
#endif
}

//byte timeSyncReadingNumber = 0;
long lastLocalReading;
void localReadings(unsigned long everyMS) {
  long nowMillis = millis();
  if ((nowMillis - lastLocalReading) >= everyMS) {  // this logic accounts for wrap-around of unsigned long millis at 47 days
    lastLocalReading = ++nowMillis;
    unsigned long currSeconds = lp.Seconds();
    byte radioTemp = RadioTemperature();
    int cpuTemp = CpuTemp();
    int cpumVolts = readPowermVolts();

    IFECHO {
      Serial << F("[") << NOdid << F("] ZT:") << currSeconds << F(" C:") << radioTemp; // << F(";");
      Serial << F(" CpuC:") << cpuTemp;
      Serial << F(" mV:") << cpumVolts;
      Serial << F(" Syncd:");
      //if ((long)(lp.Seconds() - lastSyncSeconds) > 7200UL) {timeKnown = false;} // if we have not found time in an hour, then we dont know...
      //if (timeKnown) timeKnown = lp.OK(); //check if time is still good or if it has timed out - base not available for too long
      if (timeSyncReadingNumber > 19) {
        Serial << 0;    // force a time check every 20 printouts if not otherwise checked.
      }
      else {
        timeSyncReadingNumber++;
        Serial << lp.OK(); // display true value of sync status
      }
      Serial << F(" [RSSI:") << myRSSI << F("]\n");
    }
  }
}

// put your setup code here, to run once:
void setup() {
  db.begin();             // initialise database
  //IFECHO 
  Serial.begin(SERIAL_BAUD);
  printCompileVersion();
  IFECHO Serial << F("# CONNECTED.");
  //IFNOECHO Serial << F("# NO COMM.");
  Serial << 1;
  initFlash();            // set up flash
  Serial << 2;
  initNetworkAdapter();   // set up network
  Serial << 3;
  nextPeriodmillis = 0;// transmitperiod;         // set up to start send asap
  LastSend = nextPeriodmillis - transmitperiod;   // set up to start transmit asap
  Serial << 4;
  IFECHO packet.SpSetVerbose(true);
  Serial << 5;
  IFECHO repacket.SpSetVerbose(true);
  Serial << 6;
  lp.Seconds(0UL);   // hard code to zero time - host will fix time in a few seconds
  led.Blink(2);          // show Setup() is complete
  Serial << 7 << endl;
  //Serial << F("# EEPROM size: ") << EEPROM.length() << endl; // 4K in the ATMega1284P
  printTime();  // show time so it can be corrected
}

//bool takeReadings(unsigned long currSeconds) {
//    bool newdata = false; // is there new data?
//    if (rdgPowerVolts.Add(readPowermVolts(), currSeconds)) newdata = true;
//    if (rdgRadioTemp.Add(RadioTemperature(), currSeconds)) newdata = true;
//    if (rdgRadioRSSI.Add(_rssi, currSeconds)) newdata = true;
//    if (rdgCpuTemp.Add(CpuTemp(), currSeconds)) newdata = true;
//    return newdata;
//}

//bool sendReadings() {
//    long beginMillis = lp.myMillis();
//    unsigned long currSeconds = beginMillis / 1000UL;
//    // begin packet
//    packet.Begin(currSeconds);
//    boolean sent = false;
//    {
//      // Serial << F("stackavailable:") << stackAvailable() << endl; //F(" heapavailable:") << heapAvailable() << endl;
//      if (rdgPowerVolts.dataChanged()) 
//      {
//        IFECHO Serial << F("T: ") << rdgPowerVolts.GetTime() << F(", P: ") << _HEX(rdgPowerVolts.Tag()) << F(", Vcc: ") << rdgPowerVolts.GetData() << F("mV") << endl;
//        packet.Add(rdgPowerVolts.GetTime(), rdgPowerVolts.Tag(), rdgPowerVolts.GetData());
//      }
//      if (rdgCpuTemp.dataChanged()) 
//      {
//        unsigned int tm = rdgCpuTemp.GetTime();
//        char temp = rdgCpuTemp.GetData();
//        //int tempF = tempC * 9 / 5 + 32;
//        IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(rdgCpuTemp.Tag()) << F(", CpuTemp: ") << (int)temp << F("C") << endl;
//        packet.Add(tm, rdgCpuTemp.Tag(), temp);
//      }
//      if (rdgRadioTemp.dataChanged()) 
//      {
//        unsigned int tm = rdgRadioTemp.GetTime();
//        char tempC = rdgRadioTemp.GetData();
//        IFECHO Serial << F("T: ") << tm << F(", P: ") << _HEX(rdgRadioTemp.Tag()) << F(", RadioTemp: ") << (int)tempC << F("C") << endl;
//        packet.Add(tm, rdgRadioTemp.Tag(), tempC);
//      }
//      if (rdgRadioRSSI.dataChanged()) 
//      {
//        IFECHO Serial << F("T: ") << rdgRadioRSSI.GetTime() << F(", P: ") << _HEX(rdgRadioRSSI.Tag()) << F(", RSSI: ") << rdgRadioRSSI.GetData() << F("db") << endl;
//        packet.Add(rdgRadioRSSI.GetTime(), rdgRadioRSSI.Tag(), rdgRadioRSSI.GetData());
//      }
//      byte datasize = packet.dataSize();
//      if (datasize) {
//        packet.Close((uint8_t*) &HMACkey); // close is expensive, only do this if we really have something to say
//        IFECHO packet.SerialDisplay();
//        byte sendsize = packet.sendSize();
//        sent = radio.sendWithRetry(GATway, packet.Data(), sendsize);
//        _rssi = radio.RSSI; // save RSSI
//        //long packetTime = lp.myMillis() - beginMillis;
//        //IFECHO Serial << F("packetTime: ") << packetTime << F("ms") << endl;
//        if (sent) {
//          rdgPowerVolts.Sent();
//          rdgCpuTemp.Sent();
//          rdgRadioTemp.Sent();
//          rdgRadioRSSI.Sent();
//          IFECHO Serial << F("Send worked.") << endl;
//          led.Blink(3);          // show Send worked
//        }
//        else {
//          rdgPowerVolts.Unsent();
//          rdgCpuTemp.Unsent();
//          rdgRadioTemp.Unsent();
//          rdgRadioRSSI.Unsent();
//          IFECHO Serial << F("Send failed.") << endl;
//          for (byte ix = 0; ix++ < 3;) {
//            led.Blink(10);          // show Send failed
//            delay(80);
//          }
//        }
//      }
//    }
//    return sent;
//}

// sendResponse // sends back RSSI and current time // might send back new configuration settings
bool sendResponse(byte theSenderID) {
    unsigned long currSeconds = lp.Seconds();
    int currMillis = lp.SecondsMillis();
    bool sent = false;
    bool tosend = false;

    //IFECHO Serial << F("\n>@"); 
    repacket.nonce = packet.nonce; // copy nonce to output, to prove it's really us!
    repacket.Begin(currSeconds);

    // update node time of time by adding a zero * 
    if ((currSeconds != packet.Time()) && lp.OK()) { // is current real time different than received time? and is current time valid?
      IFECHO {Serial << F(">ZT:");  lp.printSecondsMillis(); Serial << F("Update NodeTime:") << packet.Time() << F(" Diff:") << (long)(currSeconds - packet.Time()) << endl;}// << F("s") << endl;
      repacket.Add(currSeconds, 3, currMillis);  // what milliseconds time is it? // might send only if node doesn't match current time
      repacket.Add(currSeconds, rdgTime.Tag(), currSeconds);  // what time is it? // might send only if node doesn't match current time
      tosend = true;
    }
    //Serial << F("stackavailable:") << stackAvailable() << endl; //F(" heapavailable:") << heapAvailable() << endl;
    if (_rssi > -95) {
      IFECHO {Serial << F(">ZT:");  lp.printSecondsMillis(); Serial << F("Update RSSI:") << _DEC(_rssi) << endl;} // << F("db") << endl;
      repacket.Add(currSeconds, rdgRadioRSSI.Tag(), _rssi);
      tosend = true;
    }

    byte datasize = repacket.dataSize();
    if (tosend) {
      printTime();
      IFECHO Serial << F("#Updating [") << _DEC(theSenderID) << F("] ");
      repacket.Close((uint8_t*) &HMACkey); // close is expensive, only do this if we really have something to say
      IFECHO repacket.SerialDisplay();
      byte sendsize = repacket.sendSize();
      sent = radio.sendWithRetry(theSenderID, repacket.Data(), sendsize, 3, 40); // send values back to node, 3 times, 80 ms timeout
      if (sent) {
        _rssi = radio.RSSI; // save RSSI
        IFECHO Serial << F(">Update:done") << endl;
        led.Blink(2);          // show Send worked
      }
      else {
       _rssi = -128;  // no rssi
       IFECHO Serial << F(">Update:Fail") << endl;
       for (byte ix = 0; ix++ < 3;) {
          led.Blink(10);          // show Send failed
          delay(80);
      }
    }
  }
  return sent;
}

// main code here to run forever
//void loop() {
//  unsigned long currMillis = lp.myMillis();
//  unsigned long sinceLastSend = currMillis - LastSend;
//
//  if (sinceLastSend >= transmitperiod) {
//    unsigned long currSeconds = currMillis / 1000UL;
//    LastSend = nextPeriodmillis;
//    nextPeriodmillis += transmitperiod;
//
//    // take readings, then start a packet if needed
//    if (takeReadings(currSeconds)) {
//      // process readings, old and then new
//      bool sent = sendReadings(); // send first set of readings
//      if (sent) sendReadings();   // if sent old readings, then try to send send new readings
//    }
//    //led.Blink(2);          // show send is complete
//    radio.sleep();    // put radio to sleep, save power, cool it down.
//  }
//
//}

// ======================================== GATEWAY ======================================

byte processRadioInput() {
  byte theSenderID = 0;
  byte theTargetID;
  byte theDatalen;
  int16_t theRSSI;
  //int16_t myRSSI = -127;
  if (radio.receiveDone()) {
    theRSSI = radio.RSSI; // very volatile!
    unsigned long currSeconds = lp.Seconds();
    rdgRadioRSSI.Add(theRSSI, currSeconds);
    bool packetSaved = savePacket();  // save the data from the radio buffer to the packet handler
    // this gets lost on reply/ACK!
    theSenderID = radio.SENDERID;
    theTargetID = radio.TARGETID;
    theDatalen = radio.DATALEN;
    if (radio.ACKRequested()) {
      // When a node requests an ACK, respond to the ACK
      radio.sendACK();
      IFECHO Serial.println(F("#[->ACKd]"));
      ackCount++;
    }

    IFECHO {
      //Serial << F("@"); 
      printTime();
      Serial << F("#Packet:") << (++packetCount); // << F("; ");
      Serial << F(" [") << _DEC(theSenderID); // << F("; ");
      Serial << F("] [") << _DEC(theTargetID); // << F("; ");
      Serial << F("] Len:") << theDatalen; // << F("; ");
    }
    ShortPacket2::status packetAccepted = packet.validPayload(currSeconds, (uint8_t*) &HMACkey); //validatePacket(packetSaved);
    
    IFECHO {
      //Serial.println();
      Serial << F(" Valid:") << _DEC(packetAccepted) << endl;
      if (0 != packetAccepted) {  // bad packet - REPORT FOR DEBUG PURPOSES
        packet.SerialDisplay();
      }
      else {                      // GOOD PACKET HANDLE IT
        unsigned long ptime = 0;
        boolean mp_time = false;
        long value = 0;
        boolean isRSSI = false;
        if (packet.sPayload.len == 0) packet.sPayload.len = 49; // if packet length isn't valid, go ahdead and dump it all
        // TODO: process packet here...
        // sample received packet header
        //IFECHO Serial << F("!@:") << NETworkid << F(".") << theSenderID << F(";");
        IFECHO Serial << F("[") << theSenderID << F("]");

        // now, print each mini packet:  tag and property value
        //byte theMiniPacket = packet.firstPacket();
        for (byte mp = packet.firstPacket(); mp < 0xff ; mp = packet.nextPacket(mp)) {
          byte bminiPacketName = packet.miniPacketName(mp);
          if (bminiPacketName == 0) break;  // no name? done!
          byte bminiPacketLength = packet.miniPacketDataLength(mp);
          //Serial << F("[") << bminiPacketName << F(" ");
          mp_time = false;
          switch (bminiPacketName) {
            case 1:                   // types of time
            case 2:
            case 3: 
            case 4: 
                     Serial << F(" ZT:");
                     //Serial << F("Time"); 
                     //ptime = currSeconds;  // set packet time
                     mp_time = true;
                     break;
            case  8: Serial << F(" mV:"); break;   // battery millivolts
            case 11: Serial << F(" CpuC:"); break;
            case 10: Serial << F(" rC:"); break;      // use RadioC a proxy for C reading.
            case  9: isRSSI = true;
                     Serial << F(" BaseRSSI:"); 
                     break;
//Reading <int> rdgBMETemp(17);   // tempC
//Reading <int> rdgBMEHumid(18);  // humidity
//Reading <int> rdgBMEPress(19);  // pressure
//Reading <int> rdgBATmv(20); // battery
//Reading <int> rdgCDS(20);   // cadmium cell
            case 17: Serial << F(" C:"); break;    // temperature
            case 18: Serial << F(" H:"); break;    // humidity 
            case 19: Serial << F(" P:"); break;    // pressure ?
            case 20: Serial << F(" WMV:"); break;  // weather millivolts 
            case 21: Serial << F(" Light:"); break; // CDS light level
            
            default: //Serial << F("?"); break;       // other reading, tag not known yet
                     IFECHO Serial << F(" _M") << _DEC(bminiPacketName) <<  F(":"); break;
          }
          //Serial << F(" ~") << bminiPacketLength << F("]: ");
          switch (bminiPacketLength) {
            case 1: if (mp_time) {ptime = (currSeconds & 0xffffff00UL) | (value=(unsigned char)packet.miniPacketValue_c(mp));}
                    else {Serial << _DEC(value=(char)packet.miniPacketValue_c(mp));}
                    break;
            case 2: if (mp_time) {ptime = (currSeconds & 0xffff0000UL) | (value=(unsigned int)packet.miniPacketValue_i(mp));}
                    else {Serial << (value=packet.miniPacketValue_i(mp)); }
                    break;
            case 4: if (mp_time) {ptime = packet.miniPacketValue_L(mp);}
                    else {Serial << (value=packet.miniPacketValue_L(mp)); }
                    break;
            default: break;
          }
          if (mp_time) {
            //Serial << F("[T:") << ptime << F("]");
            IFECHO Serial << ptime;
          }
          else if (isRSSI) {
            myRSSI = value;
          }
          else {
            
          }
          //Serial << F(";");
        }
        IFECHO Serial << F(" [RSSI:") << (theRSSI) << F("]\n");
        //IFECHO Serial << F("\n");  // end of line
        //Serial << endl;
      }
    }
    led.Blink(2);
    // finished processing receipt
    // also send a packet updating the node
    // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
    if ((ackCount % 3 == 0) || (0 != packetAccepted)) {
      return theSenderID;
    }
  }
  return 0; // nobody there
}


// queue size should be kept small
#define REQSIZE 5
unsigned long respondtime = 0;  // when do we respond?
byte respondTo[REQSIZE] = {0};  // list of nodes gateway will talk back to
byte responding = 0;            // currently responding to this one

// send results back out, if anything is needed
void processRadioOutput(byte sender) {
  if (sender) {
    for (int ix=0; ix<REQSIZE; ix++) {
      if (respondTo[ix] == sender) sender = 0;  // if already in queue, skip it
    }
    for (int ix=0; ix<REQSIZE; ix++) {
      if (!respondTo[ix]) respondTo[ix] = sender;
    } // save sender in first open slot, if too many, will ignore new request. overrun.
    respondtime = millis() + 70UL;  // wait 120 milliseconds to respond, in case of a two-fer from the node
    //IFECHO Serial << F("Re: ") << _HEX(sender) << F(" > ") << respondtime << endl;
    IFECHO Serial << F("# Re:") << _DEC(sender) << endl;
  }
  else {  // no sender, go ahead and update node - nodes wait about a second for a response... at least for now
    if (responding) {
      if (millis() > respondtime) { // error prone respond time // if wraparound (every 47 days), we'll skip the response. oh well.
        sendResponse(responding); // send radio response
        for (int ix=0; ix<REQSIZE; ix++) {
          if (respondTo[ix] == responding) respondTo[ix] = 0;  // if we just processed it, and its still in the queue, skip second response it
        }
        responding = 0;           // no current responding, go around the loop again
        boolean newpower = false;  // change power level on radio, if needed
        if (myRSSI > -80 && powerLevel >  2) {powerLevel--; newpower = true;}    
        if (myRSSI > -127 && myRSSI < -95 && powerLevel < 30) {powerLevel++; newpower = true;}    
        if (newpower) {
          radio.setPowerLevel(powerLevel); 
          IFECHO Serial << F("#Set RadioPower:") << powerLevel << endl;
        }
      }
    }
    else {  // find someone to respond to, no current responding node
      for (int ix=0; ix<REQSIZE; ix++) {
        if(respondTo[ix]) {
          responding = respondTo[ix];   // next talk to node
          respondTo[ix]=0;              // and node has been dequeued
        }
      } // save sender in first slot, empty the slot
    }
  }
}


void loop() {
  //IFECHO 
  processSerialInput();  //process any serial input // serial data handled at end of loop /?
  
  byte sender = 0;
  if (listenMode) {
    sender = processRadioInput();  // listen hard for radio input.
  }
  processRadioOutput(sender); // check on sender and also no sender situation, always call
  
  if (led.BlinkEvery(2, 20000)) { // say hello every 20000 milliseconds
    //Serial << ".";
  }
  else{   // print readings every 'transmitperiod' milliseconds
    if ((unsureperiod < transmitperiod)) {
      localReadings(unsureperiod);
    }
    else {
      localReadings(transmitperiod);
    }
    delay(1);                       // or reduce cpu load in case of overclocking
  }
}


