// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
//#define SERIAL_BUFFER_SIZE 64  // default is 64 - somehow, there is a way to set this without hacking the AVR tree...

#include <avr/pgmspace.h>

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <Streaming.h> // support serial << stuff; http://arduiniana.org/libraries/streaming/
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <CRC16.h>    // CRC 16 from internet
#include <ShortPacket.h>  // data packet definitions


#include "wirelessnetworksetup.h"
#define NODEID        1    //unique for each node on same network

//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define SERIAL_BAUD   115200

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif


//unsigned char traffic_IX = 0;  // #[L^ - 3rd position is zeroth in array, max of 57


RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

// time management ===================
unsigned int currentepoch;    // what 47 day epoch are we in?
unsigned long currentmillis;  // what is current millis for readings
unsigned long currentunixepoch;  // old seconds since 1-1-1970 UTC
unsigned long currentunixmillis;  // seconds sinec 1-1-1970 UTC
unsigned int baseepoch;      // base- readings from when time was set - miniepoch is 47 day intervals
unsigned long basemillis;    // base- milliseconds for base unix time
unsigned long baseunixepoch;  // seconds since 1-1-1970 UTC
unsigned long baseunixmillis;  // seconds since 1-1-1970 UTC
unsigned long clockdriftCpuMillis;    // how many CPU millis are base to add per how many millis
unsigned long clockdriftClockMillis;    // how many clock millis to add per how many millis
unsigned int oldepoch = -1;      // old- readings from when time was set - miniepoch is 47 day intervals, use for seconds per milliseconds
unsigned long oldmillis;    // old- milliseconds for base unix time
unsigned long oldunixepoch;  // old seconds since 1-1-1970 UTC
unsigned long oldunixmillis;  // old seconds since 1-1-1970 UTC

unsigned int serialepoch;    // what 47 day epoch are we in? for serial reads
unsigned long serialmillis;  // what is timeout millis for serial reads

unsigned int seroutepoch;    // what 47 day epoch are we in? for serial output
unsigned long seroutmillis;  // what is timeout millis for serial output

// end time management ===============


byte ackCount=0;
uint32_t packetCount = 0;

const char PROGMEM programID[] = __FILE__ ", " __DATE__ ", " __TIME__ " \0";

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);  // give serial and radio time to warm up
  Serial << programID << endl;
  Serial << F(__FILE__ ", " __DATE__ ", " __TIME__ "*");
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  //radio.setPowerLevel(0);    // radio power level // ? radiohead: For RF69W, valid values are from -18 to +13
  radio.encrypt(ENCRYPTGUID);
  radio.promiscuous(promiscuousMode);
  //radio.setFrequency(919000000);
  char buff[50];
  Serial << F("Listening at ") << (FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915) << F(" Mhz...\n") ;
  if (flash.initialize())
  {
    Serial.print(F("SPI Flash Init OK. Unique MAC = ["));
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial << _HEX(flash.UNIQUEID[i]) << ((i<7) ? F(":") : F("]\n"));
    }
    //Serial << F("]\n");
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  Serial.print(MAC[i], HEX);
    //  Serial.print(' ');
    //}
    
  }
  else
    Serial << F("SPI Flash Init FAIL! (is chip present?)\n");
}

// serial readline
int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;
  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  if (pos >= len - 1) {
    //line is long enough, return it all
    rpos = pos;
    pos = 0;
    return rpos;
  }
  else {
    // No end of line has been found, so return -1.
    return -1;
  }
}

// process time command string
void process_time_command(int serlen, char* serbuffer) 
{
  if (serlen > 11) {
    //Serial << F("Time: ") << serbuffer << F(";\n"); // debug echo
    int timeparts[6]; //t hh:mm:ss mm-dd-yyyy
    int timex = -1;  // leading 't' masks first value
    int value = 0;
    for (int ix = 0; ix <= serlen; ix++) {
      if (isdigit(serbuffer[ix])) {
        value = value * 10 + serbuffer[ix] - '0';
      }
      else {
        if (timex > -1) {
          timeparts[timex] = value;
        }
        value = 0;
        timex++;
      }
    }
    if (timex > 5) { 
      // save current time
      unsigned long thismillis = millis();
      baseepoch = currentepoch + (thismillis < currentmillis ? 1 : 0);
      currentepoch = baseepoch;
      currentmillis = thismillis;
      basemillis = currentmillis;
      // turn time into a number
      if (timeparts[5] < 15) timeparts[5] = 15; // fix year
      if (timeparts[5] < 100) timeparts[5] += 2000; // fix century
      unsigned long fyear = timeparts[5] - ((timeparts[3]<=2)?1:0);  // functional year
      unsigned long gmonth = timeparts[3] + ((timeparts[3]<=2)?13:1); // functional month
      unsigned long daynum = (1461ul * fyear / 4ul); //Serial << F("daynumY:") << daynum; 
      daynum += (153ul * gmonth / 5ul); //Serial << F(" daynumYM: ") << daynum;
      daynum += timeparts[4] - 719606ul;  //Serial << F(" daynumYMD: ") << daynum; // - 719606;  // calc day to unix day number offset is 719606
      unsigned long unixtime = (daynum * 86400ul) + (timeparts[0] * 3600ul + timeparts[1] * 60ul + timeparts[2]); // unix second
      //Serial << F("timeparts ") << timeparts[0] << ":" << timeparts[1] << ":" << timeparts[2] << " " << timeparts[3] << "/"  << timeparts[4] << "/" << timeparts[5] << "\n";
      //Serial << F("fyear: ") << fyear << F(" gmonth: ") << gmonth << "\n";
      //Serial << F("Day: ") << daynum << F(" unixtime: ") << unixtime << "\n";
      Serial << F("Uxtime: ") << unixtime;
      //Serial << F("check out following for sure\n");
      unsigned long uxtimehi;
      unsigned long uxtimelo;
      uxtimehi = (unixtime >> 16) * 1000ul;  // top part of time
      uxtimelo = (unixtime & 0xffff) * 1000ul; // low part of time
      //Serial << F("uxhi: ") << uxtimehi << F("uxlo: ") << uxtimelo << F("\n");
      unsigned long uxtimelo2 = ((uxtimehi & 0xffff) << 16);  // get lo part of hi part
      //Serial << F("uxlo2a: ") << uxtimelo2;
      uxtimelo2 += uxtimelo; // add back original lo part
      uxtimehi = (uxtimehi >> 16) + (uxtimelo2 < uxtimelo ? 1 : 0);  // if uxtime low wrapped around, add one to epoch number
      baseunixepoch = uxtimehi;  // 47.1 day periods (mini-epoch) since 1-jan-1970 since 1-1-1970 UTC
      baseunixmillis = uxtimelo2;  // milliseconds in current mini-epoch  since 1-1-1970 UTC
      currentunixepoch = baseunixepoch;
      currentunixmillis = baseunixmillis;
      Serial << F(" (0x") << _HEX(baseunixepoch) << F(":") << _HEX(baseunixmillis) << F("ms)\n");
    }
    else {
      Serial << F("thh:nn:ss mm/dd/yyyy\n");
    }
  }
  else {
    Serial << F("Time: ") << currentunixepoch << F(":") << currentunixmillis << F("; ") << getSessionid() << F(".") << getMessageid() << F("\n");
    //Serial << F("too short.");
  }
}

void printTime() {
    unsigned long millisNow = millis();
    unsigned long second = millisNow / 1000UL;
    unsigned int milli = millisNow % 1000UL;
    Serial << second << F(".") << (milli < 100 ? "0" : "") << (milli < 10 ? "0" : "") << milli << " ";
}


void process_serial_input() {
  static char serbuffer[22];    // serial input buffer
  char serlen;
  // process any serial input
  if ((serlen = readline(Serial.read(), serbuffer, 60)) > 0)
  //if (Serial.available() > 0)
  {
    char input = serbuffer[0];  // peek at first letter of input
    //char input = Serial.read();
    if (input == 'r') //d=dump all register values
      radio.readAllRegs();
    if (input == 'E') //E=enable encryption
      radio.encrypt(ENCRYPTGUID);
    if (input == 'e') //e=disable encryption
      radio.encrypt(null);
    if (input == 'p')
    {
      promiscuousMode = !promiscuousMode;
      radio.promiscuous(promiscuousMode);
      Serial << F("Promiscuous mode ") << (promiscuousMode ? F("on") : F("off"));
    }
    if (input == 'd') //d=dump flash area
    {
      Serial.println(F("Flash content:"));
      int counter = 0;

      while(counter<=256){
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while(flash.busy());
      Serial.println();
    }
    if (input == 'D')
    {
      Serial.print(F("Deleting Flash chip ... "));
      flash.chipErase();
      while(flash.busy());
      Serial.println(F("DONE"));
    }
    if (input == 'i')
    {
      Serial.print(F("DeviceID: "));
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
    }
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      //byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial << F("Radio Temp is ") << temperature << F("C");
      //Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      //Serial.println('F');
    }
    if (input == 'T') {
      process_time_command(serlen, serbuffer);
      printTime();
    }
  }
}

byte savePacket() {
  byte accepted = false;
  if ((radio.DATALEN <= 64) && (radio.DATALEN > 0)) {
    //Serial << "[ getpayload():" << (unsigned long) getPayload() << "]";
    accepted = setPayload((const void *) radio.DATA, radio.DATALEN);
  }
  else {
    Serial << F(" packet too long.");
  }
  return accepted;
}


byte validatePacket(byte accepted) {
  //byte accepted = false;
    if (accepted) {
      // dump the radio copy to Serial
      for (byte i = 0; i < min(radio.DATALEN,2); i++)
        Serial.print((char)radio.DATA[i]);
      Serial.print(F("$"));
      for (byte i = 2; i < radio.DATALEN-2; i++)
        Serial << (radio.DATA[i]<16 ? "0" : "") << _HEX(radio.DATA[i]) << ( i < radio.DATALEN-3 ? ":" : "$");
      for (byte i = radio.DATALEN-2; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i]);
      //memcpy((void *)getPayload(),(const void *) radio.DATA, radio.DATALEN);
      Serial << endl;
    }
    else {
      Serial << F(" packet rejected.");
    }
  return accepted;
}

void process_radio_input() {
  byte theSenderID;
  byte theTargetID;
  byte theDatalen;
  int16_t theRSSI;
  if (radio.receiveDone())
  {
    theRSSI = radio.RSSI; // very volatile!
    byte packetSaved = savePacket();
    // this gets lost on reply/ACK!
    theSenderID = radio.SENDERID;
    theTargetID = radio.TARGETID;
    theDatalen = radio.DATALEN;
    if (radio.ACKRequested())
    { 
      radio.sendACK();
      Serial.print(F(" - ACK sent."));

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial << F("\n@"); printTime();
        Serial << F("Pinging node ") << _HEX(theSenderID) << F(" - ACK:");
        //delay(1); //need this when sending right after reception .. ? // it has been failing at 3
        if (radio.sendWithRetry(theSenderID, "ACK TEST", 8)) // 2 retries seems to work!, 0))  // 0 = only 1 attempt, no retries
          Serial.print(F("ok;"));
        else Serial.print(F("nack;"));
      }
    }
    Serial << F("\n@"); printTime();
    Serial << F("#:") << (++packetCount) << F(";");
    Serial << F("From:") << _HEX(theSenderID) << F("; ");
    Serial << F("To:") << _HEX(theTargetID) << F("; ");
    Serial << F("Len:") << theDatalen << F(";");
    Serial << F(" RSSI:") << (theRSSI) << F(";");
    byte packetAccepted = validatePacket(packetSaved);
    
   Serial.println();
   if (packetAccepted) {
        serial_display_packet();
        Serial << F("Valid:") << validPayload();
        // TODO: process packet here...
        // for now, print each packet
        byte theMiniPacket = firstPacket();
        byte mp;
        for (mp = firstPacket(); mp != 0xff; mp = nextPacket(mp)) {
          byte bminiPacketName = miniPacketName(mp);
          byte bminiPacketType = miniPacketType(mp);
          byte bminiPacketLength = miniPacketLength(mp);
          Serial << F("[") << mp << F("{") << bminiPacketName << F(" ");
          //Serial << TagNames[bminiPacketName] 
          Serial << F(":") << bminiPacketType << F("~") << bminiPacketLength << F("] = ");
          if (bminiPacketLength = 1 ) Serial << miniPacketValue_c(mp);
          if (bminiPacketLength = 2 ) Serial << miniPacketValue_i(mp);
          if (bminiPacketLength = 4 ) Serial << miniPacketValue_L(mp);
          Serial << "}]\n";
        }
   }
   Blink(LED,3);
  }
}

void loop() {

  process_serial_input();  //process any serial input
  process_radio_input();  // listen hard for radio input.
 }

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

