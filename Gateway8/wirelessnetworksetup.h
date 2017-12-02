#ifndef WirelessNetworkSetup_h
#define WirelessNetworkSetup_h


#define ENCRYPTGUID "\xE9\xCB\xB5\x0E\xDC\x5D\x4d\x4d\xAE\xF3\xEA\x39\xBB\x23\xDF\x2C" // 16 characters long

#define NETWORKID     109  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define ENCRYPTKEY    "279CrockettDrive" //exactly the same 16 characters/bytes on all nodes!
#endif
