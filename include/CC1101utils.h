#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Wire.h>

//#define TICC1101
#define E07M1101D
//#define E07M1101D2

#ifdef E07M1101D
 #define CCGDO0 25
 #define CCGDO2 -1
 #define CCSCK 0
 #define CCMISO 33
 #define CCMOSI 32
 #define CCCSN 26
#endif

#ifdef E07M1101D2
 #define CCGDO0 3
 #define CCGDO2 8
 #define CCSCK 5
 #define CCMISO 7
 #define CCMOSI 6
 #define CCCSN 4
#endif

#ifdef TICC1101
 #define CCGDO0 27
 #define CCGDO2 32
 #define CCSCK 26
 #define CCMISO 2
 #define CCMOSI 25
 #define CCCSN 33
#endif

void CCSetTx() {
  #ifndef CARDP
  pinMode(CCGDO0,OUTPUT);
  #endif
  ELECHOUSE_cc1101.SetTx();
}

void CCSetMhz(float freq) {
  ELECHOUSE_cc1101.setMHZ(freq);
}

void CCSetRxBW(int bandwidth) {
  ELECHOUSE_cc1101.setRxBW(bandwidth);
}
/*
 #define CCGDO0 3
 #define CCGDO2 8
 #define CCSCK 5
 #define CCMISO 7
 #define CCMOSI 6
 #define CCCSN 4
*/
void CCInit() {
  ELECHOUSE_cc1101.setI2C(&Wire,0x45,32,33);
  ELECHOUSE_cc1101._spiOverI2C=true;
  ELECHOUSE_cc1101._sda=32;
  ELECHOUSE_cc1101._scl=33;
  ELECHOUSE_cc1101._I2Caddr=0x45;
  ELECHOUSE_cc1101.setSpiPin(5, 7, 6, 4);
  ELECHOUSE_cc1101.setGDO(3, 8);
  ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
  Serial.println();

 // ELECHOUSE_cc1101.setPA(8);
                                          //The lib calculates the frequency automatically (default = 433.92).
                                          //The cc1101 can do: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setClb(1,13,15);
  ELECHOUSE_cc1101.setClb(2,16,19);
  ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setDRate(512);         // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
  ELECHOUSE_cc1101.setRxBW(256);          // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
  ELECHOUSE_cc1101.setPktFormat(3);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 
                                          // 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 
                                          // 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 
                                          // 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
}

void CCSetRx() {
  #ifndef CARDP
  pinMode(CCGDO0,INPUT);  
  #endif  

  ELECHOUSE_cc1101.SetRx();
}


#define BAVGSIZE 11
byte bavg[BAVGSIZE];
byte pb = 0;
byte cres = 0;

byte CCAvgRead() {
  cres -= bavg[pb];
  #ifndef CARDP
  bavg[pb] = digitalRead(CCGDO0);
  #else
  ELECHOUSE_cc1101.readI2CBytes(0x20 + CCGDO0, &bavg[pb],1);
  #endif
  cres += bavg[pb];
  pb++;
  if (pb >= BAVGSIZE) pb = 0;
  if (cres > BAVGSIZE/2) return 1;
  return 0;
}

void CCWrite(uint8_t b) {
  #ifndef CARDP
  digitalWrite(CCGDO0,b);
  #else
  ELECHOUSE_cc1101.writeI2CBytes(0x10 + CCGDO0, &b,1);
  #endif
}
