#include <SPI.h>
////////////////////////////////////////////
//set up pins for arudino due
//MOSI:ICSP-4
//MISO:ICSP-1
//SCK:ICSP-3
//NCS:52
//EN:2
//
///////////////////////////////////////////////

// Set up chip-select pin as NCS and enable pin as EN
const int NCS = 52;
const int EN = 2;

//SPI setting
SPISettings settingsA(100000, MSBFIRST, SPI_MODE3);

void setup() {

  pinMode (NCS, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);//set enable to high
  Serial.begin(9600);

  SPI.begin();
  SPI.beginTransaction(settingsA); //I wrote to the following 2 main control registers, register description is at the data sheet

  digitalWrite (NCS, LOW);
  SPI.transfer(0b00000000); // write and register addressL: Device staus register
  SPI.transfer(0b00000001); // value for this register: Turn RF-On
  digitalWrite (NCS, HIGH);

  digitalWrite (NCS, LOW);
  SPI.transfer(0b00000001); // write and register addressL: Protocal Selection Register
  SPI.transfer(0b00000000); // value for this register: Choose EPC Gen2
  digitalWrite (NCS, HIGH);

   SPI.endTransaction();
}


int8_t O;
void loop() {
  //
  for (int i = 0; i <= 255; i++) {

      SPI.beginTransaction(settingsA); //read from my RSSI_Display register
    digitalWrite (NCS, LOW);

    SPI.transfer16(0b011010100000000);
 //   SPI.transfer(0b0100000);

    digitalWrite (NCS, HIGH);
     SPI.endTransaction();

  }
  Serial.print(O);
}
