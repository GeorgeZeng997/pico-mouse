#include <BLEAttribute.h>
#include <BLEBondStore.h>
#include <BLECentral.h>
#include <BLECentralRole.h>
#include <BLECharacteristic.h>
#include <BLEConstantCharacteristic.h>
#include <BLEDescriptor.h>
#include <BLEDevice.h>
#include <BLEDeviceLimits.h>
#include <BLEFixedLengthCharacteristic.h>
#include <BLEHID.h>
#include <BLEHIDPeripheral.h>
#include <BLEHIDReportMapCharacteristic.h>
#include <BLEHIDReportReferenceDescriptor.h>
#include <BLEKeyboard.h>
#include <BLELocalAttribute.h>
#include <BLEManager.h>
#include <BLEMouse.h>
#include <BLEMultimedia.h>
#include <BLENode.h>
#include <BLEPeripheral.h>
#include <BLEProgmemConstantCharacteristic.h>
#include <BLERemoteAttribute.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <BLESerial.h>
#include <BLEService.h>
#include <BLESystemControl.h>
#include <BLETypedCharacteristic.h>
#include <BLETypedCharacteristics.h>
#include <BLEUtil.h>
#include <BLEUuid.h>
#include <EddystoneBeacon.h>
#include <iBeacon.h>
#include <nRF51822.h>
#include <nRF8001.h>

/*
 * Program to interface FCT3065-XY Optical Mouse
 * Sensor with Arduino.
 * 
 * This code detects when mouse is moved and also
 * prints the change in x direction and change in y
 * direction in the serial monitor.
 * 
 * Author: Vineet Sukhthanker
 * Date: 6 October 2020
 * 
 * 
 */

#include <Mouse.h>

//Register Addresses from datasheet
#define PROD_ID1 0x00
#define PROD_ID2 0x01
#define MOTION_STATUS 0X02
#define DEL_X 0x03
#define DEL_Y 0x04
#define OP_MODE 0x05
#define CONF 0x06
#define IMG_QUALITY 0x07
#define OP_STATE 0x08
#define W_PROTECT 0x09
#define SLEEP1_SET 0x0A
#define ENTER_TIME 0x0B
#define SLEEP2_SET 0x0C
#define IMG_THRESHOLD 0x0D
#define IMG_RECOG 0x0E

const int SCLK = 4;
const int SDIO = 3;
const int MOTSWK = 2;

byte noSleep = 0xA0;

int i = 0;
void setup() {

  delay(5000);

  Serial.begin(115200);
  pinMode (SCLK, OUTPUT);
  pinMode (MOTSWK, INPUT);
  mouseInit();
  
  byte prodId1 = readRegister(PROD_ID1);
  Serial.print((prodId1==0x30)?"\nDevice OK":"\nUnknown Device"); // checks for product ID

  Mouse.begin();
}

void loop() {
  if(digitalRead(MOTSWK) == LOW)
  {
    int motion = readRegister(MOTION_STATUS); // read motion status register

    //mouse move
    if(motion==0x82){
      Serial.print("\nMoved!");
      //Mouse.begin();
      signed char delta_x = (signed char)readRegister(DEL_X); // read delta x register
      signed char delta_y = (signed char)readRegister(DEL_Y); // read delta y register
      Serial.print("\nX:");
      Serial.print(delta_x);
      Serial.print(" Y:");
      Serial.print(delta_y);
      Mouse.move(delta_x, -delta_y, 0);
    }
  }

  //Mouse.end();

  //delay(100);
}

void mouseInit(void) // function to initialize optical sensor.
{
  digitalWrite(SCLK, HIGH);
  digitalWrite(SCLK, LOW);
  delayMicroseconds(1); // tRESYNC = 1us (mentioned in datasheet)
  digitalWrite(SCLK, HIGH);
  delay(320); // tSIWTT = 320ms (mentioned in datasheet)
  digitalWrite(SCLK, LOW);
  writeRegister(OP_MODE, noSleep); // disable sleep mode
}

byte readRegister(byte address) {
  pinMode (SDIO, OUTPUT);

  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  pinMode (SDIO, INPUT);

  delayMicroseconds(3); // tHOLD = 3us (mentioned in datasheet)

  byte res = 0;
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SCLK, HIGH);
    if( digitalRead (SDIO) == HIGH )
      res |= i;
  }

  delayMicroseconds(3); // tHOLD = 3us min. (mentioned in datasheet)
 
  return res;
}

void writeRegister(byte address, byte data) {
  address |= 0x80; // MSB indicates write mode.
  pinMode (SDIO, OUTPUT);

  for (byte i = 128; i > 0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  for (byte i = 128; i > 0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (data & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  delayMicroseconds(100);
}

