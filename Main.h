/*
    En este header file se declaran y definen los sistemas principales y de iniciación del programa principal
*/
#include "Telemetry.h"
#include "Lora.h"


void Begins(){

  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 32, 4);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin();
  
  delay(150);
  LoraConfig();
}



