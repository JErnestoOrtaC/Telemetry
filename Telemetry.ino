
#include "Main.h"

void setup() {
  // put your setup code here, to run once:
  Begins();
  delay(100);
  Sensorcheck();
  LoraSend("Toamando Prersion en pista de despegue");
  Serial.println("Toamando Prersion en pista de despegue");
  delay(500);
  datos.Pz = bme.readPressure() / 100.F;
  Mqsetup();

}

void loop() {
  // put your main code here, to run repeatedly:
  Get_Sensors();
  Mqsense();
  PacageTelemetry();
  LoraSend(mensaje);
  SerialDisplay();
  delay(150);


}
