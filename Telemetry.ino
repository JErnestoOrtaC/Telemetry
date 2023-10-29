
#include "Main.h"
#include "mq.h"

void setup() {
  // put your setup code here, to run once:
  Begins();
  delay(100);
  Sensorcheck();
  LoraSend("Tomando Prersion en pista y calibrando el MQ2");
  Serial.println("Tomando Prersion en pista y calibrando el MQ2");
  delay(500);
  datos.Pz = bme.readPressure() / 100;
  Ro= MQCalibration (MQ_PIN);
  delay(150);
}

void loop() {
  // put your main code here, to run repeatedly:
  Get_Sensors();
  datos.LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  datos.CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  datos.Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  PacageTelemetry();
  LoraSend(mensaje);
  SerialDisplay();
  delay(250);

}
