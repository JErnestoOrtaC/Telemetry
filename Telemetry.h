
/*
    En este header file se declaran y definen las funciones referentes a la telemetría
    Asi como la construcción de los objetos de cada sensor y estructura de datos
    Se empleo una estructura de datos para guardar los datos de telemetría
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <TinyGPS++.h>

#define EARTH_RADIUS 6371.0
#define MQpin 34

struct GPSData {
  double latitude;
  double longitude;
  double altitude;
};

struct Telemetria {
  float Pz=0;
  float temperatura;
  float presion;
  float humedad;
  float altura;
  float airQuality;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float vx, vy, vz, vt;
  GPSData gpsData;  // Agrega el miembro de tipo GPSData
};

String mensaje;

Adafruit_BME280 bme;
MPU6050 mpu;
TinyGPSPlus gps;

Telemetria datos;
GPSData data;



float Get_Pz(){
  long double x=0;
  for(int i=0; i<11;i++){
    x += ( bme.readPressure() / 100.F );
  }
    return (bme.readPressure()/100.0F);
}

void Sensorcheck(){
  unsigned status;
  status = bme.begin(0X76);
  if( !status ){
    Serial.println("Error sensor bme280");
    Serial.print("ID: 0x");
    Serial.println( bme.sensorID(), 16 );
  }

  // Inicializar el sensor MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("No se pudo encontrar el sensor MPU6050. Verifica las conexiones.");
    while (1);
  }
}

void BMPSensor() {
  datos.temperatura = bme.readTemperature();
  datos.presion = bme.readPressure() / 100.0F;
  datos.altura = bme.readAltitude(datos.Pz);
  datos.humedad = bme.readHumidity();
}

void IMU() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  datos.ax = az;
  datos.ay = ax;
  datos.az = ay;

  datos.gx = gx;
  datos.gy = gy;
  datos.gz = gz;
}

void getGPSData() {
  // Leer los datos disponibles desde el módulo GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
    // Obtener la latitud, longitud y altitud
    datos.gpsData.latitude = gps.location.lat();
    datos.gpsData.longitude = gps.location.lng();
}

void Get_vt() {
  // Integración de la aceleración lineal para obtener la velocidad
  datos.vx += (datos.ax - datos.gx) *3;
  datos.vy += (datos.ay - datos.gy)*3;
  datos.vz += (datos.az - datos.gz)*3;

  datos.vt = sqrt(datos.vx * datos.vx + datos.vy * datos.vy + datos.vz * datos.vz);

}

void Mqsensor(){
  int sensorVal = analogRead(MQpin);
  datos.airQuality = map(sensorVal, 0, 4095, 0, 100);
}

void Get_Sensors(){
  getGPSData();
  BMPSensor();
  IMU();
  Get_vt();
}

void SerialDisplay(){

  Serial.println("----------------------Telemetry---------------------");

  Serial.print("Temperatura: ");
  Serial.println(datos.temperatura);

  Serial.print("PZ:");
  Serial.println(datos.Pz);

  Serial.print("Presión: ");
  Serial.println(datos.presion);

  Serial.print("Altitud: ");
  Serial.println(datos.altura);

  Serial.print("Humedad: ");
  Serial.println(datos.humedad);

  Serial.print("Aceleración (x, y, z): ");
  Serial.print(datos.ax);
  Serial.print(", ");
  Serial.print(datos.ay);
  Serial.print(", ");
  Serial.println(datos.az);

  Serial.print("Giroscopio (x, y, z): ");
  Serial.print(datos.gx);
  Serial.print(", ");
  Serial.print(datos.gy);
  Serial.print(", ");
  Serial.println(datos.gz);

  Serial.print("Velocidad (x, y, z, total): ");
  Serial.print(datos.vx);
  Serial.print(", ");
  Serial.print(datos.vy);
  Serial.print(", ");
  Serial.print(datos.vz);
  Serial.print(", ");
  Serial.println(datos.vt);

  Serial.print("Datos GPS: ");
  Serial.print("Latitud: ");
  Serial.print(datos.gpsData.latitude,6 );
  Serial.print(", Longitud: ");
  Serial.println(datos.gpsData.longitude,6);
}

void PacageTelemetry(){
  mensaje = "";
  // Realiza lo mismo para cada variable, por ejemplo:
  mensaje += "Temp: " + String(datos.temperatura);
  mensaje += ",Pres: " + String(datos.presion);
  mensaje += ",Alt: " + String(datos.altura);
  mensaje += ",Hum: " + String(datos.humedad);
  mensaje += ",Ax: " + String(datos.ax);
  mensaje += ",Ay: " + String(datos.ay);
  mensaje += ",Az: " + String(datos.az);
  mensaje += ",Gx: " + String(datos.gx);
  mensaje += ",Gy: " + String(datos.gy);
  mensaje += ",Gz: " + String(datos.gz);
  mensaje += ",Vx: " + String(datos.vx);
  mensaje += ",Vy: " + String(datos.vy);
  mensaje += ",Vz: " + String(datos.vz);
  mensaje += ",Vt: " + String(datos.vt);
}






