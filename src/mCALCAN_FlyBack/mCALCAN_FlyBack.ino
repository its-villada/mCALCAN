#include "CustomQMC5883L.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define DRDYPIN 3
#define servoDerPin 9
#define servoIzqPin 10

CustomQMC5883L compass;
TinyGPSPlus TINY_GPS_OBJECT;
SoftwareSerial softSerial(12, 11);
Servo servoIzq;
Servo servoDer;

float magX,magY, magZ;
double miLatitud, miLongitud, destinoLatitud = -31.362777, destinoLongitud = -64.276557, miVelocidad, miOrientacion, orientacionADestino;
float deltaOrientacion, distanciaADestino;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("Iniciando CANSAT_flyback");
  compassSetup();
  GPSSetup();
servoSetup();
  Serial.println("cansat_flyback iniciado");
}

void loop() {
  leerGPS();
  leerCompass();

  if (TINY_GPS_OBJECT.location.isUpdated()) {
    Serial.print("LAT = ");
    Serial.println(miLatitud, 6);
    Serial.print("LONG = ");
    Serial.println(miLongitud, 6);
    Serial.print("SPEED = ");
    Serial.println(miVelocidad);
    Serial.print("COURSE TO DESTINATION = ");
    Serial.println(orientacionADestino, 2);
    Serial.print("DISTANCE TO DESTINATION = ");
    Serial.println(distanciaADestino, 2);
    Serial.print("heading: ");
    Serial.println(miOrientacion);
  }
}

void compassSetup() {
  while (!compass.init()) {
    Serial.println("Fallo al conectarse al modulo");
    delay(250);
  }
  compass.useCalibration(true);
  compass.setSmoothingSteps(5);
  pinMode(DRDYPIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DRDYPIN), leerCompass, CHANGE);
}

void leerCompass() {
  compass.read();
  magX = compass.getX();
  magY = compass.getY();
  magZ = compass.getZ();
  miOrientacion = compass.getAzimuth();
  deltaOrientacion = miOrientacion - orientacionADestino;
}



void GPSSetup() {
  softSerial.begin(9600);
    while (!softSerial);
    softSerial.println("CFG-UART1-BAUDRATE 0x1C200");
    softSerial.end();
  

  softSerial.begin(115200);
  softSerial.println("CFG-RATE-MEAS 0xC8");
  delay(1000);
}

void leerGPS() {

  while (softSerial.available()) {
    if (TINY_GPS_OBJECT.encode(softSerial.read())) {
      miLatitud = TINY_GPS_OBJECT.location.lat();
      miLongitud = TINY_GPS_OBJECT.location.lng();
      miVelocidad = TINY_GPS_OBJECT.speed.mps();
      distanciaADestino = (unsigned long)TinyGPSPlus::distanceBetween(miLatitud, miLongitud, destinoLatitud, destinoLongitud);
      orientacionADestino = (unsigned long)TinyGPSPlus::courseTo(miLatitud, miLongitud, destinoLatitud, destinoLongitud);
    }
  }
}

void servoSetup(){
  servoIzq.attach(servoIzqPin);
  servoDer.attach(servoDerPin);

  servoIzq.write(0);
  servoDer.write(0);

}
void servoControl(){


}