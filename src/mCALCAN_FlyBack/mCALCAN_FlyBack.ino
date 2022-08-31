#include "CustomQMC5883L.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Servo.h>

#define DRDYPIN 3
#define servoDerPin 10
#define servoIzqPin 9

CustomQMC5883L compass;
TinyGPSPlus TINY_GPS_OBJECT;
Servo servoIzq;
Servo servoDer;

const int servoDerMaxAngle = 0, servoDerMinAngle = 180;
const int servoIzqMaxAngle = 180, servoIzqMinAngle = 0;
float magX, magY, magZ;
double miLatitud, miLongitud, destinoLatitud = -31.380368, destinoLongitud = -64.243142, miVelocidad, miOrientacion, orientacionADestino;
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
}

void compassSetup() {
  while (!compass.init()) {
    Serial.println("Fallo al conectarse al modulo compass");
    delay(250);
  }
  compass.useCalibration(true);
  compass.setSmoothingSteps(5);
  pinMode(DRDYPIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DRDYPIN), leerCompass, CHANGE);
  Serial.println("Compass ready");
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
  /* Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("CFG-UART1-BAUDRATE 0x1C200");
  Serial.end();
*/

  Serial.begin(115200);
  digitalWrite(4, HIGH);
  Serial.println("CFG-RATE-MEAS 0xc8");
  delay(1000);
  digitalWrite(4, LOW);

  Serial.println("GPS ready");
}

void leerGPS() {

  while (Serial.available()) {
    if (TINY_GPS_OBJECT.encode(Serial.read())) {
      miLatitud = TINY_GPS_OBJECT.location.lat();
      miLongitud = TINY_GPS_OBJECT.location.lng();
      miVelocidad = TINY_GPS_OBJECT.speed.mps();
      distanciaADestino = (unsigned long)TinyGPSPlus::distanceBetween(miLatitud, miLongitud, destinoLatitud, destinoLongitud);
      orientacionADestino = (unsigned long)TinyGPSPlus::courseTo(miLatitud, miLongitud, destinoLatitud, destinoLongitud);


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
      Serial.print("Orientacion: ");
      Serial.println(miOrientacion);
      Serial.print("Delta orientacion: ");
      Serial.println(deltaOrientacion);
    }
  }
}

void servoSetup() {
  servoIzq.attach(servoIzqPin);
  servoDer.attach(servoDerPin);

  servoIzq.write(servoIzqMinAngle);
  servoDer.write(servoDerMinAngle);
  delay(1000);
  Serial.println("Servo ready");
}
void servoControl() {
}