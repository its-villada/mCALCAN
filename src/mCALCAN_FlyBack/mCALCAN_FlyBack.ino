/*
  El modulo gps se debe configurar desde U-center, cambiando:
  el baudrate de 9600 a 115200
  el protocolo de salida de NMEA+UBX a UBX
  se debe activar en cfg-msg el mensaje nav-posllh en uart1
  cambiar el rate a 100ms o 200ms

  IMPORTANTE: guardar la configuracion del gps en el icono que se un engranaje con el simbolo de guardado

  Usar el codigo en extras->serial-bridge-gps.ino para conectar el modulo a la pc
*/

/*
  IMPORTANTE: para que el compas funcione correctamente hay que calibrarlo ver: extras->Calibracion compas. Luego cargar los datos de calibracion en CustomQMC583L.h, en las variables calibration matrix y bias
*/

#include <CustomQMC5883L.h>
#include <Wire.h>
#include "ServoTimer2.h"
#include <UbxGpsNavPosllh.h>

#define GPS_BAUDRATE 115200
#define DRDYPIN 3
#define servoDerPin 2
#define servoIzqPin 3
#define servoDerMaxMicros 400
#define servoDerMinMicros 2400
#define servoIzqMaxMicros 2400
#define servoIzqMinMicros 400
#define compassYawCorrection 0
#define minHorizontalAcc 5000//mm


CustomQMC5883L compass;
UbxGpsNavPosllh<HardwareSerial> gps(Serial);  //cambiar a serial2 para usar con el bluepill
ServoTimer2 servoIzq;
ServoTimer2 servoDer;



double miLatitud, miLongitud, destinoLatitud = -31.380396, destinoLongitud = -64.243394;
int16_t deltaOrientacion, distanciaADestino, miOrientacion, orientacionADestino;
bool gpsDRDY = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("Iniciando CANSAT_flyback");
  compassSetup();
  GPSSetup();
  servoSetup();
  pinMode(13,OUTPUT);
  Serial.println("cansat_flyback iniciado");
}

void loop() {
  leerCompass();
  leerGPS();
  digitalWrite(13,gpsDRDY);
  if (gpsDRDY) {
    servoControl();
    /*
    Serial.print("LAT = ");
    Serial.println(miLatitud, 6);
    Serial.print("LONG = ");
    Serial.println(miLongitud, 6);
    Serial.print("COURSE TO DESTINATION = ");
    Serial.println(orientacionADestino);
    Serial.print("DISTANCE TO DESTINATION = ");
    Serial.println(distanciaADestino);
    Serial.print("Orientacion: ");
    Serial.println(miOrientacion);
    Serial.print("Delta orientacion: ");
    Serial.println(deltaOrientacion);
    */
    gpsDRDY = 0;
  }
}


void compassSetup() {
  while (!compass.init()) {
    Serial.println("Fallo al conectarse al modulo compass");
    delay(250);
  }
  //compass.useCalibration(true);
  compass.setSmoothingSteps(10);
  pinMode(DRDYPIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DRDYPIN), leerCompass, CHANGE);
  Serial.println("Compass ready");
}

void leerCompass() {
  compass.read();
  miOrientacion = compass.getAzimuth() + compassYawCorrection;
  deltaOrientacion = orientacionADestino - miOrientacion;

  deltaOrientacion = (deltaOrientacion < (-180)) ? deltaOrientacion + 360 : deltaOrientacion;
  deltaOrientacion = (deltaOrientacion > 180) ? deltaOrientacion - 360 : deltaOrientacion;
}



void GPSSetup() {
  gps.begin(GPS_BAUDRATE);

  Serial.println("GPS ready");
}

void leerGPS() {
  if (gps.ready()) {

    miLongitud = (gps.lon / 10000000.0);

    miLatitud = (gps.lat / 10000000.0);

    if (gps.hAcc < minHorizontalAcc && gps.hAcc > 0) { 
      distanciaADestino = distanceBetween(miLatitud, miLongitud, destinoLatitud, destinoLongitud);
      orientacionADestino = courseTo(miLatitud, miLongitud, destinoLatitud, destinoLongitud);
      gpsDRDY = 1;
    }
    else {
      servoIzq.write(servoIzqMinMicros);
      servoDer.write(servoDerMinMicros);
    }
  }
}

void servoSetup() {
  servoIzq.attach(servoIzqPin);
  servoDer.attach(servoDerPin);

  servoIzq.write(servoIzqMinMicros);
  servoDer.write(servoDerMinMicros);

  Serial.println("Servo ready");
}
void servoControl() {
  float girDerPor = 0, girIzqPor = 0, coefDist = 0;  //0% - 100%

  if (deltaOrientacion > 0) {
    girDerPor = map(deltaOrientacion, 0, 180, 5, 100);
  }
  if (deltaOrientacion < 0) {
    girIzqPor = map(deltaOrientacion, -180, 0, 100, 5);
  }

  if (deltaOrientacion >= 0 && deltaOrientacion <= 10) {
    girIzqPor = girIzqPor + (100 - deltaOrientacion * 5);
    girDerPor = girDerPor + (100 - deltaOrientacion * 5);
  }

  if (deltaOrientacion <= 0 && deltaOrientacion >= -10) {
    girIzqPor = girIzqPor + (100 - deltaOrientacion * -5);
    girDerPor = girDerPor + (100 - deltaOrientacion * -5);
  }

  coefDist = constrain(distanciaADestino / 1, 0, 1);

  girDerPor *= coefDist;
  girIzqPor *= coefDist;

  servoDer.write(map(constrain(girDerPor, 0, 100), 0, 100, servoDerMinMicros, servoDerMaxMicros));
  servoIzq.write(map(constrain(girIzqPor, 0, 100), 0, 100, servoIzqMinMicros, servoIzqMaxMicros));
}

double distanceBetween(double lat1, double long1, double lat2, double long2) {
  // returns distance in meters between two positions, both specified
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2) {
  // returns course in degrees (North=0, West=270) from position 1 to position 2,

  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  return degrees(a2);
}


void stopGPS() {
  byte packet[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74 };
  for (byte i = 0; i < sizeof(packet); i++) {
    Serial.write(packet[i]);
  }
}

void startGPS() {
  byte packet[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76 };
  for (byte i = 0; i < sizeof(packet); i++) {
    Serial.write(packet[i]);
  }
}
