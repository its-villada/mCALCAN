#include <CustomQMC5883L.h>
#include <Wire.h>
CustomQMC5883L compass;

float magX, magY, magZ;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(100000);

  compassSetup();

}

void loop() {

  leerCompass();

  
}

void compassSetup() {
  while (!compass.init()) {
    Serial.println("Fallo al conectarse al modulo compass");
    delay(250);
  }
  //compass.useCalibration(true);
  //compass.setSmoothingSteps(5);
}

void leerCompass() {
  compass.read();
  Serial.print( compass.getX());
  Serial.print( ",");
  Serial.print( compass.getY());
  Serial.print( ",");
  Serial.println( compass.getZ());

}
