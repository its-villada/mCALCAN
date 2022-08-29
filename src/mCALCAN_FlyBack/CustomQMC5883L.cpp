/*
  ===============================================================================================================
  Originaly QMC5883LCompass.h
  Library for using QMC5583L series chip boards as a compass.
  Learn more at [https://github.com/mprograms/QMC5883LCompass]

  Supports:

  - Getting values of XYZ axis.
  - Calculating Azimuth.
  - Smoothing of XYZ readings via rolling averaging
  - Tilt compensation using given pitch and roll
  - Calibration using transformation matrix and bias matrix edit matrices in CustomQMC5883L.h
  ===============================================================================================================

  Release under the GNU General Public License v3
  [https://www.gnu.org/licenses/gpl-3.0.en.html]

  ===============================================================================================================

  This is a modified version of the original QMC5883LCompass.
  Origianl version by MRPrograms (Github: [https://github.com/mprograms/])

  Modified by Alejo Lopez
  Intended to be used on mCALCAN mision, at the CANSAT ARGENTINA 2022 Competition

  ===============================================================================================================


  FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
  -----------------------------------------------
  MODE CONTROL (MODE)
	Standby			0x00
	Continuous		0x01

  OUTPUT DATA RATE (ODR)
	10Hz        	0x00
	50Hz        	0x04
	100Hz       	0x08
	200Hz       	0x0C

  FULL SCALE (RNG)
	2G          	0x00
	8G          	0x10

  OVER SAMPLE RATIO (OSR)
	512         	0x00
	256         	0x40
	128         	0x80
	64          	0xC0

*/



#include "Arduino.h"
#include "CustomQMC5883L.h"
#include <Wire.h>


CustomQMC5883L::CustomQMC5883L() {
}


/**
	INIT
	Initialize Chip

  @return true if started correctly false if not
**/
bool CustomQMC5883L::init() {


  Wire.begin();
  writeReg(0x0B, 0x01);
  setMode(0x01, 0x0C, 0x00, 0x00);

  if (readRegister8(0x0D) != 0xFF || readRegister8(0x09) != 0x0D) { // modificar si se cambia la configuracion del modulo
    return false;
  }
  return true;
}


/**
	SET ADDRESS
	Set the I2C Address of the chip. This needs to be called in the sketch setup() function.

	@since v0.1;
**/
// Set I2C Address if different then default.
void CustomQMC5883L::setADDR(byte b) {
  _ADDR = b;
}




/**
	REGISTER
	Write the register to the chip.

	@since v0.1;
**/
// Write register values to chip
void CustomQMC5883L::writeReg(byte reg, byte val) {
  Wire.beginTransmission(_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}


/**
	CHIP MODE
	Set the chip mode.

	@since v0.1;
**/
// Set chip mode
void CustomQMC5883L::setMode(byte mode, byte odr, byte rng, byte osr) {
  writeReg(0x09, mode | odr | rng | osr);
}


/**
	RESET
	Reset the chip.

**/
// Reset the chip
void CustomQMC5883L::setReset() {
  writeReg(0x0A, 0x80);
}

// 1 = Basic 2 = Advanced
void CustomQMC5883L::setSmoothingSteps(byte steps) {
  _smoothUse = true;
  _smoothSteps = (steps > 20) ? 20 : steps;
}

/**
  SET CALIBRATION
	Enables or disables calibration

**/
void CustomQMC5883L::useCalibration(bool a) {
  _calibrationUse = a;
}



/**
	READ
	Read the XYZ axis and save the values in an array.

	@since v0.1;
**/
void CustomQMC5883L::read() {
  magdata rawValues;
  Wire.beginTransmission(_ADDR);
  Wire.write(0x00);

  Wire.requestFrom(_ADDR, (byte)6);
  rawValues.Xaxis = (int)(int16_t)(Wire.read() | Wire.read() << 8);
  rawValues.Yaxis = (int)(int16_t)(Wire.read() | Wire.read() << 8);
  rawValues.Zaxis = (int)(int16_t)(Wire.read() | Wire.read() << 8);

  Wire.endTransmission();
  if (_smoothUse == false && _calibrationUse == false) {
    magValues = rawValues;
    return;
  }

  if (_smoothUse == false && _calibrationUse == true) {
    _applyCalibration(rawValues);
    magValues = calibratedValues;
    return;
  }

  if (_smoothUse == true && _calibrationUse == false) {
    _smoothing(rawValues);
    magValues = smoothedValues;
    return;
  }

  if (_smoothUse == true && _calibrationUse == true) {
    _applyCalibration(rawValues);
    _smoothing(calibratedValues);
    magValues = smoothedValues;
    return;
  }
}

/**
  APPLY CALIBRATION

	Extracted from: [https://www.instructables.com/Easy-hard-and-soft-iron-magnetometer-calibration/]

**/
void CustomQMC5883L::_applyCalibration(magdata rawValues) {
  int uncalibrated_values[3];
  int result[3] = {0, 0, 0};

  //calculation
  uncalibrated_values[0] = rawValues.Xaxis - bias[0];
  uncalibrated_values[1] = rawValues.Yaxis - bias[1];
  uncalibrated_values[2] = rawValues.Zaxis - bias[2];


  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];

  calibratedValues.Xaxis = result[0];
  calibratedValues.Yaxis = result[1];
  calibratedValues.Zaxis = result[2];
}


/**
	SMOOTH OUTPUT
	This function smooths the output for the XYZ axis. Depending on the options set in

	First we store (n) samples of sensor readings for each axis and store them in a rolling array.
	As each new sensor reading comes in we replace it with a new reading. Then we average the total
	of all (n) readings.

	NOTE: This function does several calculations and can cause your sketch to run slower.
**/
void CustomQMC5883L::_smoothing(magdata nonSmooth) {
  magdata sum = {0, 0, 0,};

  if (scannCount >= _smoothSteps) {
    _firstSmooth = false;
    scannCount = 0;
  }
  scannCount++;
  historyValues[scannCount - 1].Xaxis = nonSmooth.Xaxis;
  historyValues[scannCount - 1].Yaxis = nonSmooth.Yaxis;
  historyValues[scannCount - 1].Zaxis = nonSmooth.Zaxis;

  if (_firstSmooth) {
    for (int i = 0; i < scannCount; i++) {
      sum.Xaxis += historyValues[i].Xaxis;
      sum.Yaxis += historyValues[i].Yaxis;
      sum.Zaxis += historyValues[i].Zaxis;
    }

    smoothedValues.Xaxis = sum.Xaxis / scannCount;
    smoothedValues.Yaxis = sum.Yaxis / scannCount;
    smoothedValues.Zaxis = sum.Zaxis / scannCount;


  }


  if (scannCount < 10 && !_firstSmooth) {
    for (int i = 0; i < _smoothSteps; i++) {
      sum.Xaxis += historyValues[i].Xaxis;
      sum.Yaxis += historyValues[i].Yaxis;
      sum.Zaxis += historyValues[i].Zaxis;
    }

    smoothedValues.Xaxis = sum.Xaxis / _smoothSteps;
    smoothedValues.Yaxis = sum.Yaxis / _smoothSteps;
    smoothedValues.Zaxis = sum.Zaxis / _smoothSteps;
  }
}


/**
	GET X AXIS
	Read the X axis

	@return int x axis
**/
int CustomQMC5883L::getX() {
  return magValues.Xaxis;
}


/**
	GET Y AXIS
	Read the Y axis

	@return int y axis
**/
int CustomQMC5883L::getY() {
  return magValues.Yaxis;
}


/**
	GET Z AXIS
	Read the Z axis

	@return int z axis
**/
int CustomQMC5883L::getZ() {
  return magValues.Zaxis;
}


/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);
  Tilt compensates the azimuth using given pitch and roll in deg/s
	@return int azimuth
**/
int CustomQMC5883L::getCompensatedAzimuth(float pitch, float roll) {

  float radRoll = roll * 0.01745329 , radPitch = pitch * 0.01745329;

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(radRoll);
  float sinRoll = sin(radRoll);
  float cosPitch = cos(radPitch);
  float sinPitch = sin(radPitch);

  // Tilt compensation
  float Xh = magValues.Xaxis * cosPitch + magValues.Zaxis * sinPitch;
  float Yh = magValues.Xaxis * sinRoll * sinPitch + magValues.Yaxis * cosRoll - magValues.Zaxis * sinRoll * cosPitch;

  float heading = (atan2(Yh, Xh) * 180) / M_PI;

  return heading < 0 ? 360 + heading : heading;
}

/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);
	@return int azimuth
**/
int CustomQMC5883L::getAzimuth(void) {
  int a = atan2(getY(), getX()) * 180.0 / PI;
  return a < 0 ? 360 + a : a;
}

/**
  READ REGISTER 8
  Reads a byte from selected register

  @param  byte register
  @return byte value
**/
byte CustomQMC5883L::readRegister8(uint8_t reg) {
  byte value;
  Wire.beginTransmission(_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(_ADDR);
  Wire.requestFrom(_ADDR, 1);

  value = Wire.read();


  return value;
}
