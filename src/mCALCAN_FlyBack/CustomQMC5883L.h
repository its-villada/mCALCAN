#ifndef Custom_QMC5883L
#define Custom_QMC5883L

#include "Arduino.h"
#include "Wire.h"


class CustomQMC5883L {

public:
  CustomQMC5883L();
  bool init();
  void setADDR(byte b);
  void setMode(byte mode, byte odr, byte rng, byte osr);
  void setSmoothingSteps(byte steps);
  void useCalibration(bool a);
  void setReset();
  void read();
  int getX();
  int getY();
  int getZ();
  int getCompensatedAzimuth(float pitch,float roll);
  int getAzimuth(void);
  void writeReg(byte reg, byte val);
  byte readRegister8(uint8_t reg);
  typedef struct {
    int Xaxis;
    int Yaxis;
    int Zaxis;
  } magdata;

private:
  float magDeclination = -6.433;
  bool _firstSmooth = true;
  bool _smoothUse = false;
  byte _smoothSteps = 5;
  byte _ADDR = 0x0D;  //default qmc5883 addr
  bool _calibrationUse = false;
  void _smoothing(magdata nonSmooth);
  void _applyCalibration(magdata rawValues);
  magdata rawValues;
  magdata calibratedValues;
  magdata smoothedValues, historyValues[20] = {0,0,0};
  magdata magValues;
  int scannCount = 0;
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = {
    { 3.13, 0.259, -0.267 },
    { -0.243, 4.178, -0.517 },
    { 0.314, 5.085, 3.275 }
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = {422.611,-790.265,-1972.646};
};

#endif