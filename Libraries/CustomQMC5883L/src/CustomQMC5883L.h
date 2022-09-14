#ifndef Custom_QMC5883L
#define Custom_QMC5883L

#include "Arduino.h"
#include "Wire.h"


class CustomQMC5883L {

	public:
		CustomQMC5883L();
		typedef struct {
			int Xaxis;
			int Yaxis;
			int Zaxis;
		} magdata;		
		double calibration_matrix[3][3];
		double bias[3];
		
		bool init();
		void setADDR(uint8_t b);
		void setMode(byte mode, byte odr, byte rng, byte osr);
		void setSmoothingSteps(uint8_t steps);
		void useCalibration(bool a = true);
		void setReset();
		void read();
		int getX();
		int getY();
		int getZ();
		int getCompensatedAzimuth(float pitch,float roll);
		int getAzimuth(void);


	private:

		float magDeclination = -6.433;
		bool _firstSmooth = true;
		bool _smoothUse = false;
		bool _calibrationUse = false;
		uint8_t _ADDR = 0x0D;  //default qmc5883 addr
		byte _smoothSteps;

		magdata historyValues[20] = {0,0,0};
		magdata magValues;
		uint8_t scannCount = 0;		
		void writeReg(byte reg, byte val);
		uint8_t readRegister8(uint8_t reg);
		CustomQMC5883L::magdata _smoothing(magdata nonSmooth);
		CustomQMC5883L::magdata _applyCalibration(magdata rawValues);

};

#endif