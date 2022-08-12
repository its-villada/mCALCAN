#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <STM32_ISR_Timer.hpp>
#include <LoRa_STM32.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050_light.h>

SFE_BMP180 pressure;
MPU6050 mpu(Wire);

double baseline;
double T, P, A, giroX, giroY, giroZ, accX, accY, accZ;
unsigned int pktNumber = 0;
bool transmit = 0, sensors = 0, bmpIsInit = 0, actualizarMpu = 0, leerMpu = 0;
int disparoMedicion = 0 , batteryLevel;

#define HW_TIMER_INTERVAL_MS 1

#define INTpin PB8
#define BAT PA0

// Init STM32 timer TIM1 and TIM2
STM32Timer Temp(TIM1);
STM32Timer LoRaTx(TIM2);

STM32_ISR_Timer ISR_Timer1_Temp;
STM32_ISR_Timer ISR_Timer2_LoRaTx;

#define TIMER_INTERVAL_1 5L   // TIMER1 salta cada 5 ms
#define TIMER_INTERVAL_2 500L // Timer2 salta cada 500 ms

void TimerHandler()
{
  ISR_Timer1_Temp.run();
  ISR_Timer2_LoRaTx.run();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT); // FUNCIONA EN LOGICA INVERSA!
  pinMode(BAT, INPUT);
  pinMode(INTpin, INPUT);

  loraSetup(); // Setup modulo lora

  bmpSetup(); // Setup modulo de temperatura

  mpuSetup(); // Setup modulo de giroscopio

  timerSetup(); // Setup de interrupciones de timer

  attachInterrupt(digitalPinToInterrupt(INTpin), readMPU, FALLING);
}

void loop()
{
  if (transmit == 1)
  {
    packetSending();
    transmit = 0;
  }
  if (sensors == 1)
  {
    readSensors();
    sensors = 0;
  }
  if (actualizarMpu == 1)
  {
    mpu.update();
    actualizarMpu = 0;
  }
  
  if (leerMpu == 1)
  {
    leerGyro();
    leerMpu = 0;
  }
}

void readSensors()
{
  // Funcion que lee la temperatura y presion del módulo BMP180 y calcula la Altura aproximada.
  char status;
  if (!bmpIsInit)
  {
    status = pressure.startTemperature();
    if (status != 0)
    {
      // Espera que termine la medicion de Temperatura
      delay(status);
      status = pressure.getTemperature(T);
      if (status != 0)
      {
        // Comienza medicion de Presion
        status = pressure.startPressure(3);
        if (status != 0)
        {
          // Espera que termine la medicion de Presion
          delay(status);
          status = pressure.getPressure(P, T);
        }
      }
    }
  }
  else
  {
    actualizarMpu = 1;
    batteryLevel = analogRead(BAT);
    switch (disparoMedicion)
    {
    case 0:
      status = pressure.startTemperature();
      if (status != 0)
        disparoMedicion++;

      break;

    case 1:

      status = pressure.getTemperature(T);
      disparoMedicion++;
      break;

    case 2:
      status = pressure.startPressure(0);
      if (status != 0)
        disparoMedicion++;
      break;

    case 3:
      status = pressure.getPressure(P, T);
      disparoMedicion++;
      break;

    default:
      break;
    }

    if (disparoMedicion >= 4)
      disparoMedicion = 0;
  }
}

void leerGyro()
{
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();
  giroX = mpu.getAngleX();
  giroY = mpu.getAngleY();
  giroZ = mpu.getAngleZ();
}

void loraSetup()
{
#define RST PC14             // Pin de Reset del módulo LoRa
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE      // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 17            // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7     // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5       // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  // Inicializar módulo LoRa
  while (!LoRa.begin(LORA_FREQUENCY))
    ;
  LoRa.setTxPower(LORA_POWER);
  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIG_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void bmpSetup()
{
  // Inicializar el BMP180
  // Si el BMP180 no inicializa, no arranca la placa y el led onboard queda encendido!
  digitalWrite(LED_BUILTIN, LOW);
  do
  {
    delay(100);
    if (pressure.begin())
      readSensors();
    baseline = P;
    bmpIsInit = true;
    digitalWrite(LED_BUILTIN, HIGH);
  } // LoRa OK!, Iniciando BMP180, BMP180 inicio OK, baseline
  while (!bmpIsInit);
}

void timerSetup()
{
  // Timer1, lectura de temperatura.
  Temp.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_1, sensorsBegin);

  // Timer2, lora transmit
  LoRaTx.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  ISR_Timer2_LoRaTx.setInterval(TIMER_INTERVAL_2, transmitir);
}

void mpuSetup()
{
  byte status = mpu.begin();

  while (status != 0)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // stop everything if could not connect to MPU6050
    delay(1000);
  }

  mpu.writeData(0x38, 0x01);
  mpu.writeData(0x37, 0xB0);

  mpu.calcOffsets(true, true); // gyro and accelero
}

void packetSending()
{
  // Send LoRa packet to receiver
  digitalWrite(PC13, LOW);
  LoRa.beginPacket();
  LoRa.print("1234"); // Cadena de comienzo de packet
  LoRa.print(",");
  LoRa.print(T);
  LoRa.print(",");
  LoRa.print(P);
  LoRa.print(",");
  LoRa.print(giroX);
  LoRa.print(",");
  LoRa.print(giroY);
  LoRa.print(",");
  LoRa.print(giroZ);
  LoRa.print(",");
  LoRa.print(accX);
  LoRa.print(",");
  LoRa.print(accY);
  LoRa.print(",");
  LoRa.print(accZ);
  LoRa.print(",");
  LoRa.print(batteryLevel);
  LoRa.print(",");
  LoRa.print(baseline);
  LoRa.print(",");
  LoRa.print(millis());
  LoRa.print(",");
  LoRa.print("4321"); // Cadena de fin de packet
  LoRa.endPacket();
  digitalWrite(PC13, HIGH);
  // Volver a 0 para que no haga overflow
  if (pktNumber >= 65500)
    pktNumber = 0;
  pktNumber++;
}

void transmitir()
{
  transmit = 1;
}

void sensorsBegin()
{
  sensors = 1;
}

void readMPU()
{
  leerMpu = 1;
}
