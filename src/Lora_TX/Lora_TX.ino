#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <STM32_ISR_Timer.hpp>
#include <LoRa_STM32.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>

SFE_BMP180 pressure;

double baseline;
double T, P, A;
unsigned int pktNumber = 0;
bool transmit = 0;

#define TIMER_INTERVAL_MS         100
#define HW_TIMER_INTERVAL_MS      50

// Init STM32 timer TIM1
STM32Timer Temp(TIM1);
STM32Timer LoRaTx(TIM2);

STM32_ISR_Timer ISR_Timer1_Temp;
STM32_ISR_Timer ISR_Timer2_LoRaTx;

#define TIMER_INTERVAL_1           1L   //TIMER1 salta cada 1 ms
#define TIMER_INTERVAL_2           500L // Timer2 salta cada 500 ms


void TimerHandler()
{
  ISR_Timer1_Temp.run();
  ISR_Timer2_LoRaTx.run();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN,  OUTPUT);  // FUNCIONA EN LOGICA INVERSA!

  loraSetup();  //Setup modulo lora

  bmpSetup();   //Setup modulo de temperatura
 
  timerSetup(); // Setup de interrupciones de timer
}

void loop() {
  
}

void readTempPressure() {
  // Funcion que lee la temperatura y presion del módulo BMP180 y calcula la Altura aproximada.
  char status;
  // Comienza medicion de Temperatura
  status = pressure.startTemperature();
  if (status != 0) {
    // Espera que termine la medicion de Temperatura
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      // Comienza medicion de Presion
      status = pressure.startPressure(3);
      if (status != 0) {
        // Espera que termine la medicion de Presion
        delay(status);
        status = pressure.getPressure(P, T);
      }
      else Serial.println("Error iniciando medicion presion\n");
    }
    else Serial.println("Error leyendo temperatura\n");
  }
  else Serial.println("Error iniciando medicion temperatura\n");
}

void loraSetup() {

#define RST PC14 // Pin de Reset del módulo LoRa
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 17 // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7 // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5 // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  //Inicializar módulo LoRa
  while (!LoRa.begin(LORA_FREQUENCY));
  LoRa.setTxPower(LORA_POWER);
  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIG_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void bmpSetup() {
  // Inicializar el BMP180
  bool bmpIsInit = false;
  // Si el BMP180 no inicializa, no arranca la placa y el led onboard queda encendido!
  digitalWrite(LED_BUILTIN, LOW);
  do {
    delay(100);
    if (pressure.begin())
      readTempPressure();
    baseline = P;
    bmpIsInit = true;
    digitalWrite(LED_BUILTIN, HIGH);
  } // LoRa OK!, Iniciando BMP180, BMP180 inicio OK, baseline
  while (!bmpIsInit);
}

void timerSetup() {
  // Timer1, lectura de temperatura.
  Temp.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_1, readTempPressure);

  //Timer2, lora transmit
  LoRaTx.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  ISR_Timer2_LoRaTx.setInterval(TIMER_INTERVAL_2, packetSending);
}

void packetSending() {
  //Send LoRa packet to receiver
  digitalWrite(LED_BUILTIN, LOW);
  LoRa.beginPacket();
  LoRa.print("1234"); // Cadena de comienzo de packet
  LoRa.print(",");
  LoRa.print(pktNumber);
  LoRa.print(",");
  LoRa.print(baseline);
  LoRa.print(",");
  LoRa.print(P);
  LoRa.print(",");
  LoRa.print(A);
  LoRa.print(",");
  LoRa.print(T);
  LoRa.print(",");
  LoRa.print("4321"); // Cadena de fin de packet
  LoRa.endPacket();
  digitalWrite(LED_BUILTIN, HIGH);
  // Volver a 0 para que no haga overflow
  if (pktNumber >= 65500)
  {
    pktNumber = 0;
  }
  pktNumber++;
}
