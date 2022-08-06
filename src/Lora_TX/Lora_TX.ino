#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

SFE_BMP180 pressure;

#define CS 8 // Pin de CS del módulo LoRa
#define RST 4 // Pin de Reset del módulo LoRa
#define IRQ 7 // Pin del IRQ del módulo LoRa
#define LED 13 // Pin del LED onboard
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define INTERVAL_TIME_TX 1000 // Cantidad de Segundos entre cada TX

// Configuraciones del módulo LoRa. Tener en cuenta que esta configuración debe ser igual en el Rx!
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
// 915E6 to 928E3 for Argentina
#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 17 // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7 // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5 // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.

double baseline;
double T, P, A;
unsigned int pktNumber = 0;

void setup()
{
  // Set Led onboard con Output
  pinMode(LED, OUTPUT);
  // Set de pin RST como Output y se pone a High para que pueda operar
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  //Inicializar módulo LoRa
  LoRa.setPins(CS, RST, IRQ);
  while (!LoRa.begin(LORA_FREQUENCY));
  LoRa.setTxPower(LORA_POWER);
  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIG_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  // Inicializar el BMP180
  bool bmpIsInit = false;
  // Si el BMP180 no inicializa, no arranca la placa y el led onboard queda encendido!
  digitalWrite(LED, HIGH);
  do {
    delay(100);
    if (pressure.begin())
      readTempPressure();
    baseline = P;
    bmpIsInit = true;
    digitalWrite(LED, LOW);
  } // LoRa OK!, Iniciando BMP180, BMP180 inicio OK, baseline
  while (!bmpIsInit);
}

void loop()
{
  // Leer Presion en hectopascales, Temperatura en celsius y calcular Altura en metros
  readTempPressure();
  //Send LoRa packet to receiver
  digitalWrite(LED, HIGH);
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
  digitalWrite(LED, LOW);
  // Volver a 0 para que no haga overflow
  if (pktNumber >= 65500)
  {
    pktNumber = 0;
  }
  pktNumber++;
  // Tiempo de espera entre Tx y Tx
  delay(INTERVAL_TIME_TX);
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
        // Sino hubo error al obtener la Presion, calcular la altura
        if (status != 0) {
          // Calcular altura
          A = pressure.altitude(P, baseline);
          if (A <= 0) {
            A = 0;
          }
        }
        else {
          Serial.println("Error leyendo medicion\n");
        }
      }
      else Serial.println("Error iniciando medicion presion\n");
    }
    else Serial.println("Error leyendo temperatura\n");
  }
  else Serial.println("Error iniciando medicion temperatura\n");
}
