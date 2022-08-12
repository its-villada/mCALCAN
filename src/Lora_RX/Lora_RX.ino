#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <SFE_BMP180.h>

SFE_BMP180 pressure;

// Defino los pines a ser usados por el modulo transceptor LoRa
#define CS 8                 // Pin de CS del módulo LoRa
#define RST 4                // Pin de Reset del módulo LoRa
#define IRQ 7                // Pin del IRQ del módulo LoRa
#define LED 13               // Pin del LED onboard
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define LORA_FREQUENCY 915000000

#define LORA_SYNC_WORD 0xDE

#define LORA_POWER 17 // TX power in dB, defaults to 17.

#define LORA_SPREAD_FACTOR 7 // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)

#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62. 5E3, 125E3, 250E3, and 500E3

#define LORA_CODING_RATE 5

#define DATA_TIME 500

double altitud;

String input;

int missionTime = 0;

void setup()
{
  // Set Led onboard con Output
  pinMode(LED, OUTPUT);
  // Incializo el Serial Monitor
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
  {
    // Mientras el COM no esté disponible el LED onbooard encendido
    digitalWrite(LED, HIGH);
  }
  // Apaga el LED si se conecta al COM
  digitalWrite(LED, LOW);
  Serial.println("LoRa Receiver");
  // Inicializar módulo LoRa
  LoRa.setPins(CS, RST, IRQ);
  while (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println(".");
    delay(DATA_TIME);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_POWER);
  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIG_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);

  Serial.println("LoRa Initializing OK!");
}

void loop()
{
  // los datos deben imprimirse en el siguiente orden para
  // ser interpretados por la estación terrena
  // 0 - tiempo mision (ms)
  // 1 - altitud (m)
  // 2 - caída libre (1,0)
  // 3 - temperatura (°C)
  // 4 - presión
  // 5 - giro
  // 6 - giro
  // 7 - giro
  // 8 - velocidad (m/s)
  // 9 - velocidad (m/s)
  // 10 - velocidad (m/s)
  // 11 - nivel de batería

  // Trato de parsear el paquete
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // Encender LED onboard
    digitalWrite(LED, HIGH);
    // Paquete recibido
    // Lectura del paquete
    while (LoRa.available())
    {
      String LoRaData = LoRa.readString();

      int confirmacion1 = LoRaData.indexOf(',');
      int indicador1 = LoRaData.indexOf(',', confirmacion1 + 1);
      int indicador2 = LoRaData.indexOf(',', indicador1 + 1);
      int indicador3 = LoRaData.indexOf(',', indicador2 + 1);
      int indicador4 = LoRaData.indexOf(',', indicador3 + 1);
      int indicador5 = LoRaData.indexOf(',', indicador4 + 1);
      int indicador6 = LoRaData.indexOf(',', indicador5 + 1);
      int indicador7 = LoRaData.indexOf(',', indicador6 + 1);
      int indicador8 = LoRaData.indexOf(',', indicador7 + 1);
      int indicador9 = LoRaData.indexOf(',', indicador8 + 1);
      int indicador10 = LoRaData.indexOf(',', indicador9 + 1);
      int indicador11 = LoRaData.indexOf(',', indicador10 + 1);
      int confirmacion2 = LoRaData.indexOf(',', indicador11 + 1);
      String caidaLibre = "1";

      String strconf1 = LoRaData.substring(0, confirmacion1);
      String strconf2 = LoRaData.substring(indicador11 + 1, confirmacion2);

      if (strconf1 == "1234" && strconf2 == "4321")
      { // Verifica si la informacion reciciba viene de nuestro lora a partir de la cadena de comienzo y fin
        // tiempo de la misión
        //missionTime = missionTime + DATA_TIME;
        //String time;
        //time = String(missionTime);

        String temperatura = LoRaData.substring(confirmacion1 + 1, indicador1);

        String presion = LoRaData.substring(indicador1 + 1, indicador2);

        String giro1 = LoRaData.substring(indicador2 + 1, indicador3);
        String giro2 = LoRaData.substring(indicador3 + 1, indicador4);
        String giro3 = LoRaData.substring(indicador4 + 1, indicador5);

        String vel1 = LoRaData.substring(indicador5 + 1, indicador6);
        String vel2 = LoRaData.substring(indicador6 + 1, indicador7);
        String vel3 = LoRaData.substring(indicador7 + 1, indicador8);

        String bat = LoRaData.substring(indicador8 + 1, indicador9);

        String presionBase = LoRaData.substring(indicador9 + 1, indicador10);

        String tiempo = LoRaData.substring(indicador10 + 1, indicador11);

        altitud = pressure.altitude(presion.toDouble(), presionBase.toDouble());

        //Calculo de valores crudos recibidos desde el satelite

        String sAltitud = String(altitud, 2);

        bat = ((bat.toInt() * 100) / 1023);

        vel1 = vel1.toDouble() * 9.8066;
        vel2 = vel2.toDouble() * 9.8066;
        vel3 = vel3.toDouble() * 9.8066;

        //Printeo de los datos recibidoscaidaLibre

        Serial.println(tiempo + "," + sAltitud + "," + caidaLibre +"," + temperatura + "," + presion + "," + giro1 + "," + giro2 + "," + giro3 + "," + vel1 + "," + vel2 + "," + vel3 + "," + bat);
        // Apagar LED onboard
        digitalWrite(LED, LOW);
      }
    }
  }
}
