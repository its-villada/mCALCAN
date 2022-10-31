/* 
Codigo hecho por alumnos del ITS Villada 7° año B, Electronica. Año 2022
Placa: Adafruit LoRa 32u4

*/

#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <EEPROM.h>

// Defino los pines a ser usados por el modulo transceptor LoRa
#define CS 8                 // Pin de CS del módulo LoRa
#define RST 4                // Pin de Reset del módulo LoRa
#define IRQ 7                // Pin del IRQ del módulo LoRa
#define LED 13               // Pin del LED onboard
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define LORA_FREQUENCY 915000000
#define LORA_SYNC_WORD 0xDE
#define LORA_POWER 19            // TX power in dB, defaults to 17.
#define LORA_SPREAD_FACTOR 7     // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62. 5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5
#define DATA_TIME 500

#define comma ','

#define intervalo 1500

bool responder = false, error = false, cansatReset = false, telemetria = false, serial = false;
uint8_t comandoFracasado;
String LoRaData, presion, data = "", recepcionSerial, latencia;

void (*resetFunc)(void) = 0;
void LoRa_Transmit(uint8_t, uint8_t, String);

void setup()
{
    // Set Led onboard con Output
    pinMode(LED, OUTPUT);
    // Incializo el Serial Monitor
    Serial.begin(SERIAL_BAUDRATE);
    EEPROM.begin();
    // Turn on the transmission, reception, and Receive interrupt
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
    LoRa.onReceive(onReceive);
    LoRa.receive();
}

void loop()
{
    if (Serial.available())
    {
        recepcionSerial = Serial.readString();
        recepcionSerial.trim();
        serial = true;
    }
    if (serial)
    {
        LoRa.beginPacket();
        LoRa.print(recepcionSerial);
        LoRa.endPacket(true);
        LoRa.receive();
        serial = false;
    }
}

void onReceive(int packetSize)
{
    String LoRaData = LoRa.readString();

    int confirmacion0 = LoRaData.indexOf(',');            // Header
    int tipo1 = LoRaData.indexOf(',', confirmacion0 + 1); // Tipo de mensaje
    int codigo2 = LoRaData.indexOf(',', tipo1 + 1);       // Codigo de solicitud/respuesta

    String strconf1 = LoRaData.substring(0, confirmacion0);
    String tipomsg = LoRaData.substring(confirmacion0 + 1, tipo1);
    String codigomsg = LoRaData.substring(tipo1 + 1, codigo2);

    // Desglosaje del protocolo GVIE
    if (strconf1 == "gvie")
    { // Verifica si la informacion reciciba viene de nuestro lora a partir de la cadena de comienzo

        switch (tipomsg.toInt()) // Puede ser 0, 1, 2, o 3
        {
        case 0:
            responder = true; // Se espera que el mensaje sea de tipo confirmable (Se confirma con "OK")
            break;

        case 1:
            responder = false; // Se espera que el mensaje sea de tipo no confirmable
            break;

        case 2:
            Serial.println(LoRaData);
            break;

        case 3:
            Serial.println(LoRaData);
            break;

        case 4:
            error = true;
            break;

        default:
            break;
        }

        switch (codigomsg.toInt())
        {

        case 37:
            resetFunc(); // El mensaje recibido es para resetear el sistema
            break;

        case 14:
            telemetria = true;
            break;

        case 77:
            // Indefinido
            break;

        default:
            break;
        }

        // Recepcion de telemetria raw
        if (telemetria)
        {
            int indicador1 = LoRaData.indexOf(',', codigo2 + 1); // Datos
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
            int indicador12 = LoRaData.indexOf(',', indicador11 + 1);
            int indicador13 = LoRaData.indexOf(',', indicador12 + 1);
            int indicador14 = LoRaData.indexOf(',', indicador13 + 1);
            int indicador15 = LoRaData.indexOf(',', indicador14 + 1);
            int indicador16 = LoRaData.indexOf(',', indicador15 + 1);
            int indicador17 = LoRaData.indexOf(',', indicador16 + 1);

            String temperatura = LoRaData.substring(codigo2 + 1, indicador1);

            String presion = LoRaData.substring(indicador1 + 1, indicador2);

            String giro1 = LoRaData.substring(indicador2 + 1, indicador3);
            String giro2 = LoRaData.substring(indicador3 + 1, indicador4);
            String giro3 = LoRaData.substring(indicador4 + 1, indicador5);

            String vel1 = LoRaData.substring(indicador5 + 1, indicador6);
            String vel2 = LoRaData.substring(indicador6 + 1, indicador7);
            String vel3 = LoRaData.substring(indicador7 + 1, indicador8);

            String bat = LoRaData.substring(indicador8 + 1, indicador9);

            String tiempo = LoRaData.substring(indicador9 + 1, indicador10);

            String caidaLibre = LoRaData.substring(indicador10 + 1, indicador11);

            String calidadDeAire = LoRaData.substring(indicador11 + 1, indicador12);

            String humedad = LoRaData.substring(indicador12 + 1, indicador13);

            String latitud = LoRaData.substring(indicador13 + 1, indicador14);

            String longitud = LoRaData.substring(indicador14 + 1, indicador15);

            String bat2 = LoRaData.substring(indicador15 + 1, indicador16);

            String altitud = LoRaData.substring(indicador16 + 1, indicador17);

            calidadDeAire = calidadDeAire.toInt() - 355;


            // Calculo de valores crudos recibidos desde el satelite

            bat = ((bat.toInt() * 3.7) / 1023);

            bat2 = ((bat2.toInt() * 3.7) / 1023);

            vel1 = vel1.toDouble() * 9.8066;
            vel2 = vel2.toDouble() * 9.8066;
            vel3 = vel3.toDouble() * 9.8066;

            // Printeo de los datos recibidos
            Serial.println("gvie,1,14," + tiempo + "," + altitud + "," + caidaLibre + "," + temperatura + "," + presion + "," + giro1 + "," + giro2 + "," + giro3 + "," + vel1 + "," + vel2 + "," + vel3 + "," + humedad + "," + calidadDeAire + "," + latitud + "," + longitud + "," + bat + "," + bat2);
            telemetria = false;
        }
        // Se responde a algun mensaje
        if (responder)
        {
            confirmacion(codigomsg.toInt());
            responder = false;
        }
        // Significa que hubo un error en el cansat
        if (error)
        {
            int indicador1 = LoRaData.indexOf(',', codigo2 + 1);
            String sError = LoRaData.substring(codigo2 + 1, indicador1);
            Serial.println(sError);
            error = false;
        }
        // Se envia el reset del cansat
        if (cansatReset)
            resetCansat();
    }
}

// Funcion para confirmar algun proceso
void confirmacion(uint8_t codigomsg)
{
    digitalWrite(LED_BUILTIN, HIGH);
    LoRa_Transmit(2, codigomsg, "");
    digitalWrite(LED_BUILTIN, LOW);
}

// Funcion para resetear el cansat
void resetCansat()
{
    digitalWrite(LED_BUILTIN, HIGH);
    LoRa_Transmit(1, 37, "");
    digitalWrite(LED_BUILTIN, LOW);
}

// Funcion estandard para enviar mensajes por LoRa con el Protocolo gVIE
void LoRa_Transmit(uint8_t type, uint8_t reqs, String data)
{
    digitalWrite(LED_BUILTIN, LOW);
    LoRa.beginPacket();
    LoRa.print(String("gvie," + String(type) + comma + String(reqs) + comma + data));
    LoRa.endPacket(true);
    LoRa.receive();
    digitalWrite(LED_BUILTIN, HIGH);
}
