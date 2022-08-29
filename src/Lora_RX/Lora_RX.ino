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
bool responder = false, haRespondido = false, noPudoProcesar = false, escribirDatos = false, ejecutarAccion = false, inicializacion = false, error = false, cansatReset = false, telemetria = false;

String LoRaData, presionBase, data = "";

String tiempoDeTransmision, tiempoAnterior;

int pktNumber = 0;

void (*resetFunc)(void) = 0;

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
    LoRa.onReceive(onReceive);
    LoRa.receive();
}

void loop()
{
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
            haRespondido = true; // Significa que el paquete enviado es para confirmar un comando
            break;

        case 3:
            noPudoProcesar = true; // Significa que el paquete enviado pudo ser procesado
            break;

        case 4:
            error = true;
            break;

        default:
            break;
        }

        if (!haRespondido && !noPudoProcesar)
        {
            switch (codigomsg.toInt())
            {

            case 17:
                escribirDatos = true; // El mensaje recibido es para escribir datos
                break;

            case 27:
                ejecutarAccion = true; // EL mensaje recibido es para ejecutar una accion
                break;

            case 37:
                resetFunc(); // El mensaje recibido es para resetear el sistema
                break;

            case 10:
                inicializacion = true; // El mensaje recibido es para inicializar un dato
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
        }

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

            calidadDeAire = (calidadDeAire.toInt() - 282);

            altitud = pressure.altitude(presion.toDouble(), presionBase.toDouble());

            // Calculo de valores crudos recibidos desde el satelite

            String sAltitud = String(altitud, 2);

            if (sAltitud.toDouble() >= 10)
            {
                sAltitud = sAltitud.toDouble() - 10;
            }

            bat = ((bat.toInt() * 100) / 1023);

            vel1 = vel1.toDouble() * 9.8066;
            vel2 = vel2.toDouble() * 9.8066;
            vel3 = vel3.toDouble() * 9.8066;

            tiempoDeTransmision = (tiempo.toDouble() - tiempoAnterior.toDouble());

            // Printeo de los datos recibidos

            Serial.println(tiempo + "," + sAltitud + "," + caidaLibre + "," + temperatura + "," + presion + "," + giro1 + "," + giro2 + "," + giro3 + "," + vel1 + "," + vel2 + "," + vel3 + "," + bat + "," + calidadDeAire + "," + humedad + "%" + "," + tiempoDeTransmision);
            tiempoAnterior = tiempo;
            // Apagar LED onboard
            digitalWrite(LED, LOW);
            telemetria = false;
        }
        if (inicializacion)
        {
            int presion = LoRaData.indexOf(',', codigo2 + 1);
            presionBase = LoRaData.substring(codigo2 + 1, presion);
            if (presionBase.toInt() <= 500)
            {
                Serial.println("Error obteniendo la presion Base");
                responder = false;
            }
            else
            {
                Serial.println("Presion Base:" + presionBase);
                data = presionBase;
                responder = true;
                inicializacion = false;
            }
        }
        if (responder)
        {
            confirmacion(data, codigomsg);
            responder = false;
        }
        if (error)
        {
            int indicador1 = LoRaData.indexOf(',', codigo2 + 1);
            String sError = LoRaData.substring(codigo2 + 1, indicador1); // Significa que hubo un error en el cansat, se reinicia el sistema
            Serial.println(sError);
            error = false;
        }
        if (cansatReset = true)
        {
            reset();
        }
    }
}

void confirmacion(String datos, String codigomsg)
{
    digitalWrite(LED_BUILTIN, HIGH);
    LoRa.beginPacket();
    LoRa.print("gvie");
    LoRa.print(",");
    LoRa.print("2");
    LoRa.print(",");
    LoRa.print(codigomsg);
    LoRa.print(",");
    LoRa.print(datos);
    LoRa.endPacket(true);
    LoRa.receive();
    digitalWrite(LED_BUILTIN, LOW);
}

void reset()
{
    digitalWrite(LED_BUILTIN, HIGH);
    LoRa.beginPacket();
    LoRa.print("gvie");
    LoRa.print(",");
    LoRa.print("1");
    LoRa.print(",");
    LoRa.print("3X");
    LoRa.endPacket(true);
    LoRa.receive();
    digitalWrite(LED_BUILTIN, LOW);
}
