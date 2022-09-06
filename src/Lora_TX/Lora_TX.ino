#include <AHTxx.h>
#include <EEPROM.h>
#include <LoRa.h>
#include <MPU6050_light.h>
#include <SFE_BMP180.h>
#include <SPI.h>
#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <STM32_ISR_Timer.hpp>
#include <SD.h>
#include <Wire.h>

#define SERIAL_BAUDRATE 9600     // Velocidad del Puerto Serie
#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE      // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 17            // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7     // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5       // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.
#define RST PB3                  // Pin Reset del transciver LoRa
#define IRQ PA1                  // Pin DIO0 del transciver LoRa
#define NSS PA15                 // Pin CS del transciver LoRa
#define INTpin PA2               // Pin de interrupcion del MPU6050
#define DRDY PA11                // Pin de interrupcion del GY-273
#define BAT PA0                  // Pin de tension de bateria
#define MainS PA12               // Pin Sensors enable
#define AQS PA4                  // Pin del sensor MQ-135
#define buzzer PA8               // Pin del buzzer de recuperacion
#define servoDerPin PB1          // Pin del servo derecha
#define servoIzqPin PB0          // Pin del servo izquierda
#define HW_TIMER_INTERVAL_MS 1
#define TIMER_INTERVAL_1 5L   // Intervalo1 salta cada 5 ms
#define TIMER_INTERVAL_2 500L // Intervalo2 salta cada 500 ms
#define TIMER_INTERVAL_3 50L  // Intervalo3 salta cada 50 ms
#define comma ','
#define memInit 0
#define memMision 1
#define memDesc 2
#define memLat 3
#define memLong 4

File csvTelemetry;
STM32Timer Timer1(TIM1);
STM32_ISR_Timer ISR_Timer1_Temp;
SPIClass mySPI_2(PB15, PB14, PB13);
SFE_BMP180 pressure;
MPU6050 mpu(Wire);
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); // sensor address, sensor type

float giroX, giroY, giroZ, accX, accY, accZ, ahtValue;
double T, P, pAnterior, latitud, longitud;
bool transmit = false, sensors = false, bmpIsInit = false, actualizarMpu = false, leerMpu = false, isFreeFall = false, responder = false, haRespondido = false, noPudoProcesar = false, escribirDatos = false, ejecutarAccion = false, inicializacion = false, inicializado = false, enviarDevuelta = true, escritura = false, coordenadas = false, listoParaDespegar = false, descenso = false, wipeEeprom = false, leerCompass = false, actualizarCompass = false, iniciarMision = false;
uint8_t disparoMedicion = 0, readAht = 0, cicloAht = 0, preparacion = 0;
uint16_t MQ135, batteryLevel;
uint32_t humidity, tiempoMision;
String latitudAterrizaje, longitudAterrizaje;

void Timer1Handler();
void onReceive(int);
void telemetrySend();
void enviarPresionBase();
void reportar(String);
void readSensors();
void beginMeasurementAht();
void readHumidityAht();
void leerMPU();
void loraSetup();
void bmpSetup();
void timerSetup();
void mpuSetup();
void aht10Setup();
void sdCardSetup();
void transmitir();
void sensorsBegin();
void readMPU();
void escribirArchivo();
void errorCheck(char);
bool checkSensorStatus(char);
void LoRa_Transmit(uint8_t, uint8_t, String);
void putEepromFloat(uint8_t, float);
float getEepromFloat(uint8_t);
void song(int);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();
    EEPROM.begin();
    pinMode(LED_BUILTIN, OUTPUT); // FUNCIONA EN LOGICA INVERSA!
    pinMode(MainS, OUTPUT);
    pinMode(BAT, INPUT);
    pinMode(INTpin, INPUT);
    pinMode(DRDY, INPUT);
    pinMode(AQS, INPUT);
    pinMode(buzzer, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    inicializacion = EEPROM.read(memInit);

    if (inicializacion)
    {
        descenso = EEPROM.read(memDesc);
        iniciarMision = EEPROM.read(memMision);
        latitudAterrizaje = EEPROM.read(memLat);
        longitudAterrizaje = EEPROM.read(memLong);
        digitalWrite(MainS, HIGH);
        sdCardSetup();
        bmpSetup();
        aht10Setup();
        mpuSetup();
        actualizarMpu = true;
        inicializado = true;
    }

    loraSetup(); // Setup modulo lora

    timerSetup(); // Setup de interrupciones de timer

    attachInterrupt(digitalPinToInterrupt(INTpin), readMPU, FALLING);
    LoRa.receive();
}

void loop()
{
    if (!inicializado)
    {
        if (inicializacion)
        {
            digitalWrite(MainS, HIGH);
            sdCardSetup();
            bmpSetup();
            aht10Setup();
            mpuSetup();
            while (!haRespondido)
                enviarPresionBase();
            actualizarMpu = true;
            haRespondido = false;
            inicializado = true;
            EEPROM.update(memInit, true);
        }
    }
    else
    {
        if ((listoParaDespegar) || (iniciarMision))
        {
            if (cicloAht == 1)
                beginMeasurementAht();

            if (cicloAht == 2)
            {
                readHumidityAht();
                cicloAht = 0;
            }
            if (transmit)
            {
                telemetrySend();
                transmit = false;
            }
            if (sensors)
            {
                readSensors();
                sensors = false;
            }
            if (actualizarMpu)
            {
                mpu.update();
                actualizarMpu = false;
            }
            if (leerMpu)
            {
                byte data = mpu.readData(0x3A);
                if (data == 0x81)
                {
                    leerMPU();
                    isFreeFall = true;
                    descenso = true;
                    EEPROM.update(memDesc, true);
                }
                else if (data == 0x01)
                    leerMPU();
                else if (data == 0x80)
                {
                    isFreeFall = true;
                    descenso = true;
                    EEPROM.update(memDesc, true);
                }
                actualizarMpu = true;
                leerMpu = false;
            }
            if (escritura)
            {
                if (csvTelemetry)
                {
                    if (!transmit)
                    {
                        csvTelemetry.println(String(String(millis()) + comma + String(isFreeFall) + comma + String(T) + comma + String(P) + comma + String(giroX) + comma + String(giroY) + comma + String(giroZ) + comma + String(accX) + comma + String(accY) + comma + String(accZ) + comma + String(ahtValue) + comma + String(MQ135) + comma + latitud + comma + longitud + comma + String(batteryLevel)));
                        csvTelemetry.flush();
                        escritura = false;
                    }
                }
                else
                {
                    reportar("Fallo al abrir archivo de MicroSD");
                    escritura = false;
                }
            }
        }
    }
}

// Inicializacion del Timer
void timerSetup()
{
    // Timer1, lectura de temperatura y transmision
    Timer1.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, Timer1Handler);
    ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_1, sensorsBegin);    // Intervalo de timer para los ciclos de lectura de sensor
    ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_2, transmitir);      // Intervalo de timer para la transmision de datos a la ET
    ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_3, escribirArchivo); // Intervalo de timer para escribir a la microsd
}

// Inicializacion del LoRa
void loraSetup()
{
    pinMode(RST, OUTPUT);
    digitalWrite(RST, HIGH);
    LoRa.setPins(NSS, RST, IRQ);
    // Inicializar módulo LoRa
    while (!LoRa.begin(LORA_FREQUENCY))
        ;
    LoRa.setTxPower(LORA_POWER);
    LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIG_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);

    LoRa.onReceive(onReceive);
    LoRa.receive();
}

// Inicializacion del MPU6050
void mpuSetup()
{
    errorCheck(1);

    mpu.writeData(0x38, 0x81);
    mpu.writeData(0x1D, 17);
    mpu.writeData(0x1E, 2);
    mpu.writeData(0x37, 0xB0);
    mpu.calcOffsets(true, true); // gyro and accelero
}

// Inicializacion del BMP180
void bmpSetup()
{
    errorCheck(3);
    readSensors();
    bmpIsInit = true;
}

// Inicializacion del AHT10
void aht10Setup()
{
    errorCheck(2);
}

// Inicializacion de la MicroSD
void sdCardSetup()
{
    if (!SD.begin(SS2))
    {
        reportar("No se pudo iniciar MicroSD1");
        while (1)
            ;
    }
    csvTelemetry = SD.open("mCALCAN.csv", FILE_WRITE);
}

// Subortina ISR para interrupciones
void Timer1Handler()
{
    ISR_Timer1_Temp.run();
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

// Funcion de transmision de datos a la Estacion Terrena
void transmitir()
{
    transmit = true; // Se transmiten datos a la estacion terrena
    if (!haRespondido)
    {
        enviarDevuelta = !enviarDevuelta; // Si han pasado 1000 ms, y no se respondio al comando, se vuelve a enviar
    }
    if (inicializado && listoParaDespegar)
        preparacion++;
    if (preparacion >= 31)
    {
        listoParaDespegar = false;
        preparacion = 0;
    }
}

// Funcion que envia la telemetria por el modulo LoRa a la estacion terrena
void telemetrySend()
{
    LoRa_Transmit(1, 14, String(String(T) + comma + String(P) + comma + String(giroX) + comma + String(giroY) + comma + String(giroZ) + comma + String(accX) + comma + String(accY) + comma + String(accZ) + comma + String(batteryLevel) + comma + String(millis() - tiempoMision) + comma + String(isFreeFall) + comma + String(MQ135) + comma + String(ahtValue) + comma + String(latitud) + comma + String(longitud)));
    if (isFreeFall)
    {
        isFreeFall = false;
    }
}

// Funcion encargada de realizar las operaciones necesarias al recibir informacion por el modulo LoRa
void onReceive(int packetSize)
{
    String LoRaData = LoRa.readString();
    Serial.println(LoRaData);

    // Busca y almacena la posicion (estilo arreglo) del caracter ingresado + X posiciones
    uint8_t confirmacion0 = LoRaData.indexOf(',');            // Header
    uint8_t tipo1 = LoRaData.indexOf(',', confirmacion0 + 1); // Tipo de mensaje
    uint8_t codigo2 = LoRaData.indexOf(',', tipo1 + 1);       // Codigo de solicitud/respuesta

    // Situa el cursor en X y extrae caracteres hasta Y
    String strconf1 = LoRaData.substring(0, confirmacion0);
    String tipomsg = (LoRaData.substring(confirmacion0 + 1, tipo1));
    String codigomsg = (LoRaData.substring(tipo1 + 1, codigo2));

    if (strconf1 == "gvie")
    { // Verifica si la informacion reciciba viene de nuestro lora a partir de la cadena de comienzo
        switch (tipomsg.toInt())
        { // Puede ser 0, 1, 2, o 3
        case 0:
            responder = true; // Se espera que el mensaje sea de tipo confirmable (Se confirma con "OK")
            break;

        case 1:
            responder = false; // Se espera que el mensaje sea de tipo no confirmable
            haRespondido = false;
            break;

        case 2:
            haRespondido = true; // Significa que el paquete enviado es para confirmar un comando
            break;

        default:
            break;
        }

        if (!haRespondido)
        {
            switch (codigomsg.toInt())
            {

            case 3:
                inicializacion = true;
                listoParaDespegar = true;
                break;

            case 12:
                coordenadas = true;
                break;

            case 20:
                tiempoMision = millis();
                iniciarMision = true;
                EEPROM.update(memMision, true);
                break;

            case 21:
                latitudAterrizaje = latitud;
                longitudAterrizaje = longitud;
                break;

            case 35:
                wipeEeprom = true;
                break;

            case 37:
                NVIC_SystemReset(); // El mensaje recibido es para resetear el sistema
                break;

            case 77:
                // Indefinido
                break;

            default:
                LoRa_Transmit(3, 77, "");
                break;
            }
        }
        if (responder)
        {
            LoRa_Transmit(2, codigomsg.toInt(), "");
            responder = false;
        }
        if (coordenadas)
        {
            uint8_t indicador1 = LoRaData.indexOf(',', codigo2 + 1); // Datos
            uint8_t indicador2 = LoRaData.indexOf(',', indicador1 + 1);
            latitudAterrizaje = LoRaData.substring(codigo2 + 1, indicador1);
            longitudAterrizaje = LoRaData.substring(indicador1 + 1, indicador2);
            Serial.print(latitudAterrizaje);
            Serial.print(longitudAterrizaje);
            EEPROM.put(memLat, latitudAterrizaje);
            EEPROM.put(memLong, longitudAterrizaje);
            coordenadas = false;
        }
        if (!inicializacion)
        {
            if (codigomsg.toInt() == 20)
                LoRa_Transmit(3, 77, "");
        }
        if (!listoParaDespegar)
        {
            if (codigomsg.toInt() == 03)
                LoRa_Transmit(3, 77, "");
        }
        if (!iniciarMision)
        {
            if (codigomsg.toInt() == 00)
                LoRa_Transmit(3, 77, "");
        }
        if (wipeEeprom)
        {
            for (int i = 0; i < (memLong + 1); i++)
            {
                EEPROM.put(i, false);
            }
            wipeEeprom = 0;
        }
    }
}

// Funcion que envia la presion base a la Estacion Terrena
void enviarPresionBase()
{
    if (enviarDevuelta)
    {
        LoRa_Transmit(0, 10, String(P));
        enviarDevuelta = false;
    }
}

// Funcion basica de envio de errores a la Estacion Terrena
void reportar(String data)
{
    LoRa_Transmit(4, 77, String(data));
}

// Se completa un ciclo de la lectura de sensores
void sensorsBegin()
{
    sensors = true;
}

// Funcion que lee todos los sensores a su maxima velocidad. Controlada por banderas de interrupcion
void readSensors()
{
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
                    pAnterior = P;
                }
            }
        }
    }
    else
    {
        readAht++;
        if (readAht == 1)
            cicloAht++;

        if (readAht == 16)
            cicloAht++;

        if (readAht == 400)
            readAht = 0;

        switch (disparoMedicion)
        {
        case 0:
            status = pressure.startTemperature();
            batteryLevel = analogRead(BAT);
            actualizarCompass = true;
            disparoMedicion++;
            break;

        case 1:
            status = pressure.getTemperature(T);
            disparoMedicion++;
            break;

        case 2:
            MQ135 = analogRead(AQS);
            status = pressure.startPressure(0);
            disparoMedicion++;
            break;

        case 3:
            status = pressure.getPressure(P, T);
            if ((P - pAnterior) >= 100)
                P = pAnterior;
            disparoMedicion++;
            break;

        default:
            disparoMedicion = false;
            break;
        }
    }
}

// Funcion callback del pin INT para iniciar la lectura del MPU6050
void readMPU()
{
    leerMpu = true; // Comienza la actualizacion de datos del MPU6050
}

// Lectura de los datos del sensor MPU6050
void leerMPU()
{
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
    giroX = mpu.getAngleX();
    giroY = mpu.getAngleY();
    giroZ = mpu.getAngleZ();
}

void beginMeasurementAht()
{
    Wire.beginTransmission(0x38);

    Wire.write(AHTXX_START_MEASUREMENT_REG);      // send measurement command, strat measurement
    Wire.write(AHTXX_START_MEASUREMENT_CTRL);     // send measurement control
    Wire.write(AHTXX_START_MEASUREMENT_CTRL_NOP); // send measurement NOP control

    Wire.endTransmission(true);
}

// Lectura de los datos del sensor AHT10
void readHumidityAht()
{
    uint8_t _rawData[7] = {0, 0, 0, 0, 0, 0, 0}; //{status, RH, RH, RH+T, T, T, CRC}, CRC for AHT2x only
    Wire.requestFrom(0x38, 7, (uint8_t) true);   // read n-byte to "wire.h" rxBuffer, true-send stop after transmission

    for (uint8_t i = 0; i < 7; i++)
    {
        _rawData[i] = Wire.read();
    }

    humidity = _rawData[1]; // 20-bit raw humidity data
    humidity <<= 8;
    humidity |= _rawData[2];
    humidity <<= 4;
    humidity |= _rawData[3] >> 4;

    ahtValue = ((float)humidity / 0x100000) * 100;
}

// Funcion callback del Timer para escribir en el archivo de la MicroSD
void escribirArchivo()
{
    escritura = true;
}

// Funcion encargada de resetear el micro si algun dispositivo I2C no se inicializa
void errorCheck(char device)
{
    int deberiaResetear = 0;
    while (checkSensorStatus(device))
    {
        digitalWrite(LED_BUILTIN, !LED_BUILTIN);
        // stop everything if could not connect to I2C device
        delay(1000);
        deberiaResetear++;
        if (deberiaResetear >= 5)
        {
            reportar(String("Error al iniciar el dispositivo I2C N" + String((int)device)));
            delay(1000);
            deberiaResetear = 0;
        }
    }
}

// Funcion encargada de detectar el estado de inicializacion en el dispositivo seleccionado
bool checkSensorStatus(char device)
{
    switch (device)
    {
    case 1:
        return mpu.begin(); // return begin status of MPU6050
        break;
    case 2:
        return !aht10.begin(); // return begin status of AHT10
        break;
    case 3:
        return !pressure.begin(); // return begin status of bmp
        break;
    case 4:
        // innit de la brujula
        break;
    }
}

// Melodia para recuperar el cansat al finalizar mision
void song(int buzzerPin)
{

    tone(buzzerPin, 988);
    delay(278);
    noTone(buzzerPin);

    tone(buzzerPin, 1480);
    delay(556);
    noTone(buzzerPin);

    tone(buzzerPin, 1976);
    delay(278);
    noTone(buzzerPin);

    tone(buzzerPin, 1109);
    delay(139);
    noTone(buzzerPin);

    tone(buzzerPin, 988);
    delay(972);
    noTone(buzzerPin);

    tone(buzzerPin, 1480);
    delay(972);
    noTone(buzzerPin);

    tone(buzzerPin, 988);
    delay(278);
    noTone(buzzerPin);

    tone(buzzerPin, 1480);
    delay(556);
    noTone(buzzerPin);

    tone(buzzerPin, 1976);
    delay(278);
    noTone(buzzerPin);

    tone(buzzerPin, 1109);
    delay(139);
    noTone(buzzerPin);

    tone(buzzerPin, 1480);
    delay(556);
    noTone(buzzerPin);

    tone(buzzerPin, 2960);
    delay(556);
    noTone(buzzerPin);
}
