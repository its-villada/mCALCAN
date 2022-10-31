#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <AHTxx.h>
#include <EEPROM.h>
#include <LoRa.h>
#include <SPI.h>
#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <STM32_ISR_Timer.hpp>
#include <SD.h>
#include <Wire.h>
#include "MPU9250.h"
#include <Servo.h>
#include <BMP280_DEV.h>

#define GPS_BAUDRATE 115200L
#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE      // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 19            // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7     // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5       // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.
#define RST PB3                  // Pin Reset del transciver LoRa
#define IRQ PA4                  // Pin DIO0 del transciver LoRa
#define NSS PA15                 // Pin CS del transciver LoRa
#define INTMPU PA3               // Pin de interrupcion del MPU6050
#define gpsOn PA11               // Pin de activacion del GPS
#define BAT0 PA0                 // Pin de tension de bateria 1
#define BAT1 PA1                 // Pin de tension de bateria 2
#define MainS PA12               // Pin Sensors enable
#define AQS PA2                  // Pin del sensor MQ-135
#define buzzer PA8               // Pin del buzzer de recuperacion
#define servoDerPin PB1          // Pin del servo derecha
#define servoIzqPin PB0          // Pin del servo izquierda
#define HW_TIMER_INTERVAL_MS 1
#define TIMER_INTERVAL_1 6L   // Intervalo1 salta cada 6 ms
#define TIMER_INTERVAL_2 500L // Intervalo2 salta cada 500 ms
#define TIMER_INTERVAL_3 50L  // Intervalo3 salta cada 50 ms
#define servoDerMaxMicros 400
#define servoDerMinMicros 2400
#define servoIzqMaxMicros 2400
#define servoIzqMinMicros 400
#define minHorizontalAcc 5000 // mm
#define USE_STM32_TIMER_NO TIM2
#define comma ','
#define memInit 0
#define memMision 1
#define memDesc 2
#define memLat 3
#define memLong 4
#define memReset 5
#define memTiempo 6
#define MPUaddress 0x68
#define altitudMar 429

File csvTelemetry;
STM32Timer Timer1(TIM1);
STM32_ISR_Timer ISR_Timer1_Temp;
MPU9250 mpu;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); // sensor address, sensor type
TinyGPSPlus tGPS;
BMP280_DEV bmp280;
Servo servoDer;
Servo servoIzq;

float T = 0, P = 0, giroX, giroY, giroZ, accX, accY, accZ, ahtValue, miOrientacion = 0, mx, my, mz, altitud, altitudMax = 0;
double tAnterior = 0, pAnterior = 0, latitud, longitud;
bool pressureDRDY = false, busqueda = false, finalizarMision = false, gpsDRDY = false, transmit = false, sensors = false, actualizarMpu = false, leerMpu = false, responder = false, haRespondido = false, noPudoProcesar = false, escribirDatos = false, ejecutarAccion = false, inicializacion = false, inicializado = false, enviarDevuelta = true, escritura = false, coordenadas = false, listoParaDespegar = false, descenso = false, wipeEeprom = false, iniciarMision = false;
uint8_t disparoMedicion = 0, cicloAht = 0, preparacion = 0, tiempoGps = 0;
uint16_t MQ135, batteryLevel1, batteryLevel2, deltaOrientacion, distanciaADestino, orientacionADestino, readAht = 0;
uint32_t humidity, tiempoMision;
String latitudAterrizaje, longitudAterrizaje;

void cansatStartUp();
void Timer1Handler();
void onReceive(int);
void telemetrySend();
void reportar(String);
void readSensors();
void beginMeasurementAht();
void readHumidityAht();
void leerMPU();
void loraSetup();
void bmpSetup();
void timerSetup();
void servoSetup();
void GPSSetup();
void mpuSetup();
void aht10Setup();
void sdCardSetup();
void transmitir();
void sensorsBegin();
void readMPU();
void servoControl();
void escribirArchivo();
void errorCheck(char);
bool checkSensorStatus(char);
void LoRa_Transmit(uint8_t, uint8_t, String);
void leerGPS();
double courseTo(double lat1, double long1, double lat2, double long2);
double distanceBetween(double lat1, double long1, double lat2, double long2);
void song(int);

void setup()
{
    // put your setup code here, to run once:
    // EEPROM.begin();
    pinMode(LED_BUILTIN, OUTPUT); // FUNCIONA EN LOGICA INVERSA!
    pinMode(MainS, OUTPUT);
    pinMode(gpsOn, OUTPUT);             // FUNCIONA EN LOGICA INVERSA!
    pinMode(buzzer, OUTPUT_OPEN_DRAIN); // FUNCIONA EN LOGICA INVERSA!
    pinMode(BAT0, INPUT);
    pinMode(BAT1, INPUT);
    pinMode(INTMPU, INPUT);
    pinMode(AQS, INPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(MainS, LOW);
    digitalWrite(buzzer, HIGH);
    digitalWrite(gpsOn, LOW);

    loraSetup(); // Setup modulo lora

    // if (EEPROM.read(memReset))
    // {
    //     reportar("Reset Exitoso");
    //     EEPROM.update(memReset, 0);
    // }

    timerSetup(); // Setup de interrupciones de timer

    attachInterrupt(digitalPinToInterrupt(INTMPU), readMPU, FALLING);
}

void loop()
{
    if (!inicializado)
    {
        // if (EEPROM.read(memInit))
        // {
        //     cansatStartUp();
        //     tiempoMision = EEPROM.read(memTiempo);
        //     descenso = EEPROM.read(memDesc);
        //     iniciarMision = EEPROM.read(memMision);
        //     latitudAterrizaje = EEPROM.read(memLat);
        //     longitudAterrizaje = EEPROM.read(memLong);
        //     inicializacion = false;
        //     inicializado = true;
        // }

        if (inicializacion)
        {
            cansatStartUp();
            haRespondido = false;
            inicializado = true;
            // EEPROM.update(memInit, true);
        }
    }
    else
    {
        if (listoParaDespegar || iniciarMision)
        {
            if (!finalizarMision)
            {

                leerGPS();

                if (pressureDRDY)
                {
                    altitud = altitud - altitudMar;
                    if (altitud <= 0)
                        altitud = 0;

                    if (altitud >= altitudMax)
                        altitudMax = altitud;

                    if (altitud <= 10 && altitudMax >= 50)
                        finalizarMision = true;

                    pressureDRDY = false;
                }

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
                if (descenso && gpsDRDY)
                {
                    servoControl();
                    gpsDRDY = false;
                }

                if (actualizarMpu)
                {
                    mpu.update();
                    actualizarMpu = false;
                }
                if (leerMpu)
                {

                    Wire.beginTransmission(MPUaddress);
                    Wire.write(0x3A); // Registro de status de interrupciones
                    Wire.endTransmission();
                    Wire.requestFrom(MPUaddress, (uint8_t)1); // Se pide la informacion del registro anterior

                    byte data = Wire.read();

                    if (data == 0x81)
                    {
                        leerMPU();
                        descenso = true;
                        // EEPROM.update(memDesc, true);
                    }
                    else if (data == 0x01)
                        leerMPU();
                    else if (data == 0x80)
                    {
                        descenso = true;
                        // EEPROM.update(memDesc, true);
                    }
                    actualizarMpu = true;
                    leerMpu = false;
                }

                if (escritura && !listoParaDespegar)
                {
                    if (csvTelemetry)
                    {
                        if (!transmit)
                        {
                            csvTelemetry.println(String(String(millis() - tiempoMision) + comma + String(altitud) + comma + String(descenso) + comma + String(T) + comma + String(P) + comma + String(giroX) + comma + String(giroY) + comma + String(giroZ) + comma + String(accX) + comma + String(accY) + comma + String(accZ) + comma + String(ahtValue) + comma + String(MQ135) + comma + latitud + comma + longitud + comma + String(batteryLevel1)));
                            csvTelemetry.flush();
                        }
                    }
                    else
                    {
                        reportar("Fallo al abrir archivo de MicroSD");
                    }
                    escritura = false;
                }
            }
            else
            {
                servoDer.detach();
                servoIzq.detach();
                digitalWrite(servoDerPin, LOW);
                digitalWrite(servoIzqPin, LOW);
                digitalWrite(MainS, LOW);
                if (busqueda)
                {
                    song(buzzer);
                    delay(1000);
                }
            }
        }
    }
}

void cansatStartUp()
{
    digitalWrite(MainS, HIGH);
    digitalWrite(gpsOn, LOW);

    Wire.begin();
    delay(2000);
    bmpSetup();
    delay(100);
    aht10Setup();
    delay(100);
    mpuSetup();
    delay(100);
    sdCardSetup();
    delay(100);
    GPSSetup();
    delay(100);
    servoSetup();
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

    // reportar("Please leave the device still on a flat plane.");
    // mpu.verbose(true);
    // delay(5000);
    // mpu.calibrateAccelGyro();

    // reportar("Please Wave device in a figure eight until done.");
    // delay(5000);
    // mpu.calibrateMag();

    // reportar("Calibration Complete");

    Wire.beginTransmission(MPUaddress); // Se le escribe datos al mpu9250
    Wire.write(0x37);                   // Registro configuracion del pin int
    Wire.write(0xB2);
    Wire.endTransmission();

    reportar("MPU inicializado");

    actualizarMpu = true;
}

// Inicializacion del BMP280
void bmpSetup()
{
    errorCheck(3);
    bmp280.setTimeStandby(TIME_STANDBY_250MS);
    bmp280.startNormalConversion();
    bmp280.setPresOversampling(OVERSAMPLING_X4); // Set the pressure oversampling to X4
    bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
    bmp280.getTempPres(T, P);
    tAnterior = T;
    pAnterior = P;
    reportar("BMP inicializado");
}

// Inicializacion del AHT10
void aht10Setup()
{
    errorCheck(2);
    reportar("AHT inicializado");
}

// Inicializacion de la MicroSD
void sdCardSetup()
{
    while (!SD.begin(PB9))
    {
        reportar("No se pudo iniciar MicroSD1");
        delay(500);
    }
    csvTelemetry = SD.open("mCALCAN.csv", FILE_WRITE);
    reportar("Micro SD iniciado");
}

// Setup del modulo del GPS
void GPSSetup()
{
    Serial.begin(GPS_BAUDRATE);
    reportar("GPS inicializado");
}

// Setup de los servos
void servoSetup()
{
    servoDer.attach(servoDerPin);
    servoIzq.attach(servoIzqPin);
}

// Subortina ISR para interrupciones
void Timer1Handler()
{
    ISR_Timer1_Temp.run();
}

// Funcion estandard para enviar mensajes por LoRa con el Protocolo gVIE
void LoRa_Transmit(uint8_t type, String reqs, String data)
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
    LoRa_Transmit(1, "14", String(String(T) + comma + String(P) + comma + String(giroX) + comma + String(giroY) + comma + String(miOrientacion) + comma + String(accX) + comma + String(accY) + comma + String(accZ) + comma + String(batteryLevel1) + comma + String(millis() - tiempoMision) + comma + String(descenso) + comma + String(MQ135) + comma + String(ahtValue) + comma + String(latitud, 6) + comma + String(longitud, 6) + comma + String(batteryLevel2) + comma + String(altitud)));
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
                digitalWrite(gpsOn, HIGH);
                tiempoMision = millis();
                iniciarMision = true;
                // EEPROM.update(memMision, true);
                break;

            case 21:
                latitudAterrizaje = latitud;
                longitudAterrizaje = longitud;
                break;

            case 26:
                finalizarMision = true;
                busqueda = true;
                break;

            case 35:
                wipeEeprom = true;
                break;

            case 37:
                // EEPROM.put(memTiempo, (millis() - tiempoMision));
                // EEPROM.update(memReset, 1);
                digitalWrite(MainS, LOW);
                NVIC_SystemReset(); // El mensaje recibido es para resetear el sistema
                break;

            case 77:
                // Indefinido
                break;

            default:
                LoRa_Transmit(3, "77", "");
                break;
            }
        }
        if (responder)
        {
            LoRa_Transmit(2, codigomsg, "");
            responder = false;
        }
        if (coordenadas)
        {
            uint8_t indicador1 = LoRaData.indexOf(',', codigo2 + 1); // Datos
            uint8_t indicador2 = LoRaData.indexOf(',', indicador1 + 1);
            latitudAterrizaje = LoRaData.substring(codigo2 + 1, indicador1);
            longitudAterrizaje = LoRaData.substring(indicador1 + 1, indicador2);
            // EEPROM.put(memLat, latitudAterrizaje);
            // EEPROM.put(memLong, longitudAterrizaje);
            coordenadas = false;
        }
        if (!inicializacion)
        {
            if (codigomsg.toInt() == 20)
                LoRa_Transmit(3, "77", "");
        }
        if (!listoParaDespegar)
        {
            if (codigomsg.toInt() == 03)
                LoRa_Transmit(3, "77", "");
        }
        if (!iniciarMision)
        {
            if (codigomsg.toInt() == 00)
                LoRa_Transmit(3, "77", "");
        }
        if (wipeEeprom)
        {
            // for (int i = 0; i < (memTiempo + 1); i++)
            //     EEPROM.put(i, false);
            wipeEeprom = false;
        }
    }
}

// Funcion basica de envio de errores a la Estacion Terrena
void reportar(String data)
{
    LoRa_Transmit(4, "77", String(data));
}

// Se completa un ciclo de la lectura de sensores
void sensorsBegin()
{
    sensors = true;
}

// Funcion que lee todos los sensores a su maxima velocidad. Controlada por banderas de interrupcion
void readSensors()
{
    readAht++;
    if (readAht == 1)
        cicloAht++;

    if (readAht == 16)
        cicloAht++;

    if (readAht >= 400)
        readAht = 0;

    switch (disparoMedicion)
    {
    case 0:
        batteryLevel1 = (analogRead(BAT0) * 1.2);
        MQ135 = analogRead(AQS);
        batteryLevel2 = (analogRead(BAT1) * 1.2);
        disparoMedicion++;
        break;

    case 1:
        bmp280.getCurrentMeasurements(T, P, altitud);
        pressureDRDY = true;
        disparoMedicion = 0;
        break;

    default:
        disparoMedicion = 0;
        break;
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
    if (accX >= 3 || accY >= 3 || accZ >= 3)
        digitalWrite(gpsOn, HIGH);
    else
        digitalWrite(gpsOn, LOW);
    giroX = mpu.getGyroX();
    giroY = mpu.getGyroY();
    giroZ = mpu.getGyroZ();
    mx = mpu.getMagX();
    my = mpu.getMagY();
    mz = mpu.getMagZ();
    miOrientacion = atan2(mx, my) * 180 / 3.14;
    deltaOrientacion = orientacionADestino - miOrientacion;
    deltaOrientacion = (deltaOrientacion < (-180)) ? deltaOrientacion + 360 : deltaOrientacion;
    deltaOrientacion = (deltaOrientacion > 180) ? deltaOrientacion - 360 : deltaOrientacion;
}

// Lectura de los datos del sensor de GPS
void leerGPS()
{
    while (Serial.available())
    {
        if (tGPS.encode(Serial.read()))
        {

            longitud = (tGPS.location.lng());

            latitud = (tGPS.location.lat());

            if (tGPS.location.isValid() && (tGPS.hdop.hdop() < 2))
            {
                distanciaADestino = distanceBetween(latitud, longitud, latitudAterrizaje.toDouble(), longitudAterrizaje.toDouble());
                orientacionADestino = courseTo(latitud, longitud, latitudAterrizaje.toDouble(), longitudAterrizaje.toDouble());
                gpsDRDY = true;
            }
            else
            {
                servoIzq.write(servoIzqMinMicros);
                servoDer.write(servoDerMinMicros);
            }
        }
    }
}

// Ceba el AHT para que comienze una medicion y despues leer los datos recuperados
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
    if (tiempoGps >= 70)
        digitalWrite(gpsOn, LOW);
    else
        tiempoGps++;

    escritura = true;
}

// Funcion encargada de resetear el micro si algun dispositivo I2C no se inicializa
void errorCheck(char device)
{
    int deberiaResetear = 0;
    int times = 0;
    while (checkSensorStatus(device))
    {

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // stop everything if could not connect to I2C device
        delay(1000);
        deberiaResetear++;
        if (deberiaResetear >= 5)
        {
            times++;
            reportar(String("Error al iniciar el dispositivo I2C N" + String((int)device)));
            checkSensorStatus(device);
            deberiaResetear = 0;
        }
        if (times >= 3)
        {
            reportar(String("No se pudo iniciar el dispositivo I2C N" + String((int)device)));
            break;
        }
    }
}

// Funcion encargada de detectar el estado de inicializacion en el dispositivo seleccionado
bool checkSensorStatus(char device)
{
    switch (device)
    {
    case 1:
        return !mpu.setup(MPUaddress); // return begin status of MPU6050
        break;
    case 2:
        return !aht10.begin(); // return begin status of AHT10
        break;
    case 3:
        return !bmp280.begin(BMP280_I2C_ALT_ADDR); // return begin status of bmp
        break;
    }
}

// Melodia para recuperar el cansat al finalizar mision
void song(int tonePin)
{
    tone(tonePin, 987, 29.347826087);
    delay(32.6086956522);
    tone(tonePin, 246, 48.9130434783);
    delay(54.347826087);
    delay(173.913043478);
    delay(10.8695652174);
    tone(tonePin, 1479, 29.347826087);
    delay(32.6086956522);
    tone(tonePin, 369, 19.5652173913);
    delay(21.7391304348);
    delay(141.304347826);
    tone(tonePin, 246, 68.4782608696);
    delay(76.0869565217);
    delay(152.173913043);
    delay(54.347826087);
    tone(tonePin, 246, 29.347826087);
    delay(32.6086956522);
    delay(358.695652174);
    tone(tonePin, 369, 58.6956521739);
    delay(65.2173913043);
    delay(10.8695652174);
    tone(tonePin, 987, 97.8260869565);
    delay(108.695652174);
    digitalWrite(buzzer, HIGH);
}

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
    // returns distance in meters between two positions, both specified
    double delta = radians(long1 - long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
    // returns course in degrees (North=0, West=270) from position 1 to position 2,

    double dlon = radians(long2 - long1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double a1 = sin(dlon) * cos(lat2);
    double a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0)
    {
        a2 += TWO_PI;
    }
    return degrees(a2);
}

void servoControl()
{
    float girDerPor = 0, girIzqPor = 0, coefDist = 0; // 0% - 100%

    if (deltaOrientacion > 0)
    {
        girDerPor = map(deltaOrientacion, 0, 180, 5, 100);
    }
    if (deltaOrientacion < 0)
    {
        girIzqPor = map(deltaOrientacion, -180, 0, 100, 5);
    }

    if (deltaOrientacion >= 0 && deltaOrientacion <= 10)
    {
        girIzqPor = girIzqPor + (100 - deltaOrientacion * 5);
        girDerPor = girDerPor + (100 - deltaOrientacion * 5);
    }

    if (deltaOrientacion <= 0 && deltaOrientacion >= -10)
    {
        girIzqPor = girIzqPor + (100 - deltaOrientacion * -5);
        girDerPor = girDerPor + (100 - deltaOrientacion * -5);
    }

    coefDist = constrain(distanciaADestino / 10, 0, 1);

    girDerPor *= coefDist;
    girIzqPor *= coefDist;

    servoDer.write(map(constrain(girDerPor, 0, 100), 0, 100, servoDerMinMicros, servoDerMaxMicros));
    servoIzq.write(map(constrain(girIzqPor, 0, 100), 0, 100, servoIzqMinMicros, servoIzqMaxMicros));
}
