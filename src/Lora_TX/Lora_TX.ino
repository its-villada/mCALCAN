#include <LoRa.h>
#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <STM32_ISR_Timer.hpp>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050_light.h>
#include <AHTxx.h>

SFE_BMP180 pressure;
MPU6050 mpu(Wire);
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); // sensor address, sensor type

float ahtValue;
double baseline;
double T, P, A, giroX, giroY, giroZ, accX, accY, accZ;
unsigned int pktNumber = 0;
bool transmit = false, sensors = false, bmpIsInit = false, actualizarMpu = false, leerMpu = false, isFreeFall = false;
bool responder = false, haRespondido = false, noPudoProcesar = false, escribirDatos = false, ejecutarAccion = false, inicializacion = true, inicializado = false, enviarDevuelta = true, cicloAht = true;
int disparoMedicion = 0, batteryLevel, MQ135, readAht = 0;
uint32_t tiempoMision = 0, empezarMision = 0;
String dataEnviar = "", dataRecibir;

#define HW_TIMER_INTERVAL_MS 1

#define NSS PA15
#define RST PB3
#define IRQ PA1
#define INTpin PA2
#define DRDY PA11
#define BAT PA0
#define MainS PA12
#define AQS PA4

// Init STM32 timer TIM1
STM32Timer Timer1(TIM1);

STM32_ISR_Timer ISR_Timer1_Temp;

#define TIMER_INTERVAL_1 5L   // TIMER1 salta cada 5 ms
#define TIMER_INTERVAL_2 500L // Timer2 salta cada 500 ms


void Timer1Handler()
{
  ISR_Timer1_Temp.run();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT); // FUNCIONA EN LOGICA INVERSA!
  pinMode(MainS, OUTPUT);
  pinMode(BAT, INPUT);
  pinMode(INTpin, INPUT);
  pinMode(AQS, INPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(MainS, HIGH);

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
      empezarMision = millis();
      bmpSetup(); // Setup modulo de temperatura
      aht10Setup();
      // mpuSetup(); // Setup modulo de giroscopio
      // while (!haRespondido)
      //   enviarPresionBase();
      inicializado = true;
    }
  }
  else
  {
    if (cicloAht == true)
    {
      ahtValue = aht10.readHumidity();
      cicloAht = false;
    }
    
    if (transmit == true)
    {
      packetSending();
      transmit = false;
    }
    if (sensors == true)
    {
      readSensors();
      sensors = false;
    }
    if (actualizarMpu == true)
    {
      // mpu.update();
      actualizarMpu = false;
    }
    if (leerMpu == true)
    {
      if ((mpu.readData(0x3A) == 0x81) || (mpu.readData(0x3A) == 0x01))
        leerGyro();
      if ((mpu.readData(0x3A) == 0x81) || (mpu.readData(0x3A) == 0x80))
        isFreeFall = true;
      leerMpu = false;
    }
  }
}

void packetSending()
{
  tiempoMision = (millis() - empezarMision);
  // Send LoRa packet to receiver
  digitalWrite(LED_BUILTIN, LOW);
  LoRa.beginPacket();
  LoRa.print("gvie"); // Cadena de comienzo de packet
  LoRa.print(",");
  LoRa.print("1"); // Tipo de mensaje, este es un mensaje no confirmable
  LoRa.print(",");
  LoRa.print("1"); // Codigo de solicitud/respuesta (2 bytes) Primer byte, escribir datos
  LoRa.print("4"); // Segundo Byte, Telemetria
  LoRa.print(",");
  LoRa.print(T); // Datos
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
  LoRa.print(tiempoMision);
  LoRa.print(",");
  if (isFreeFall)
  {
    LoRa.print(isFreeFall);
    isFreeFall = false;
  }
  else
    LoRa.print("0");
  LoRa.print(",");
  LoRa.print(MQ135);
  LoRa.print(",");
  LoRa.print(ahtValue);
  LoRa.endPacket(true);
  // LoRa.receive();
  digitalWrite(LED_BUILTIN, HIGH);
  // Volver a 0 para que no haga overflow
  if (pktNumber >= 65500)
    pktNumber = 0;
  pktNumber++;
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
      responder = 0; // Se espera que el mensaje sea de tipo no confirmable
      break;

    case 2:
      haRespondido = true; // Significa que el paquete enviado es para confirmar un comando
      break;

    case 3:
      noPudoProcesar = true; // Significa que el paquete enviado pudo ser procesado
      break;

    default:
      break;
    }

    if (!haRespondido && !noPudoProcesar)
    {
      switch (codigomsg.toInt())
      {
      case 1:
        escribirDatos = true; // El mensaje recibido es para escribir datos
        break;

      case 2:
        ejecutarAccion = true; // EL mensaje recibido es para ejecutar una accion
        break;

      case 3:
        NVIC_SystemReset(); // El mensaje recibido es para resetear el sistema
        break;

      case 10:
        inicializacion = true; // El mensaje recibido es para comenzar la inicializacion de datos
        break;

      default:
        break;
      }
    }

    if (responder)
    {
      confirmacion(dataEnviar, codigomsg);
      responder = false;
    }
  }
}

void confirmacion(String datos, String codigomsg)
{
  digitalWrite(LED_BUILTIN, HIGH);
  LoRa.beginPacket();
  LoRa.print("gvie"); // Cadena de comienzo de packet
  LoRa.print(",");
  LoRa.print("2"); // Tipo de mensaje, este es un mensaje respondiendo a un comando
  LoRa.print(",");
  LoRa.print(codigomsg); // Codigo del comando
  LoRa.print(",");
  LoRa.print(datos); // Si es necesario, se ingresan datos
  LoRa.endPacket(true);
  LoRa.receive();
  digitalWrite(LED_BUILTIN, LOW);
}

void enviarPresionBase()
{
  if (enviarDevuelta)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    LoRa.beginPacket();
    LoRa.print("gvie"); // Cadena de comienzo de packet
    LoRa.print(",");
    LoRa.print("0"); // Tipo de mensaje, este es un mensaje confirmable
    LoRa.print(",");
    LoRa.print("1"); // Codigo de solicitud/respuesta (2 bytes) Primer byte, escribir datos
    LoRa.print("0"); // Segundo Byte, es de inicializacion
    LoRa.print(",");
    LoRa.print(baseline);
    LoRa.endPacket(true);
    LoRa.receive();
    enviarDevuelta = false;
  }
}

void reportarError(String data)
{
  LoRa.beginPacket();
  LoRa.print("gvie"); // Cadena de comienzo de packet
  LoRa.print(",");
  LoRa.print("1"); // Tipo de mensaje, este es un mensaje no confirmable
  LoRa.print(",");
  LoRa.print("4"); // Codigo de solicitud/respuesta (2 bytes) Primer byte, se reporta un error
  LoRa.print("X"); // Segundo Byte, no se toma en cuenta
  LoRa.print(",");
  LoRa.print(data); // Se informa cual fue el error
  LoRa.endPacket(true);
  LoRa.receive();
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
    readAht++;
    if (readAht == 400)
    {
      cicloAht = true;
      readAht = 0;
    }
    
    switch (disparoMedicion)
    {
    case 0:
      status = pressure.startTemperature();
      batteryLevel = analogRead(BAT);
      MQ135 = analogRead(AQS);
      actualizarMpu = true;
      if (status != 0)
        disparoMedicion++;
      break;

    case 1:
      status = pressure.getTemperature(T);
      disparoMedicion++;
      break;

    case 2:
      actualizarMpu = true;
      status = pressure.startPressure(0);
      if (status != 0)
        disparoMedicion++;
      break;

    case 3:
      // status = pressure.getPressure(P, T);
      disparoMedicion++;
      break;

    default:
      ahtValue = aht10.readHumidity(AHTXX_USE_READ_DATA);
      disparoMedicion = false;
      break;
    }
  }
}

void leerGyro() // Se toman los datos del sensor MPU
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
#define SERIAL_BAUDRATE 9600 // Velocidad del Puerto Serie

#define LORA_FREQUENCY 915000000 // Frecuencia en Hz a la que se quiere transmitir.
#define LORA_SYNC_WORD 0xDE      // Byte value to use as the sync word, defaults to 0x12
#define LORA_POWER 17            // TX power in dB, defaults to 17. Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.
#define LORA_SPREAD_FACTOR 7     // Spreading factor, defaults to 7. Supported values are between 6 and 12 (En Argentina se puede utilizar entre 7 a 10)
#define LORA_SIG_BANDWIDTH 125E3 // Signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15. 6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_CODING_RATE 5       // Denominator of the coding rate, defaults to 5. Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.

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
  } // Iniciando BMP180, BMP180 inicio OK, baseline
  while (!bmpIsInit);
}

void timerSetup()
{
  // Timer1, lectura de temperatura y transmision
  Timer1.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, Timer1Handler);
  ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_1, sensorsBegin); // Intervalo de timer para los ciclos de lectura de sensor
  ISR_Timer1_Temp.setInterval(TIMER_INTERVAL_2, transmitir);   // Intervalo de timer para la transmision de datos a la ET
}

void mpuSetup()
{
  byte status = mpu.begin();
  int deberiaResetear = 0;

  while (status != 0)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // stop everything if could not connect to MPU6050
    delay(1000);
    deberiaResetear++;
    if (deberiaResetear >= 2)
    {
      reportarError("Error iniciando el mpu. Se vuelve a iniciar");
      status = mpu.begin();
    }
  }

  mpu.writeData(0x38, 0x81);
  mpu.writeData(0x1D, 17);
  mpu.writeData(0x1E, 2);
  mpu.writeData(0x37, 0xB0);

  mpu.calcOffsets(true, true); // gyro and accelero
}

void aht10Setup()
{
  int deberiaResetear = 0;

  while (!aht10.begin())
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // stop everything if could not connect to MPU6050
    delay(1000);
    deberiaResetear++;
    if (deberiaResetear >= 4)
    {
      reportarError("Error iniciando el AHT10");
    }
  }
  // aht10.readTemperature();
  // ahtValue = aht10.readHumidity(AHTXX_USE_READ_DATA);
}

void transmitir()
{
  transmit = true; // Se transmiten datos a la estacion terrena
  if (!haRespondido)
  {
    enviarDevuelta = !enviarDevuelta; // Si han pasado 1000 ms, y no se respondio al comando, se vuelve a enviar
  }
}

void sensorsBegin()
{
  sensors = true; // Se completa un ciclo de la lectura de sensores
}

void readMPU()
{
  leerMpu = true; // Comienza la lectura del mpu
}
