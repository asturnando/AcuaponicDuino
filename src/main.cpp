/*
  AcuaponicDuino - Automatización y control de cultivos acuapónicos con Arduino y Node-RED sobre protocolo MQTT
  Codigo perteneciente al TFG del Grado de Ingenieria Informatica de la Universitat Oberta de Catalunya
  Autor: Fernando Suarez Rodriguez
  Fecha: 2022/06/06
  Version: 1.0
  Descripcion: Programa que controlara el funcionamiento de la placa de control del sistema de acuaponia (Arduino Mega 2560 rev3)).
  El programa esta realizado en FreeRTOS, y se encargara de la automatizacion de las diferentes tareas del sistema asi como de la recepcion de datos de los sensores
  Los datos recibidos de los sensores, seran enviados por el puerto serie hacia un ESP8266 que sera el encargado de transmitir los datos a la red MQTT para luego ser
  leidos en Node-RED.
*/

#include <Arduino.h>
#include <temperaturaHumedad.h>    // Libreria necesaria para controlar los sensores de temperatura y humedad
#include <sensorTemperaturaAgua.h> // Libreria necesaria para controlar el sensor de temperatura del agua
#include <Arduino_FreeRTOS.h>      // Libreria necesaria para controlar el sistema operativo
#include <semphr.h>                // Libreria necesaria para añadir semaforos a los procesos y dar comunicacion entre tareas
#include <queue.h>                 // Libreria necesaria para añadir colas a los procesos y dar comunicacion entre tareas
#include <DFRobot_PH.h>            // Libreria de terceros para controlar el sensor de PH
#include <EEPROM.h>                // Libreria necesaria para almacenar los datos en la eeprom usada por el sensor de PH
#include <GravityTDS.h>            // Libreria necesaria para controlar el sensor de TDS basada en librerias de terceros

//*---------------------------------------------------
//*------------------- Definiciones ------------------
//*---------------------------------------------------
// Modo DEBUG
#define DEBUG true
#define Serial \
  if (DEBUG)   \
  Serial
#define TASK_PRIORITY (tskIDLE_PRIORITY + 2) // Definimos la prioridad de las tareas
#define QUEUE_SIZE (5)                       // Definimos el tamaño de la cola

// Definicion de pines
#define SENSOR_TEMP (A10)
#define SENSOR_HUM (A11)
#define SENSOR_LUZ (A12)
#define SENSOR_TDS (A5)
#define SENSOR_PH (A14)
#define STOP_BUTTON (2)
#define START_BUTTON (3)
#define CAUDALIMETRO1 (18)
#define CAUDALIMETRO2 (19)
#define SENSOR_NIVEL_AGUA1 (20)
#define SENSOR_NIVEL_AGUA2 (21)
#define SENSOR_TEMP_AGUA (25)
#define ELECTROVALVULA_1 (26)
#define ELECTROVALVULA_2 (27)
#define ELECTROVALVULA_3 (28)
#define ELECTROVALVULA_4 (29)
#define RELE_LIBRE (30)
#define COMPRESOR (33)
#define CALENTADOR (32)
#define BOMBA_ONOF (31)
#define LED_ON (34)
#define LED_FALLO (35)
#define LED_OFF (36)

// Declaramos los manejadores de tareas
TaskHandle_t Start_Handler;
TaskHandle_t Stop_Handler;
TaskHandle_t StartBlink_Handler;
TaskHandle_t EnvMeasure_Handler;
TaskHandle_t ProcessCommand_Handler;
TaskHandle_t CaudalMeasure_Handler;
TaskHandle_t WaterMeasure_Handler;
TaskHandle_t TempWaterMeasure_Handler;

// Declaramos las tareas
void Start(void *pvParameters);
void Stop(void *pvParameters);
void StartBlink(void *pvParameters);
void EnvMeasure(void *pvParameters);
void ProcessCommand(void *pvParameters);
void CaudalMeasure(void *pvParameters);
void WaterMeasure(void *pvParameters);
void TempWaterMeasure(void *pvParameters);

// Declaramos semaforos y colas
SemaphoreHandle_t xStart;     // semaforo para activar la tarea Start cuando se pulsa el boton de arranque.
SemaphoreHandle_t xStop;      // semaforo para activar la tarea Stop cuando se pulsa el boton de parada.
SemaphoreHandle_t xLevel;     // semaforo para activar el control de nivel
QueueHandle_t xQueueCommands; // cola para que tanto la tarea para el paso de mensajes entre tareas
SemaphoreHandle_t xPlayMutex; // mutex

/*---------------------------------------------------
 *------------------- Callbacks ----------------------
 *---------------------------------------------------*/
void callbackButtonStart();   // Funcion para gestionar las interrupciones del boton de arranque
void callbackButtonStop();    // Funcion para gestionar las interrupciones del boton de parada
void callbackCaudalimetro2(); // Funcion para gestionar las interrupciones del caudalimetro 2
void callbackCaudalimetro1(); // Funcion para gestionar las interrupciones del caudalimetro 1
void callbackNivelAgua1();    // Funcion para gestionar las interrupciones del sensor de nivel de agua 1
void callbackNivelAgua2();   // Funcion para gestionar las interrupciones del sensor de nivel de agua 2
/*---------------------------------------------------
 *------------- Funciones auxiliares ----------------
 *---------------------------------------------------*/

float obtenerFrecuencia2(); // Funcion para obtener la frecuencia del caudalimetro 2
float obtenerFrecuencia1(); // Funcion para obtener la frecuencia del caudalimetro 1

/*---------------------------------------------------
 *------------------- Variables ----------------------
 *---------------------------------------------------*/
volatile bool pressedStart = false;   // Variable para controlar el estado del boton de arranque
volatile bool pressedStop = true;     // Variable para controlar el estado del boton de parada
volatile bool nivelAlcanzado = false; // Variable para controlar el estado del nivel
volatile int numPulsos1 = 0;          // Variable que almacena el numero de pulsos del sensor de flujo de agua 1
volatile int numPulsos2 = 0;          // Variable que almacena el numero de pulsos del sensor de flujo de agua 2
String mensajes = String(100);        // Variable que almacena los mensajes a enviar
float temperaturaAgua = 0;            // Variable que almacena la temperatura de la agua
float tempMax = 27;                   // Variable que almacena la temperatura maxima
float tempMin = 20;                   // Variable que almacena la temperatura minima

// Declaramos las variables globales para las configuraciones de las cadencias de las tareas
unsigned long cadenciaMedidaAmbiente = 180000; // Cadencia de medida del ambiente
unsigned long cadenciaMedidaCaudal = 5000;     // Cadencia de medida del caudal
unsigned long cadenciaMedidaAgua = 30000000;   // Cadencia de medida de la agua
unsigned long cadenciaLed = 1000;
unsigned long cadenciaMedidaTempAgua = 300000; // Cadencia de medida de la temperatura de la agua

void setup()
{
  BaseType_t xReturned; // Variable de control de creacion de tareas

  // Inicializamos los puertos de comunicaciones tanto par el debug como para la comunicacion con NodeMCU
  Serial.begin(115200);
  Serial3.begin(115200);
  // Creamos la tarea Start encargada de arrancar el sistema
  xReturned = xTaskCreate(Start, "Start", 200, NULL, TASK_PRIORITY + 1, &Start_Handler);
  if (xReturned != pdPASS)
  {
    Serial.println("ERROR AL CREAR START");
  }
  // Creamos la tareas ProcessCommand encargada de procesar comandos recibidos desde Node - Red
  xReturned = xTaskCreate(ProcessCommand, "ProcessCommand", 200, NULL, TASK_PRIORITY + 1, &ProcessCommand_Handler);
  if (xReturned != pdPASS)
  {
    Serial.println("ERROR AL CREAR PROCESSCOMMAND");
  }

  // Inicializamos el boton de arranque y de paro
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  // Adjuntamos las interrupciones que gestionaran los botones
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), callbackButtonStart, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON), callbackButtonStop, FALLING);

  // Creamos los semaforos y colas
  xStart = xSemaphoreCreateBinary(); // creamos el semeforo de arranque
  xStop = xSemaphoreCreateBinary();  // creamos el semeforo de  paro
  xLevel = xSemaphoreCreateBinary(); // creamos el semeforo de nivel
  // creamos el mutex que usaremos para garantizar condiciones de carrera
  xPlayMutex = xSemaphoreCreateMutex();
  // creamos la cola de comandos
  xQueueCommands = xQueueCreate(QUEUE_SIZE, sizeof(int));

  // Inicializamos los leds de control del sistema
  pinMode(LED_ON, OUTPUT);
  pinMode(LED_FALLO, OUTPUT);
  pinMode(LED_OFF, OUTPUT);
  // Encendemos el led de paro
  digitalWrite(LED_OFF, HIGH);

  // Sensor de nivel de agua 1
  attachInterrupt(digitalPinToInterrupt(SENSOR_NIVEL_AGUA1), callbackNivelAgua1, RISING);
  // Sensor de nivel de agua 2
  attachInterrupt(digitalPinToInterrupt(SENSOR_NIVEL_AGUA2), callbackNivelAgua2, RISING);
  // Añadimos las interrupciones de los caudalimetros
  attachInterrupt(digitalPinToInterrupt(CAUDALIMETRO1), callbackCaudalimetro1, RISING);
  attachInterrupt(digitalPinToInterrupt(CAUDALIMETRO2), callbackCaudalimetro2, RISING);

  // Mandamos el valor de las cadencias por defecto hacia node-Red
  Serial3.println("S [AcuaponicDuino/Start/Agua] <" + String(cadenciaMedidaAgua) + ">");
  Serial3.println("S [AcuaponicDuino/Start/TempAgua] <" + String(cadenciaMedidaTempAgua) + ">");
  Serial3.println("S [AcuaponicDuino/Start/Flujo] <" + String(cadenciaMedidaCaudal) + ">");
  Serial3.println("S [AcuaponicDuino/Start/Ambiental] <" + String(cadenciaMedidaAmbiente) + ">");
  Serial3.println("S [AcuaponicDuino/Start/tempMax] <" + String(tempMax) + ">");
  Serial3.println("S [AcuaponicDuino/Start/tempMin] <" + String(tempMin) + ">");

  // Inicializamos los pines de las electrovalvulas
  pinMode(ELECTROVALVULA_1, OUTPUT);
  digitalWrite(ELECTROVALVULA_1, HIGH);
  pinMode(ELECTROVALVULA_2, OUTPUT);
  digitalWrite(ELECTROVALVULA_2, HIGH);
  pinMode(ELECTROVALVULA_3, OUTPUT);
  digitalWrite(ELECTROVALVULA_3, HIGH);
  pinMode(ELECTROVALVULA_4, OUTPUT);
  digitalWrite(ELECTROVALVULA_4, HIGH);
  // Inicializamos los pines de la bomba
  pinMode(BOMBA_ONOF, OUTPUT);
  digitalWrite(BOMBA_ONOF, HIGH);
  // Inicializamos los pines del calentador
  pinMode(CALENTADOR, OUTPUT);
  digitalWrite(CALENTADOR, HIGH);
  // Inicializamos los pines del compresor
  pinMode(COMPRESOR, OUTPUT);
  digitalWrite(COMPRESOR, HIGH);
}

void loop()
{
  //--------------------------------------------------
  //*-------------- Controlado por tareas -------------
  //*--------------------------------------------------
}

//*--------------------------------------------------*/
//*---------------------- Tareas ---------------------*/
//*--------------------------------------------------*/

void ProcessCommand(void *pvParameters __attribute__((unused)))
{
  /* Tarea encarga de recibir y procesar los comandos de Node-Red. Estos comandos son envidado por el NodeMCU y recibidos por el puerto
     Serial3. De la misma manera tambien envia comandos y actualizaciones al NodeMCU.
   */
  // Funcion auxiliar para medir la marca de agua y ajustar el tamaño en memoria de nuestras tareas
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.println("\tHIGH WATER MARK PROCESS INICANDO: " + String(uxHighWaterMark));
  // Varibales auxiliares
  BaseType_t xStatus;
  String topic, command;
  char buff[50];
  unsigned int numero = 0;
  for (;;)
  {
    // Los mensajes llega a traves de la cola de comandos provenientes del puerto serie 3
    // Esperamos a que llegue un mensaje
    xStatus = xQueueReceive(xQueueCommands, &numero, portMAX_DELAY);
    if (xStatus == pdPASS)
    {
      // Se comprueba que el mensaje que ha llegado es correcto comprobando su extensión tanto aqui como en el serialEvent
      // si es correcto empezamos a procesar el mensaje
      if ((numero != 0) && (numero == mensajes.length()))
      {
        // Fragmentamos el mensaje en topic y command
        xSemaphoreTake(xPlayMutex, portMAX_DELAY);
        topic = mensajes.substring(mensajes.indexOf("[") + 1, mensajes.indexOf("]"));
        command = mensajes.substring(mensajes.indexOf("<") + 1, mensajes.indexOf(">"));
        mensajes = "";
        xSemaphoreGive(xPlayMutex);
        //   Procesamos el comando en funcion del topic del que proviene
        if (topic == "AcuaponicDuino/Commands")
        {
          if (command == "START")
          {
            if (pressedStart == false)
            {
              xSemaphoreGive(xStart);
              xSemaphoreTake(xPlayMutex, portMAX_DELAY);
              pressedStart = true;
              xSemaphoreGive(xPlayMutex);
            }
          }
          if (command == "STOP")
          {
            if (pressedStop == false)
            {
              xSemaphoreGive(xStop);
              xSemaphoreTake(xPlayMutex, portMAX_DELAY);
              pressedStop = true;
              xSemaphoreGive(xPlayMutex);
            }
          }
        }
        // A partir de aqui se procesan las cadencias que provienen de Node-RED si ha sido modificada se actualiza
        // con el nuevo valor y se vuelve a reenviar hacia Node-RED
        if (topic == "AcuaponicDuino/Config/Flujo")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          ltoa(cadenciaMedidaCaudal, buff, 10);
          if (strcmp(buff, command.c_str()) != 0)
          {
            cadenciaMedidaCaudal = command.toInt();
            Serial3.println("S [AcuaponicDuino/Start/Flujo] <" + String(cadenciaMedidaCaudal) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }
        if (topic == "AcuaponicDuino/Config/Temperatura")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          ltoa(cadenciaMedidaTempAgua, buff, 10);
          if (strcmp(buff, command.c_str()) != 0)
          {
            cadenciaMedidaTempAgua = command.toInt();
            Serial3.println("S [AcuaponicDuino/Start/TempAgua] <" + String(cadenciaMedidaTempAgua) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }
        if (topic == "AcuaponicDuino/Config/Agua")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          ltoa(cadenciaMedidaAgua, buff, 10);
          if (strcmp(buff, command.c_str()) != 0)
          {
            cadenciaMedidaAgua = command.toInt();
            Serial3.println("S [AcuaponicDuino/Start/Agua] <" + String(cadenciaMedidaAgua) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }

        if (topic == "AcuaponicDuino/Config/Ambiente")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          ltoa(cadenciaMedidaAmbiente, buff, 10);
          if (strcmp(buff, command.c_str()) != 0)
          {
            cadenciaMedidaAmbiente = command.toInt();
            Serial3.println("S [AcuaponicDuino/Start/Ambiental] <" + String(cadenciaMedidaAmbiente) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }

        if (topic == "AcuaponicDuino/Config/tempMax")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          if (command.toFloat()!= tempMax)
          {
            tempMax = command.toFloat();
            Serial3.println("S [AcuaponicDuino/Start/tempMax] <" + String(tempMax) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }

         if (topic == "AcuaponicDuino/Config/tempMin")
        {
          xSemaphoreTake(xPlayMutex, portMAX_DELAY);
          if (command.toFloat()!= tempMin)
          {
            tempMin = command.toFloat();
            Serial3.println("S [AcuaponicDuino/Start/tempMin] <" + String(tempMin) + ">");
          }
          xSemaphoreGive(xPlayMutex);
        }
      }
    }
    // Marca de agua en ejecucion
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println("\tHIGH WATER MARK PROCESS EJECUTANDO: " + String(uxHighWaterMark));
    // Serial.println("\tTarea ProcessCommand EJECUTANDO");
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void Start(void *pvParameters __attribute__((unused)))
{
  // Marcas de agua
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.println("\tMARCA DE AGUA DE START AL INICIAR: " + String(uxHighWaterMark));

  //  Variables de control para la creacion de tareas y espera del semaforo
  BaseType_t xReturned = pdFAIL, xStatus = pdFAIL;
  // Inicializamos la variable de pulsacion de boton de arranque
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  pressedStart = false;
  xSemaphoreGive(xPlayMutex);
  for (;;)
  {
    // Esperamos a que se pulse el boton de arranque y nos devuelva el semaforo por interrupcion
    xStatus = xSemaphoreTake(xStart, portMAX_DELAY);
    if (xStatus == pdPASS)
    {
      //  Una vez devuelto el semaforo por interrupcion, solo procesamos la primera pulsacion evitando asi rebotes
      if (pressedStart == true)
      {
        //  Comenzamos el arranque del sistema con la creacion de las diferentes tareas necesarias para la ejecucion
        //  Creamos la tarea de paro y le asignamos una mayor prioridad que al resto
        xReturned = xTaskCreate(Stop, "Stop", 200, NULL, TASK_PRIORITY + 1, &Stop_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR STOP");
        }
        // Creamos la tarea de señalizacion de los led para indicar que el sistema esta arrancado
        xReturned = xTaskCreate(StartBlink, "StartBlink", 100, NULL, TASK_PRIORITY, &StartBlink_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR STARTBLINK");
        }
        // Creamos la tarea de medicion de parametros ambientales
        xReturned = xTaskCreate(EnvMeasure, "EnvMeasure", 200, NULL, TASK_PRIORITY, &EnvMeasure_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR ENVMESURE");
        }
        // Creamos la tarea de medicion de flujo que controlara tambien la bomba
        xReturned = xTaskCreate(CaudalMeasure, "CaudalMeasure", 200, NULL, TASK_PRIORITY, &CaudalMeasure_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR CAUDALMEASURE");
        }
        // Creamos la tarea de medicion de parametros del agua
        xReturned = xTaskCreate(WaterMeasure, "WaterMeasure", 320, NULL, TASK_PRIORITY, &WaterMeasure_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR WATERMEASURE");
        }
        // Creamos la tarea de medicion de temperatura del agua que se ha separado de la tarea anterior por tener cadencias diferentes
        xReturned = xTaskCreate(TempWaterMeasure, "TempWaterMeasure", 200, NULL, TASK_PRIORITY, &TempWaterMeasure_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR TEMPWATERMEASURE");
        }

        // Apagamos el led de paro
        digitalWrite(LED_OFF, LOW);
        // Enecendemos la bomba de impulsion
        digitalWrite(BOMBA_ONOF, LOW);
        // Desviamos el flujo por la tuberia principal y reseteamos las electrovalvulas
        digitalWrite(ELECTROVALVULA_1, LOW);

        // Marca de agua en ejecucion
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.println("\tMARCA DE AGUA DE START EJECUTANDO: " + String(uxHighWaterMark));
        // Autodestrimos la tarea para liberar memoria
        vTaskDelete(Start_Handler);
      }
    }
  }
}
void Stop(void *pvParameters __attribute__((unused)))
{
  /* Tarea encargada de Parar el sistema. Para ello se queda esperando un semaforo que puede ser activado bien por interrupcion
     o bien por un comando recibido desde el dashboard de control. Una vez activado el semaforo y despreciado rebotes, creara las distintas
     tareas necesarias para la ejecucion del sistema asi como la tarea de paro. Activara la bomba principal y abrira las electrovalvulas necesarias
     para que el agua circule. Una vez realizado esto se autodestruye para liberar memoria.
   */
  // Marcas de agua
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.println("\tMARCA DE AGUA DE STOP AL INICIAR: " + String(uxHighWaterMark));
  // Variables de control para la creacion de tareas y espera del semaforo
  BaseType_t xStatus = pdFAIL, xReturned = pdFAIL;
  // Inicializamos la variable de pulsacion de boton de paro
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  pressedStop = false;
  xSemaphoreGive(xPlayMutex);
  for (;;)
  {
    // Esperamos a que se pulse el boton de paro y nos devuelva el semaforo por interrupcion
    xStatus = xSemaphoreTake(xStop, portMAX_DELAY);
    //  Una vez devuelto el semaforo por interrupcion, solo procesamos la primera pulsacion evitando asi rebotes
    if (xStatus == pdPASS)
    {
      if (pressedStop == true)
      {
        xSemaphoreTake(xPlayMutex, portMAX_DELAY);
        cadenciaLed = 1000; // reinciamos la cadencia del led por si hubiera fallos
        xSemaphoreGive(xPlayMutex);
        // Apagamos el led de fallo si lo hubiera
        digitalWrite(LED_FALLO, LOW);
        // Borramos las tareas escepto la que procesa los comandos externos
        vTaskDelete(StartBlink_Handler);
        vTaskDelete(EnvMeasure_Handler);
        vTaskDelete(CaudalMeasure_Handler);
        vTaskDelete(WaterMeasure_Handler);
        vTaskDelete(TempWaterMeasure_Handler);
        // Se vuelve a crear la tarea Start para volver a arrancar el sistema
        xReturned = xTaskCreate(Start, "Start", 200, NULL, TASK_PRIORITY + 1, &Start_Handler);
        if (xReturned != pdPASS)
        {
          Serial.println("ERROR AL CREAR START");
        }
        // Encendemos el led de paro
        digitalWrite(LED_OFF, HIGH);
        // Apagamos el led de arranque
        digitalWrite(LED_ON, LOW);
        // Apagamos la bomba de impulsion
        digitalWrite(BOMBA_ONOF, HIGH);
        // Apagamos el calentador y el compresor si estuvieran encencidos
        digitalWrite(CALENTADOR, HIGH);
        digitalWrite(COMPRESOR, HIGH);
        // reseteamos las electrovalvulas
        digitalWrite(ELECTROVALVULA_1, HIGH);
        digitalWrite(ELECTROVALVULA_2, HIGH);
        digitalWrite(ELECTROVALVULA_3, HIGH);
        digitalWrite(ELECTROVALVULA_4, HIGH);
        // Se manda un mensaje de reset a los gaujes del interfaz del sistema
        Serial3.println("S [AcuaponicDuino/Config/Stop] <STOP>");

        // Marca de agua en ejecucion
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.println("\tMARCA DE AGUA DE STIP: " + String(uxHighWaterMark));
        // Autodestrimos la tarea para liberar memoria
        vTaskDelete(Stop_Handler);
      }
    }
  }
}
void StartBlink(void *pvParameters __attribute__((unused)))
{
  /* Tarea encargada de la señalizacion de los leds del sistema.*/
  for (;;)
  {
    digitalWrite(LED_ON, HIGH);
    vTaskDelay(pdMS_TO_TICKS(cadenciaLed));
    digitalWrite(LED_ON, LOW);
    vTaskDelay(pdMS_TO_TICKS(cadenciaLed));
  }
}

void EnvMeasure(void *pvParameters __attribute__((unused)))
{
  /* Tarea encargada de la medicion de los parametros del Ambientales. Recopila datos y los envia por el puerto Serial3 hacia Mqtt */
  // Declaramos una instancia del sensor encargado de medir la humedad y la temperatura
  temperaturaHumedad sensorTempHumAmb(SENSOR_TEMP, SENSOR_HUM);
  // Variables que albergaran los valores de temperatura, humedad y luz ambientales
  float tempAmb, humedadAmb;
  int luz = 0;
  // Marcas de agua para memoria
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  //  Serial.println("\tHIGH WATER MARK MEDIDA AMBIENTAL INICANDO: " + String(uxHighWaterMark));
  for (;;)
  {
    // Obtenemos las lecturas de los sensores
    tempAmb = sensorTempHumAmb.getTemp();
    humedadAmb = sensorTempHumAmb.getHum();
    luz = analogRead(SENSOR_LUZ);
    // Serial.println("TEMPERATURA AMBIENTE: " + String(tempAmb));
    // Serial.println("HUMEDAD AMBIENTE: " + String(humedadAmb));
    // Serial.println("LUZ: " + String(luz));
    //     Las enviamos hacia el puerto serial para ser enviadas por Mqtt
    Serial3.println("S [AcuaponicDuino/Ambiente/Temperatura] <" + String(tempAmb) + ">");
    Serial3.println("S [AcuaponicDuino/Ambiente/Humedad] <" + String(humedadAmb) + ">");
    Serial3.println("S [AcuaponicDuino/Ambiente/Luz] <" + String(luz) + ">");
    // Marca de agua
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println("\tMARCA DE AGUA MEDIDA AMBIENTAL EJECUTANDO: " + String(uxHighWaterMark));
    // Entramos en espera de la siguiente medicion
    vTaskDelay(pdMS_TO_TICKS(cadenciaMedidaAmbiente));
  }
}
void CaudalMeasure(void *pvParameters __attribute__((unused)))
{
  /* Tarea encargada de la medicion del caudal. Recopila datos y los envia por el puerto Serial3 hacia Mqtt.*/
  // Variable para el calculo del caudal del sistema para los dos caudalimetros tanto el de entrada como el de salida
  float frecuencia1 = 0, frecuencia2 = 0;
  float caudalEntrada_L_m = 0, caudalSalida_L_m = 0;
  // Marcas de agua para memoria
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  //  Serial.println("\tHIGH WATER MARK CAUDAL INICANDO: " + String(uxHighWaterMark));
  for (;;)
  {
    // Obtenemos la frecuencia del caudalimetro de entrada y con ella calculamos el caudal en litros por minuto
    frecuencia1 = obtenerFrecuencia1();    // obtenemos la frecuencia del sensor 1
    caudalEntrada_L_m = frecuencia1 / 3.5; // calculamos el caudal en litros por minuto del sensor 1 usando parametro fabricante
    /// Serial.println("CAUDAL ENTRADA: " + String(caudalEntrada_L_m) + " L/m");
    //  Obtenemos la frecuencia del caudalimetro de salida y con ella calculamos el caudal en litros por minuto
    frecuencia2 = obtenerFrecuencia2();   // obtenemos la frecuencia del sensor 2
    caudalSalida_L_m = frecuencia2 / 3.5; // calculamos el caudal en litros por minuto del sensor 2 usando parametro fabricante
    // Serial.println("CAUDAL SALIDA: " + String(caudalSalida_L_m) + " L/m");
    //  Enviamos los datos por el puerto serial hacia Mqtt
    Serial3.println("S [AcuaponicDuino/Flujo/Entrada] <" + String(caudalEntrada_L_m) + ">");
    Serial3.println("S [AcuaponicDuino/Flujo/Salida] <" + String(caudalSalida_L_m) + ">");
    // Marcas de agua
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println("\tMARCA DE AGUA CAUDAL EJECUTANDO: " + String(uxHighWaterMark));
    // Entramos en espera de la siguiente medicion
    vTaskDelay(pdMS_TO_TICKS(cadenciaMedidaCaudal));
  }
}
void WaterMeasure(void *pvParameters __attribute__((unused)))
{
  // Instanciamos e inicializamos los sensores y variables auxiliares que necesitaremos
  DFRobot_PH sensorPh;
  float voltage, phValue, temperature, tds;
  sensorPh.begin();
  GravityTDS sensorTDS;
  sensorTDS.setPin(SENSOR_TDS);
  sensorTDS.setAref(5.0);
  sensorTDS.setAdcRange(1024);
  sensorTDS.begin();
  BaseType_t xStatus;
  // Marcas de agua para memoria
  // BaseType_t uxHighWaterMark, xStatus;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.println("\tMARCA DE AGUA MEDIDA-AGUA INICANDO: " + String(uxHighWaterMark));
  // Una vez creada la tarea esperamos 3 minutos a que el sistema se estabilice para tomar la medida
  vTaskDelay(pdMS_TO_TICKS(10000));

  for (;;)
  {
    // desviamos el flujo principal hacia el bypass
    digitalWrite(ELECTROVALVULA_1, HIGH);
    digitalWrite(ELECTROVALVULA_2, LOW);
    digitalWrite(ELECTROVALVULA_3, LOW);
    // Esperamos unos minutos para que se limpie la tuberia
    vTaskDelay(pdMS_TO_TICKS(180000)); // Esperamos 3 minutos
    // Llenamos la tuberia con agua
    digitalWrite(ELECTROVALVULA_3, HIGH);
    // El sensor de nivel devolverar el semaforo cuando la tuberia este llena
    xStatus = xSemaphoreTake(xLevel, portMAX_DELAY);
    if (xStatus == pdPASS)
    {
      // Reconducimo el flujo principal
      digitalWrite(ELECTROVALVULA_2, HIGH);
      digitalWrite(ELECTROVALVULA_1, LOW);
      // y se toma la medida
      // Obtenemos la temperatura del sensor temperatura del agua para compensar en la formula de ph y tds
      xSemaphoreTake(xPlayMutex, portMAX_DELAY);
      temperature = temperaturaAgua;
      xSemaphoreGive(xPlayMutex);
      voltage = analogRead(SENSOR_PH) / 1024.0 * 5000; // leemos el voltaje del sensor
      phValue = sensorPh.readPH(voltage, temperature); // lo transformamos en ph
      // Enviamos el valor medida hacia node-red
      Serial3.println("S [AcuaponicDuino/Agua/pH] <" + String(phValue) + ">");
      // Serial.println("PH: " + String(phValue));
      //  Compensamos el sensor con la temperatura del agua
      sensorTDS.setTemperature(temperature);
      sensorTDS.update();
      tds = sensorTDS.getTdsValue();
      // Serial.println("TDS: " + String(tds));
      //  Enviamos el valor medida hacia node-red
      Serial3.println("S [AcuaponicDuino/Agua/TDS] <" + String(tds) + ">");
      // Una vez medido vaciamos la tuberia
      digitalWrite(ELECTROVALVULA_4, LOW);
      // Esperamos al vaciado
      vTaskDelay(pdMS_TO_TICKS(60000)); // Esperamos 3 minutos
      digitalWrite(ELECTROVALVULA_4, HIGH);
    }

    // Marca de agua en ejecucion
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println("\tMARCA DE AGUA MEDIDA-AGUA EJECUTANDO: " + String(uxHighWaterMark));
    // esperamos a la proxima medicion
    vTaskDelay(pdMS_TO_TICKS(cadenciaMedidaAgua));
  }
}
void TempWaterMeasure(void *pvParameters __attribute__((unused)))
{
  /* Tarea encargada de la medicion de la temperatura del agua. Recopila datos y los envia por el puerto Serial3 hacia Mqtt.
     Se ha separado la medicion de la temperatura del agua del resto de parametros de medidas del agua por la diferencia de
     cadencias de tiempo entre las mediciones necesarias para la supervivencia de los peces.
  */
  // Instanciamos un sesnor de temperatura del agua
  sensorTemperaturaAgua sensor(SENSOR_TEMP_AGUA);
  // Variable para almacenar el valor de la temperatura del agua
  float temp = 0;
  // Marcas de agua
  // BaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.println("\tHIGH WATER MARK TEMPERATURA AGUA INICANDO: " + String(uxHighWaterMark));
  for (;;)
  {
    // Obtenemos el valor de la temperatura del agua
    temp = sensor.getTemp();
    // Fijamos el valor de la temperatura en la variable global compartida mediante un mutex
    xSemaphoreTake(xPlayMutex, portMAX_DELAY);
    temperaturaAgua = temp;
    xSemaphoreGive(xPlayMutex);
    // Enviamos el valor obtendido por el puerto serial hacia Mqtt
    // Serial.println("TEMPERATURA AGUA: " + String(temp) + "º C");
    Serial3.println("S [AcuaponicDuino/Agua/Temperatura] <" + String(temp) + ">");
    // Marcas de agua
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println("\tMARCA DE AGUA TEMPERATURA AGUA EJECUTANDO: " + String(uxHighWaterMark));
    // En funcion de la temperatura calentamos o no el agua
    if (temp > tempMax)
    {
      digitalWrite(CALENTADOR, HIGH);
    }
    if (temp < tempMin)
    {
      digitalWrite(CALENTADOR, LOW);
    }
    // Esperamos a la proxima medicion
    vTaskDelay(pdMS_TO_TICKS(cadenciaMedidaTempAgua));
  }
}

//*--------------------------------------------------*/
//*---------------------- Callbacks -----------------*/
//*--------------------------------------------------*/

void callbackButtonStart()
{
  // Si el boton no estaba pulsado ya... gestionamos la pulsacion
  if (pressedStart == false)
  {
    xSemaphoreTakeFromISR(xPlayMutex, NULL);
    pressedStart = true;
    xSemaphoreGiveFromISR(xPlayMutex, NULL);
    xSemaphoreGiveFromISR(xStart, NULL);
  }
}
void callbackButtonStop()
{
  // Si el boton no estaba pulsado ya... gestionamos la pulsacion
  if (pressedStop == false)
  {
    xSemaphoreTakeFromISR(xPlayMutex, NULL);
    pressedStop = true;
    xSemaphoreGiveFromISR(xPlayMutex, NULL);
    xSemaphoreGiveFromISR(xStop, NULL);
  }
}

void callbackCaudalimetro1()
{
  // simplemente cuenta los pulso del sensor
  xSemaphoreTakeFromISR(xPlayMutex, NULL);
  numPulsos1++;
  xSemaphoreGiveFromISR(xPlayMutex, NULL);
}
void callbackCaudalimetro2()
{
  // simplemente cuenta los pulso del sensor
  xSemaphoreTakeFromISR(xPlayMutex, NULL);
  numPulsos2++;
  xSemaphoreGiveFromISR(xPlayMutex, NULL);
}
// Devuelve el semaforo cuando el nivel de agua es alcanzado
void callbackNivelAgua1()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 1000)
  {
    xSemaphoreTakeFromISR(xPlayMutex, NULL);
    nivelAlcanzado = true;
    xSemaphoreGiveFromISR(xPlayMutex, NULL);
    xSemaphoreGiveFromISR(xLevel, NULL);
  }
  last_interrupt_time = interrupt_time;
}
// Si hay desbordamiento en el acuario, para bomba y señaliza fallo
void callbackNivelAgua2()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 1000)
  {
    xSemaphoreTakeFromISR(xPlayMutex, NULL);
    digitalWrite(LED_FALLO, HIGH);
    digitalWrite(BOMBA_ONOF,HIGH);
    cadenciaLed= 100;
    xSemaphoreGiveFromISR(xLevel, NULL);
  }
  last_interrupt_time = interrupt_time;
}

float obtenerFrecuencia1()
{
  float frecuencia1 = 0;
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  numPulsos1 = 0;
  xSemaphoreGive(xPlayMutex);
  interrupts();
  delay(1000);
  noInterrupts();
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  frecuencia1 = numPulsos1;
  xSemaphoreGive(xPlayMutex);
  return frecuencia1;
}

float obtenerFrecuencia2()
{
  float frecuencia2 = 0;
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  numPulsos2 = 0;
  xSemaphoreGive(xPlayMutex);
  interrupts();
  delay(1000);
  noInterrupts();
  xSemaphoreTake(xPlayMutex, portMAX_DELAY);
  frecuencia2 = numPulsos2;
  xSemaphoreGive(xPlayMutex);
  return frecuencia2;
}

void serialEvent3()
{

  // Se obtienes el contenido del serial y se manda la longitud de lo obtenido por la cola para la comprobacion
  // correcta del mensaje.
  int num = 0;
  xSemaphoreTakeFromISR(xPlayMutex, NULL);
  if (Serial3.available() > 0)
  {
    mensajes = Serial3.readStringUntil('\n');
    if (mensajes.startsWith("R"))
    {
      num = mensajes.length();
      xQueueSendFromISR(xQueueCommands, &num, NULL);
    }
  }
  xSemaphoreGiveFromISR(xPlayMutex, NULL);
}