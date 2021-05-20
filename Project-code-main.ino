/* Sundhedsteknologi Aalborg Universitet
   Gruppe: 21gr4406
   Dato 21-05-2021
   Dette er koden som overføres til MCU'en, som indeholder sampling, bluetooth, SPIFFS samt analyse.
*/

/***** Indlæser SPI/SPIFFS biblioteker *****/
#include <SPI.h>
#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true
SPIClass ADXL_SPI(HSPI);

// Indlæser BLE biblioteker
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define BAUDRATE 115200
#define max_sample 45001

/********ADXL355 register adresses & RANGE & FS ******/
#define ZDATA3                0x0E
#define ZDATA2                0x0F
#define ZDATA1                0x10
#define FILTER                0x28
#define RANGE                 0x2C
#define POWER_CTL             0x2D

#define  RANGE_2G             0x01

#define ADXL355_ODR_500HZ 0x03u
#define ADXL355_ODR_1000HZ 0x02u

// Pinkonfiguration for ADXL355 & AD8232
#define SCLK  17
#define MISO  21
#define MOSI  2
#define SS    22
#define DRDY  13

#define ECG   39

// Operations
const int READ_BYTE =         0x01;
const int WRITE_BYTE =        0x00;
bool first_write = true;

/**** Globale variabler ****/
double AC_amplitude_mean = 0;
// variabler til rå data
uint32_t zdata;
int32_t z;
int32_t axisMeasures[] = {0, 0, 0};

// Struct til at sende data via queue
struct queue_data
{
  int32_t skg;
  int32_t ekg;
};
queue_data data_ekg_skg;
queue_data data_ekg_skg_receive;

#define QUEUE_LENGTH 1000
#define ITEM_SIZE sizeof( queue_data )

SemaphoreHandle_t syncSem; // Laver Semaphore
static QueueHandle_t  msgQ;  // Laver Queue

int cnt_ISR = 0;

int32_t array_gem1[1000];
int32_t array_gem2[1000];
int32_t array_length = sizeof(array_gem2) / sizeof(int32_t);
int array_gem_state = 1;
bool save_flag = false;
bool flag_start = false;

int cnt = 0;

float locate_thr_R = 0;

//SPIFFS READ
int32_t skg_SPIFFS[2000];
int32_t ekg_SPIFFS[2000];
int cnt_stop = 4000;
int cnt_start = 0;

//Analyse
int cnt_R_test = 0;
int32_t AC_amplitude;
int cnt_AC = 0;

// til debug
byte error = 2;   // 2 = no error, 3 = buffer full, 4 = time out
int file_error = 0;
int32_t dummy = 0;
int32_t data_to_plot; //Fejlfinding

/**** Taskhandle *****/
TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;
TaskHandle_t TaskHandle_4;


// Funktioner til accelerometer
/* Skriv data til specifikke register adresser - er brugt i setup */
void writeRegister(byte thisRegister, byte thisRegisterContent) {
  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
  digitalWrite(SS, LOW);
  ADXL_SPI.transfer(dataToSend);
  ADXL_SPI.transfer(thisRegisterContent);
  digitalWrite(SS, HIGH);
}

/* Læs data fra specifikke register adresser */
unsigned int readRegistry(byte thisRegister) {
  unsigned int registerContent = 0;
  byte dataToSend = (thisRegister << 1) | READ_BYTE;
  digitalWrite(SS, LOW);
  ADXL_SPI.transfer(dataToSend);
  registerContent = ADXL_SPI.transfer(0x00);
  digitalWrite(SS, HIGH);
  return registerContent;
}

/* Læs data fra 3 registre fra z aksen - bruges i get_acc_z()*/
void read_XYZ_Data(int *registerContentArray) {
  byte dataToSend = (ZDATA3 << 1) | READ_BYTE;

  digitalWrite(SS, LOW);
  ADXL_SPI.transfer(dataToSend);
  for (int i = 0; i < 3; i = i + 1) {
    registerContentArray[i] = ADXL_SPI.transfer(0x00);
  }
  digitalWrite(SS, HIGH);
}

// ISR (interrupt service routine)
void IRAM_ATTR ISR() {
  xSemaphoreGiveFromISR(syncSem, NULL);
}

/************** BLUETOOTH *******************/

// BLE UUID
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Opsætter UUID
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// deviceConnected = true, hvis client er forbundet til server. deviceConnected = false, hvis modsatte er gældende.
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("device disconnected");
      deviceConnected = false;
    }
};


// Modtager startkommando fra MatLab vha. funktionen write(c,'data')
class StartKommando: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string startKom = pCharacteristic->getValue();

      switch (startKom[0])
      {
        case 's':
          Serial.println("Optagelse startet");
          writeRegister(POWER_CTL, 0);                // Enable measure mode
          delay(100);                                 // Give the sensor time to set up:
          attachInterrupt(DRDY, ISR, RISING);
          break;
        default:
          break;
      }

      Serial.println(" ");
      Serial.println("================Start==================");
      Serial.println(" ");
      Serial.print("Startkommando modtaget: ");
      //      Serial.println(startKom1);

    }
};

void BLE() {
  Serial.println(deviceConnected);

  // notify AC-amplitude
  if (deviceConnected) {
    pCharacteristic->setValue((uint8_t*)&AC_amplitude_mean, 8); // Vi modtager AC-amplituden som en double (8 bytes) og typecaster den til en uint8_t
    pCharacteristic->notify();
    delay(1000);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    //Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

/********************SPIFFS*************************/
//funktion til at skrive til SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
//funktion til at appende et array til en fil
void append_array_to_File(fs::FS &fs, const char * path, int32_t * data2save, int datalength) {
  File file;
  char str[9]; // negativt fortegn + 7 tal + ","
  if (first_write == true) {
    file = fs.open(path, FILE_WRITE);
    first_write = false;
  }
  else {
    file = fs.open(path, FILE_APPEND);
  }
  if (!file) {
    file_error = 1;
  }
  else
  {
    file_error = 0;
    for (int i = 0; i < datalength; i++)
    {
      sprintf(str, "%d,", data2save[i]);
      file.print(str);
    }
  }
}

//funktion til at slette en fil i SPIFFS
void deleteFile(fs::FS &fs, const char * path) {    // Slet alt der lå i SPIFFS
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

//funktion til at læse en fil i SPIFFS
void readFile(fs::FS &fs, const char * path) {
  //Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path);

  char buffer_SPIFFS[64];

  int cnt_read = 0;
  int cnt_read_skg = 0;
  int cnt_read_ekg = 0;

  while (file.available()) {
    int32_t SPIFFS_data_holder = file.readBytesUntil(',', buffer_SPIFFS, sizeof(buffer_SPIFFS) - 1);
    buffer_SPIFFS[SPIFFS_data_holder] = 0;
    if (cnt_read < cnt_stop && cnt_start <= cnt_read) {
      //array_SPIFFS_read[cnt_read] = atoi(buffer_SPIFFS);
      if ( (cnt_read % 2) == 0) {
        skg_SPIFFS[cnt_read_skg] = atoi(buffer_SPIFFS);
        cnt_read_skg++;
      }
      else {
        ekg_SPIFFS[cnt_read_ekg] = atoi(buffer_SPIFFS);
        cnt_read_ekg++;
      }
    }
    cnt_read++;
    if (cnt_read == cnt_stop) {
      // Kør analyse af data
      cnt_read = 0;
      cnt_read_skg = 0;
      cnt_read_ekg = 0;
      cnt_start = cnt_stop;
      cnt_stop += 4000;
      file.close();
    }
  }
}

// Konverter data fra accelerometer til two's complement
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
  int32_t volatile i32Conversion = 0;

  if ((ui32SensorData & 0x00080000)  == 0x00080000) { // Hvis tallet fra sensoren har et 1-tal på x19 konverteres tallet til to-komplement
    // Sensorvalue & 10000000000000000000 == 10000000000000000000 (1-tallet på plads 20 fra acc sammenlignes med 0x00080000 - hvis det er sandt skal den konverteres)
    i32Conversion = (ui32SensorData | 0xFFF00000);
  } else
  {
    i32Conversion = ui32SensorData;
  }
  return i32Conversion;
}

// Læs accelerometer værdier
int32_t get_acc_z() {
  read_XYZ_Data(axisMeasures);         // Read accelerometer data

  uint32_t zdata = (axisMeasures[2] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[0] << 12);

  int32_t z = ADXL355_Acceleration_Data_Conversion(zdata);

  //return dummy++;
  return z;
}


/***************************************************** TASKS **************************************************************/

// t1_sample starter når den får en semaphore. Semaphoren bliver givet hver gang der kommer et interrupt. Interrupt kommer hver gang, accelerometer data er klar.
// t1_sample sender det data den får til queue.
void t1_sample( void * parameter )
{
  for (;;)
  {
    if ( xSemaphoreTake(syncSem, (TickType_t)1000)  == pdTRUE )  // Her modtager vi semaphore
    {

      data_ekg_skg.skg = get_acc_z();          //sampler accelerometer data
      data_ekg_skg.ekg = analogRead(ECG);      //sampler ECG data

      if (xQueueSend(msgQ,  (void *)&data_ekg_skg , 0) == pdTRUE) {  //send dataet i køen (som struct). If statement er brugt til fejlfinding.
        error = 2;
      }
      else {
        error = 3;
        Serial.println("msg not send - buffer full ?");   //buffer er fyldt
      }
      cnt_ISR ++;
      if (cnt_ISR == max_sample) {                        // max sample er de antal samples vi ønsker i alt.
        Serial.print("Sampling færdig");
        detachInterrupt(DRDY);
        writeRegister(POWER_CTL, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);            //Giver CPU tid til andre tasks, for at sikre andre task når at blive færdig inden de bliver suspended
        vTaskSuspend(TaskHandle_2);
        vTaskSuspend(TaskHandle_3);
        vTaskResume(TaskHandle_4);
        vTaskSuspend(TaskHandle_1);
      }
    }
    else {
      error = 4;
      //Serial.println("time out - DRDY er ikke gået høj indenfor 1000ms");  //hvis semaphor ikke bliver givet.
    }
  }
}


// t2_save2array læser fra queue og gemmer i arrays. Disse arrays skal gemmes til SPIFFS (t3).
void t2_save2array( void * parameter )
{
  for (;;)
  {
    if (xQueueReceive(msgQ, (void *)&data_ekg_skg_receive, 100) == pdTRUE) {     // Tjekker om der er data i queue. Henter dataet i køen som ligger som et struct med en SKG og EKG sample.
      if (cnt < array_length) {
        array_gem1[cnt] = data_ekg_skg_receive.skg;                              // SKG og EKG data bliver sorteret i array_gem1 som [SKG, EKG, SKG, EKG, SKG, EKG ...]
        data_to_plot = array_gem1[cnt];
        cnt++;
        array_gem1[cnt] = data_ekg_skg_receive.ekg;
        if (cnt == array_length - 1) {
          save_flag = true;                                                      // Sætter flag når array er fyldt op
          array_gem_state = 1;
        }
      }
      else if (cnt >= array_length && cnt < 2 * array_length) {                  // Når array_gem1 er fyldt, begynder den at fylde array_gem2 op.
        array_gem2[cnt - array_length] = data_ekg_skg_receive.skg;
        data_to_plot = array_gem2[cnt - array_length];
        cnt++;
        array_gem2[cnt - array_length] = data_ekg_skg_receive.ekg;
        if (cnt == 2 * array_length - 1) {
          save_flag = true;
          array_gem_state = 0;
        }
      }
      cnt++;
      if (cnt == 2 * array_length) {
        cnt = 0;
      }
      // Debug
      //Serial.print(data_to_plot);
      //Serial.print(" ");
      //Serial.print(array_gem_state);
      //Serial.print(" ");
      //Serial.println(save_flag);
      //Serial.print(" ");
      //Serial.print(file_error);
      //Serial.print(" ");
      //Serial.println(error);
      //Serial.println(save_flag);
    }
  }
}

// t3_save2SPIFFS gemmer arrays i SPIFFS.
void t3_save2SPIFFS( void * parameter )
{
  while (1)
  {

    if (save_flag == true) {
      //long x = micros();
      save_flag = false;
      if (array_gem_state == 1)                                                  //array_gem_state bestemmer hvilket array der skal gemmes
      {
        append_array_to_File(SPIFFS, "/data.txt", array_gem1, array_length);
      }
      else if (array_gem_state == 0)
      {
        append_array_to_File(SPIFFS, "/data.txt", array_gem2, array_length);
      }
      //Serial.printf("tid = %.5f\n", ((float)(micros() - x)) / 1000000.0);      // Brugt til at teste skrivetiden
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);

  }
}

//t4_read_analyse læser dataet fra SPIFFS og kører det igennem analyse koden. Her bliver gennemsnittet af AC-amplituden også udregnet. 
//t4_read_analyse sender den gennemsnitlige AC-amplitude via BLE, til matlab GUI. 
void t4_read_analyse( void * parameter ) {
  while (1)
  {
    if (cnt_ISR == max_sample) {
      readFile(SPIFFS, "/data.txt");
      analyze_ECG_and_SCG();
      vTaskDelay(1 / portTICK_PERIOD_MS);
      if (cnt_start > (2 * max_sample) - 100 ) {
        Serial.printf("R-tak: %d \n", cnt_R_test);
        Serial.printf("AC-amplituder: %d \n", cnt_AC);
        AC_amplitude_mean = (double)AC_amplitude / (double)cnt_AC;
        Serial.println(AC_amplitude_mean);
        BLE();
        vTaskSuspend(TaskHandle_4);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/************************************************* SETUP ****************************************************/

void setup() {
  Serial.begin(BAUDRATE);

  delay(1000); // Konfigurer serial

  /************EKG SETUP ************/
  pinMode(ECG, INPUT);

  /*************SPIFFS *************/
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  // formater SPIFFS hvis skrivetiden er ved at blive for høj
  /*bool formatted = SPIFFS.format(); // Kør hvis det går langsomt

    if (formatted) {
    Serial.println("\n\nSuccess formatting");
    } else {
    Serial.println("\n\nError formatting");
    }
  */

  deleteFile(SPIFFS, "/data.txt");
  writeFile(SPIFFS, "/data.txt", " ");


  /**********BLUETOOTH****************/

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->setCallbacks(new StartKommando());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");



  /********** ADXL setup **********/
  ADXL_SPI.begin(SCLK, MISO, MOSI, SS);
  ADXL_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // 1000khz clock
  pinMode(SS, OUTPUT);
  pinMode(DRDY, INPUT_PULLDOWN);                       /* Set DRDY pin as input */
  writeRegister(RANGE, RANGE_2G);             // 2G
  writeRegister(FILTER, ADXL355_ODR_1000HZ);   // Enable measure mode

  /*********** freeRTOS ********/
  msgQ = xQueueCreate( QUEUE_LENGTH,   // Elementer som køen kan indeholde
                       ITEM_SIZE );    // Størrelsen af hvert element

  syncSem = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(t1_sample, "t1_sample", 20000, NULL,  4, &TaskHandle_1, 0);
  xTaskCreatePinnedToCore(t2_save2array, "t2_save2array", 20000, NULL,  3, &TaskHandle_2, 0);
  xTaskCreatePinnedToCore(t3_save2SPIFFS, "t3_save2SPIFFS", 10000, NULL,  1, &TaskHandle_3, 0);
  xTaskCreatePinnedToCore(t4_read_analyse, "t4_read_analyse", 10000, NULL,  5, &TaskHandle_4, 0);
  vTaskSuspend(TaskHandle_4);
  /*                                                          ^--  ´core (0or 1 on esp32)
                                                       ^------  handle to task code
                                                  ^---------- priority 0(lowest) to  (configMAX_PRIORITIES-1)
                                             ^---------------- pointer to parameter to be used as pParm in task
                                       ^--------------------- amoont of stak to be allocated (bytes in ESP32)
                                 ^--------------------------- name of task
                             ^---------------------------------- function to be called*/
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {}
