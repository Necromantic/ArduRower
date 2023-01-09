#include <Arduino.h>

#include "RunningAverage.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage

#define USE_SSD1306      1
#define USE_BATTERY      0
#define USE_BLE          1
#define USE_MQTT         1
#define USE_HTTP         1

#if USE_SSD1306
#include "SSD1306.h"
#include "font.h" //// Created by http://oleddisplay.squix.ch/
#endif

#if USE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#endif

#if USE_MQTT || USE_HTTP
#include <WiFi.h>
#include <esp_wifi.h>
#endif

#if USE_MQTT
#include <PubSubClient.h>
#endif

#if USE_HTTP
#include <ESPAsyncWebServer.h>
#include "index.h"  //Web page header file
#endif

#if USE_MQTT || USE_HTTP
const char* ssid = "*****";
const char* password = "*****";
WiFiClient wifiClient;
#endif

#define MSG_BUFFER_SIZE	(200)
char msg[MSG_BUFFER_SIZE];

#if USE_MQTT
PubSubClient mqttClient;
const char* mqttServer = "192.168.0.10";
const uint16_t mqttPort = 1883;
unsigned long lastMsg = 0;
long lastReconnectAttempt = 0;
const char* mqttOutTopic = "WaterRower/Out";
const char* mqttInTopic = "WaterRower/In";
#endif

#if USE_HTTP
AsyncWebServer* webServer;
AsyncWebSocket* webSocket;
#endif

QueueHandle_t xQueue = xQueueCreate(1, sizeof(char)*MSG_BUFFER_SIZE);

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#if USE_SSD1306
// Oled Display
#define SDA_OLED   17
#define SCL_OLED   18
#define RST_OLED   21 //RST must be set by software

SSD1306  display(0x3c, SDA_OLED, SCL_OLED, RST_OLED);
#endif

#if USE_BATTERY
#define BATPIN 37
#define MinADC 1095
#define MaxADC 1437
/*#define Fbattery    3700  //The default battery is 3700mv when the battery is fully charged.
#define XS 0.0025f;      //The returned reading is multiplied by this XS to get the battery voltage.
#define MUL 1000;*/
#endif

#define _VERSION          0.05

#if USE_BLE
// Global Define 
#define BLE_SERVICE_NAME "WR S4BL3"           // name of the Bluetooth Service 
#endif
      
#if USE_BLE
#define FitnessMachineService       0x1826
#define FitnessMachineControlPoint  0x2AD9    // Beta Implementation
#define FitnessMachineFeature       0x2ACC    // CX Not implemented yet
#define FitnessMachineStatus        0x2ADA    // Beta Implementation
#define FitnessMachineRowerData     0x2AD1    // CX Main cx implemented
#define DEVICE_INFORMATION          0x180A

#if USE_BATTERY
#define batteryService              0x180F    // Additionnal battery service
#define batteryLevel                0x2A19    // Additionnal cx to battery service
#endif

BLEServer *pServer = NULL;
BLECharacteristic * pCtrCharacteristic;
BLECharacteristic * pDtCharacteristic;
BLECharacteristic * pFmfCharacteristic;
BLECharacteristic * pStCharacteristic;
#if USE_BATTERY
BLECharacteristic * pBatCharacteristic;
#endif
BLEAdvertising *pAdvertising;

BLECharacteristic * pCharacteristic24;
BLECharacteristic * pCharacteristic25;
BLECharacteristic * pCharacteristic26;
BLECharacteristic * pCharacteristic27;
BLECharacteristic * pCharacteristic28;
BLECharacteristic * pCharacteristic29;

bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint32_t value = 0;
//uint8_t txValue = 0;

// Service
int32_t fitnessMachineServiceId;

#if USE_BATTERY
int32_t batteryServiceId;
#endif

uint16_t  rowerDataFlagsP1=0b0000011111110;
uint16_t  rowerDataFlagsP2=0b1111100000001;

  //P1
  // 0000000000001 - 1   - 0x001 - More Data 0 <!> WARNINNG <!> This Bit is working the opposite way, 0 means field is present, 1 means not present
  // 0000000000010 - 2   - 0x002 - Average Stroke present
  // 0000000000100 - 4   - 0x004 - Total Distance Present
  // 0000000001000 - 8   - 0x008 - Instantaneous Pace present
  // 0000000010000 - 16  - 0x010 - Average Pace Present
  // 0000000100000 - 32  - 0x020 - Instantaneous Power present
  // 0000001000000 - 64  - 0x040 - Average Power present
  // 0000010000000 - 128 - 0x080 - Resistance Level present
  //P2
  // 0000100000000 - 256 - 0x080 - Expended Energy present
  // 0001000000000 - 512 - 0x080 - Heart Rate present
  // 0010000000000 - 1024- 0x080 - Metabolic Equivalent present
  // 0100000000000 - 2048- 0x080 - Elapsed Time present
  // 1000000000000 - 4096- 0x080 - Remaining Time present

  //  C1  Stroke Rate             uint8     Position    2 (After the Flag 2bytes)
  //  C1  Stroke Count            uint16    Position    3 
  //  C2  Average Stroke Rate     uint8     Position    5
  //  C3  Total Distance          uint24    Position    6
  //  C4  Instantaneous Pace      uint16    Position    9
  //  C5  Average Pace            uint16    Position    11
  //  C6  Instantaneous Power     sint16    Position    13
  //  C7  Average Power           sint16    Position    15
  //  C8  Resistance Level        sint16    Position    17
  //  C9  Total Energy            uint16    Position    19
  //  C9  Energy Per Hour         uint16    Position    21
  //  C9  Energy Per Minute       uint8     Position    23
  //  C10 Heart Rate              uint8     Position    24
  //  C11 Metabolic Equivalent    uint8     Position    25
  //  C12 Elapsed Time            uint16    Position    26
  //  C13 Remaining Time          uint16    Position    28

  // fitnessMachineControlPointId
  // OPCODE     DESCRIPTION                                             PARAMETERS
  // 0x01       Reset                                                   N/A
  // 0x02       Fitness Machine Stopped or Paused by the User
  // 0x03       Fitness Machine Stopped by Safety Key                   N/A
  // 0x04       Fitness Machine Started or Resumed by the User          N/A
  // 0x05       Target Speed Changed
  // 0x06       Target Incline Changed
  // 0x07       Target Resistance Level Changed
  // 0x08       Target Power Changed
  // 0x09       Target Heart Rate Changed
  // 0x0B       Targeted Number of Steps Changed                        New Targeted Number of Steps Value (UINT16, in Steps with a resolution of 1)
  // 0x0C       Targeted Number of Strides Changed                      New Targeted Number of Strides (UINT16, in Stride with a resolution of 1)
  // 0x0D       Targeted Distance Changed                               New Targeted Distance (UINT24, in Meters with a resolution of 1)
  // 0x0E       Targeted Training Time Changed                          New Targeted Training Time (UINT16, in Seconds with a resolution of 1)

bool bleInitFlag=false;
bool bleConnectionStatus=false;
#endif


struct rowerDataKpi{
  int bpm; // Start of Part 1
  int strokeCount;
  //int tmpstrokeRate;
  int strokeRate;
  int averageStokeRate;
  int totalDistance;
  int instantaneousPace;
  int tmpinstantaneousPace;
  int averagePace;
  int instantaneousPower;
  int averagePower;
  int resistanceLevel;
  int totalEnergy; // Start of Part 2
  int energyPerHour;
  int energyPerMinute;
  int heartRate;
  int metabolicEquivalent;
  int elapsedTime;
  int elapsedTimeSec;
  int elapsedTimeMin;
  int elapsedTimeHour;
  int remainingTime;
};

struct rowerDataKpi rdKpi;

unsigned long currentTime=0;
unsigned long previousTime=0;
unsigned long battPreviousTime=0;

/*-----------------SETUP--PINS-----------------------------*/
const int ROWERINPUT = 2; // the input pin where the waterrower sensor is connected
const int BUTTONSPIN = 0;

/*-----------------CONSTANTS-------------------------------*/
//const float Ratio = 4.8; // from old script 4.8; meters per rpm = circumference of rotor (D=34cm) -> 1,068m -> Ratio = 0.936 ; WaterRower 7,3 m/St. -> Ratio: 3.156
//const float Ratio = 0.79; //one magnet
const float Ratio = 3.156;

RunningAverage stm_RA(21); // size of array for strokes/min
RunningAverage mps_RA(7); // size of array for meters/second -> emulates momentum of boat

#if USE_BATTERY
RunningAverage battery_RA(40);
#endif

/*-----------------VARIABLES-----------------------------*/
long start;
float rounds;
volatile unsigned long click_time = 0;
volatile unsigned long last_click_time = 0;
volatile unsigned long old_split = 0;
volatile unsigned long split_time = 0;
volatile unsigned long start_split;
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
volatile int clicks = 0;
volatile int clicks_old = 0;
volatile float data_output = 0;
volatile long rpm = 0;
volatile long old_rpm = 0;
volatile long stm = 0;
volatile int old_strokes = 0;
volatile float spm = 0;
int old_spm = 0;
volatile long stmra = 0;
int accel = 0;
int puffer = 0;
volatile float Ms = 0;
volatile int meters = 0;
volatile int meters_old = 0;
//volatile int trigger = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
int stage = 0; // sets case for what parameter to be displayed
int rotation = 0;
int strokes = 0;
int trend = 0;
int resetTimer = 0;


#if USE_BLE
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("CONNECTED");
      resetTimer = 0;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("DISCONNECTED");
      
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        if (rxValue.empty())
          Serial.print("-Null-");
        else {
          for (int i = 0; i < rxValue.length(); i++) {
            char chr = rxValue[i];
            Serial.print(chr);
            Serial.print(" Int: ");
            Serial.print((int)chr);
          }
        }  
          Serial.println();
        Serial.println("*********");
      }
    }
};

void initBLE(){
  // Initialise the module
  Serial.println("Init BLE:");
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  // Create the BLE Device
  BLEDevice::init(BLE_SERVICE_NAME);
  
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Add the Fitness Machine Service definition 
  // Service ID should be 1 
  Serial.println(F("Adding the Fitness Machine Service definition (UUID = 0x1826): "));
  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)FitnessMachineService));

  // Add the Fitness Machine Rower Data characteristic 
  // Chars ID for Measurement should be 1 
    pDtCharacteristic = pService->createCharacteristic(
          BLEUUID((uint16_t)FitnessMachineRowerData),
          BLECharacteristic::PROPERTY_NOTIFY
          );
    pDtCharacteristic->addDescriptor(new BLE2902());      
    pDtCharacteristic->setCallbacks(new MyCallbacks());

    pFmfCharacteristic = pService->createCharacteristic(
          BLEUUID((uint16_t)FitnessMachineFeature),
          BLECharacteristic::PROPERTY_READ
          );
    pFmfCharacteristic->addDescriptor(new BLE2902());

    // Add the Fitness Machine Control Point characteristic 
    pCtrCharacteristic = pService->createCharacteristic(
          BLEUUID((uint16_t)FitnessMachineControlPoint),
          BLECharacteristic::PROPERTY_WRITE 
          );
    pCtrCharacteristic->addDescriptor(new BLE2902());
    pCtrCharacteristic->setCallbacks(new MyCallbacks());

#if USE_BATTERY
    pBatCharacteristic = pService->createCharacteristic(
          BLEUUID((uint16_t)batteryLevel),
          BLECharacteristic::PROPERTY_NOTIFY
          );
    pBatCharacteristic->addDescriptor(new BLE2902());
#endif

    // Add the Fitness Machine Status characteristic 
    pStCharacteristic = pService->createCharacteristic(
          BLEUUID((uint16_t)FitnessMachineStatus),
          BLECharacteristic::PROPERTY_NOTIFY
          );
    pStCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();
 
  //-------------------------------------------------------------------------------------

  BLEService *pService2 = pServer->createService(BLEUUID((uint16_t)DEVICE_INFORMATION));
  pCharacteristic24 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A24),
    BLECharacteristic::PROPERTY_READ
    );
  pCharacteristic25 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A25),
    BLECharacteristic::PROPERTY_READ
    );
  pCharacteristic26 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A26),
    BLECharacteristic::PROPERTY_READ
    );    
  pCharacteristic27 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A27),
    BLECharacteristic::PROPERTY_READ
    );
  pCharacteristic28 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A28),
    BLECharacteristic::PROPERTY_READ
    );
  pCharacteristic29 = pService2->createCharacteristic(
    BLEUUID((uint16_t)0x2A29),
    BLECharacteristic::PROPERTY_READ
    );            
  pService2->start();

  pCharacteristic24->setValue("4");
  pCharacteristic25->setValue("0000");
  pCharacteristic26->setValue("0.30");
  pCharacteristic27->setValue("2.2BLE");
  pCharacteristic28->setValue("4.3");
  pCharacteristic29->setValue("Waterrower");

  char cRower[8];
  cRower[0]=0x26;
  cRower[1]=0x56;
  cRower[2]=0x00;
  cRower[3]=0x00;
  cRower[4]=0x00;
  cRower[5]=0x00;
  cRower[6]=0x00;
  cRower[7]=0x00;
  pFmfCharacteristic->setValue((uint8_t* )cRower, 8);

//--------------------------------------------------------------------------------------------------------

  // Start advertising
  Serial.println(F("Adding Fitness Machine Service UUID to the advertising payload "));
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData oAdvertisementData;
  oAdvertisementData.addData(std::string{ 0x02, 0x01, 0x06, 0x05, 0x02, 0x26, 0x18, 0x0a, 0x18});
  //oAdvertisementData.addData(std::string{ 0x01, 0xFC});
  pAdvertising->setScanResponse(true);
  pAdvertising->setAdvertisementData(oAdvertisementData);

  //pAdvertising->addServiceUUID(BLEUUID((uint16_t)GATT_CSC_UUID));
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)FitnessMachineService));
  //pAdvertising->setAppearance(833);
  //pAdvertising->setAppearance(1154); //BLE_APPEARANCE_CYCLING_SPEED_SENSOR
  BLEDevice::startAdvertising();

  bleInitFlag=true;
  Serial.println();
}

void initBleData(){
  rdKpi.bpm=0;
  rdKpi.strokeCount=0;
  //rdKpi.tmpstrokeRate=0;
  rdKpi.strokeRate=0;
  rdKpi.averageStokeRate=0;
  rdKpi.totalDistance=0;
  rdKpi.tmpinstantaneousPace=0;
  rdKpi.instantaneousPace=0;
  rdKpi.averagePace=0;
  rdKpi.instantaneousPower=0;
  rdKpi.averagePower=0;
  rdKpi.resistanceLevel=0;
  rdKpi.totalEnergy=0;
  rdKpi.energyPerHour=0;
  rdKpi.energyPerMinute=0;
  rdKpi.heartRate=0;
  rdKpi.metabolicEquivalent=0;
  rdKpi.elapsedTime=0;
  rdKpi.elapsedTimeSec=0;
  rdKpi.elapsedTimeMin=0;
  rdKpi.elapsedTimeHour=0;
  rdKpi.remainingTime=0;
}

void setCxLightRowerData(){
#if DEEPTRACE
  Serial.printf("sendBleLightData() start");
#endif
  // This function is a subset of field to be sent in one piece
  // An alternative to the sendBleData()
  uint16_t  rowerDataFlags=0b0000001111110; //0x7E
  
  // 0000000000001 - 1   - 0x001 + More Data 0 <!> WARNINNG <!> This Bit is working the opposite way, 0 means field is present, 1 means not present
  // 0000000000010 - 2   - 0x002 + Average Stroke present
  // 0000000000100 - 4   - 0x004 + Total Distance Present
  // 0000000001000 - 8   - 0x008 + Instantaneous Pace present
  // 0000000010000 - 16  - 0x010 + Average Pace Present
  // 0000000100000 - 32  - 0x020 + Instantaneous Power present
  // 0000001000000 - 64  - 0x040 + Average Power present
  // 0000010000000 - 128 - 0x080 - Resistance Level present
  // 0000100000000 - 256 - 0x080 + Expended Energy present
  // 0001000000000 - 512 - 0x080 - Heart Rate present
  // 0010000000000 - 1024- 0x080 - Metabolic Equivalent present
  // 0100000000000 - 2048- 0x080 - Elapsed Time present
  // 1000000000000 - 4096- 0x080 - Remaining Time present

  //  C1  Stroke Rate             uint8     Position    2  + (After the Flag 2bytes)
  //  C1  Stroke Count            uint16    Position    3  +
  //  C2  Average Stroke Rate     uint8     Position    5  +
  //  C3  Total Distance          uint24    Position    6  +
  //  C4  Instantaneous Pace      uint16    Position    9  +
  //  C5  Average Pace            uint16    Position    11 +
  //  C6  Instantaneous Power     sint16    Position    13 +
  //  C7  Average Power           sint16    Position    15 +
  //  C8  Resistance Level        sint16    Position    17 -
  //  C9  Total Energy            uint16    Position    19 +
  //  C9  Energy Per Hour         uint16    Position    21 +
  //  C9  Energy Per Minute       uint8     Position    23 +
  //  C10 Heart Rate              uint8     Position    24 -
  //  C11 Metabolic Equivalent    uint8     Position    25 -
  //  C12 Elapsed Time            uint16    Position    26 -
  //  C13 Remaining Time          uint16    Position    28 -

  char cRower[17];

  cRower[0]=rowerDataFlags & 0x000000FF;
  cRower[1]=(rowerDataFlags & 0x0000FF00) >> 8;
  
  //rdKpi.strokeRate=rdKpi.tmpstrokeRate*2;
  cRower[2] = rdKpi.strokeRate & 0x000000FF;
  
  cRower[3] = rdKpi.strokeCount & 0x000000FF;
  cRower[4] = (rdKpi.strokeCount & 0x0000FF00) >> 8;
  
  cRower[5] = rdKpi.averageStokeRate & 0x000000FF;
  
  cRower[6] = rdKpi.totalDistance &  0x000000FF;
  cRower[7] = (rdKpi.totalDistance & 0x0000FF00) >> 8;
  cRower[8] = (rdKpi.totalDistance & 0x00FF0000) >> 16;
  
  if (rdKpi.tmpinstantaneousPace>0) // Avoid Divide by Zero
    rdKpi.instantaneousPace=(100000/rdKpi.tmpinstantaneousPace)/2;
  cRower[9] = rdKpi.instantaneousPace & 0x000000FF;
  cRower[10] = (rdKpi.instantaneousPace & 0x0000FF00) >> 8;
  
  cRower[11] = rdKpi.averagePace & 0x000000FF;
  cRower[12] = (rdKpi.averagePace & 0x0000FF00) >> 8;

  cRower[13] = rdKpi.instantaneousPower & 0x000000FF;
  cRower[14] = (rdKpi.instantaneousPower & 0x0000FF00) >> 8;

  cRower[15] = rdKpi.averagePower & 0x000000FF;
  cRower[16] = (rdKpi.averagePower & 0x0000FF00) >> 8;

  //gatt.setChar(fitnessMachineRowerDataId, cRower, 17);
  pDtCharacteristic->setValue((uint8_t* )cRower, 17);
  pDtCharacteristic->notify();


#if DEEPTRACE
  Serial.printf("sendBleLightData() end");
#endif
}

void setCxRowerData(){
  // Due the size limitation of the message in the BLE Stack of the NRF
  // the message will be split in 2 parts with the according Bitfield (read the spec :) )
  // rowerDataFlagsP1=0b0000011111110 = 0xFE
  // rowerDataFlagsP2=0b1111100000001 = 0x1F01
 
  char cRower[19]; // P1 is the biggest part whereas P2 is 13
  
  // Send the P1 part of the Message
  cRower[0]=rowerDataFlagsP1 & 0x000000FF;
  cRower[1]=(rowerDataFlagsP1 & 0x0000FF00) >> 8;
  
  //rdKpi.strokeRate=rdKpi.tmpstrokeRate*2;
  cRower[2] = rdKpi.strokeRate & 0x000000FF;
  
  cRower[3] = rdKpi.strokeCount & 0x000000FF;
  cRower[4] = (rdKpi.strokeCount & 0x0000FF00) >> 8;
  
  cRower[5] = rdKpi.averageStokeRate & 0x000000FF;
  
  cRower[6] = rdKpi.totalDistance &  0x000000FF;
  cRower[7] = (rdKpi.totalDistance & 0x0000FF00) >> 8;
  cRower[8] = (rdKpi.totalDistance & 0x00FF0000) >> 16;
  
  cRower[9] = rdKpi.instantaneousPace & 0x000000FF;
  cRower[10] = (rdKpi.instantaneousPace & 0x0000FF00) >> 8;
  
  cRower[11] = rdKpi.averagePace & 0x000000FF;
  cRower[12] = (rdKpi.averagePace & 0x0000FF00) >> 8;

  cRower[13] = rdKpi.instantaneousPower & 0x000000FF;
  cRower[14] = (rdKpi.instantaneousPower & 0x0000FF00) >> 8;

  cRower[15] = rdKpi.averagePower & 0x000000FF;
  cRower[16] = (rdKpi.averagePower & 0x0000FF00) >> 8;

  cRower[17] = rdKpi.resistanceLevel & 0x000000FF;
  cRower[18] = (rdKpi.resistanceLevel & 0x0000FF00) >> 8;

  pDtCharacteristic->setValue((uint8_t* )cRower, 19);
  pDtCharacteristic->notify();

  // Send the P2 part of the Message
  cRower[0]=rowerDataFlagsP2 & 0x000000FF;
  cRower[1]=(rowerDataFlagsP2 & 0x0000FF00) >> 8;
         
  cRower[2] = rdKpi.totalEnergy & 0x000000FF;
  cRower[3] = (rdKpi.totalEnergy & 0x0000FF00) >> 8;

  cRower[4] = rdKpi.energyPerHour & 0x000000FF;
  cRower[5] = (rdKpi.energyPerHour & 0x0000FF00) >> 8;

  cRower[6] = rdKpi.energyPerMinute & 0x000000FF;

  cRower[7] = rdKpi.bpm & 0x000000FF;

  cRower[8] = rdKpi.metabolicEquivalent& 0x000000FF;

  //rdKpi.elapsedTime=rdKpi.elapsedTimeSec+rdKpi.elapsedTimeMin*60+rdKpi.elapsedTimeHour*3600;
  cRower[9] =   rdKpi.elapsedTime & 0x000000FF;
  cRower[10] = (rdKpi.elapsedTime & 0x0000FF00) >> 8;

  cRower[11] = rdKpi.remainingTime & 0x000000FF;
  cRower[12] = (rdKpi.remainingTime & 0x0000FF00) >> 8;
  
  pDtCharacteristic->setValue((uint8_t* )cRower, 13);
  pDtCharacteristic->notify();

}

#if USE_BATTERY
void setCxBattery(){
  #if DEEPTRACE
    Serial.printf("sendBleBattery() start");
  #endif
  char hexBat[2];
  int batteryLevelPercent = round(battery_RA.getAverage() * 100 / 127);
  /*
  float measuredVbat = analogRead(VBATPIN);
  measuredVbat *= 2;    // we divided by 2, so multiply back
  measuredVbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredVbat /= 1024; // convert to voltage

  batteryLevelPercent=((measuredVbat-2.9)/(5.15-2.90))*100;

  Serial.print("Mesured Bat:");
  Serial.println(batteryLevelPercent);
  */

  hexBat[0]=batteryLevelPercent & 0x000000FF;
  hexBat[1]='\0'; // just in case 

  pBatCharacteristic->setValue((uint8_t* )hexBat, 1);
  pBatCharacteristic->notify();

  #if DEEPTRACE
    Serial.printf("sendBleBattery() end");
  #endif
}
#endif
#endif

//reset function (set to select button)
void reset() {
  unsigned long now = millis();
  clicks = 0;
  clicks_old = 0;
  meters = 0;
  meters_old = 0;
  click_time = 0;
  old_split = 0;
  split_time = 0;
  Ms = 0;
  timer1 = 0;
  timer2 = 0;
  start = now;
  old_split = now;
  strokes = 0;
  old_strokes = 0;
  stmra = 0;
  stm = 0;
  rpm = 0;
  stm_RA.clear();
  mps_RA.clear();
  trend = 0;
  resetTimer = 0;
  //lcd.clear();
  
#if USE_MQTT
  lastMsg = now;
#endif
  
//#if USE_MQTT || USE_HTTP
  snprintf(msg, MSG_BUFFER_SIZE, "{\"strokeRate\":0,\"strokeCount\":0,\"averageStokeRate\":0,\"totalDistance\":0,\"instantaneousPace\":0,\"averagePace\":0,\"instantaneousPower\":0,\"averagePower\":0,\"elapsedTime\":0}");
  xQueueOverwrite(xQueue, &msg);
//#endif
}

//calculate and return meters per second
float calcmetersmin() {
  meters = clicks / Ratio;
  stm = (strokes*60000)/(millis()-start);
  mps_RA.addValue((clicks - clicks_old) / Ratio);
  //float x = (clicks - clicks_old) / Ratio;
  float x = mps_RA.getAverage(); // use of floating average emulates the momentum of a boat
  clicks_old = clicks;
  return x; 
}

//calculate strokes/minute as rolling average
void calcstmra() {
  stm_RA.addValue(strokes-old_strokes);
  old_strokes = strokes;
  stmra = stm_RA.getAverage()*60;
  if (stmra > stm){
    trend = 2;
  }
  if (stmra < stm){
    trend = 1;
  }
  if (stmra == stm){
    trend = 0;
  }
}

//calculate 500m split time (in seconds)
void split() {
  start_split = millis();
  split_time = (start_split - old_split) / 1000;
  old_split = millis();
}

int read_LCD_buttons()
{
  bool key_in;
  key_in = digitalRead(BUTTONSPIN);
  vTaskDelay(20); //switch debounce delay. Increase this delay if incorrect switch selections are returned.
  if ((!digitalRead(BUTTONSPIN)) && (!key_in)) 
    return btnLEFT;
  else
    return btnNONE;

  /*
  int adc_key_in = 0;
  adc_key_in = analogRead(BUTTONSPIN);      // read the value from the sensor
  vTaskDelay(20); //switch debounce delay. Increase this delay if incorrect switch selections are returned.
  int k = (analogRead(BUTTONSPIN) - adc_key_in); //gives the button a slight range to allow for a little contact resistance noise
  if (5 < abs(k)) return btnNONE;  // double checks the keypress. If the two readings are not equal +/-k value after debounce delay, it tries again.
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
  */
}

#if USE_SSD1306 && USE_BATTERY
void displayTimeBattery()
{
  float h,m,s;
  unsigned long over;
  h=int((millis()-start)/3600000);
  over=long(millis()-start)%3600000;
  m=int(over/60000);
  over=over%60000;
  s=int(over/1000);
  String str = "";
  if (int(h) > 0){
    str = str + String((int)round(h)) + ":";
  }
  if (m < 10){
    str = str + "0";
  }
  str = str + String((int)round(m));

  if (int(h) == 0) {
    str = str + ':';
    if (s < 10){
       str = str + "0";
    }
    str = str + String((int)round(s));
  }

  display.setFont(Roboto_Slab_Bold_38);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 24, str);

  display.fillRect(0, 0, 2, 5);
  display.fillRect(31, 0, 2, 5);
  display.fillRect(63, 0, 2, 5);
  display.fillRect(95, 0, 2, 5);
  display.fillRect(0, 0, 128, 1);
  display.fillRect(128 - battery_RA.getAverage(), 0, 128, 4);

}
#endif

void rowing() {
  while (read_LCD_buttons() != btnLEFT) {
    unsigned long now = millis();

    // prints the variable parameter as selected by keypad
    // variableParameter();

    /*calculate split times every 500m*/
    if ((meters % 500) == 0 && meters > 0 ) {
      timer2 = now;
      if ((now - timer2) >= 500) {
        split();
      }
    }
    
    if ((now - timer1) >= 500) {
      timer1 = millis();
      /* calculate meters/min*/
      Ms = calcmetersmin();
      //Serial.println(Ms);
      /*calculate moving average of strokes/min*/
      old_spm = (int)spm;
      calcstmra();
      spm = stm_RA.getAverage()*60;

#if USE_BLE
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        //vTaskDelay(5000);    
      }
#endif

      long sec = (now - start) / 1000;
      rdKpi.strokeRate = (int)round(spm + old_spm);
      rdKpi.strokeCount = strokes;
      rdKpi.averageStokeRate = sec > 0 ? (int)(strokes * 60 * 2 / sec) : 0;
      rdKpi.totalDistance = meters;
      rdKpi.instantaneousPace = Ms == 0 ? 0 : (int)round(500 / Ms); // pace for 500m
      float avrMs = sec > 0 ?  meters / sec : 0;
      rdKpi.averagePace = avrMs == 0 ? 0 : (int)round(500 / avrMs);
      rdKpi.instantaneousPower = (int)round(2.8 * Ms * Ms * Ms); //https://www.concept2.com/indoor-rowers/training/calculators/watts-calculator
      rdKpi.averagePower = (int)round(2.8 * avrMs * avrMs * avrMs);
      rdKpi.elapsedTime = sec;

//#if USE_MQTT || USE_HTTP
      snprintf (msg, MSG_BUFFER_SIZE, "{\"strokeRate\":%d,\"strokeCount\":%d,\"averageStokeRate\":%d,\"totalDistance\":%d,\"instantaneousPace\":%d,\"averagePace\":%d,\"instantaneousPower\":%d,\"averagePower\":%d,\"elapsedTime\":%d}",
                                      rdKpi.strokeRate, rdKpi.strokeCount, rdKpi.averageStokeRate, rdKpi.totalDistance, rdKpi.instantaneousPace, rdKpi.averagePace, rdKpi.instantaneousPower, rdKpi.averagePower, rdKpi.elapsedTime);
      xQueueOverwrite(xQueue, &msg);
      Serial.println(msg);
//#endif

#if USE_BLE
      if (deviceConnected) {        //** Send a value to protopie. The value is in txValue **//
        setCxRowerData();
        //setCxLightRowerData();
        //vTaskDelay(500); // bluetooth stack will go into congestion, if too many packets are sent
        Serial.println("Send data " + String(rdKpi.strokeCount));

        vTaskDelay(10);
#if USE_BATTERY
        setCxBattery();
#endif

        //Why coxswain need this notify for start???
        char cRower[3];
        cRower[0]=0x01;
        cRower[1]=0x00;
        cRower[2]=0x00;
        pDtCharacteristic->setValue((uint8_t* )cRower, 3);
        pDtCharacteristic->notify();
      }
 
      // disconnecting
      if (!deviceConnected && oldDeviceConnected) {
        //vTaskDelay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
        resetTimer = 21;
      }
#endif

      if (resetTimer > 0) {
        if (resetTimer == 1) {
          resetTimer = 0;
          clicks = 0;
          break;
        }
        resetTimer = resetTimer - 1;
      }

#if USE_BATTERY
      adcAttachPin(BATPIN);
      //battery 3.2 - 4.2V
      //int batt = map(analogRead(BATPIN), MinADC, MaxADC, 0, 127);
      if (batt > 127) batt = 127;
      if (batt < 0) batt = 0;
      battery_RA.addValue(batt);
      //Serial.println(String(analogRead(BATPIN)) + " batt: "+ String(batt)+ " battery: "+ String(batt));
#endif
    }

#if USE_SSD1306
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(5, 23, "meters");
    //display.drawString(24, 53, "time");
    display.drawString(100, 23, "m/s");
    display.drawString(102, 53, "st/min");

    // set time display
#if USE_BATTERY
    displayTimeBattery();
#endif

    display.setFont(Roboto_Slab_Bold_27);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 26, String(round((spm + old_spm)/2), 0));
    display.drawString(128, -3, String(Ms, 2)); // "m/s"
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, -3, String(meters, DEC)); //"m"
    
    display.display();
#endif

    /* if select button is pressed reset */
    switch (read_LCD_buttons())
    {
      case btnSELECT:
        {
          reset();
          break;
        }
      case btnUP:
        {
          if (stage < 6){
            stage ++;
            vTaskDelay(500);
            //lcd.clear();
          }
          break;
        }
      case btnDOWN:
      {
        if (stage > 0){
          stage --;
          vTaskDelay(500);
          //lcd.clear();
        }
        break;
      }
    }

/*-------SIMULATE ROWING-------------*/
//      if (random(0, 100) < 30) {
//         rowerinterrupt();
//         }
/*-------SIMULATE ROWING-------------*/
    
    vTaskDelay(10);
  }

  //Serial.println("*****EXIT*********");

}

void row_start() {
#if USE_SSD1306
  display.clear();
  display.setColor(WHITE);
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 10, "Waiting");
  display.drawString(64, 34, "for start");
  display.display();
  vTaskDelay(100);
#endif
  
  while (clicks == 0) {
    vTaskDelay(10);
  };

  reset();
  rowing();
}

void variableParameter(){
    //lcd.setCursor(9, 0);
    switch (stage) {
    case 1:
      //lcd.print("Rot");
      data_output = clicks;
      break;
    case 2:
      //lcd.print("Rpm");
      data_output = rpm;
    break;
    case 3:
        //lcd.print("Trend ");
        if (trend == 0){
          //lcd.print("c");
        }
        if (trend == 1){
          //lcd.print("-");
        }
        if (trend == 2){
          //lcd.print("+");
        }
      data_output = 1;
    break;
    case 4:
      //lcd.print("St ");
      data_output = strokes;
    break;
    case 5:
      //lcd.print("StA");
      data_output = stm;
    break;
    case 6:
      //lcd.print("StM");
      data_output = stmra;
    break;
    default:
       //lcd.print("Sp5");
       data_output = split_time;
    break;
    }
    if(data_output >= 10) {
      if(data_output >= 100) {
      //lcd.print(" ");
      } else {
        //lcd.print("  ");
      }
    } else {
    //lcd.print("   ");
    }
    //lcd.print(data_output);
}

/*------------------INTERRUPT-----------------------------------*/
void IRAM_ATTR calcrpm() {
  click_time = millis();
  rpm = 60000 / (click_time - last_click_time);
  last_click_time = click_time;
  accel = rpm - old_rpm;
  if ((accel > 20) && (puffer == 0)){ // > 20 to eliminate micro "acceleration" due to splashing water (3 for one magnet)
    strokes ++;
    puffer = 20; // prevents counting two strokes due to still accelerating rotor
  }
  if (puffer > 0){
    puffer --;
  }
  old_rpm = rpm;
}

void IRAM_ATTR rowerinterrupt() {
  /*  if (trigger == 0) {
      reset();
      trigger = 1;
    }*/
  if (rotation == 0){ // prevents double-count due to switch activating twice on the magnet passing once
    clicks++;
    rotation = 1;
    calcrpm();
  } else {
    rotation = 0;
  }
}
/*------------------INTERRUPT END-----------------------------------*/


/*-----debounce------*/
void IRAM_ATTR rowerdebounceinterrupt() {
  noInterrupts();
  if ((long)(micros() - last_micros) >= debouncing_time * 1000) {
    rowerinterrupt();
    last_micros = micros();
  }
  interrupts();
}

#if USE_MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
#endif

void mainCode(void * parameters) {
#if USE_SSD1306
  // Start the OLED Display
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();                 // this is to flip the screen 180 degrees

  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 4, "ArduRower");
  display.display();
#endif

  Serial.println("/************************************");
  Serial.println(" * DIY_Rower BLE Module ");
  Serial.print  (" * Version ");
  Serial.println(_VERSION);
  Serial.println(" ***********************************/");
    
#if USE_BLE
  initBLE();
  initBleData();
#endif

  currentTime=millis();
  previousTime=millis();

  stm_RA.clear(); // explicitly start clean
  mps_RA.clear(); // explicitly start clean

  pinMode(ROWERINPUT, INPUT_PULLUP);
  pinMode(BUTTONSPIN, INPUT);

#if USE_BATTERY
  battery_RA.clear();

  adcAttachPin(BATPIN);
  int battery = map(analogRead(BATPIN), MinADC, MaxADC, 0, 127);
  if (battery > 127) battery = 127;
  if (battery < 0) battery = 0;
  for (int i = 1; i < battery_RA.getSize(); ++i) {
     battery_RA.addValue(battery);
  }
#endif

  //digitalWrite(ROWERINPUT, HIGH);
  vTaskDelay(500);
  attachInterrupt(ROWERINPUT, rowerdebounceinterrupt, CHANGE);
  timer1 = millis();
  timer2 = millis();
  reset();
  
  for(;;) {
    row_start();

    vTaskDelay(10);
  }
}

#if USE_MQTT || USE_HTTP
void wifiCode(void * parameters) {
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  WiFi.onEvent([](WiFiEvent_t event) {
    switch (event) {
      case SYSTEM_EVENT_STA_CONNECTED:
        Serial.print("Connected to ");
        Serial.println(ssid);
        break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.print("Trying to connect to ");
        Serial.println(ssid);
        WiFi.begin(ssid, password);
        break;
      case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        break;
      default:
        break;
    }
  });

  WiFi.begin(ssid, password);
  
  for(;;) {
    vTaskDelay(1000);
  }
}
#endif

#if USE_MQTT
void mqttCode(void * parameters) {
  mqttClient = PubSubClient(wifiClient);
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
  lastReconnectAttempt = 0;
  
  for(;;) {
    
    if (WiFi.status() != WL_CONNECTED)
      continue;

    long now = millis();

    if (!mqttClient.connected()) {
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;

        if (mqttClient.connect("WaterRower")) {
          Serial.print("Connected to ");
          Serial.print(mqttServer);
          Serial.print(":");
          Serial.println(mqttPort);

          mqttClient.subscribe(mqttInTopic);

          lastReconnectAttempt = 0;
        }
      }
    } else {
      mqttClient.loop();

      if (now - lastMsg > 500) {
        char msgCopy[MSG_BUFFER_SIZE];
        if(xQueuePeek(xQueue, &msgCopy, 10)) {
          lastMsg = now;
          Serial.print("Publish message: ");
          char msgCopy[MSG_BUFFER_SIZE];
          memcpy(msgCopy, msg, MSG_BUFFER_SIZE);
          Serial.println(msgCopy);
          mqttClient.publish(mqttOutTopic, msgCopy);
        }
      }
    }

    vTaskDelay(10);
  }
}
#endif

#if USE_HTTP
void httpCode(void * parameters) {
  vTaskDelay(500);

  webServer = new AsyncWebServer(80);
  webSocket = new AsyncWebSocket("/ws");
 
  webServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String s = MAIN_page;
    if (WiFi.status() == WL_CONNECTED)
      request->send(200, "text/html", s);
  });
  webServer->on("/getData", HTTP_GET, [](AsyncWebServerRequest *request) {
    char msgCopy[MSG_BUFFER_SIZE];
    if(xQueuePeek(xQueue, &msgCopy, 10)) {
      if (WiFi.status() == WL_CONNECTED)
        request->send(200, "application/json", msgCopy);
    }
  });

  webSocket->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
  });
  
  webServer->addHandler(webSocket);
  
  vTaskDelay(500);

  webServer->begin();

  for(;;) {
    webSocket->cleanupClients();

    char msgCopy[MSG_BUFFER_SIZE];
    if(xQueuePeek(xQueue, &msgCopy, 10))
      webSocket->textAll(msgCopy);

    vTaskDelay(500);
  }
}
#endif

void setup() {
  randomSeed(micros());

  Serial.begin(115200);

  xTaskCreatePinnedToCore(
                    mainCode,   /* Task function. */
                    "Main",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

#if USE_MQTT || USE_HTTP
  xTaskCreatePinnedToCore(
                    wifiCode,   /* Task function. */
                    "Wifi",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
#endif

#if USE_MQTT
  xTaskCreatePinnedToCore(
                    mqttCode,   /* Task function. */
                    "MQTT",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
#endif

#if USE_HTTP
 xTaskCreatePinnedToCore(
                    httpCode,   /* Task function. */
                    "HTTP",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
#endif
                  
vTaskDelete(NULL);
}

void loop() {
}