#include "src/BattSense/INA219.h"
#include "src/BattSense/SystemConfig.h"
#include <TinyGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DEBUG
#define GPS_Switch PA0

TwoWire i2cPort(ACSIP_SDA, ACSIP_SCL);
INA219 SolarSense(0X41);
INA219 BatterySense(0X40);
PowerMonitor Battery;
PowerMonitor Solar;

byte ModbusCommand[2][11] = {
    {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xc5, 0xcb}, // Get temperature and DO
    {0x01, 0x10, 0x20, 0x00, 0x00, 0x01, 0x02, 0x00, 0xFF, 0x07,
     0x95}}; // Set brush interval to 255 mins

//#define DEBUG
TinyGPS gps;
Adafruit_MPU6050 mpu;
HardwareSerial gpsSensor(PA3, PC2);     // gps
TwoWire _wire(PB7, PB6);                 // accelerometer
HardwareSerial interCom(PD2, PC12);

float totAccumulation;
uint16_t waitingTime;                                // window waiting time for on-motion event
uint16_t waterLevel, waterLevelOffset = 0;
uint16_t waterHighThreshold           = 70,          // Water level sensor settings
         waterCriticalHighThreshold   = 100, 
         waterLowThreshold            = 50,
         waterCriticalLowThreshold    = 20;
bool movementDetected                 = false;
uint16_t motionThreshold              = 5;           // Accelerometer Motion Sensitivity
uint16_t motionDuration               = 25;          // Accelerometer Motion Duration in millisecs
float battlevel;                                     // battery level
bool solarCharging;
float gpsLon, gpsLat;
uint16_t gpsTransmissionHours, gpsWaitTime;

const long checkupTime = 3600000;
const long deviceSleepTime = 300000;
unsigned long lastCheckupTime = 0;
const int maxArraySize = 10; 
int receivedData[maxArraySize];

void readGPS() ;
void deviceWakeInit();


void deviceWakeInit() {
  #ifdef DEBUG
    Serial.println();
    Serial.println(F("[SYSTEM] Initializing Device Configurations."));
  #endif // DEBUG

  pinMode(PA0, OUTPUT);
  digitalWrite(PA0, LOW);             // DIO (GPS loadswitch)
  delay(1000);

  #ifdef DEBUG
    Serial.println(F("[SYSTEM] Device Initialized."));
  #endif // DEBUG
}

void ChargeCurrent(byte current){
       for(int x = 0; x < current; x++){
         chargePulse();
       }
     };

 void chargePulse(){
 digitalWrite(CHARGER_CTRL, HIGH);
 delayMicroseconds(500);
 digitalWrite(CHARGER_CTRL, LOW);
 delayMicroseconds(500);
}

void readBatt() {
  Serial.println("[SYSTEM] Reading Battery Percentage");
  pinMode(CHARGER_CTRL, OUTPUT); 
  digitalWrite(CHARGER_CTRL, LOW);
  pinMode(CHARGER_STAT, INPUT);
  if (!SolarSense.begin(&i2cPort)) {
     Serial.println("[ERROR] Failed to find Solar Sensing chip");
     //while (1) { delay(10); }
  }
  if (!BatterySense.begin(&i2cPort)) {
     Serial.println("[ERROR] Failed to find Battery Sensing chip");
     //while (1) { delay(10); }
  }
  /* Set maximum charge current to 935mA */
  ChargeCurrent(_935MA);
  Solar.voltage = SolarSense.getBusVoltage_V();
  Solar.current = SolarSense.getShuntVoltage_mV() / RSHUNT;
  Battery.voltage = BatterySense.getBusVoltage_V();
  Battery.current = BatterySense.getShuntVoltage_mV() / RSHUNT;
  ChargeCurrent(_935MA);
  bool ChargingStatus = digitalRead(CHARGER_STAT);
  Serial.println("\n - - - - - - - - - - - ");
  Serial.println("Solar Voltage: " + String(Solar.voltage) + "V");
  Serial.println("Solar Current: " + String(Solar.current) + "mA");
  Serial.println("Battery Voltage: " + String(Battery.voltage) + "V");
  Serial.println("Battery Current: " + String(Battery.current) + "mA");
  Serial.println("Charging Status: " + String((ChargingStatus) ? "OFF" : "ON")); 
  battlevel = Battery.voltage;
  solarCharging = !ChargingStatus;
}

void readGPS() {
  Serial.println("[SYSTEM] Reading GPS Location");
  digitalWrite(GPS_Switch, HIGH);                     // Turn on GPS Load Switch
  delay(50);
    bool stopReading       = false,             // Set GPS checkpoints 
         tried             = false,
         newData           = false;
    uint16_t tryWindowTime = (millis() / 1000) + gpsWaitTime;
    tryWindowTime=(millis() / 1000)+0;   //30 seconds if debug mode
    Serial.println(F("Waking up GPS!"));
    gpsSensor.begin(9600);                      // Start GPS Serial Communication
    delay(3000);
    unsigned long chars;
    unsigned short sentences, failed;
    while (!stopReading) {
      if (millis() % 1000 == 0) {
        Serial.print("Waiting for GPS Lock: ");
        Serial.print(tryWindowTime - (millis() / 1000));
        Serial.println("s");
        delay(1);
      }
      if (gpsSensor.available()) {
        while (gpsSensor.available()) {
          char c = gpsSensor.read();                 // Read GPS serial data
          if (gps.encode(c)) {
            newData = true;
          }
        }
        if (newData) {                               // Successful GPS Location Lock
          unsigned long age;
          long flat, flon;
          gps.get_position(&flat, &flon);
          gpsLat = flat / 100;
          gpsLon = flon / 100;
          Serial.println(F("GPS Location: "));
          Serial.print("Latitude: ");
          Serial.println(gpsLat);
          Serial.print("Longitude: ");
          Serial.println(gpsLat);
          stopReading = true;
          break;
        }
      } 
      else {                                         // Failed GPS Locking
        if ((millis() / 1000) >= gpsWaitTime) {
            Serial.println(F("[ERROR] Can\'t read GPS data"));
          stopReading = true;
        }
        if (tried == true) {
          stopReading = true;
          break;
        } 
        else {
          if (millis() / 1000 >= tryWindowTime) {    // Exceeded GPS Tries-for-Locking window
            tried = true;
            stopReading = true;
              Serial.println(F("[ERROR] Can\'t lock GPS"));
            break;
          }
        }
      }
    }
  Serial.println(F("Sleeping GPS!"));
  gpsSensor.end();                                  // End GPS Serial Communication
  delay(100);
  digitalWrite(GPS_Switch, LOW);                          // Turn Off GPS
}

void deviceSleep(){
  Serial.println("[SYSTEM] Device Sleeping...");
  delay(deviceSleepTime);
  Serial.println("\n[SYSTEM] Device Wakeup!");
  delay(50);
  deviceWakeInit();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[MASTER] Device Setup ");
  interCom.begin(9600);
  delay(100);
  deviceWakeInit();
}

void parseDataArray() {
  int arrayIndex = 0;

  // Read until the end of the array or timeout
  while (arrayIndex < maxArraySize && interCom.available()) {
    receivedData[arrayIndex] = interCom.parseFloat();
    arrayIndex++;
  }
}

void processDataArray(int data[], int arraySize){
    for (int i = 0; i < arraySize; i++) {
    Serial.print("Data ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data[i]);
  }
}

void loop() {
  Serial.println("[SYSTEM] Inside Loop");
  unsigned long currentMillis = millis();

  if (currentMillis - lastCheckupTime >= checkupTime) {
    lastCheckupTime = currentMillis;
    Serial.println("[SYSTEM] Fetching Sensor Data");
    interCom.println("DataFetch");
    delay(50);
    if(interCom.available()>0){
      parseDataArray();
    }
    // readBatt();
    // Serial.println();
    // readGPS();
    // Serial.println();
  }
  int arraySize = sizeof(receivedData) / sizeof(receivedData[0]); 
  processDataArray(receivedData, arraySize);
  delay(100);
  deviceSleep();
}
