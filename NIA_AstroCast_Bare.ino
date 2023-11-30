#include "src/BattSense/INA219.h"
#include "src/BattSense/SystemConfig.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS.h>
#include <Wire.h>

#define DEBUG
#define DEBUG_WATER_LEVEL
#define DEBUG_BAT
#define DEBUG_ACCEL
#define DEBUG_GPS
#define SLEEP_INTERRUPT

#define WQ_MODBUS_EN PC4 // B5 // PA0
#define MODBUS_LS_5V PC6
#define MODBUS_LS_12V PB1
#define WDT_DONE PC0
/* Defining interrupt pin */
#define INTERRUPT_PIN PC5    
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
HardwareSerial waterLevelData(PA3, PA2); // water level
HardwareSerial gpsSensor(PD2, PC12);     // gps
HardwareSerial Modbus(PC11, PC10);   // rain gauge
TwoWire _wire(PB7, PB6);                 // accelerometer


float totAccumulation;
float currentAcc, RInt;
uint16_t waitingTime;                                // window waiting time for on-motion event
uint16_t waterLevel, waterLevelOffset = 0;
uint16_t waterHighThreshold           = 70,            // Water level sensor settings
         waterCriticalHighThreshold   = 100, 
         waterLowThreshold            = 50,
         waterCriticalLowThreshold    = 20;
bool movementDetected                 = false;
uint16_t motionThreshold              = 5;             // Accelerometer Motion Sensitivity
uint16_t motionDuration               = 25;             // Accelerometer Motion Duration in millisecs
float battlevel;        // battery level
bool solarCharging;
volatile bool onMotion = false;

float gpsLon, gpsLat;
uint16_t gpsTransmissionHours, gpsWaitTime;
uint16_t motionCooldownDelay;


/* @brief ACSIP lmic pin map */

void readGPS(bool alert);
void initAccel();
int readWaterLevel();
void deviceWakeInit();
void readRain();

int RAIN_GAUGE_BAUD_RATE = 9600; // default baudrate
int myIndex = 0;
float mydata[6];
float previousTotal = 0;
bool gotData = false;
bool flag1 = false, flag2 = false, flag3 = false;
bool decodeMode = false;
bool serialTrigger = false;
int baudIndex, baud;
String baudtest[7] = {"echo0", "echo1", "echo2", "echo3",
                      "echo4", "echo5", "echo6"};
int baudrates[7] = {1200, 2400, 4800, 9600, 19200, 38400, 57600};
String raingauge;
bool newbaud = false;
int command;
bool keyword = false;
int dataCounter = 0;

uint8_t ModbusResponse[20];
unsigned long int responseCapturedMillis = 0;
int responseByteIndex = 0;
bool responseCaptured = false;
byte responseLength = 0;
union bytesToFloat {
  byte arr[4];
  float data;
};

bytesToFloat Temperature;
bytesToFloat DOLevel;


void motionEvent(){
  #ifdef DEBUG
    Serial.println(F("[SYSTEM] Motion Interupt TRIGGERED"));
  #endif
  movementDetected = true;
  onMotion = true;                 // Detach motion interrupt to give way to sensor telemetry readings

  return;
  }
void deviceWakeInit() {
  #ifdef DEBUG
    Serial.println();
    Serial.println(F("[SYSTEM] Initializing Device Configurations."));
  #endif // DEBUG


  pinMode(PC1, OUTPUT);
  digitalWrite(PC1, LOW);             // DIO (GPS loadswitch)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Modbus.begin(RAIN_GAUGE_BAUD_RATE); // init with default
  delay(1000);
  readRain();
   delay(1000);
  initAccel();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), motionEvent, CHANGE);
   delay(50);
  delay(1000);

  pinMode(WQ_MODBUS_EN, OUTPUT);
  pinMode(MODBUS_LS_5V, OUTPUT);
  pinMode(MODBUS_LS_12V, OUTPUT);
  digitalWrite(MODBUS_LS_5V, HIGH);
  digitalWrite(MODBUS_LS_12V, HIGH);
  pinMode(WDT_DONE, OUTPUT);
  digitalWrite(WDT_DONE, LOW);
  delay(100);
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
  pinMode(CHARGER_CTRL, OUTPUT); 
  digitalWrite(CHARGER_CTRL, LOW);
  pinMode(CHARGER_STAT, INPUT);
  if (!SolarSense.begin(&i2cPort)) {
     Serial.println("Failed to find Solar Sensing chip");
     //while (1) { delay(10); }
  }
  if (!BatterySense.begin(&i2cPort)) {
     Serial.println("Failed to find Battery Sensing chip");
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
int readWaterLevel() {
  /* Begin ModBus Serial Communication */
  waterLevelData.begin(9600, SERIAL_8N1);
  delay(100);
  int counter = 0;
  int waterReadings[11];
  int waterSensorFailures = 0;

  /* Capture incoming modbus response */
  while (counter != 11) {
    digitalWrite(WQ_MODBUS_EN, HIGH);
    delay(100);
    for (int i = 0; i < 8; i++) {
      waterLevelData.write(ModbusCommand[0][i]);
    }

    delay(10);
    digitalWrite(WQ_MODBUS_EN, LOW);
    delay(100);

    /* Capture incoming modbus response */
    if (!waterLevelData.available()) {

      waterLevel = 0;
      waterSensorFailures++;
      if (waterSensorFailures >= 5) {
        #ifdef DEBUG_WATER_LEVEL
          Serial.println("[ERROR] Failed to initialize water level sensor");
        #endif // DEBUG_WATER_LEVEL
        waterLevelData.end();
        return -1;
      }
    }
    if (waterSensorFailures >= 5) {
      break;
      return -1;
    }
    while (waterLevelData.available()) {
      responseCapturedMillis = millis();
      uint8_t incoming = waterLevelData.read();
      ModbusResponse[responseByteIndex] = incoming;
      responseByteIndex++;
    }
    if (responseByteIndex >= 5) {
      waterLevel = (ModbusResponse[3] << 8) | ModbusResponse[4];
      waterLevel+=waterLevelOffset;
      waterReadings[counter] = waterLevel;
      counter++;
       #ifdef DEBUG_WATER_LEVEL
         Serial.print("Water Level: ");
         Serial.print(waterLevel);
         Serial.println(" mm");
       #endif //DEBUG_WATER_LEVEL
      responseLength = responseByteIndex;
      responseByteIndex = 0;
    }
  }

  /* Calculate Median Sensor Reading */
  int size = sizeof(waterReadings) / sizeof(waterReadings[0]);
  bubbleSort(waterReadings, size);             // Sort Water Level Readings
  waterLevel = getMedian(waterReadings, size); // Get the Median from the Readings
  Serial.print("Water Level: ");
  Serial.println(waterLevel);
  /* Check Water Level */
  if (waterLevel <= waterCriticalLowThreshold) {
      Serial.println("Water Level Critically Low");
  } else if (waterLevel <= waterLowThreshold) {
      Serial.println("Water Level Low");
  } else if (waterLevel <= waterHighThreshold) {
      Serial.println("Water Level Normal");
  } else if (waterLevel <= waterCriticalHighThreshold) {
      Serial.println("Water Level High");
  } else {
      Serial.println("Water Level Critically High");
  }
  
  delay(100);
  waterLevelData.end();
}

void bubbleSort(int arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

double getMedian(int arr[], int size) {
  if (size % 2 == 0) { // If the size is even, return the average of the two middle elements
    return (arr[size / 2 - 1] + arr[size / 2]) / 2.0;
  } else { // If the size is odd, return the middle element
    return arr[size / 2];
  }
}

void initAccel() {
  _wire.begin();      // Start I2C Communication
  delay(200);
  if (!mpu.begin()) { // Initialize MPU6050
    Serial.println("[ERROR] Failed to find MPU6050 chip");
  } else {
    #ifdef DEBUG
      Serial.println(F("[SYSTEM] Initializing MPU-6000 Motion Detection..."));
    #endif
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(motionThreshold);
    mpu.setMotionDetectionDuration(motionDuration);
    mpu.setInterruptPinLatch(false);  
    mpu.setInterruptPinPolarity(false);
    mpu.setMotionInterrupt(true);
    delay(100);
    #ifdef DEBUG
      Serial.println(F("[SYSTEM] MPU-6000 Motion Detection Initialized"));
    #endif
  }
}

void decodeSerial() {
  /*Serial reading variables*/
  String message;
  message = Modbus.readStringUntil(' ');
  if (message != "") {
    if (gotData) {
      mydata[myIndex] = message.toFloat();
      if (myIndex == 2) {
        currentAcc = mydata[myIndex];
        Serial.print("Current Acc: ");
        Serial.println(currentAcc);
      } else if (myIndex == 3) {
        totAccumulation = mydata[myIndex];
        Serial.print("Total Acc: ");
        Serial.println(totAccumulation);
      } else if (myIndex == 4) {
        RInt = mydata[myIndex];
        Serial.print("Rain Intensity: ");
        Serial.println(RInt);
      }
      
      gotData = false;
    }
    if (message == "EventAcc" && !flag2) {
      flag2 = true;
      gotData = true;
      myIndex = 2;
    }
    if (message == "TotalAcc" && !flag3) {
      flag3 = true;
      gotData = true;
      myIndex = 3;
    }
    if (message == "RInt") {
      gotData = true;
      myIndex = 4;
    }
    String messages[7] = {"N", "M", "W", "O", "U", "B", "D"};
    // N-Normal Power Up, M-MCLR, W-Watchdog Timer Reset,[O,U]-Stack
    // Overflow,B-Brownout,D-Other
    for (int i = 0; i <= 6; i++) {
      if (message == messages[i]) {
        mydata[5] = i + 1;
        keyword = true;
      }
    }
    if (message == "LensBad") { // not sufficient light
      mydata[5] = 8;
      keyword = true;

    }
    if (message == "EmSat") { // emitter is saturated
      mydata[5] = 9;
      keyword = true;
    }
  }

}

void readRain() {
  Modbus.begin(RAIN_GAUGE_BAUD_RATE); // init with default
  delay(1000);
  previousTotal = mydata[3];
  delay(500);
  Modbus.print("P\n");
  delay(500);
  Modbus.print("R\n");
  delay(500);
  if (Modbus.available() > 0) {
    while (Modbus.available() > 0) {
      decodeSerial();                // Decode data from Modbus Serial
    }
  } 
  else {}
  Serial.print("totAccumulation: ");
  Serial.println(totAccumulation);
  Serial.print("currentAcc: ");
  Serial.println(currentAcc);
  Serial.print("RInt: ");
  Serial.println(RInt);
}


void readGPS( ) {
  digitalWrite(PA13, HIGH);                     // Turn on GPS Load Switch
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
  digitalWrite(PA13, LOW);                          // Turn Off GPS
}


void setup() {

  Serial.begin(115200);
  deviceWakeInit();
}


void loop() {
  readBatt();
  Serial.println();
  readGPS();
  Serial.println();
  readRain();
  Serial.println();
  readWaterLevel();
  Serial.println();
  delay(1000);
}
