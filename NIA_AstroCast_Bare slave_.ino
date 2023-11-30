
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DEBUG
#define DEBUG_WATER_LEVEL
#define DEBUG_BAT
#define DEBUG_ACCEL

#define WQ_MODBUS_EN PC4 // B5 // PA0
#define MODBUS_LS_5V PC6
#define MODBUS_LS_12V PB1
#define WDT_DONE PC0
/* Defining interrupt pin */
#define INTERRUPT_PIN PC5  

/*Variables for Data Transmission*/
const int maxArraySize = 5;    
float sensorData[maxArraySize];
uint8_t movementCount = 0;
uint16_t debounceTime = 30000;
byte ModbusCommand[2][11] = {
    {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xc5, 0xcb}, // Get temperature and DO
    {0x01, 0x10, 0x20, 0x00, 0x00, 0x01, 0x02, 0x00, 0xFF, 0x07,
     0x95}}; // Set brush interval to 255 mins

//#define DEBUG
Adafruit_MPU6050 mpu;
HardwareSerial waterLevelData(PA3, PA2); // water level
HardwareSerial Modbus(PC11, PC10);   // rain gauge
HardwareSerial interCom(PD2, PC12);
TwoWire _wire(PB7, PB6);                 // accelerometer

int prevMillis = 0, prevMillis2;
float totAccumulation;
float currentAcc, RInt;
uint16_t waterLevel, waterLevelOffset = 0;

uint8_t waterCriticalHighReadings = 0,waterCriticalLowReadings = 0;
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


/* @brief ACSIP lmic pin map */

void initAccel();
int readWaterLevel();
void startupConfig();
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
  movementCount++;
  prevMillis = millis();
  detachInterrupt(INTERRUPT_PIN); 
  return;
  }
void startupConfig() {
  #ifdef DEBUG
    Serial.println();
    Serial.println(F("[SYSTEM] Initializing Device Configurations."));
  #endif // DEBUG
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
      waterCriticalLowReadings++;
  } 
  else if (waterLevel >= waterCriticalHighThreshold) {
      Serial.println("Water Level Criticaly High");
      waterCriticalHighReadings++;
  } else {
      Serial.println("Water Level normal");
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

void sendDataArray() {
  // Replace this function with your actual sensor reading code
  sensorData[0] = waterLevel; //Water Level
  sensorData[1] = waterCriticalHighReadings; //Water High Counter
  sensorData[2] = waterCriticalLowReadings; //Water Low Counter
  sensorData[3] = totAccumulation; //Rain Total Accumulation
  sensorData[4] = movementCount; //Motion Total Interrupts

  // Send the sensor data array as a string to the master
  for (int i = 0; i < maxArraySize; i++) {
    interCom.print(sensorData[i]);

    // Add a delimiter (comma) except for the last element
    if (i < maxArraySize - 1) {
      interCom.print(",");
    }
  }
  // End the transmission with a newline
  interCom.println();
}

void setup() {
  Serial.begin(115200);
  interCom.begin(9600); 
  startupConfig();
}

void loop() {
  if(movementDetected){
     if((millis()-prevMillis)>=debounceTime){ //default - 30 seconds passed 
       prevMillis = millis();
       movementDetected = false;
       attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), motionEvent, CHANGE);
       delay(1000);
    }
  }
  readRain();
  if((millis()-prevMillis2)>=30000){ //default 5 minutes passed 
     prevMillis2 = millis();
     readWaterLevel();               // read water level every 5 minutes
     delay(1000);
  }
  if (interCom.available() > 0) {
    String request = interCom.readStringUntil('\n');
    if (request == "DataFetch") {
      sendDataArray();
    }
  }
}