#include <Wire.h>

#include "INA219.h"
#include "SystemConfig.h"

TwoWire i2cPort(ACSIP_SDA, ACSIP_SCL);

INA219 SolarSense(0X41);
INA219 BatterySense(0X40);

PowerMonitor Battery;
PowerMonitor Solar;


void setup() {
 Serial.begin(9600);
 
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
 delay(1000);
 
}

void loop() {

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

   delay(2000);
   
}
