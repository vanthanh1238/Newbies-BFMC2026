#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
  if(!bno.begin()) while(1);

  // 1. Check if we have a saved profile in EEPROM (at address 0)
  int eeAddress = 0;
  adafruit_bno055_offsets_t savedOffsets;
  EEPROM.get(eeAddress, savedOffsets);

  // 2. Load the profile if it exists (you can add a 'magic number' check here)
  Serial.println("Loading saved calibration...");
  bno.setSensorOffsets(savedOffsets);
  
  bno.setExtCrystalUse(true);
}

void loop(void) {
  // 3. Monitor calibration status
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("Sys:"); Serial.print(system);
  Serial.print(" G:"); Serial.print(gyro);
  Serial.print(" A:"); Serial.print(accel);
  Serial.print(" M:"); Serial.println(mag);

  // 4. Save the profile once fully calibrated (all 3s)
  if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
    saveToEEPROM();
    Serial.println("PROFILE SAVED! You can now restart.");
    while(1); // Stop after saving
  }
  delay(500);
}

void saveToEEPROM() {
  adafruit_bno055_offsets_t currentOffsets;
  // This function reads all 22 bytes from the BNO055 registers
  bno.getSensorOffsets(currentOffsets);
  
  EEPROM.put(0, currentOffsets);
}