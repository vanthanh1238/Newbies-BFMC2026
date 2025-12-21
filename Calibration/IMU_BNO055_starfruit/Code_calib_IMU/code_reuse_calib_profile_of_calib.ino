#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
  if(!bno.begin()) {
    while(1); // Halt if sensor not found
  }

  // --- EEPROM LOADING LOGIC ---
  int eeAddress = 0;
  long bnoID;
  adafruit_bno055_offsets_t calibrationData;

  EEPROM.get(eeAddress, bnoID);
  sensor_t sensor;
  bno.getSensor(&sensor);

  // If the ID matches, load the 22-byte profile
  if (bnoID == sensor.version) {
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData); 
  }

  bno.setExtCrystalUse(true);
}

void loop(void) {
  // 1. Get Orientation (Quaternion)
  imu::Quaternion quat = bno.getQuat();
  // 2. Get Angular Velocity (rad/s)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // 3. Get Linear Acceleration (m/s^2)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Send CSV: qw,qx,qy,qz,gx,gy,gz,ax,ay,az
  Serial.print(quat.w(), 4); Serial.print(",");
  Serial.print(quat.x(), 4); Serial.print(",");
  Serial.print(quat.y(), 4); Serial.print(",");
  Serial.print(quat.z(), 4); Serial.print(",");
  Serial.print(gyro.x(), 4); Serial.print(",");
  Serial.print(gyro.y(), 4); Serial.print(",");
  Serial.print(gyro.z(), 4); Serial.print(",");
  Serial.print(accel.x(), 4); Serial.print(",");
  Serial.print(accel.y(), 4); Serial.print(",");
  Serial.println(accel.z(), 4);

  delay(20); // 50Hz output rate
}
