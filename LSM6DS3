

#include <Arduino_LSM6DS3.h>

const float threshold = 50.0;
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("LSM6DS3 initialized successfully!");
}

void loop() {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    Serial.print("Acceleration X: ");
    Serial.print(accelX);
    Serial.print(" | Y: ");
    Serial.print(accelY);
    Serial.print(" | Z: ");
    Serial.println(accelZ);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    Serial.print("Gyroscope X: ");
    Serial.print(gyroX);
    Serial.print(" | Y: ");
    Serial.print(gyroY);
    Serial.print(" | Z: ");
    Serial.println(gyroZ);

    // Check for significant changes in the gyroscope data
    if (gyroX < -10 - threshold || gyroX > 8 + threshold || 
        gyroY < -10 - threshold || gyroY > 8 + threshold || 
        gyroZ < -10 - threshold || gyroZ > 8 + threshold) {
      Serial.println("HALA NAHULOG!!!");
    }
  }

  delay(1000); 
}
// DON'T JUDGE US PO HAHAHAHAAHAHAHAHA HI KUYAAAAA!
