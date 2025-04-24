#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
#include <ITG3200.h>

// Initialize the sensors
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
HMC5883L mag;
ITG3200 gyro;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize the ADXL345
  if (!accel.begin()) {
    Serial.println("No ADXL345 Accelerometer detected, check your wiring!");
    while (1);
  }
  Serial.println("ADXL345 Accelerometer detected!");

  // Initialize the HMC5883L
  mag.initialize();
  if (!mag.testConnection()) {
    Serial.println("No HMC5883L Magnetometer detected, check your wiring!");
    while (1);
  }
  Serial.println("HMC5883L Magnetometer detected!");

  // Initialize the ITG3200
  gyro.initialize();
  if (!gyro.testConnection()) {
    Serial.println("No ITG3200 Gyroscope detected, check your wiring!");
    while (1);
  }
  Serial.println("ITG3200 Gyroscope detected!");
}

void loop() {
  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("\nAccelerometer X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" Z: ");
  Serial.println(event.acceleration.z);

  // Read magnetometer data
  int16_t mx, my, mz;
  mag.getHeading(&mx, &my, &mz);
  Serial.print("Magnetometer X: ");
  Serial.print(mx);
  Serial.print(" Y: ");
  Serial.print(my);
  Serial.print(" Z: ");
  Serial.println(mz);

  // Calculate and print heading
  float heading = atan2((float)my, (float)mx) * 180 / PI;
  if (heading < 0) {
    heading += 360; // Correct for negative angles
  }
  Serial.print("Heading: ");
  Serial.println(heading);

  // Determine and print cardinal direction
  const char* direction;
  if (heading >= 45 && heading < 135) {
    direction = "East";
  } else if (heading >= 135 && heading < 225) {
    direction = "South";
  } else if (heading >= 225 && heading < 315) {
    direction = "West";
  } else {
    direction = "North";
  }
  Serial.print("Robot is facing: ");
  Serial.println(direction);



  // Read gyroscope data
  int16_t gx, gy, gz;
  gyro.getRotation(&gx, &gy, &gz);
  Serial.print("Gyroscope X: ");
  Serial.print(gx);
  Serial.print(" Y: ");
  Serial.print(gy);
  Serial.print(" Z: ");
  Serial.println(gz);

  delay(1000); // Wait for 1 second before reading again
}

/*

The GY-85 sensor module is an IMU sensor board that combines three different types of sensors:

1) ITG3200 Gyroscope: Angular velocity along the x, y, and z axes.

2) ADXL345 Accelerometer: Linear acceleration in three dimensions.

3) HMC5883L Magnetometer: Digital compass sensor that measures magnetic fields. It provides heading information based on the Earth's magnetic poles, which is invaluable for navigation systems to determine directional orientation.

*/