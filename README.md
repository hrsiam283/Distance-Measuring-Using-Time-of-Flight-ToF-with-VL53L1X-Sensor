# Distance-Measuring-Using-Time-of-Flight-ToF-with-VL53L1X-Sensor

## A library is needed for this sensor

Search VL53L1X in arduino IDE. You will find VL53L1X (Pololu Arduino library).
Install it, and try running continous.ino from the example code.

```php
/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

void loop()
{
  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}

```

## Working With Multiple sensor

An Arduino Uno or Nano only has one set of SDA and SCL pins, limiting the number of I2C devices you can connect directly. That's where the PCA9548A multiplexer comes in handy.

Here's how you can use the PCA9548A to check if your two ToF sensors (VL53L1X) are connected:

## 1. Hardware Connections:

Connect the SDA and SCL pins of your Arduino Uno/Nano to the corresponding pins of the PCA9548A (usually labeled SDA and SCL).
Connect each VL53L1X sensor's SDA pin to a separate SDA channel of the PCA9548A (e.g., SD0 and SD1). Similarly, connect the SCL pins of the sensors to the corresponding SCL channels (e.g., SC0 and SC1).
Refer to the PCA9548A datasheet for specific pin configurations. You might need to adjust the address pins (A0, A1, and A2) of the multiplexer to avoid conflicts with other I2C devices.

## 2. Checking Sensor Connection:

There are two main approaches to check if the sensors are connected:

I2C Scanner Library:

Use an I2C scanner library for your Arduino platform. These libraries typically scan the I2C bus for devices and report their addresses.
By scanning each channel of the PCA9548A (using its channel select functionality), you can identify the addresses of your connected VL53L1X sensors.

```cpp


//i2c device connected in bus checking code
// we are connecting a sensor in channel 6 of the mux
// if sensor is connected then we will find an address (0x29)
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor to open
  Serial.println("\nI2C Scanner");
  selectMuxChannel(6);

}
// void check()
// {
//     selectMuxChannel(7);
// }
void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.println("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.println("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done\n");
  }
  delay(5000); // Wait 5 seconds before scanning again
}


void selectMuxChannel(uint8_t channel) {
  // Ensure channel is within range (0 to 7)
  if (channel < 8) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel); // Set the corresponding bit for the channel
    Wire.endTransmission();
    delay(10); // Delay to allow the multiplexer to settle
  } else {
    Serial.println("Invalid channel number!");
    }
}
```

## Distance measuring with two sensor

```cpp
//mux channels are working
//i2c device connected in bus checking code
#include <Wire.h>
#include <VL53L1X.h>
VL53L1X sensor,sensor2; //sensor in mux->7,sensor2 in mux->6
void setup() {
  Wire.begin();
  pinMode(12 , OUTPUT);
  Serial.begin(9600);

  while (!Serial); // Wait for serial monitor to open
  // selectMuxChannel(7);
  Serial.println("\nI2C Scanner");
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
}

void loop() {
  delay(200); // Wait 5 seconds before scanning again
  selectMuxChannel(6);
  // sensor.init();
  int n = sensor.read();
  // while(n < 100)
  // digitalWrite(12 , 1);
  if(n < 100)
  {
    Serial.print("HI");
    digitalWrite(12 , 1);
  }
  // tone(1);
  else
  digitalWrite(12 , 0);
  Serial.print("First Sensor: ");
  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  selectMuxChannel(7);
  // sensor.init();
  Serial.print("Second Sensor: ");
  Serial.print(sensor.read());
  n = sensor.read();

  // if(n < 100)
  // tone(12 , 100);
  // else
  // noTone(12);
  if(n < 100)
  // tone(1);
  digitalWrite(12 , 1);
  else
  digitalWrite(12 , 0);
  // while(se < 100)
  // digitalWrite(12 , 1);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}


void selectMuxChannel(uint8_t channel) {
  // Ensure channel is within range (0 to 7)
  if (channel < 8) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel); // Set the corresponding bit for the channel
    Wire.endTransmission();
    delay(10); // Delay to allow the multiplexer to settle
  } else {
    Serial.println("Invalid channel number!");
  }
}

// void beep(int dist , int pin)
// {
//   // if(dist < 100)
//   if(dist < 100)
//   tone(12 , 40 , )
// }
```
