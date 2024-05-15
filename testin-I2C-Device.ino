//i2c device connected in bus checking code
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