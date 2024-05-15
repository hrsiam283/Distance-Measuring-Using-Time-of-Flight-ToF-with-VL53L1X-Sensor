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