
#include <Wire.h>
void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

void loop() {
  
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("Coordinates are ");        // sends five bytes
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  x++;
  delay(500);
}
