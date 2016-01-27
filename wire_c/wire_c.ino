extern "C"
{
  #include "avr_i2c.h"
  #include "utility/twi.h"
};

const uint8_t ADDRESS = 0x68;
const uint8_t REGISTER = 0x75;

void setup()
{
  avr_i2c_begin();       // Join i2c bus
  Serial.begin(115200);
}


void loop()
{
  unsigned char loopData;
  
  avr_i2c_read(ADDRESS, REGISTER, 1, &loopData);
  Serial.print("Hoping for 0x71, read: 0x");
  Serial.println(loopData, HEX);

  delay(1000);
}
