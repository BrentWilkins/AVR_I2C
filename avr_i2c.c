#include <util/delay.h> // TODO: Remove this debug
#include "avr_i2c.h"

#define BUFFER_LENGTH 32

typedef struct
{
  uint8_t data[BUFFER_LENGTH];
  uint8_t index;
  uint8_t length;
  uint8_t address;
} buffer;

// rxAddress shouldn't be used in master mode
static buffer rxBuffer = { .index = 0, .length = 0, .address = 0 };
static buffer txBuffer = { .index = 0, .length = 0, .address = 0 };

static uint8_t transmitting = 0;

uint8_t avr_i2c_endTransmission(bool sendStop);
size_t  avr_i2c_write_bytes(const uint8_t *data, size_t quantity);


// TODO: Remove this debug flash?
void flash(uint8_t times)
{
  // Writing a logic one to PINxn toggles the value of PORTxn,
  int i;
  for(i = 0; i < times; ++i)
  {
    bitSet(PORTB, 5);
    _delay_ms(200);
    bitClear(PORTB, 5);
    _delay_ms(200);
  }
}

void avr_i2c_begin(void)  // Master only
{
  rxBuffer.index = 0;
  rxBuffer.length = 0;

  txBuffer.index = 0;
  txBuffer.length = 0;

  twi_init();
}

void avr_i2c_end(void)
{
  twi_disable();
}

void avr_i2c_beginTransmission(uint8_t slave_addr)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txBuffer.address = slave_addr;
  // reset tx buffer iterator vars
  txBuffer.index = 0;
  txBuffer.length = 0;
}

size_t avr_i2c_write_byte(uint8_t data)
{
  if(transmitting)
  {
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBuffer.length >= BUFFER_LENGTH){
      //setWriteError();
      return 0;
    }
    // put byte in tx buffer
    txBuffer.data[txBuffer.index] = data;
    ++txBuffer.index;
    // update amount in buffer   
    txBuffer.length = txBuffer.index;
  }
  else
  {
    // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
  return 1;
}


/*
int avr_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char const * data)
{
  int ret = 4; // Other error
  avr_i2c_beginTransmission(slave_addr);  // Initialize the Tx buffer

  if (avr_i2c_write_byte(reg_addr))
  {
    ret = avr_i2c_write_bytes(data, length);
    if (ret)
    {
      switch (avr_i2c_endTransmission(true))
      {
      case 0:
        ret = 0;  // Success
        break;
      case 1: // Data too long to fit in transmit buffer
        ret = 1;
        break;
      case 2: // Received NACK on transmit of address
        ret = 2;
        break;
      case 3: // Received NACK on transmit of data
        ret = 3;
        break;
      case 4: // Other Error
        ret = 4;
        break;
      default:
        ret = 4;
      }
    }
    else
    {
      ret = 4;
    }
  }

  return ret;
}
*/

// STOP is performed on the I2C bus
uint8_t avr_i2c_endTransmission(bool sendStop)
{
  // transmit buffer (blocking)
  uint8_t ret = twi_writeTo(txBuffer.address, txBuffer.data,
                            txBuffer.length, 1, sendStop);
  // reset tx buffer iterator vars
  txBuffer.index = 0;
  txBuffer.length = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

// TODO: Probably not compatible with newer boards like Due (internal registers)
uint8_t avr_i2c_requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer.data, quantity, sendStop);
  // set rx buffer iterator vars
  rxBuffer.index = 0;
  rxBuffer.length = read;

  return read;
}


int avr_i2c_available(void)
{
  return rxBuffer.length - rxBuffer.index;
}

int avr_i2c_read_byte(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if(rxBuffer.index < rxBuffer.length)
  {
    value = rxBuffer.data[rxBuffer.index++];
  }

  return value;
}

/*
int avr_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                 unsigned char length, unsigned char *data)
{
  int ret = 4;  // Other error
  avr_i2c_beginTransmission(slave_addr);  // Initialize the Tx buffer
  if (avr_i2c_write_byte(reg_addr))  // Put slave register address in Tx buff
  {
    if (avr_i2c_endTransmission(false))  // Send Tx, send restart to keep alive
    {
      ret = 4;  // Error
    }
    //else if (avr_i2c_requestFrom(slave_addr, length))
    else if (avr_i2c_requestFrom(slave_addr, length, (uint8_t)true))
    {
      unsigned char count = 0;
      while (avr_i2c_available())
      {
        data[count++] = avr_i2c_read_byte();
      }
      if (count == length)
      {
        ret = 0;  // Succcess
      }
      else
      {
        ret = 4;  // Error
      }
    }
    else
    {
      ret = 4;  // Error
    }
  }
  else
  {
    //debug_println("Error: couldn't send slave register address");
  }
  return ret;
}
*/

