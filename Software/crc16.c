//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_db.h"

//----------------------------------------------------------------------------------------------------------------------------------

#define CRC16_POLY  0x8005

//----------------------------------------------------------------------------------------------------------------------------------

uint16_t update_crc16(uint16_t crc16, uint8_t val)
{
  int bit_flag;
  int i;

  for(i=7;i>=0;i--)
  {
    bit_flag = crc16 >> 15;

    /* Get next bit: */
    crc16 <<= 1;

    crc16 |= (val >> i) & 1; // item a) work from the least significant bits

    /* Cycle check: */
    if(bit_flag)
    {
      crc16 ^= CRC16_POLY;
    }
  }
  
  return(crc16);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16_t finalise_crc16(uint16_t crc16)
{
  // item b) "push out" the last 16 bits
  int i;
  int bit_flag;
  
  for(i=0;i<16;++i)
  {
    bit_flag = crc16 >> 15;
    
    crc16 <<= 1;
    
    if(bit_flag)
    {
      crc16 ^= CRC16_POLY;
    }
  }

  return(crc16);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16_t calculate_crc16(uint8_t *buffer, uint32_t length)
{
  uint16_t crc16 = 0;
  
  //Add all the given bytes to the crc
  while(length)
  {
    //Add the current byte to the crc
    crc16 = update_crc16(crc16, *buffer);
    
    //Point to the next byte to add to the crc
    buffer++;
    
    //Done one byte
    length--;
  }
  
  //finish the crc and return it to the caller
  return(finalise_crc16(crc16));
}

//----------------------------------------------------------------------------------------------------------------------------------
