//----------------------------------------------------------------------------------------------------------------------------------
// Delay functionality based on the system tick timer
//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_db.h"

//----------------------------------------------------------------------------------------------------------------------------------

void delayinit(void)
{
  //Enable the system timer for delay function
  //In default setting with 72MHz AHB clock this timer runs on 9MHz, so 111ns per tick. Takes 9 ticks for a microsecond
  STK->LOAD = 0x00FFFFFF;
  STK->CTRL = STK_CTRL_ENABLE;
}

//----------------------------------------------------------------------------------------------------------------------------------
//The number of microseconds must be less then 1864135. (just over 1.8 second)
//Multiplied by 9 gives the number of ticks.
//With this it can only lead to a single timer overflow, which this function can handle
void usdelay(int32_t usec)
{
  int32_t end = STK->VAL - (usec * 9);

  //Check if there is the need to wait for an timer overflow
  if(end <= 0)
  {
    //Wait for the overflow to occur
    while((STK->CTRL & STK_CTRL_OVERFLOW) == 0);

    //calculate the new end value
    end += 0x00FFFFFF;
  }

  //Wait till the timer reaches the intended value for the given delay
  while(STK->VAL >= end);
}

//----------------------------------------------------------------------------------------------------------------------------------
