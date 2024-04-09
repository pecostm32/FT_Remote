//----------------------------------------------------------------------------------------------------------------------------------
// IO functionality for STM32F103 MCU
//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_db.h"

//----------------------------------------------------------------------------------------------------------------------------------

//Simple function for setup of an IO pin
void InitIOPin(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t conf)
{
  //Mix the mode and configuration for single instruction usage
  uint32_t data = mode | (conf << 2);

  //Create a base pointer for either the lower or the higher control register
  __IO uint32_t *reg;

  //See if the lower control register or the higher control register needs to be used
  if(pin < 8)
  {
    //Low control register used for first 8 pins
    reg = &port->CRL;
  }
  else
  {
    //Force pin into 8 pins per register range
    pin -= 8;

    //High control register used for upper 8 pins
    reg = &port->CRH;
  }

  //4 control bits used per pin
  pin *= 4;

  //Reset bits first and set new mode and configuration.
  *reg &= ~(0x0F << pin);
  *reg |=  (data << pin);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint32_t readIOPin(GPIO_TypeDef *port, uint32_t pin)
{
  //Convert pin number to a bit mask
  register uint32_t mask = (1 << pin);
  
  //Get the masked status of the pin and match it with the mask to return either 0 or 1
  return((port->IDR & mask) == mask);
}

//----------------------------------------------------------------------------------------------------------------------------------

void setIOPin(GPIO_TypeDef *port, uint32_t pin)
{
  //Convert pin number to a set bit mask and set the output
  port->ODR |= (1 << pin);
}

//----------------------------------------------------------------------------------------------------------------------------------

void clearIOPin(GPIO_TypeDef *port, uint32_t pin)
{
  //Convert pin number to a clear bit mask and clear the output
  port->ODR &= ~(1 << pin);
}

//----------------------------------------------------------------------------------------------------------------------------------
