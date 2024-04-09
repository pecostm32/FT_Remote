//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_db.h"
#include "stm32f103_io.h"

//----------------------------------------------------------------------------------------------------------------------------------

void adcInit(void)
{
  //Enable the used peripherals. PORTA and ADC1
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
  
  //Analog inputs
  InitIOPin(GPIOA, 0, GPIO_MODE_INPUT, GPIO_CONF_I_ANA);   //Pot 1
  InitIOPin(GPIOA, 1, GPIO_MODE_INPUT, GPIO_CONF_I_ANA);   //Pot 2

  //ADC functionality
  //Single conversion mode
  //Start up the ADC
  ADC1->CR2  = ADC_CR2_ADON;

  //Set the sample time for channels 1 and 0 to 28.5 adc clk cycles
  ADC1->SMPR2 = ADC_SMPR2_CH1_28 | ADC_SMPR2_CH0_28;

  //For the ADC the channels can be handled in a group. In the SQR registers a group of up to 16 channels can be defined
  //for automated conversion. Here this is not used and for single conversion only the first entry found in SQR3 is used.
  
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16_t adcRead(uint8_t channel)
{
  //Select the channel.
  ADC1->SQR3 = channel & 0x1F;

  //Start the conversion
  ADC1->CR2 = ADC_CR2_ADON;

  //Wait until conversion is done
  while((ADC1->SR & 2) == 0);

  //Give back the sampled value
  return (uint16_t)(ADC1->DR & 0xFFF);
}

//----------------------------------------------------------------------------------------------------------------------------------
