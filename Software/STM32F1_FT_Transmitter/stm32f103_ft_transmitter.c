//----------------------------------------------------------------------------------------------------------------------------------
//Run openocd for debugging
//openocd -f STM32F103C8T6.cfg
//----------------------------------------------------------------------------------------------------------------------------------

#include "../fischertechnik_remote_control.h"
#include "../stm32f103_db.h"
#include "../stm32f103_io.h"
#include "../stm32f103_delay.h"
#include "../stm32f103_nrf905.h"
#include "../stm32f103_analog.h"
#include "../crc16.h"

//----------------------------------------------------------------------------------------------------------------------------------

#define STACK_TOP 0x20005000

//----------------------------------------------------------------------------------------------------------------------------------

extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

//----------------------------------------------------------------------------------------------------------------------------------

int main(void);
void resetHandler(void);
void tim1IrqHandler(void);
void tim2IrqHandler(void);

//----------------------------------------------------------------------------------------------------------------------------------

//Vectors for other STM32F10xxx devices. Connectivity line and XL-density devices have different tables
//Not all peripherals are available in the STM32F103C8 or STM32F103CB devices
const void * intVectors[76] __attribute__((section(".vectors"))) =
{
    (void*) STACK_TOP,     //0x00000000
    resetHandler,          //0x00000004  Reset
    0,                     //0x00000008  NMI
    0,                     //0x0000000C  HardFault
    0,                     //0x00000010  MemManage
    0,                     //0x00000014  BusFault
    0,                     //0x00000018  UsageFault
    0,                     //0x0000001C  Reserved
    0,                     //0x00000020  Reserved
    0,                     //0x00000024  Reserved
    0,                     //0x00000028  Reserved
    0,                     //0x0000002C  SVCall
    0,                     //0x00000030  Debug Monitor
    0,                     //0x00000034  Reserved
    0,                     //0x00000038  PendSV
    0,                     //0x0000003C  SysTick
    0,                     //0x00000040  WWDG
    0,                     //0x00000044  PVD
    0,                     //0x00000048  Tamper
    0,                     //0x0000004C  RTC
    0,                     //0x00000050  Flash
    0,                     //0x00000054  RCC
    0,                     //0x00000058  EXTI0
    0,                     //0x0000005C  EXTI1
    0,                     //0x00000060  EXTI2
    0,                     //0x00000064  EXTI3
    0,                     //0x00000068  EXTI4
    0,                     //0x0000006C  DMA1_Channel1
    0,                     //0x00000070  DMA1_Channel2
    0,                     //0x00000074  DMA1_Channel3
    0,                     //0x00000078  DMA1_Channel4
    0,                     //0x0000007C  DMA1_Channel5
    0,                     //0x00000080  DMA1_Channel6
    0,                     //0x00000084  DMA1_Channel7
    0,                     //0x00000088  ADC1_2
    0,                     //0x0000008C  USB_HP_CAN_TX
    0,                     //0x00000090  USB_LP_CAN_RX0
    0,                     //0x00000094  CAN_RX1
    0,                     //0x00000098  CAN_SCE
    0,                     //0x0000009C  EXTI9_5
    0,                     //0x000000A0  TIM1_BRK
    tim1IrqHandler,        //0x000000A4  TIM1_UP
    0,                     //0x000000A8  TIM1_TRG_COM
    0,                     //0x000000AC  TIM1_CC
    tim2IrqHandler,        //0x000000B0  TIM2
    0,                     //0x000000B4  TIM3
    0,                     //0x000000B8  TIM4
    0,                     //0x000000BC  I2C1_EV
    0,                     //0x000000C0  I2C1_ER
    0,                     //0x000000C4  I2C2_EV
    0,                     //0x000000C8  I2C2_ER
    0,                     //0x000000CC  SPI1
    0,                     //0x000000D0  SPI2
    0,                     //0x000000D4  USART1
    0,                     //0x000000D8  USART2
    0,                     //0x000000DC  USART3
    0,                     //0x000000E0  EXTI15_10
    0,                     //0x000000E4  RTCAlarm
    0,                     //0x000000E8  USBWakeup
    0,                     //0x000000EC  TIM8_BRK
    0,                     //0x000000F0  TIM8_UP
    0,                     //0x000000F4  TIM8_TRG_COM
    0,                     //0x000000F8  TIM8_CC
    0,                     //0x000000FC  ADC3
    0,                     //0x00000100  FSMC
    0,                     //0x00000104  SDIO
    0,                     //0x00000108  TIM5
    0,                     //0x0000010C  SPI3
    0,                     //0x00000110  UART4
    0,                     //0x00000114  UART5
    0,                     //0x00000118  TIM6
    0,                     //0x0000011C  TIM7
    0,                     //0x00000120  DMA2_Channel1
    0,                     //0x00000124  DMA2_Channel2
    0,                     //0x00000128  DMA2_Channel3
    0,                     //0x0000012C  DMA2_Channel4_5
};

//----------------------------------------------------------------------------------------------------------------------------------

void resetHandler(void)
{
  unsigned char volatile *src;
  unsigned char volatile *dst;
  unsigned len;

  src= &INIT_DATA_VALUES;
  dst= &INIT_DATA_START;
  len= &INIT_DATA_END - &INIT_DATA_START;

  while(len--)
    *dst++ = *src++;

  dst = &BSS_START;

  len = &BSS_END - &BSS_START;

  while(len--)
    *dst++=0;

  main();
}

//----------------------------------------------------------------------------------------------------------------------------------
//To range the readings of the joysticks to a usable 8 bit range the defines below are used
//The minimum value is what the ADC returns with the joystick in the lowest possible position
//The range is the delta between the lowest and the highest reading from min to max position without using the center adjust
//The maximum is just the full 12 bit range of the ADC

#define JOYSTICK_MIN        800
#define JOYSTICK_RANGE     1660
#define JOYSTICK_MAX       4095

//----------------------------------------------------------------------------------------------------------------------------------

FTMESSAGE ftmessage = { 0 };

uint8_t button0 = 0;
uint8_t button1 = 0;

//----------------------------------------------------------------------------------------------------------------------------------

uint8_t ProcessPotentiometer(int32_t channel)
{
  int16_t sample;
  
  //Range the 12 bit sample read from the ADC to the set full range
  sample = (((int16_t)adcRead(channel) - JOYSTICK_MIN) * JOYSTICK_MAX) / JOYSTICK_RANGE;

  //Make sure it stays within the limits
  if(sample > JOYSTICK_MAX)
    sample = JOYSTICK_MAX;
  else if(sample < 0)
    sample = 0;

  //Only need the top 8 bit, which filters it to get rid of some noise
  return(sample >> 4);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint8_t ProcessButton(uint8_t *data, GPIO_TypeDef *inport, uint32_t inpin, GPIO_TypeDef *outport, uint32_t outpin)
{
  //Get the previous state from the data stored in the 8th bit
  uint8_t previous = ((*data & 0x80) == 0x80);
  uint8_t state = 0;
  
  //Filter the input to de-bounce by shifting the result into a register
  //Make room in the register
  *data <<= 1;
  
  //Add the new sample
  *data |= readIOPin(inport, inpin);
  
  //Need three samples in a row to be low to decide button is pressed
  if((*data & 0x07) == 0x00)
  {
    //Check if button was not already down
    if(previous == 0)
    {
      //Show the user that the button is pressed
      setIOPin(outport, outpin);
      
      //Set the previous state to signal button is down
      previous = 1;
    }
  }
  //Need three samples in a row to be high to decide button is released
  else if((*data & 0x07) == 0x07)
  {
    //Check if button was not already up
    if(previous == 1)
    {
      //Show the user that the button is released
      clearIOPin(outport, outpin);

      //Clear the previous state to signal button is up
      previous = 0;
    }
  }

  //Set the state to return based on what previous now signals
  if(previous == 0)
  {
    //Button up so send 0
    state = 0;
    
    //Reflect this in the button data
    *data &= 0x7F;
  }
  else
  {
    //Button down so send 255
    state = 255;

    //Reflect this in the button data
    *data |= 0x80;
  }
  
  //Return the found state to the caller
  return(state);
}

//----------------------------------------------------------------------------------------------------------------------------------

int main(void)
{
  //Setup flash to work with 72MHz clock
  //Enable the Prefetch Buffer and Set to 2 wait states
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  //Configure system clock
  //External oscillator: 8MHz
  //PLL multiplicator: x9
  //SYSCLK: 72MHz
  //AHB: SYSCLK = 72MHz
  //APB1: SYSCLK/2 = 36MHz  //Timer 2,3 and 4 run on 72MHz since APB1 divider is not 1
  //APB2: SYSCLK/2 = 36MHz  //Timer 1 also runs on 72MHz since APB2 divider is not 1
  //ADC: SYSCLK/6 = 12MHz
  //USB: SYSCLK/1.5 = 48MHz
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

  //Enable external oscillator
  RCC->CR |= RCC_CR_HSEON;

  //Wait for the clock to become stable
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  //Enable the PLL
  RCC->CR |= RCC_CR_PLLON;

  //Wait for the PLL to become stable
  while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  //Switch to the PLL clock as system clock source. Since on reset these bits are set to 0 no need to clear first.
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  //Wait for the PLL to become the clock source
  while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

  //From this point on it is not possible to change the clock configuration without switching back to HSI
    
  //Enable the used peripherals. Need to reroute PB4 and PA15 to its alternate function
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN;

  //Enable timer 2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // RCC_APB1ENR_TIM3EN | 

  //Disable the JTAG interface, but leave SWD enabled. This allows the usage of GPIO pins PA15, PB3 and PB4
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  
  //Enable the system timer for delay functionality
  delayinit();
  
  //IO pins need to be configured first
  //Pin with LED to show activity
  InitIOPin(GPIOA, 15, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);

  //Button 0 input
  InitIOPin(GPIOB, 4, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);

  //Button 1
  InitIOPin(GPIOB, 5, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);

  //Button 0 output. A led to show the user the button is pressed
  InitIOPin(GPIOB, 9, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  
  //Button 1 output. A led to show the user the button is pressed
  InitIOPin(GPIOB, 8, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  
  //Timer 2
  TIM2->CNT = 0;
  TIM2->PSC = 15999;  //72MHz / 16000 = 4500Hz
  TIM2->ARR = 1124;   //4500Hz / 1125 = 4Hz;

  //Timer 2 generates an interrupt at a 4Hz rate
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

  //Initialize the ADC for reading the potentiometers
  adcInit();
  
  //Initialize the NRF905 module for transmitting data to the receiver
  nrf905_init();
  
  //Select the receiver to be addressed based on the switch and jumper settings
  nrf905_select_receiver();

  //Send first message
  nrf905_send_packet((uint8_t *)&ftmessage);
  
  //Timer 1 is used to time the transmissions on a 10ms interval
  TIM1->CNT = 0;
  TIM1->PSC = 719;   //72MHz / 720 = 100KHz
  TIM1->ARR = 999;   //100KHz / 1000 = 100Hz

  //Timer 1 up counting with update interrupt
  TIM1->DIER = TIM_DIER_UIE;
  TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
  
  //Set priority for timer 1, 2 and 3 interrupt to be higher then the other interrupts
  //This is an array of 8 bit registers, of which only the upper 4 bits are used for the priority allowing for 16 levels
  //By grouping this is separated to allow for having sub priorities within a single group.
  //In the usb init this is set for 4 group priorities with each 4 sub priorities.
  //The higher the number the lower the priority
  NVIC->IP[TIM1_UP_IRQn] = 0x80;  //(1000b) Group priority 2, sub priority 0
  NVIC->IP[TIM2_IRQn]    = 0x90;  //(1001b) Group priority 2, sub priority 1
  
  //Enable the timer 1 and 2 interrupt
  //This is an array of 32 bit registers, only used to enable an interrupt. To disable the ICER registers need to be used
  //Each register serves 32 interrupts, so to get the register for the interrupt, shift the IRQ number right 5 times (divide by 32) and to get
  //the right interrupt enable bit, shift a unsigned 32 bit integer 1 the IRQ number anded with 31 (modulo 32) times to the right
  NVIC->ISER[TIM1_UP_IRQn >> 0x05] = (uint32_t)0x01 << (TIM1_UP_IRQn & 0x1F);
  NVIC->ISER[TIM2_IRQn >> 0x05] = (uint32_t)0x01 << (TIM2_IRQn & 0x1F);
  
  while(1)
  {
    //Select the receiver to be addressed based on the switch and jumper settings
    nrf905_select_receiver();
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//Handler for sending the current state of the inputs

void tim1IrqHandler(void)
{
  //Clear the interrupt flags
  TIM1->SR = 0;

  //Setup the message to send
  //Set the preamble for checking the synchronization of the data in the receiver 
  ftmessage.preamble = 0x55AA;
  
  //Get the samples to send
  ftmessage.channel1 = ProcessPotentiometer(0);
  ftmessage.channel2 = 255 - ProcessPotentiometer(1);

  ftmessage.channel3 = ProcessButton(&button0, GPIOB, 4, GPIOB, 9);
  ftmessage.channel4 = ProcessButton(&button1, GPIOB, 5, GPIOB, 8);

  //calculate the crc
  ftmessage.crc = calculate_crc16((uint8_t *)&ftmessage, FT_DATA_LENGTH);
  
  //Measurements show that sampling takes about 20us and transferring the data to the NRF905 also takes about 20us
  //The transmission takes slightly over 3ms, so the repetition rate could be brought down to 5ms without issues
  nrf905_send_packet((uint8_t *)&ftmessage);
}

//----------------------------------------------------------------------------------------------------------------------------------
//Power led blinker

void tim2IrqHandler(void)
{
  //Clear the interrupt flags
  TIM2->SR = 0;

  //Blink the led
  GPIOA->ODR ^= (1 << 15);
}

//----------------------------------------------------------------------------------------------------------------------------------
