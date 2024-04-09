//----------------------------------------------------------------------------------------------------------------------------------
//Run openocd for debugging
//openocd -f STM32F103C8T6.cfg
//----------------------------------------------------------------------------------------------------------------------------------

#include "../fischertechnik_remote_control.h"
#include "../stm32f103_db.h"
#include "../stm32f103_io.h"
#include "../stm32f103_delay.h"
#include "../stm32f103_nrf905.h"
#include "../stm32f103_motor_control.h"
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

int  main(void);
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
    motor_driver_interrupt,//0x000000B4  TIM3
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

//Minimum ADC reading for the FT servo
//Most servos will be around this limit on the bottom extreme
#define SERVO_MIN           1280

//Maximum ADC output value for the FT servo
//This relates to the full swing most of the FT servos have in relation to the transmitter
//The MAXDIFFERENCE in the motor driver times 2 for full range times 16 for ADC resolution
#define SERVO_MAX           2944

//Allowed ADC range for the FT servo
//This is the full range delta the FT servos actually have
//54 times 16. Is about the average of the servos I have
#define SERVO_RANGE          864

//The needed ADC offset for center alignment and proper control of the FT servo
//30 times 16. Tweak this for better alignment with the transmitter joystick centers
#define SERVO_OFFSET         480

//Limits for the calculated results
#define SERVO_BOTTOM_LIMIT     0
#define SERVO_TOP_LIMIT     4095

//Wanted center point for the motor speed control
#define SERVO_CENTER         128

//When the ADC output value is above the given value the system assumes no servo connected
//and sets center value for motor speed control
#define SET_CENTER_ABOVE    3840

//----------------------------------------------------------------------------------------------------------------------------------

uint8_t ProcessPotentiometer(int channel)
{
  int16_t sample;
  
  //Get a new sample for this channel
  sample = adcRead(channel);
  
  //Check if the read value is above the given value.
  //This indicates that there is no potentiometer connected to the analog input
  //and the reading should remain steady on center value to allow for motor speed control
  if(sample > SET_CENTER_ABOVE)
  {
    //If so return the center value
    return(SERVO_CENTER);
  }
  else
  {
    //Range the 12 bit sample read from the ADC to the set full range and range it around center by adding the offset
    sample = ((((int16_t)sample - SERVO_MIN) * SERVO_MAX) / SERVO_RANGE) + SERVO_OFFSET;

    //Make sure it stays within the limits
    if(sample > SERVO_TOP_LIMIT)
      sample = SERVO_TOP_LIMIT;
    else if(sample < SERVO_BOTTOM_LIMIT)
      sample = SERVO_BOTTOM_LIMIT;

    //Only need the top 8 bit
    //Need to reverse the result for proper servo control
    //This is needed to match the plus and minus output signals to the up and down of the potentiometer
    //Without the wires to the servo motor need to be reversed
    return(255 - (sample >> 4));
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//The incoming message is stored in this structure

FTMESSAGE ftmessage = { 0 };

//----------------------------------------------------------------------------------------------------------------------------------

//Position feedback variables for the two possible servos
volatile uint8_t feedback1 = 0;
volatile uint8_t feedback2 = 0;

//Position variables for the two channels
//Default to center position
volatile uint8_t position1 = SERVO_CENTER;
volatile uint8_t position2 = SERVO_CENTER;

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
    
  //Enable the used peripherals. Timer 1 for acting as a watchdog on loss of transmitter signal
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  //And timer 2 for the feedback potentiometer sample interval
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  
  //Enable the system timer for delay functionality
  delayinit();
 
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
  
  //Initialize the ADC for reading the potentiometers
  adcInit();

  //Initialize the NRF905 module for receiving data from the transmitter
  nrf905_init();
  nrf905_enable_receiver();

  //Setup the system for controlling the motors and servos  
  motor_driver_init();

  //Needs to be setup after the ADC initialization because the analog inputs are sampled in the interrupt routine
  //Timer 2
  TIM2->CNT = 0;
  TIM2->PSC = 719;    //72MHz / 720 = 100KHz
  TIM2->ARR = 99;     //100KHz / 100 = 1KHz;

  //Timer 2 generates an interrupt at a 1ms rate
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

  //Timer 1
  TIM1->CNT = 0;
  TIM1->PSC = 719;    //72MHz / 720 = 100KHz
  TIM1->ARR = 4999;   //100KHz / 5000 = 20Hz;

  //Timer 1 generates an interrupt at a 50ms rate
  //This means a loss of a minimal of 5 consecutive data sets received from the transmitter
  TIM1->DIER = TIM_DIER_UIE;
  TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
  
  //Main part of the receiver is to check if there is new data received from the transmitter
  while(1)
  {
    //Check if there is a valid packet for this receiver
    if(nrf905_check_address_match() && nrf905_check_data_ready())
    {
      //Get the data into the message struct
      nrf905_receive_packet((uint8_t *)&ftmessage);
      
      //Make sure the data is correctly aligned based on the preamble
      if(ftmessage.preamble == 0x55AA)
      {
        //Calculate the crc to make sure the data is valid
        uint16_t crc = calculate_crc16((uint8_t *)&ftmessage, FT_DATA_LENGTH);
        
        //Proceed when valid data
        if(crc == ftmessage.crc)
        {
          //At this point a valid message is received
          //Set the requested positions for the control interrupts to take action if needed
          position1 = ftmessage.channel1;
          position2 = ftmessage.channel2;
          
          //Set the received state for the two on/off channels
          motor_driver_control_channel_3(ftmessage.channel3);
          motor_driver_control_channel_4(ftmessage.channel4);
          
          //Reset the watchdog timeout timer
          TIM1->CNT = 0;
        }
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//Watchdog timer to detect loss of signal from the transmitter

void tim1IrqHandler(void)
{
  //Clear the interrupt flags
  TIM1->SR = 0;
  
  //Keep the servos on the current position
  position1 = feedback1;
  position2 = feedback2;
  
  //In the case of the loss of signal the motor drive outputs need to be turned off
  motor_control_emergency_stop();
}

//----------------------------------------------------------------------------------------------------------------------------------
//Sampling of feedback signals based on a set interval

void tim2IrqHandler(void)
{
  //Clear the interrupt flags
  TIM2->SR = 0;
  
  //Measure the analog signals from the feedback potentiometers
  feedback1 = ProcessPotentiometer(0);
  feedback2 = ProcessPotentiometer(1);
  
  //Go and check if the channels 1 or 2 need stopping the motor with a break action
  motor_driver_check_channel_1_2();
}

//----------------------------------------------------------------------------------------------------------------------------------
