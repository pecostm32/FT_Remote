//----------------------------------------------------------------------------------------------------------------------------------
//
//PB3, PB4, PA13 and PA15 are special pins. Need alternate function enabled and disabling of JTAG for it to be GPIO
//
//PB3 and PB5 need pullup resistors to match the reset state of PA15 and PB4
//
//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_io.h"
#include "stm32f103_motor_control.h"

//----------------------------------------------------------------------------------------------------------------------------------
//Set the enable bits for the used IO ports in this define

#define USED_IO_PORTS   (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN)

//----------------------------------------------------------------------------------------------------------------------------------

//The dead band specifies the minimum input change needed before outputting a signal
#define DEADBAND        3

//Specifies the minimum number of timer ticks for the smallest pulse
#define MINPULSE       25

//Setting to avoid getting compare interrupt
#define NO_COMPARE_INTERRUPT   150

//----------------------------------------------------------------------------------------------------------------------------------

//Port and pin couplings for channel 1
#define CH1_FWD_PORT    GPIOB
#define CH1_FWD_PIN         8
#define CH1_BWD_PORT    GPIOB
#define CH1_BWD_PIN         9

//Port and pin couplings for channel 2
#define CH2_FWD_PORT    GPIOB
#define CH2_FWD_PIN         6
#define CH2_BWD_PORT    GPIOB
#define CH2_BWD_PIN         7

//Port and pin couplings for channel 3
#define CH3_FWD_PORT    GPIOB
#define CH3_FWD_PIN         4
#define CH3_BWD_PORT    GPIOB
#define CH3_BWD_PIN         5

//Port and pin couplings for channel 4
#define CH4_FWD_PORT    GPIOB
#define CH4_FWD_PIN         3
#define CH4_BWD_PORT    GPIOA
#define CH4_BWD_PIN        15

//----------------------------------------------------------------------------------------------------------------------------------

#define MOTOR_STATE_NONE      0
#define MOTOR_STATE_FORWARD   1
#define MOTOR_STATE_BACKWARD  2

//----------------------------------------------------------------------------------------------------------------------------------

//Position feedback variables for the two possible servos
extern volatile uint8_t feedback1;
extern volatile uint8_t feedback2;

//Position variables for the two channels
//Default to center position
extern volatile uint8_t position1;
extern volatile uint8_t position2;

//----------------------------------------------------------------------------------------------------------------------------------

//Motor state variables
volatile uint8_t motorstate1 = MOTOR_STATE_NONE;
volatile uint8_t motorstate2 = MOTOR_STATE_NONE;

//----------------------------------------------------------------------------------------------------------------------------------

void motor_driver_init(void)
{
  //Enable the used peripherals. Need to reroute PB4 to its alternate function
  RCC->APB2ENR |= USED_IO_PORTS | RCC_APB2ENR_AFIOEN;
  
  //Enable timer 3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  
  //Disable the JTAG interface, but leave SWD enabled. This allows the usage of GPIO pins PA15, PB3 and PB4
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  
  //Initialize the IO pins for the servo motors
  //Setup the control pins as low speed push pull
  //Make sure motor is not turning (motor break!) by setting both pins high
  
  //Channel 1
  InitIOPin(CH1_FWD_PORT, CH1_FWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  InitIOPin(CH1_BWD_PORT, CH1_BWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  CH1_FWD_PORT->ODR |= (1 << CH1_FWD_PIN);
  CH1_BWD_PORT->ODR |= (1 << CH1_BWD_PIN);
  
  //Channel 2
  InitIOPin(CH2_FWD_PORT, CH2_FWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  InitIOPin(CH2_BWD_PORT, CH2_BWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  CH2_FWD_PORT->ODR |= (1 << CH2_FWD_PIN);
  CH2_BWD_PORT->ODR |= (1 << CH2_BWD_PIN);
  
  //Channel 3
  InitIOPin(CH3_FWD_PORT, CH3_FWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  InitIOPin(CH3_BWD_PORT, CH3_BWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  CH3_FWD_PORT->ODR |= (1 << CH3_FWD_PIN);
  CH3_BWD_PORT->ODR |= (1 << CH3_BWD_PIN);
  
  //Channel 4
  InitIOPin(CH4_FWD_PORT, CH4_FWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  InitIOPin(CH4_BWD_PORT, CH4_BWD_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  CH4_FWD_PORT->ODR |= (1 << CH4_FWD_PIN);
  CH4_BWD_PORT->ODR |= (1 << CH4_BWD_PIN);

  //Setup timer 3 for the pulse width modulation ~10ms interval
  TIM3->CNT = 0;
  TIM3->PSC = 6160;  //72MHz / 6161  = ~11500Hz
  TIM3->ARR = 114;   //~11500Hz / 117 = ~100Hz;  //min pulse 25 plus 90 transmitter active range

  //Shadow registers for the compare are used
  TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
  
  //No stop interrupts just yet
  TIM3->CCR1 = NO_COMPARE_INTERRUPT;
  TIM3->CCR2 = NO_COMPARE_INTERRUPT;

  //Make sure the active registers are loaded from the buffered ones
  TIM3->EGR = TIM_EGR_UG;
  
  //Enable the timer 3 interrupt on a higher priority then the other interrupts
  NVIC->IP[TIM3_IRQn] = 0x40;  //(0100b) Group priority 1, sub priority 0
  NVIC->ISER[TIM3_IRQn >> 0x05] = (uint32_t)0x01 << (TIM3_IRQn & 0x1F);
  
  //Clear any pending interrupts
  TIM3->SR = 0;
  
  //Timer 3 is allowed to generates an interrupt at a 100Hz rate, and on compare 1 and 2 events
  TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
  
  //Enable the timer and make loading the counter non buffered.
  TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

//----------------------------------------------------------------------------------------------------------------------------------
//Function to check if motor(s) of channel 1 or 2 need to be stopped with a break on them

void motor_driver_check_channel_1_2(void)
{
  int32_t diff;

  //Check if the channel 1 motor needs to be stopped by taking the difference between the two positions
  diff = feedback1 - position1;

  //Need the absolute value for calculating checking against the deadband
  if(diff < 0)
  {
    //Make the difference positive
    diff = -diff;
  }

  //Check if the motor of the first channel should be turned off. Dead band is used to filter unwanted jitter on the servo due to signal noise
  if(diff <= DEADBAND)
  {
    //The motor does not need to be turned on the next time, and is already stopped in the analog sampling interrupt
    motorstate1 = MOTOR_STATE_NONE;

    //Set the compare register beyond the timer count so no compare interrupt will be generated
    TIM3->CCR1 = NO_COMPARE_INTERRUPT;

    //Stop the motor by putting the break on
    CH1_FWD_PORT->ODR |= (1 << CH1_FWD_PIN);
    CH1_BWD_PORT->ODR |= (1 << CH1_BWD_PIN);
  }
  
  //Check if the channel 2 motor needs to be stopped by taking the difference between the two positions
  diff = feedback2 - position2;

  //Need the absolute value for calculating checking against the deadband
  if(diff < 0)
  {
    //Make the difference positive
    diff = -diff;
  }

  //Check if the motor of the first channel should be turned off. Dead band is used to filter unwanted jitter on the servo due to signal noise
  if(diff <= DEADBAND)
  {
    //The motor does not need to be turned on the next time, and is already stopped in the analog sampling interrupt
    motorstate2 = MOTOR_STATE_NONE;

    //Set the compare register beyond the timer count so no compare interrupt will be generated
    TIM3->CCR2 = NO_COMPARE_INTERRUPT;

    //Stop the motor by putting the break on
    CH2_FWD_PORT->ODR |= (1 << CH2_FWD_PIN);
    CH2_BWD_PORT->ODR |= (1 << CH2_BWD_PIN);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//Function to control channel 3

void motor_driver_control_channel_3(uint32_t state)
{
  //Check what action is requested
  if(state == MOTOR_STATE_OFF)
  {
    //For motor off the outputs need to go high. Only one pin is controlled to simulate a relay
    CH3_BWD_PORT->ODR |= (1 << CH3_BWD_PIN);
  }
  else
  {
    //For motor on the backward control is made low. This way the non ground pin goes high.
    CH3_BWD_PORT->ODR &= ~(1 << CH3_BWD_PIN);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//Function to control channel 4

void motor_driver_control_channel_4(uint32_t state)
{
  //Check what action is requested
  if(state == MOTOR_STATE_OFF)
  {
    //For motor off the outputs need to go high. Only one pin is controlled to simulate a relay
    CH4_BWD_PORT->ODR |= (1 << CH4_BWD_PIN);
  }
  else
  {
    //For motor on the backward control is made low. This way the non ground pin goes high.
    CH4_BWD_PORT->ODR &= ~(1 << CH4_BWD_PIN);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void motor_control_emergency_stop(void)
{
  //The channels 1 and 2 are stopped by the servo control system
  
  //Stop the channel 3 motor by putting the break on
  CH3_FWD_PORT->ODR |= (1 << CH3_FWD_PIN);
  CH3_BWD_PORT->ODR |= (1 << CH3_BWD_PIN);

  //Stop the channel 4 motor by putting the break on
  CH4_FWD_PORT->ODR |= (1 << CH4_FWD_PIN);
  CH4_BWD_PORT->ODR |= (1 << CH4_BWD_PIN);
}

//----------------------------------------------------------------------------------------------------------------------------------

void motor_driver_interrupt(void)
{
  int32_t diff1;
  int32_t diff2;
  
  //Check if it is a timer update interrupt
  if(TIM3->SR & TIM_SR_UIF)
  {
    //Check if motors need to be turned on based on previous calculations.
    //The compare registers are already set for the turn off action if so.
    //This happens when the update interrupt is generated.
    //The shadow register is loaded with the pre load register.
    
    //Control the motor of the first channel
    switch(motorstate1)
    {
      case MOTOR_STATE_FORWARD:
        //Start turning forward by making the forward pin high and the backward pin low
        CH1_FWD_PORT->ODR |=  (1 << CH1_FWD_PIN);
        CH1_BWD_PORT->ODR &= ~(1 << CH1_BWD_PIN);
        break;
        
      case MOTOR_STATE_BACKWARD:
        //Start turning backward by making the forward pin low an the backward pin high
        CH1_FWD_PORT->ODR &= ~(1 << CH1_FWD_PIN);
        CH1_BWD_PORT->ODR |=  (1 << CH1_BWD_PIN);
        break;
    }

    //Control the motor of the second channel
    switch(motorstate2)
    {
      case MOTOR_STATE_FORWARD:
        //Start turning forward by making the forward pin high and the backward pin low
        CH2_FWD_PORT->ODR |=  (1 << CH2_FWD_PIN);
        CH2_BWD_PORT->ODR &= ~(1 << CH2_BWD_PIN);
        break;
        
      case MOTOR_STATE_BACKWARD:
        //Start turning backward by making the forward pin low an the backward pin high
        CH2_FWD_PORT->ODR &= ~(1 << CH2_FWD_PIN);
        CH2_BWD_PORT->ODR |=  (1 << CH2_BWD_PIN);
        break;
    }
    
    //Calculate new settings for the next time round
    //Assume backward movement for now
    motorstate1 = MOTOR_STATE_BACKWARD;
    motorstate2 = MOTOR_STATE_BACKWARD;
    
    //Check if the servos need to be repositioned by taking the difference between the two positions
    diff1 = feedback1 - position1;
    diff2 = feedback2 - position2;

    //Need the absolute value for calculating the pulse width
    if(diff1 < 0)
    {
      //Make the difference positive
      diff1 = -diff1;
      
      //And signal forward movement needed
      motorstate1 = MOTOR_STATE_FORWARD;
    }

    //Need the absolute value for calculating the pulse width
    if(diff2 < 0)
    {
      //Make the difference positive
      diff2 = -diff2;
      
      //And signal forward movement needed
      motorstate2 = MOTOR_STATE_FORWARD;
    }
    
    //Check if the motor of the first channel should not be turned on. Dead band is used to filter unwanted jitter on the servo due to signal noise
    if(diff1 <= DEADBAND)
    {
      //The motor does not need to be turned on the next time, and is already stopped in the analog sampling interrupt
      motorstate1 = MOTOR_STATE_NONE;
      
      //Set the compare register beyond the timer count so no compare interrupt will be generated
      TIM3->CCR1 = NO_COMPARE_INTERRUPT;
    }
    else
    {
      //Calculate the needed pulse width for the distance to travel
      //It will last for at least the minimum time plus the distance to travel
      //When the result is higher than the timer count no interrupt will come meaning 100% on
      TIM3->CCR1 = MINPULSE + diff1;
    }
    
    //Check if the motor of the second channel should not be turned on. Dead band is used to filter unwanted jitter on the servo due to signal noise
    if(diff2 <= DEADBAND)
    {
      //The motor does not need to be turned on the next time, and is already stopped in the analog sampling interrupt
      motorstate2 = MOTOR_STATE_NONE;
      
      //Set the compare register beyond the timer count so no compare interrupt will be generated
      TIM3->CCR2 = NO_COMPARE_INTERRUPT;
    }
    else
    {
      //Calculate the needed pulse width for the distance to travel
      //It will last for at least the minimum time plus the distance to travel
      //When the result is higher than the timer count no interrupt will come meaning 100% on
      TIM3->CCR2 = MINPULSE + diff2;
    }
  }
  else
  {
    //Check if there is a compare interrupt for the first channel
    if(TIM3->SR & TIM_SR_CC1IF)
    {
      //If so stop the motor
      //Don't use the break feature, so pins go low
      CH1_FWD_PORT->ODR &= ~(1 << CH1_FWD_PIN);
      CH1_BWD_PORT->ODR &= ~(1 << CH1_BWD_PIN);
    }
  
    //Check if there is a compare interrupt for the second channel
    if(TIM3->SR & TIM_SR_CC2IF)
    {
      //If so stop the motor
      //Don't use the break feature, so pins go low
      CH2_FWD_PORT->ODR &= ~(1 << CH2_FWD_PIN);
      CH2_BWD_PORT->ODR &= ~(1 << CH2_BWD_PIN);
    }
  }

  //Clear the interrupt flags to allow the next one to come in
  TIM3->SR = 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
