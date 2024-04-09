//----------------------------------------------------------------------------------------------------------------------------------
//NRF905 driver for STM32F103 MCU on SPI1 tailored for the use in the FT remote control project
//
//
//In Europe short range devices are allowed to use the 862MHz to 876MHz frequency range
//Another range is 433.05MHz to 434.79MHz.
//
//It looks like the devices bought on Aliexpress are designed for the 433MHz range and do not work on the higher frequencies.
//
//STM32 SPI signal lines can be used in both push pull as open drain setup, except for NSS (CSN)
//  NSS is always open drain despite the GPIO alternate function setting. It needs a pull up resistor to function properly.
//    When hardware mode is selected it goes low when SPI is enabled and the first data is transmitted.
//    It goes high again when transmission has been completed and the SPI is disabled.
//  SCK and MOSI can be used in both and need a pull up resistor when open drain is selected.
//
//The NRF905 when set in retransmission mode pulses the Data Ready signal after finishing a packet transmission.
//When not in retransmission mode it goes high and stays high until the transmission is ended by setting either
//TRX_CE or TX_EN low. When TX_EN is set low the shockburst receive mode is activated and on reception of a valid
//packet, Data Ready will be set high again.
//----------------------------------------------------------------------------------------------------------------------------------

#include "stm32f103_db.h"
#include "stm32f103_io.h"
#include "stm32f103_delay.h"
#include "stm32f103_nrf905.h"
#include "fischertechnik_remote_control.h"

//----------------------------------------------------------------------------------------------------------------------------------
//NRF905 instruction set

#define NRF905_WC     0x00
#define NRF905_RC     0x10
#define NRF905_WTP    0x20
#define NRF905_RTP    0x21
#define NRF905_WTA    0x22
#define NRF905_RTA    0x23
#define NRF905_RRP    0x24

//----------------------------------------------------------------------------------------------------------------------------------
//Default configuration for the transmitter and receiver

uint8_t nrf905_config_buffer[10] =
{
  0x73,                                                   //CH_NO 115, 433.9MHZ
  0x0C,                                                   //output power 10dBm, Receive current on normal operation (Auto retransmission, For retransmission | 0x20)
  0x44,                                                   //4-byte addresses
  FT_MESSAGE_LENGTH, FT_MESSAGE_LENGTH,                   //send and receive data lengths are specified with the given define
#ifdef NRF905_USE_TRANSMITTER
  'F', 't', 'T', 'X',                                     //receiving address for the transmitter PCB
#else
  '1', 'O', 'n', 'e',                                     //receiving address for the receiver PCB
#endif
  0xD8,                                                   //CRC enable, 16 bit CRC, external clock disable, 16MHZ oscillator
};

//----------------------------------------------------------------------------------------------------------------------------------
//Start with a not used address to have it set on the first check

uint8_t nrf905_current_rx_address = 0xFF;

//----------------------------------------------------------------------------------------------------------------------------------

void nrf905_write_spi(uint8_t command, uint8_t *data, uint32_t length)
{
  //Start a communication session by enabling the peripheral
  SPI1->CR1 |= SPI_CR1_SPE;
  
  //To start a session with the NRF905 a command needs to be send
  SPI1->DR = command;
  
  //Need to wait for the command to be transfered before loading the next byte
  //and then transfer all the data bytes
  for(;length > 0;length--)
  {
    //Wait until the register is empty
    while((SPI1->SR & SPI_SR_TXE) == 0);
    
    //Transfer the current byte
    SPI1->DR = *data;
    
    //Point to the next byte to transfer
    data++;
  }

  //Wait until the transmit register is empty
  while((SPI1->SR & SPI_SR_TXE) == 0);

  //Wait until the peripheral is done
  while(SPI1->SR & SPI_SR_BSY);
  
  //Disable the peripheral to clear the NSS line
  SPI1->CR1 &= ~SPI_CR1_SPE;
}

//----------------------------------------------------------------------------------------------------------------------------------

#ifdef NRF905_USE_RECEIVER

//----------------------------------------------------------------------------------------------------------------------------------
//This code works but there is a gap of on average 700ns between the transmissions of the bytes.
//With the use of the RXNE flag it could do continuous, but there is a risk of overrun due to
//interrupt interference, so best to leave it with the gap between the bytes.

void nrf905_read_spi(uint8_t command, uint8_t *buffer, uint32_t length)
{
  //Start a communication session by enabling the peripheral, which sets the NSS line low
  SPI1->CR1 |= SPI_CR1_SPE;
  
  //To start a session with the NRF905 a command needs to be send
  //Make sure only 8 bits are placed in the data register. F0 and F3 series work with packed data and can go wrong when
  //in 8 bits mode and getting 16 bits written. For this the type casting is used.
  *(volatile uint8_t *)&SPI1->DR = command;

  //Wait until the transmit register is empty. Is needed because the busy flag needs some cycles to become one
  while((SPI1->SR & SPI_SR_TXE) == 0);

  //Wait until the peripheral is done
  while(SPI1->SR & SPI_SR_BSY);

  //The first byte received is the status register and needs to be ignored
  //Use the buffer to read the byte into and later overwrite it with the actual data
  *buffer = SPI1->DR;
  
  //Continue to receive all the needed data bytes
  for(;length > 0;length--)
  {
    //Transfer a null byte to clock in the byte to be received
    *(volatile uint8_t *)&SPI1->DR = 0;
    
    //Wait until the transmit register is empty. Is needed because the busy flag needs some cycles to become one
    while((SPI1->SR & SPI_SR_TXE) == 0);

    //Wait until the peripheral is done
    while(SPI1->SR & SPI_SR_BSY);

    //Load the received byte into the buffer
    *buffer = SPI1->DR;
    
    //Point to the next location in the buffer
    buffer++;
  }

  //Disable the peripheral to make the NSS line high
  SPI1->CR1 &= ~SPI_CR1_SPE;
}

//----------------------------------------------------------------------------------------------------------------------------------

#endif

//----------------------------------------------------------------------------------------------------------------------------------

void nrf905_init(void)
{
  //Enable the used peripherals. PORTA for SPI1, PORTB for NRF905 signals, SPI1 for communication with the NRF905
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_SPI1EN;

  //Initialize the IO pins for the connection with the NRF905 module
  
  //SP1 CSN (NSS)
  InitIOPin(GPIOA, 4, GPIO_MODE_50MHZ, GPIO_CONF_AF_OD);
  
  //SP1 SCK
  InitIOPin(GPIOA, 5, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);

  //SP1 MISO
  InitIOPin(GPIOA, 6, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
  
  //SP1 MOSI
  InitIOPin(GPIOA, 7, GPIO_MODE_50MHZ, GPIO_CONF_AF_PP);
  
  //Data ready input
  InitIOPin(NRF905_DR_PORT, NRF905_DR_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_PUPD);
  
  //Address match input
  InitIOPin(NRF905_AM_PORT, NRF905_AM_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_PUPD);
  
  //Carrier detect input
  InitIOPin(NRF905_CD_PORT, NRF905_CD_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_PUPD);
  
  //Chip transmit/receive enable output (TRX_CE)
  InitIOPin(NRF905_CE_PORT, NRF905_CE_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  
  //Transmit enable output (TX_EN)
  InitIOPin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);
  
  //Power enable output (PWR_UP)
  InitIOPin(NRF905_PWR_PORT, NRF905_PWR_PIN, GPIO_MODE_2MHZ, GPIO_CONF_O_PP);

#ifdef NRF905_USE_TRANSMITTER
  //Receiver address select 1
  InitIOPin(FT_TRANSMITTER_RX_SELECT_1_PORT, FT_TRANSMITTER_RX_SELECT_1_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
  
  //Receiver address select 2
  InitIOPin(FT_TRANSMITTER_RX_SELECT_2_PORT, FT_TRANSMITTER_RX_SELECT_2_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
#else
#ifdef NRF905_USE_RECEIVER
  //Receiver address select 1
  InitIOPin(FT_RECEIVER_RX_SELECT_1_PORT, FT_RECEIVER_RX_SELECT_1_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
  
  //Receiver address select 2
  InitIOPin(FT_RECEIVER_RX_SELECT_2_PORT, FT_RECEIVER_RX_SELECT_2_PIN, GPIO_MODE_INPUT, GPIO_CONF_I_FLT);
#endif
#endif

  //Setup pull up mode for the three input signals. DR, AM and CD
  //All other signals low
  NRF905_DR_PORT->ODR |= (1 << NRF905_DR_PIN);
  NRF905_AM_PORT->ODR |= (1 << NRF905_AM_PIN);
  NRF905_CD_PORT->ODR |= (1 << NRF905_CD_PIN);
  
  //SPI setup
  //CR1
  //BIDIMODE = 0 (2 line unidirectional communication)
  //BIDIOE = 0   (Not used since 2 line mode selected)
  //CRCEN = 0    (No CRC calculation needed with NRF905)
  //CRCNEXT = 0  (Not used)
  //DFF = 0      (8 bit frames)
  //RXONLY = 0   (Full duplex mode)
  //SSM = 0      (Software slave mode disabled)
  //SSI = 0      (Not used)
  //LSBFIRST = 0 (Most significant bit transmitted first)
  //SPE = 0      (This bit is set when data needs to be send or received. Controls the slave select pin)
  //BR = ???     (Need to determine a baud rate)
  //MSTR = 1     (Master device)
  //CPOL = 0     (Mode 0 selected)
  //CPHA = 0     (Mode 0 selected)
  
  //CR2
  //TXEIE = 0    (No transmit interrupt)
  //RXNEIE = 0   (No receive interrupt)
  //ERRIE = 0    (No error interrupt)
  //SSOE = 1     (Hardware slave control output enabled)
  //TXDMAEN = 0  (No transmit DMA enabled)
  //RXDMAEN = 0  (No receive DMA enabled)
  
  //Setup the NRF905 communication interface for master mode, clock/8 (4.5MHz), rising edge low start clock, 8 bit, msb first out
  //Active low hardware NSS (CSN on NRF905)
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_DIV8;
  SPI1->CR2 = SPI_CR2_SSOE;

  //Turn on the power on the NRF905. (PB13)
  //Keep it in standby mode
  NRF905_PWR_PORT->ODR |= (1 << NRF905_PWR_PIN);
 
  //Wait 3ms for the NRF905 to go in standby
  usdelay(3000);
  
  //Configure the NRF905
  //Use the command for writing the configuration and the preset configuration buffer as data
  nrf905_write_spi(NRF905_WC, nrf905_config_buffer, sizeof(nrf905_config_buffer));
  
#ifdef NRF905_USE_RECEIVER
  //Read the second receiver address select pin to determine which of the two sets needs to be used
  //This is done only at startup so not optimized
  //Per default the jumpers are not soldered and the inputs are read as one
  //So both one means use the first address
  if(readIOPin(FT_RECEIVER_RX_SELECT_2_PORT,FT_RECEIVER_RX_SELECT_2_PIN))
  {
    //Use address 1 or 2
    if(readIOPin(FT_RECEIVER_RX_SELECT_1_PORT,FT_RECEIVER_RX_SELECT_1_PIN))
    {
      //Use address 1
      nrf905_set_rx_address(FT_RECEIVER_1_ADDRESS);
    }
    else
    {
      //Use address 2
      nrf905_set_rx_address(FT_RECEIVER_2_ADDRESS);
    }
  }
  else
  {
    //Use address 3 or 4
    //Read the first receiver address select pin to determine which of the two needs to be used
    if(readIOPin(FT_RECEIVER_RX_SELECT_1_PORT,FT_RECEIVER_RX_SELECT_1_PIN))
    {
      //Use address 3
      nrf905_set_rx_address(FT_RECEIVER_3_ADDRESS);
    }
    else
    {
      //Use address 4
      nrf905_set_rx_address(FT_RECEIVER_4_ADDRESS);
    }
  }
#endif  
}

//----------------------------------------------------------------------------------------------------------------------------------

#ifdef NRF905_USE_TRANSMITTER

//----------------------------------------------------------------------------------------------------------------------------------
//The address are set based on that the jumper input is 1 and the switch input is 0 for the first receiver
//The jumper selects between the two sets (1,2) and (3,4).

const uint32_t nrf905_receiver_adresses[] =
{
  FT_RECEIVER_3_ADDRESS,
  FT_RECEIVER_4_ADDRESS,
  FT_RECEIVER_1_ADDRESS,
  FT_RECEIVER_2_ADDRESS
};

//----------------------------------------------------------------------------------------------------------------------------------
//Select the receiver to be addressed based on the switch and jumper settings

void nrf905_select_receiver(void)
{
  uint8_t needed_rx_address;
  
  needed_rx_address  = readIOPin(FT_TRANSMITTER_RX_SELECT_2_PORT,FT_TRANSMITTER_RX_SELECT_2_PIN) << 1;
  needed_rx_address |= readIOPin(FT_TRANSMITTER_RX_SELECT_1_PORT,FT_TRANSMITTER_RX_SELECT_1_PIN);

  //Check if the selection changed
  if(needed_rx_address != nrf905_current_rx_address)
  {
    //Set the new address identifier for the next check
    nrf905_current_rx_address = needed_rx_address;
    
    //Set the select address in the NRF905
    nrf905_set_tx_address(nrf905_receiver_adresses[needed_rx_address]);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//Is for selecting one of four receiver addresses. The original has only two. (receiver 1 (27,105MHz) or receiver 2 (27,135MHz))

void nrf905_set_tx_address(uint32_t address)
{
  //Switch device into standby mode. Keep the input pull-ups high. TX_EN low and TRX_CE low
  NRF905_TXEN_PORT->ODR &= ~(1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR &= ~(1 << NRF905_CE_PIN);
  
  //Use the command for writing the transmission address and pass the selected address
  nrf905_write_spi(NRF905_WTA, (uint8_t *)&address, 4);
}

//----------------------------------------------------------------------------------------------------------------------------------

#endif

#ifdef NRF905_USE_RECEIVER

//----------------------------------------------------------------------------------------------------------------------------------

void nrf905_set_rx_address(uint32_t address)
{
  //Switch device into standby and receive mode. Keep the input pull-ups high. TX_EN low and TRX_CE low
  NRF905_TXEN_PORT->ODR &= ~(1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR &= ~(1 << NRF905_CE_PIN);
  
  //Use the command for writing the configuration starting on the receiver address and pass the selected address
  nrf905_write_spi(NRF905_WC | 0x05, (uint8_t *)&address, 4);
}

//----------------------------------------------------------------------------------------------------------------------------------

void nrf905_enable_receiver(void)
{
  //Enable the receiver. TX_EN low and TRX_CE high
  NRF905_TXEN_PORT->ODR &= ~(1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR |= (1 << NRF905_CE_PIN);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint32_t nrf905_check_carrier_detect(void)
{
  //Read the status of the carrier detect pin.
  return((NRF905_CD_PORT->IDR & (1 << NRF905_CD_PIN)) == (1 << NRF905_CD_PIN));
}

//----------------------------------------------------------------------------------------------------------------------------------

uint32_t nrf905_check_address_match(void)
{
  //Read the status of the address match pin (bit 1)
  return((NRF905_AM_PORT->IDR & (1 << NRF905_AM_PIN)) == (1 << NRF905_AM_PIN));
}

//----------------------------------------------------------------------------------------------------------------------------------

#endif

//----------------------------------------------------------------------------------------------------------------------------------

uint32_t nrf905_check_data_ready(void)
{
  //Read the status of the data ready pin (bit 0)
  return((NRF905_DR_PORT->IDR & (1 << NRF905_DR_PIN)) == (1 << NRF905_DR_PIN));
}

//----------------------------------------------------------------------------------------------------------------------------------

#ifdef NRF905_USE_TRANSMITTER

//----------------------------------------------------------------------------------------------------------------------------------
//packet needs to point to a buffer of at least FT_MESSAGE_LENGTH bytes

void nrf905_send_packet(uint8_t *packet)
{
  //Switch device into standby mode. Keep the input pull-ups high
  NRF905_TXEN_PORT->ODR &= ~(1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR &= ~(1 << NRF905_CE_PIN);
  
  //Transfer the packet to the NRF905
  nrf905_write_spi(NRF905_WTP, packet, FT_MESSAGE_LENGTH);
  
  //Start the transmission. TRX_CE and TX_EN high
  NRF905_TXEN_PORT->ODR |= (1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR |= (1 << NRF905_CE_PIN);
}

//----------------------------------------------------------------------------------------------------------------------------------

#endif

#ifdef NRF905_USE_RECEIVER

//----------------------------------------------------------------------------------------------------------------------------------
//packet needs to point to a buffer of at least FT_MESSAGE_LENGTH bytes

void nrf905_receive_packet(uint8_t *packet)
{
  //Disable the receiver. Keep the input pull-ups high. TX_EN low and TRX_CE low
  NRF905_TXEN_PORT->ODR &= ~(1 << NRF905_TXEN_PIN);
  NRF905_CE_PORT->ODR &= ~(1 << NRF905_CE_PIN);
  
  //Read the data into the packet buffer by using the read received payload command
  nrf905_read_spi(NRF905_RRP, packet, FT_MESSAGE_LENGTH);
  
  //Enable the receiver. TRX_CE high and TX_EN stays low
  NRF905_CE_PORT->ODR |= (1 << NRF905_CE_PIN);
}

//----------------------------------------------------------------------------------------------------------------------------------

#endif

//----------------------------------------------------------------------------------------------------------------------------------

