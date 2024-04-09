//----------------------------------------------------------------------------------------------------------------------------------

#ifndef STM32F103_NRF905_h
#define STM32F103_NRF905_h

//----------------------------------------------------------------------------------------------------------------------------------

//Connections between NRF905 module and STM32F103 on both PCB's
//
//The SPI interface is the same for both boards and is fixed in the code
// PA4  - CSN   (Output, chip select)
// PA5  - SCK   (Output, serial clock)
// PA6  - MISO  (Input, master in, slave out)
// PA7  - MOSI  (Output, master out, slave in)
//
//The control signals differ between the two boards for ease of layout

//----------------------------------------------------------------------------------------------------------------------------------

#ifdef FT_TRANSMITTER

//Connections between NRF905 module and STM32F103 on the transmitter PCB
// PB0  - DR    (Input, data ready)
// PB1  - AM    (Input, address match)
// PB10 - CD    (Input, carrier detect)
// PB11 - CE    (Output, chip enable)
// PB12 - TXEN  (Output, transmit enable)
// PB13 - PWR   (Output, power enable)

#define NRF905_DR_PORT          GPIOB
#define NRF905_DR_PIN           0

#define NRF905_AM_PORT          GPIOB
#define NRF905_AM_PIN           1

#define NRF905_CD_PORT          GPIOB
#define NRF905_CD_PIN           10

#define NRF905_CE_PORT          GPIOB
#define NRF905_CE_PIN           11

#define NRF905_TXEN_PORT        GPIOB
#define NRF905_TXEN_PIN         12

#define NRF905_PWR_PORT         GPIOB
#define NRF905_PWR_PIN          13

//Signal the code to setup for use in the transmitter
#define NRF905_USE_TRANSMITTER

#else

//----------------------------------------------------------------------------------------------------------------------------------

#ifdef FT_RECEIVER

//Connections between NRF905 module and STM32F103 on the receiver PCB
// PB0  - DR    (Input, data ready)
// PB1  - AM    (Input, address match)
// PB10 - CD    (Input, carrier detect)
// PB12 - CE    (Output, chip enable)
// PB13 - TXEN  (Output, transmit enable)
// PB11 - PWR   (Output, power enable)

#define NRF905_DR_PORT          GPIOB
#define NRF905_DR_PIN           0

#define NRF905_AM_PORT          GPIOB
#define NRF905_AM_PIN           1

#define NRF905_CD_PORT          GPIOB
#define NRF905_CD_PIN           10

#define NRF905_CE_PORT          GPIOB
#define NRF905_CE_PIN           12

#define NRF905_TXEN_PORT        GPIOB
#define NRF905_TXEN_PIN         13

#define NRF905_PWR_PORT         GPIOB
#define NRF905_PWR_PIN          11

//Signal the code to setup for use in the receiver
#define NRF905_USE_RECEIVER
#endif
#endif

//----------------------------------------------------------------------------------------------------------------------------------

void nrf905_init(void);

void nrf905_select_receiver(void);

void nrf905_set_tx_address(uint32_t address);
void nrf905_set_rx_address(uint32_t address);

uint32_t nrf905_check_carrier_detect(void);
uint32_t nrf905_check_address_match(void);
uint32_t nrf905_check_data_ready(void);

void nrf905_enable_receiver(void);

void nrf905_send_packet(uint8_t *packet);

void nrf905_receive_packet(uint8_t *packet);


//----------------------------------------------------------------------------------------------------------------------------------

#endif  //STM32F103_NRF905_h

//----------------------------------------------------------------------------------------------------------------------------------
