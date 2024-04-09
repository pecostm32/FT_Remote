//----------------------------------------------------------------------------------------------------------------------------------

#ifndef FISCHERTECHNIK_REMOTE_CONTROL_H
#define FISCHERTECHNIK_REMOTE_CONTROL_H

//----------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>

//----------------------------------------------------------------------------------------------------------------------------------

typedef struct tagFTMessage   FTMESSAGE, *pFTMESSAGE;

//----------------------------------------------------------------------------------------------------------------------------------

struct __attribute__((packed)) tagFTMessage
{
  uint16_t preamble;
  uint8_t  channel1;
  uint8_t  channel2;
  uint8_t  channel3;
  uint8_t  channel4;
  uint16_t crc;
};

//----------------------------------------------------------------------------------------------------------------------------------
//Transmitter and receiver addresses are made up with ASCII characters to get a good discrimination between the lot

#define FT_RECEIVER_1_ADDRESS        0x314F6E65            //1One
#define FT_RECEIVER_2_ADDRESS        0x3254776F            //2Two
#define FT_RECEIVER_3_ADDRESS        0x33546872            //3Thr
#define FT_RECEIVER_4_ADDRESS        0x34466F75            //4Fou

#define FT_TRANSMITTER_ADDRESS       0x46745458            //FtTX

//----------------------------------------------------------------------------------------------------------------------------------
//Port and pin defines for the two receiver address select inputs on the receiver board

#define FT_RECEIVER_RX_SELECT_1_PORT    GPIOB
#define FT_RECEIVER_RX_SELECT_1_PIN     14

#define FT_RECEIVER_RX_SELECT_2_PORT    GPIOB
#define FT_RECEIVER_RX_SELECT_2_PIN     15

//----------------------------------------------------------------------------------------------------------------------------------
//Port and pin defines for the two receiver address select inputs on the transmitter board

#define FT_TRANSMITTER_RX_SELECT_1_PORT    GPIOB
#define FT_TRANSMITTER_RX_SELECT_1_PIN     6

#define FT_TRANSMITTER_RX_SELECT_2_PORT    GPIOB
#define FT_TRANSMITTER_RX_SELECT_2_PIN     7

//----------------------------------------------------------------------------------------------------------------------------------

#define FT_MESSAGE_LENGTH             8    //sizeof(tagFTMessage)

#define FT_DATA_LENGTH                (FT_MESSAGE_LENGTH - sizeof(uint16_t))   //CRC is not part of the data

//----------------------------------------------------------------------------------------------------------------------------------

#endif //FISCHERTECHNIK_REMOTE_CONTROL_H

//----------------------------------------------------------------------------------------------------------------------------------
