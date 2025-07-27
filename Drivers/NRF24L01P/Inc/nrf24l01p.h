#ifndef __NRF24L01P_H
#define __NRF24L01P_H

#include "main.h"
#include <string.h>

// NRF24L01P Register Map
#define CONFIG 0X00
#define EN_AA 0X01
#define EN_RXADDR 0X02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define RPD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

// NRF24L01P Commands
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP 0xFF

typedef enum {
    WAIT_FOR_STARTUP,
    POWER_DOWN_MODE,
    NRF24_STANDBY1,
    NRF24_STANDBY2
} NRF24_State_t;

void NRF24_WriteRegs(uint8_t start_reg, uint8_t* data, uint8_t size);
void NRF24_ReadRegs(uint8_t start_reg, uint8_t* data, uint8_t size);
uint8_t NRF24_ReadReg(uint8_t reg);
void NRF24_WriteReg(uint8_t reg, uint8_t data);
void NRF24_Init(SPI_HandleTypeDef *hspi);
void send_cmd(uint8_t cmd);
void cs_low(void);
void cs_high(void);
void ce_low(void);
void ce_high(void);

#endif