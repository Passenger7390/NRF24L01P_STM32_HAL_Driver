#include "nrf24l01p.h"

extern UART_HandleTypeDef huart2;

static SPI_HandleTypeDef *nrf24_spi;
static GPIO_TypeDef *CE_Port;
static uint16_t CE_Pin;
static GPIO_TypeDef *CSN_Port;
static uint16_t CSN_Pin;

static NRF24_State_t nrf24_state = POWER_DOWN_MODE;

void NRF24_RXMode(uint8_t *address, uint8_t channel) {
    ce_low();

    NRF24_WriteReg(RF_CH, channel);
    NRF24_WriteRegs(RX_ADDR_P0, address, 5);
    NRF24_WriteReg(RX_PW_P0, 32);
    
    uint8_t config = NRF24_ReadReg(CONFIG);
    config |= (1 << 1);
    config |= (1 << 0);
    NRF24_WriteReg(CONFIG, config);

    ce_high();

    HAL_Delay(2);

    nrf24_state = RX_MODE;
}

// TODO: add address to the argument, 
uint8_t NRF24_Transmit(uint8_t *address, uint8_t *data) {
    // It returns 1 if the transmission is successful, and 0 otherwise.

    // Set the CE pin to low to start the transmission.
    ce_low();

    uint8_t cmd = W_TX_PAYLOAD;
    cs_low();
    HAL_SPI_Transmit(nrf24_spi, &cmd, 1, 100);
    HAL_SPI_Transmit(nrf24_spi, data, 32, 100);
    cs_high();

    // Set the CE pin to high to start the transmission.
    ce_high();

    HAL_Delay(1);
    ce_low(); // Return CE to low after transmission starts

    // Wait for the transmission to complete or time out.
    uint32_t start_time = HAL_GetTick();
    uint8_t status;
    while (1) {
        status = NRF24_ReadReg(STATUS);
        if ((status & (1 << 5)) || (status & (1 << 4))) {
            // Data sent (TX_DS) or max retries (MAX_RT) reached.
            break;
        }
        if (HAL_GetTick() - start_time > 100) {
            // Timeout
            return 0; // Indicate failure
        }
    }

    // Clear the status bits.
    NRF24_WriteReg(STATUS, (1 << 5) | (1 << 4));

    // Flush the TX FIFO if not empty.
    uint8_t fifo_status = NRF24_ReadReg(FIFO_STATUS);
    if ((fifo_status & (1 << 4)) == 0) {
        send_cmd(FLUSH_TX);
    }

    // Return 1 if the data was sent successfully (TX_DS is set), and 0 otherwise.
    if (status & (1 << 5)) {
        return 1; // Success
    } else {
        return 0; // Failure (MAX_RT)
    }
}

void NRF24_TXMode(uint8_t *address, uint8_t channel) {
    ce_low();

    NRF24_WriteReg(RF_CH, channel);
    NRF24_WriteRegs(TX_ADDR, address, 5);
    NRF24_WriteRegs(RX_ADDR_P0, address, 5);

    uint8_t config = NRF24_ReadReg(CONFIG);
    config |= (1 << 1);
    NRF24_WriteReg(CONFIG, config);

    HAL_Delay(2);

    nrf24_state = TX_MODE;
}

uint8_t NRF24_Receive(uint8_t *data) {
    uint8_t status = NRF24_ReadReg(STATUS);

    if ((status & (1 << 6))) {
        uint8_t cmd = R_RX_PAYLOAD;
        cs_low();
        HAL_SPI_Transmit(nrf24_spi, &cmd, 1, 100);
        HAL_SPI_Receive(nrf24_spi, data, 32, 100);
        cs_high();

        NRF24_WriteReg(STATUS, (1 << 6));
        return 1;
    }

    return 0;
}

void NRF24_WriteRegs(uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];

    tx_buf[0] = W_REGISTER | (start_reg & 0x1F);

    // Copy the data to be written into the transmit buffer.
    memcpy(&tx_buf[1], data, size);

    cs_low();

    HAL_SPI_Transmit(nrf24_spi, tx_buf, size + 1, 100);

    cs_high();
}

void NRF24_ReadRegs(uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];
    uint8_t rx_buf[size + 1];

    tx_buf[0] = R_REGISTER | (start_reg & 0x1F);

    // The rest of the bytes are dummy bytes to clock out the data.
    memset(&tx_buf[1], NOP, size);

    cs_low();

    HAL_SPI_TransmitReceive(nrf24_spi, tx_buf, rx_buf, size + 1, 100);

    cs_high();

    memcpy(data, &rx_buf[1], size);
}

uint8_t NRF24_ReadReg(uint8_t reg) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = R_REGISTER | (reg & 0x1F);
    tx_buf[1] = NOP;

    cs_low();

    HAL_SPI_TransmitReceive(nrf24_spi, tx_buf, rx_buf, 2, 100);

    cs_high();

    return rx_buf[1];
}

void NRF24_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = W_REGISTER | (reg & 0x1F);
    buf[1] = data;

    cs_low();

    HAL_SPI_Transmit(nrf24_spi, buf, 2, 100);

    cs_high();
}

void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    nrf24_spi = hspi;
    CE_Port = ce_port;
    CE_Pin = ce_pin;
    CSN_Port = csn_port;
    CSN_Pin = csn_pin;
    nrf24_spi = hspi;

    ce_low();

    NRF24_WriteReg(EN_AA, 0x3F);
    NRF24_WriteReg(EN_RXADDR, 0x03);
    NRF24_WriteReg(SETUP_AW, 0x03);
    NRF24_WriteReg(SETUP_RETR, 0x03);
    NRF24_WriteReg(RF_CH, 0x02);
    NRF24_WriteReg(RF_SETUP, 0x0E);
    NRF24_WriteReg(CONFIG, 0x08);

    HAL_Delay(2);

    ce_high();
    nrf24_state = NRF24_STANDBY1;
}

void send_cmd(uint8_t cmd) {
    cs_low();

    HAL_SPI_Transmit(nrf24_spi, &cmd, 1, 100);

    cs_high();
}

void cs_low(void) {
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
}

void cs_high(void) {
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

void ce_low(void) {
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_RESET);
}

void ce_high(void) {
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_SET);
}