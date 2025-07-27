#include "nrf24l01p.h"

#define NRF24_CSN_Port   GPIOC
#define NRF24_CSN_Pin    GPIO_PIN_0

#define NRF24_CE_Port   GPIOC
#define NRF24_CE_Pin    GPIO_PIN_1

extern UART_HandleTypeDef huart2;
static SPI_HandleTypeDef *nrf24_spi;

static NRF24_State_t nrf24_state = POWER_DOWN_MODE;

uint8_t NRF24_Transmit(uint8_t *data) {
    ce_low();
    uint8_t cmd = W_TX_PAYLOAD;

    cs_low();

    HAL_SPI_Transmit(nrf24_spi, &cmd, 1, 100);
    HAL_SPI_Transmit(nrf24_spi, data, 32, 100);

    cs_high();
    ce_high();
    HAL_Delay(1);
    // Wait for the transmission to complete or time out
    uint32_t start_time = HAL_GetTick();
    uint8_t status;
    uint8_t fifo_status;
    while (1) {
        
        status = NRF24_ReadReg(STATUS);
        if ((status & (1 << 5)) || (status & (1 << 4))) {
            // Data sent or max retries reached
            break;
        }
        if (HAL_GetTick() - start_time > 100) {
            // Timeout
            return 0;
        }
    }

    // Clear the status bits
    NRF24_WriteReg(STATUS, (1 << 5) | (1 << 4));

    fifo_status = NRF24_ReadReg(FIFO_STATUS);
    if (fifo_status & (1 << 4) == 0) {
        send_cmd(FLUSH_TX);
    }

    // Check if data was sent successfully
    if (status & (1 << 5)) {
        return 1;
    } else {
        return 0;
    }
}

void NRF24_TXMode(uint8_t *address, uint8_t channel) {
    ce_low();

    NRF24_WriteReg(RF_CH, channel);
    NRF24_WriteRegs(TX_ADDR, address, 5);

    uint8_t config = NRF24_ReadReg(CONFIG);
    config |= (1 << 1);
    NRF24_WriteReg(CONFIG, config);

    HAL_Delay(2);

    nrf24_state = NRF24_STANDBY1;
}

void NRF24_WriteRegs(uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];

    // The first byte is the W_REGISTER command (0x20) OR'd with the register address.
    tx_buf[0] = W_REGISTER | (start_reg & 0x1F);

    // Copy the data to be written into the transmit buffer.
    memcpy(&tx_buf[1], data, size);

    // Select the device
    cs_low();

    // Transmit the command and data.
    HAL_SPI_Transmit(nrf24_spi, tx_buf, size + 1, 100);

    // Deselect the device
    cs_high();
}

void NRF24_ReadRegs(uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];
    uint8_t rx_buf[size + 1];

    tx_buf[0] = R_REGISTER | (start_reg & 0x1F);

    // The rest of the bytes are dummy bytes to clock out the data.
    memset(&tx_buf[1], NOP, size);

    cs_low();

    // Perform a simultaneous transmit and receive.
    // The STATUS register will be in rx_buf[0].
    // The requested register's data will be in rx_buf[1] onwards.
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

void NRF24_Init(SPI_HandleTypeDef *hspi) {
    nrf24_spi = hspi;

    ce_low();

    NRF24_WriteReg(EN_AA, 0x00);
    NRF24_WriteReg(EN_RXADDR, 0x03);
    NRF24_WriteReg(SETUP_AW, 0x03);
    NRF24_WriteReg(SETUP_RETR, 0x00);
    NRF24_WriteReg(RF_CH, 0x02);
    NRF24_WriteReg(RF_SETUP, 0x0E);
    NRF24_WriteReg(CONFIG, 0x08);

    HAL_Delay(2);

    ce_high();

}

void send_cmd(uint8_t cmd) {
    cs_low();

    HAL_SPI_Transmit(nrf24_spi, &cmd, 1, 100);

    cs_high();
}

void cs_low(void) {
    HAL_GPIO_WritePin(NRF24_CSN_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
}

void cs_high(void) {
    HAL_GPIO_WritePin(NRF24_CSN_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
}

void ce_low(void) {
    HAL_GPIO_WritePin(NRF24_CE_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
}

void ce_high(void) {
    HAL_GPIO_WritePin(NRF24_CE_Port, NRF24_CE_Pin, GPIO_PIN_SET);
}