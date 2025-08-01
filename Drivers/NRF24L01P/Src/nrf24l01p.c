#include "nrf24l01p.h"

// Private function prototypes
static void NRF24_CS_Low(NRF24_t *nrf);
static void NRF24_CS_High(NRF24_t *nrf);
static void NRF24_CE_Low(NRF24_t *nrf);
static void NRF24_CE_High(NRF24_t *nrf);

void NRF24_Init(NRF24_t *nrf, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    nrf->hspi = hspi;
    nrf->ce_port = ce_port;
    nrf->ce_pin = ce_pin;
    nrf->csn_port = csn_port;
    nrf->csn_pin = csn_pin;
    nrf->payload_size = 32; // Default payload size
    nrf->channel = 76;      // Default channel
    nrf->mode = NRF24_MODE_POWER_DOWN;

    NRF24_CE_Low(nrf);
    NRF24_CS_High(nrf);

    HAL_Delay(5);

    // Reset all registers to default
    NRF24_WriteReg(nrf, CONFIG, 0x0C);
    NRF24_WriteReg(nrf, EN_AA, 0x3F);
    NRF24_WriteReg(nrf, EN_RXADDR, 0x03);
    NRF24_WriteReg(nrf, SETUP_AW, 0x03);
    NRF24_WriteReg(nrf, SETUP_RETR, 0x03);
    NRF24_WriteReg(nrf, RF_CH, nrf->channel);
    NRF24_WriteReg(nrf, RF_SETUP, 0x0E);

    // Activate features
    uint8_t cmd = 0x50; // ACTIVATE command
    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
    cmd = 0x73;
    HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
    NRF24_CS_High(nrf);

    // Enable dynamic payload
    NRF24_WriteReg(nrf, FEATURE, (1 << 2));
    NRF24_WriteReg(nrf, DYNPD, 0x3F);

    // Go to Standby-I mode
    uint8_t config = NRF24_ReadReg(nrf, CONFIG);
    config |= (1 << 1);
    NRF24_WriteReg(nrf, CONFIG, config);
    nrf->mode = NRF24_MODE_STANDBY;

    HAL_Delay(2);
}

void NRF24_TXMode(NRF24_t *nrf, uint8_t *address, uint8_t channel) {
    NRF24_CE_Low(nrf);

    nrf->channel = channel;
    NRF24_WriteReg(nrf, RF_CH, nrf->channel);
    NRF24_WriteRegs(nrf, TX_ADDR, address, 5);
    NRF24_WriteRegs(nrf, RX_ADDR_P0, address, 5); // Use same address for ACK

    // Power up and set to TX mode
    uint8_t config = NRF24_ReadReg(nrf, CONFIG);
    config &= ~(1 << 0); // Clear PRIM_RX
    NRF24_WriteReg(nrf, CONFIG, config);
    nrf->mode = NRF24_MODE_TX;

    HAL_Delay(2);
}

void NRF24_RXMode(NRF24_t *nrf, uint8_t *address, uint8_t channel) {
    NRF24_CE_Low(nrf);

    nrf->channel = channel;
    NRF24_WriteReg(nrf, RF_CH, nrf->channel);
    NRF24_WriteRegs(nrf, RX_ADDR_P0, address, 5);

    // Power up and set to RX mode
    uint8_t config = NRF24_ReadReg(nrf, CONFIG);
    config |= (1 << 0); // Set PRIM_RX
    NRF24_WriteReg(nrf, CONFIG, config);
    nrf->mode = NRF24_MODE_RX;

    NRF24_CE_High(nrf);

    HAL_Delay(2);
}

uint8_t NRF24_Transmit(NRF24_t *nrf, uint8_t *data, uint8_t length) {
    NRF24_CE_Low(nrf);

    // Write payload
    uint8_t cmd = W_TX_PAYLOAD;
    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(nrf->hspi, data, length, 100);
    NRF24_CS_High(nrf);

    // Pulse CE to start transmission
    NRF24_CE_High(nrf);
    HAL_Delay(1); // Keep CE high for at least 10us
    NRF24_CE_Low(nrf);

    // Wait for transmission to complete or fail
    uint32_t start_time = HAL_GetTick();
    uint8_t status;
    while (1) {
        status = NRF24_ReadReg(nrf, STATUS);
        if ((status & (1 << 5)) || (status & (1 << 4))) { // TX_DS or MAX_RT
            break;
        }
        if (HAL_GetTick() - start_time > 100) { // Timeout
            return 0;
        }
    }

    // Clear status flags
    NRF24_WriteReg(nrf, STATUS, (1 << 5) | (1 << 4));

    if (status & (1 << 5)) { // TX_DS (Data Sent)
        return 1;
    } else { // MAX_RT (Max Retransmits)
        NRF24_SendCmd(nrf, FLUSH_TX);
        return 0;
    }
}

uint8_t NRF24_Receive(NRF24_t *nrf, uint8_t *data) {
    uint8_t status = NRF24_ReadReg(nrf, STATUS);

    if (status & (1 << 6)) { // RX_DR (Data Ready)
        uint8_t len = NRF24_GetPayloadWidth(nrf);
        if (len > 32) {
            NRF24_SendCmd(nrf, FLUSH_RX);
            NRF24_WriteReg(nrf, STATUS, (1 << 6)); // Clear flag
            return 0; // Error
        }

        // Read payload
        uint8_t cmd = R_RX_PAYLOAD;
        NRF24_CS_Low(nrf);
        HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
        HAL_SPI_Receive(nrf->hspi, data, len, 100);
        NRF24_CS_High(nrf);

        // Clear RX_DR flag
        NRF24_WriteReg(nrf, STATUS, (1 << 6));
        return len;
    }

    return 0; // No data available
}

uint8_t NRF24_GetPayloadWidth(NRF24_t *nrf) {
    uint8_t width;
    uint8_t cmd = R_RX_PL_WID;
    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(nrf->hspi, &width, 1, 100);
    NRF24_CS_High(nrf);
    return width;
}

void NRF24_WriteReg(NRF24_t *nrf, uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = W_REGISTER | (reg & 0x1F);
    buf[1] = data;

    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, buf, 2, 100);
    NRF24_CS_High(nrf);
}

uint8_t NRF24_ReadReg(NRF24_t *nrf, uint8_t reg) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = R_REGISTER | (reg & 0x1F);
    tx_buf[1] = NOP;

    NRF24_CS_Low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, tx_buf, rx_buf, 2, 100);
    NRF24_CS_High(nrf);

    return rx_buf[1];
}

void NRF24_WriteRegs(NRF24_t *nrf, uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];
    tx_buf[0] = W_REGISTER | (start_reg & 0x1F);
    memcpy(&tx_buf[1], data, size);

    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, tx_buf, size + 1, 100);
    NRF24_CS_High(nrf);
}

void NRF24_ReadRegs(NRF24_t *nrf, uint8_t start_reg, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];
    uint8_t rx_buf[size + 1];

    tx_buf[0] = R_REGISTER | (start_reg & 0x1F);
    memset(&tx_buf[1], NOP, size);

    NRF24_CS_Low(nrf);
    HAL_SPI_TransmitReceive(nrf->hspi, tx_buf, rx_buf, size + 1, 100);
    NRF24_CS_High(nrf);

    memcpy(data, &rx_buf[1], size);
}

void NRF24_SendCmd(NRF24_t *nrf, uint8_t cmd) {
    NRF24_CS_Low(nrf);
    HAL_SPI_Transmit(nrf->hspi, &cmd, 1, 100);
    NRF24_CS_High(nrf);
}

static void NRF24_CS_Low(NRF24_t *nrf) {
    HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, GPIO_PIN_RESET);
}

static void NRF24_CS_High(NRF24_t *nrf) {
    HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, GPIO_PIN_SET);
}

static void NRF24_CE_Low(NRF24_t *nrf) {
    HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, GPIO_PIN_RESET);
}

static void NRF24_CE_High(NRF24_t *nrf) {
    HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, GPIO_PIN_SET);
}