//
// Created by rover on 19.07.22.
//

#ifndef HELLO_2_CC1101_H
#define HELLO_2_CC1101_H

#include "libs.h"
#include <main.h>

namespace RF {

// STROBES BLOCK

u8_t SRES =     0x30; // Reset chip
u8_t SFSTXON =  0x31; // Enable and calibrate freq synthesizer
u8_t SXOFF =    0x32; // Turn off crystal oscillator
u8_t SCAL =     0x33; // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
u8_t SRX =      0x34; // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
u8_t STX =      0x35; // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear
u8_t SIDLE =    0x36; // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
u8_t SWOR =     0x38; // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
u8_t SPWD =     0x39; // Enter power down mode when CSn goes high
u8_t SFRX =     0x3A; // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
u8_t SFTX =     0x3B; // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
u8_t SWORRST =  0x3C; // Reset real time clock to Event1 value
u8_t SNOP =     0x3D; // No operation. May be used to get access to the chip status byte 0x3D for read and 0xBD for write

// STATUS BYTE BLOCK

u8_t CHIP_RDY_MASK =    0x80;
u8_t STATE_MASK =       0x70;
u8_t FIFO_AVAIL_MASK =  0x0F; // The number of bytes available in the RX FIFO or free bytes in the TX FIFO

enum EStatus {
    IDLE =              0b000, // IDLE state (Also reported for some transitional states instead of SETTLING or CALIBRATE)
    RX =                0b001, // Receive mode
    TX =                0b010, // Transmit mode
    FSTXON =            0b011, // Fast TX ready
    CALIBRATE =         0b100, // Frequency synthesizer calibration is running
    SETTLING =          0b101, // PLL is settling
    RXFIFO_OVERFLOW =   0b110, // RX FIFO has overflowed. Read out any useful data, then flush the FIFO with SFRX
    TXFIFO_UNDERFLOW =  0b111, // TX FIFO has underflowed. Acknowledge with SFTX
};

// FIFO ACCESS HEADERS

u8_t SINGLE_BYTE_TX_FIFO =      0x3F;
u8_t BURST_ACCESS_TX_FIFO =     0x7F;
u8_t SINGLE_BYTE_RX_FIFO =      0xBF;
u8_t BURST_ACCESS_RX_FIFO =     0xFF;
u8_t MARCSTATE =                0x35 | 0xC0;
u8_t MCSM1 =                    0x17; //0x3F for stay in tx, stay in rx
u8_t READ_FLAG =                0x80;
u8_t WRITE_FLAG =               0x00;
u8_t BURST_FLAG =               0x40;

class Status {
public:
    bool isChipReady;
    EStatus state;
    u8_t available;

    Status() = default;

    explicit Status(u8_t s) {
        isChipReady = (s & RF::CHIP_RDY_MASK) == 0;
        state = EStatus((s & RF::STATE_MASK) >> 4);
        available = s & RF::FIFO_AVAIL_MASK;
    }
};

class Radio {
public:
    Status SendStrobe(u8_t strobe) {
        tx_buffer[0] = strobe;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
        return Status(rx_buffer[0]);
    }

    Status WriteRegister(u8_t address, u8_t value) {
        tx_buffer[0] = address;
        HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 0xFFFF);
        tx_buffer[0] = value;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
        return Status(rx_buffer[0]);
    }

    u8_t ReadRegister(u8_t address) {
        tx_buffer[0] = address | READ_FLAG;
        HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 0xFFFF);
        HAL_SPI_Receive(&hspi1, rx_buffer, 1, 0xFFFF);
        return rx_buffer[0];
    }

    void Reset() {
        tx_buffer[0] = SRES;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
        while (!IsReady()) {}
//        WriteRegister(MCSM1, 0x3E);
        WriteRegister(MCSM1, 0x10);
    }

    bool IsReady() {
        Status status = GetStatus();
        return status.isChipReady;
    }

    Status Send(u8_t cmd) {
        tx_buffer[0] = cmd;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
        return Status(rx_buffer[0]);
    }

    u8_t Receive() {
        HAL_SPI_Receive(&hspi1, rx_buffer, 1, 0xFFFF);
        return rx_buffer[0];
    }

    void CsLow() {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }

    void CsHigh() {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }

    Status WriteTxFifo(u8_t byte) {
        Send(RF::SINGLE_BYTE_TX_FIFO);
        return Send(byte);
    }

    void EnterTxMode() {
        tx_buffer[0] = STX;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
    }

    void EnterRxMode() {
        tx_buffer[0] = SRX;
        HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 0xFFFF);
    }

    Status GetStatus() {
        tx_buffer[0] = SNOP | READ_FLAG;
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 1, 0xFFFF);
        return Status(rx_buffer[0]);
    }
private:
    u8_t tx_buffer[1];
    u8_t rx_buffer[1];
};

} // namespace RF


#endif //HELLO_2_CC1101_H
