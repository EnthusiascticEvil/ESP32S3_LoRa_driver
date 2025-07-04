#pragma once

// Команды SX126x
#define SX126X_CMD_CLEAR_IRQ_STATUS      0x02
#define SX126X_CMD_GET_IRQ_STATUS        0x12
#define SX126X_CMD_SET_DIO_IRQ_PARAMS    0x08
#define SX126X_CMD_SET_RX                0x82
#define SX126X_CMD_READ_BUFFER           0x1E
#define SX126X_CMD_GET_RX_BUFFER_STATUS  0x13

// IRQ-флаги
#define SX126X_IRQ_TX_DONE               0x0001
#define SX126X_IRQ_RX_DONE               0x0002
#define SX126X_IRQ_PREAMBLE_DETECTED     0x0004
#define SX126X_IRQ_SYNCWORD_VALID        0x0008
#define SX126X_IRQ_HEADER_VALID          0x0010
#define SX126X_IRQ_HEADER_ERR            0x0020
#define SX126X_IRQ_CRC_ERR               0x0040
#define SX126X_IRQ_CAD_DONE              0x0080
#define SX126X_IRQ_CAD_DETECTED          0x0100
#define SX126X_IRQ_TIMEOUT               0x0200
#define SX126X_IRQ_ALL                   0xFFFF

// Константы, если надо
#define SX126X_BUFFER_SIZE               256
