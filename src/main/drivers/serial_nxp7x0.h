
#pragma once

#define NXPSERIAL_BUFFER_SIZE 256

typedef enum {
    NXPSERIAL1 = 0,
    NXPSERIAL2,
    NXPSERIAL3,
    NXPSERIAL4,
    NXPSERIAL5,
    NXPSERIAL6,
    NXPSERIAL7,
    NXPSERIAL8,
} nxpSerialPortIndex_e;

serialPort_t *openNXPSerial(
	nxpSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options);

// serialPort API
void nxpSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t nxpSerialRxBytesWaiting(serialPort_t *instance);
uint8_t nxpSerialTxBytesFree(serialPort_t *instance);
uint8_t nxpSerialReadByte(serialPort_t *instance);
void nxpSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate);
bool nxpSerialTransmitBufferEmpty(serialPort_t *instance);

// i2c poller
void nxpSerialPoller(void);
