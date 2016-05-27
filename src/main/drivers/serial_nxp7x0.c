
serialPort_t *openNXPSerial(
	nxpSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options)
{
}

void nxpSerialWriteByte(serialPort_t *instance, uint8_t ch)
{
}

uint8_t nxpSerialRxBytesWaiting(serialPort_t *instance)
{
}

uint8_t nxpSerialTxBytesFree(serialPort_t *instance)
{
}

uint8_t nxpSerialReadByte(serialPort_t *instance)
{
}

void nxpSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate)
{
}

bool nxpSerialTransmitBufferEmpty(serialPort_t *instance)
{
}
