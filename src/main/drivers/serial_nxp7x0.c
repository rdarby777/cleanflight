#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"

#include "common/utils.h"
#include "common/atomic.h"

#include "nvic.h"
#include "system.h"
#include "gpio.h"
#include "timer.h"

#include "drivers/bus_i2c.h"

#include "serial.h"
#include "serial_nxp7x0.h"

#include "sc16is7x0.h"
#define nxpi2cRead(addr, chan, reg, n, buf) i2cRead(addr, ((reg) << 3)|((chan) << 1), n, buf)
#define nxpi2cWrite(addr, chan, reg, data) i2cWrite(addr, ((reg) << 3)|((chan) << 1), data)
#define nxpi2cWriteBuffer(addr, chan, reg, n, buf) i2cWriteBuffer(addr, ((reg) << 3)|((chan) << 1), n, buf)

#define nxpRead(nxp, reg, n, buf) nxpi2cRead((nxp)->addr, (nxp)->chan, reg, n, buf)
#define nxpWrite(nxp, reg, data) nxpi2cWrite((nxp)->addr, (nxp)->chan, reg, data)
#define nxpWriteBuffer(nxp, reg, n, buf) nxpi2cWrite((nxp)->addr, (nxp)->chan, reg, n, buf)

#define NXPSERIAL_MAX_RXFRAG 16
#define NXPSERIAL_MAX_TXFRAG 16

typedef struct nxpSerial_s {
    serialPort_t     port;    // Must be the first
    volatile uint8_t rxBuf[NXPSERIAL_BUFFER_SIZE];
    volatile uint8_t txBuf[NXPSERIAL_BUFFER_SIZE];
    uint8_t          nxpSerialPortIndex;

    char             active;
    uint8_t          bustype;
#define NXPSERIAL_BUSTYPE_I2C 0
#define NXPSERIAL_BUSTYPE_SPI 1
    uint8_t          addr;
    uint8_t          chan;
    uint32_t         freq;
} nxpSerial_t;

#define rxBufferLen(port) ((((port).rxBufferHead - (port).rxBufferTail)) & ((port).rxBufferSize - 1))
#define txBufferLen(port) ((((port).txBufferHead - (port).txBufferTail)) & ((port).txBufferSize - 1))
#define rxBufferRoom(port) ((port).rxBufferSize - rxBufferRoom(port) - 1)
#define txBufferRoom(port) ((port).txBufferSize - txBufferRoom(port) - 1)

extern nxpSerial_t nxpSerialPorts[];
extern const struct serialPortVTable nxpSerialVTable[];

#define MAX_NXPSERIAL_PORTS 8

nxpSerial_t nxpSerialPorts[MAX_NXPSERIAL_PORTS];

static void resetBuffers(nxpSerial_t *nxp)
{
    nxp->port.rxBufferSize = NXPSERIAL_BUFFER_SIZE;
    nxp->port.rxBuffer = nxp->rxBuf;
    nxp->port.rxBufferTail = 0;
    nxp->port.rxBufferHead = 0;

    nxp->port.txBufferSize = NXPSERIAL_BUFFER_SIZE;
    nxp->port.txBuffer = nxp->txBuf;
    nxp->port.txBufferTail = 0;
    nxp->port.txBufferHead = 0;
}

static
bool
nxpReset(uint8_t addr, uint8_t chan)
{
    uint8_t ioc, lcr, lsr;

    nxpi2cRead(addr, chan, IS7x0_REG_IOCONTROL, 1, &ioc);
    nxpi2cWrite(addr, chan, IS7x0_REG_IOCONTROL, ioc|(1 << 3));

    delay(10);

    if (!nxpi2cRead(addr, chan, IS7x0_REG_LCR, 1, &lcr)
     || !nxpi2cRead(addr, chan, IS7x0_REG_LSR, 1, &lsr))
        return false;

    if (lcr == 0x1D && (lsr & 0x60) == 0x60)
        return true;

    return false;
}

static
bool
nxpProbe(uint8_t addr, uint8_t chan)
{
    uint8_t lcr, lsr, txlvl, ioc;
    int retry;

    if (!nxpi2cRead(addr, chan, IS7x0_REG_LCR, 1, &lcr)
     || !nxpi2cRead(addr, chan, IS7x0_REG_LSR, 1, &lsr))
        return false;

    if (lcr == 0x1D && (lsr & 0x60) == 0x60)
        return true;

    // Not after reset or unknown chip.
    // Try identifying the chip non-intrusively.

    // Wait until THR and TSR to go empty.

    if (!(lsr & IS7x0_LSR_THRTSREMPTY)) {
            delay(10);
            nxpi2cRead(addr, chan, IS7x0_REG_LSR, 1, &lsr);
            if (!(lsr & IS7x0_LSR_THRTSREMPTY))
                return false;
    }

    // TX is all empty, TX FIFO Level should be 0x40 (64)

    nxpi2cRead(addr, chan, IS7x0_REG_TXLVL, 1, &txlvl);

    if (txlvl == 64)
        return true;

    return false;
}

//#define XTAL_FREQ 3072000 // 3.072MHz
#define XTAL_FREQ 12000000
//#define XTAL_FREQ 12288000
//#define XTAL_FREQ 14745600

#define MAX_BAUDRATE 57600

static
void
nxpSetSpeed(nxpSerial_t *nxp)
{
    uint32_t divisor;
    uint32_t prescaler;
    uint32_t baudrate = nxp->port.baudRate;

    if (baudrate > MAX_BAUDRATE)
        baudrate = MAX_BAUDRATE;

    prescaler= 1;
    divisor = (nxp->freq/prescaler)/(baudrate * 16);

    if (divisor > 65535) {
        prescaler = 4;
        divisor = (nxp->freq/prescaler)/(baudrate * 16);
    }

    uint8_t lcr;
    uint8_t mcr;

    nxpRead(nxp, IS7x0_REG_MCR, 1, &mcr);

    if (prescaler == 1 && (mcr & (IS7x0_MCR_CLKDIV << 7))) {
        mcr &= ~(1 << 7);
        nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    } else if (prescaler == 4 && !(mcr & (IS7x0_MCR_CLKDIV << 7))) {
        mcr |= (1 << 7);
        nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    }

    nxpRead(nxp, IS7x0_REG_LCR, 1, &lcr);
    nxpWrite(nxp, IS7x0_REG_LCR, lcr|IS7x0_LCR_DIVLATEN);

    nxpWrite(nxp, IS7x0_REG_DLL, divisor & 0xff);
    nxpWrite(nxp, IS7x0_REG_DLH, (divisor >> 8) & 0xff);

    nxpWrite(nxp, IS7x0_REG_LCR, lcr);
}

static
void nxpSetLine(nxpSerial_t *nxp)
{
    portOptions_t options = nxp->port.options; 
    uint8_t lcr = IS7x0_LCR_WLEN8;

    if (options & SERIAL_STOPBITS_2)
        lcr |= IS7x0_LCR_STOP2;

    if (options & SERIAL_PARITY_EVEN)
        lcr |= IS7x0_LCR_PAREVEN;

    nxpWrite(nxp, IS7x0_REG_LCR, lcr);
}

void nxpSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    nxp->port.baudRate = baudrate;
    nxpSetSpeed(nxp);
}

nxpEnableEnhancedFunctions(nxpSerial_t *nxp)
{
    uint8_t lcr;
    uint8_t efr;

    nxpRead(nxp, IS7x0_REG_LCR, 1, &lcr);
    nxpWrite(nxp, IS7x0_REG_LCR, 0xbf);

    nxpRead(nxp, IS7x0_REG_EFR, 1, &efr);
    efr |= IS7x0_EFR_ENH;
    nxpWrite(nxp, IS7x0_REG_EFR, efr);

    nxpWrite(nxp, IS7x0_REG_LCR, lcr);
}

nxpFIFOEnable(nxpSerial_t *nxp)
{
    uint8_t fcr;

    nxpRead(nxp, IS7x0_REG_FCR, 1, &fcr);

    fcr |= IS7x0_FCR_TXFRST|IS7x0_FCR_RXFRST|IS7x0_FCR_FIFOEN;

    nxpWrite(nxp, IS7x0_REG_FCR, fcr);
}

serialPort_t *openNXPSerial(
	nxpSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options)
{
    nxpSerial_t *nxp = &nxpSerialPorts[portIndex];

    nxp->active = 0;

    if (portIndex == 0) {
        // Switch Science BOB
        // Should obtain from cli variables
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->addr = 0x4c;
        nxp->chan = 0;
        nxp->freq = 12000000;

        if (!nxpProbe(0x4c, 0) || !nxpReset(0x4c, 0))
	    return NULL;

    } else if (portIndex == 1) {
        // Sparkfun BOB
        // Should obtain from cli variables
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->addr = 0x4c;
        nxp->chan = 0;
        nxp->freq = 14745600;

        return NULL;
    }

    nxp->active = 1;

    nxp->port.vTable = nxpSerialVTable;
    nxp->port.baudRate = baud;
    nxp->port.mode = MODE_RXTX;
    nxp->port.options = options;
    nxp->port.callback = callback;

    resetBuffers(nxp);

    nxpEnableEnhancedFunctions(nxp);
    nxpSetSpeed(nxp);
    nxpSetLine(nxp);
    nxpFIFOEnable(nxp);

    nxp->nxpSerialPortIndex = portIndex;

    return &nxp->port;
}

void nxpSerialWriteByte(serialPort_t *instance, uint8_t ch)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    nxpWrite(nxp, IS7x0_REG_THR, &ch);
}

uint8_t nxpSerialRxBytesWaiting(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    return 0;
}

uint8_t nxpSerialTxBytesFree(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    return 0;
}

uint8_t nxpSerialReadByte(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    return 0;
}

bool nxpSerialTransmitBufferEmpty(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    return false;
}

void nxpSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    nxp->port.mode = mode;

    // Disable tx/rx based on the mode?
}

void nxpSerialPoller(void)
{
    // static int scanport = 0; // Should service all ports: MAX_NXPSERIAL_PORTS
    nxpSerial_t *nxp;
    uint8_t txlvl;
    uint8_t rxlvl;
    int rxroom;
    int rxwrap;
    int rxlen;

    nxp = &nxpSerialPorts[0];


#if 1
    // Test reception
    uint8_t tmpbuf[NXPSERIAL_MAX_RXFRAG];
    nxpRead(nxp, IS7x0_REG_RXLVL, 1, &rxlvl);
    if (rxlvl) {
        rxlen = (rxlvl < NXPSERIAL_MAX_RXFRAG) ? rxlvl : NXPSERIAL_MAX_RXFRAG;
        nxpRead(nxp, IS7x0_REG_RHR, rxlen, tmpbuf);
    }
#else
    // If there's a room in rxBuf, try to read from RX FIFO.
    // Limit single transfer to MIN(rxroom, rxfifo, maxfrag)
    // Actual transfer size will further be limited by the end of phys buf. 

    rxroom = rxBufferRoom(nxp->port);
    rxwrap = nxp->port.rxBufferSize - nxp->port.rxBufferHead;

    if (rxroom) {
        nxpRead(nxp, IS7x0_REG_RXLVL, 1, &rxlvl);
        rxlen = (rxroom < rxlvl) ? rxroom : rxlvl;
        if (rxlen > NXPSERIAL_MAX_RXFRAG)
            rxlen = NXPSERIAL_MAX_RXFRAG;
        if (rxlen > rxwrap)
            rxlen = rxwrap;
        nxpRead(nxp, IS7x0_REG_RHR, rxlen, &(nxp->port.rxBuffer[nxp->port.rxBufferHead]));
    }
#endif
}

const struct serialPortVTable nxpSerialVTable[] = {
    {
        nxpSerialWriteByte,
        nxpSerialRxBytesWaiting,
        nxpSerialTxBytesFree,
        nxpSerialReadByte,
        nxpSerialSetBaudRate,
        nxpSerialTransmitBufferEmpty,
        nxpSerialSetMode,
        .writeBuf = NULL,
    }
};
