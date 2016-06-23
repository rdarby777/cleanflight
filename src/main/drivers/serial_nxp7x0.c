#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/utils.h"
#include "common/atomic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "nvic.h"
//#include "timer.h"

#include "drivers/bus_i2c.h"

#include "serial.h"
#include "serial_nxp7x0.h"

#include "sc16is7x0.h"

#define nxpi2cRead(addr, chan, reg, n, buf) \
            i2cRead(addr, ((reg) << 3)|((chan) << 1), n, buf)
#define nxpi2cWrite(addr, chan, reg, data) \
            i2cWrite(addr, ((reg) << 3)|((chan) << 1), data)
#define nxpi2cWriteBuffer(addr, chan, reg, n, buf) \
            i2cWriteBuffer(addr, ((reg) << 3)|((chan) << 1), n, buf)

#define nxpRead(nxp, reg, n, buf) \
            nxpi2cRead((nxp)->addr, (nxp)->chan, reg, n, buf)
#define nxpWrite(nxp, reg, data) \
            nxpi2cWrite((nxp)->addr, (nxp)->chan, reg, data)
#define nxpWriteBuffer(nxp, reg, n, buf) \
            nxpi2cWriteBuffer((nxp)->addr, (nxp)->chan, reg, n, buf)

#define NXPSERIAL_MAX_RXFRAG 16
#define NXPSERIAL_MAX_TXFRAG 16 

int max_rxfrag = NXPSERIAL_MAX_RXFRAG;
int max_txfrag = NXPSERIAL_MAX_TXFRAG;

// 16B at 100Hz service interval = 16*100 = 1.6KB/sec = 16kbps
// On 800KHz bus, 1KHz slot is 800 bits = 88 bytes
// 0x68 read = addr68+reg43+addr+6datar = 9 bytes
// 0x68 read = addr68+reg3b+addr+6datar = 9 bytes
 
typedef struct nxpSerial_s {
    serialPort_t     port;    // Must be the first
    volatile uint8_t rxBuf[NXPSERIAL_BUFFER_SIZE];
    volatile uint8_t txBuf[NXPSERIAL_BUFFER_SIZE];
    uint8_t          nxpSerialPortIndex;

    bool             active;
    uint8_t          bustype;
#define NXPSERIAL_BUSTYPE_I2C 0
#define NXPSERIAL_BUSTYPE_SPI 1
    uint8_t          addr;
    uint8_t          chan;
    uint32_t         freq;

    uint8_t          rxlvl;
    uint8_t          txlvl;

    uint8_t          iir;       // Last IIR read
    uint8_t          fcr;       // FCR soft copy
    uint8_t          efcr;      // EFCR soft copy
} nxpSerial_t;

extern nxpSerial_t nxpSerialPorts[];
extern const struct serialPortVTable nxpSerialVTable[];

#define MAX_NXPSERIAL_PORTS 4

nxpSerial_t nxpSerialPorts[MAX_NXPSERIAL_PORTS] = {
    { .active = false, },
    { .active = false, },
    { .active = false, },
    { .active = false, },
};

#define rxBufferLen(port) ((((port).rxBufferHead - (port).rxBufferTail)) & ((port).rxBufferSize - 1))
#define txBufferLen(port) ((((port).txBufferHead - (port).txBufferTail)) & ((port).txBufferSize - 1))

#define rxBufferRoom(port) ((port).rxBufferSize - rxBufferLen(port) - 1)
#define txBufferRoom(port) ((port).txBufferSize - txBufferLen(port) - 1)

#define rxBufferBurstLimit(port) ((port).rxBufferSize - (port).rxBufferHead)
#define txBufferBurstLimit(port) ((port).txBufferSize - (port).txBufferTail)


// SPRF3
#if 1
// RC2 (BLUE) = PA1 : Conflict with PB1 (RC8 Sonar Echo)
#define I2CSERIAL_INT_PERIPH          RCC_AHBPeriph_GPIOA
#define I2CSERIAL_INT_PIN             Pin_1
#define I2CSERIAL_INT_GPIO            GPIOA
#define I2CSERIAL_INT_EXTI_LINE       EXTI_Line1
#define I2CSERIAL_INT_EXTI_PIN_SOURCE EXTI_PinSource1
#define I2CSERIAL_INT_IRQN            EXTI1_IRQn
#endif

#if 0
// RC5 (YELLOW) = PB4 : PA4 = VBAT ADC (conflict? check ADC config!)
#define I2CSERIAL_INT_PERIPH          RCC_AHBPeriph_GPIOB
#define I2CSERIAL_INT_PIN             Pin_4
#define I2CSERIAL_INT_GPIO            GPIOB
#define I2CSERIAL_INT_EXTI_LINE       EXTI_Line4
#define I2CSERIAL_INT_EXTI_PIN_SOURCE EXTI_PinSource4
#define I2CSERIAL_INT_IRQN            EXTI4_IRQn
#endif

// RC6 (GREEN) = PB5 : PA5 = CURRENT ADC (conflict? check ADC config!)

extiConfig_t nxpIntExtiConfig = {
    .gpioAHBPeripherals	 = RCC_AHBPeriph_GPIOB,
    //.gpioAPB2Peripherals // STM32F103 only, uint32_t

    .gpioPort            = I2CSERIAL_INT_GPIO,
    .gpioPin             = I2CSERIAL_INT_PIN,

    .exti_port_source    = I2CSERIAL_INT_GPIO,
    .exti_pin_source     = I2CSERIAL_INT_EXTI_PIN_SOURCE,
    .exti_line           = I2CSERIAL_INT_EXTI_LINE,
    .exti_irqn           = I2CSERIAL_INT_IRQN,
};

volatile bool nxpInterrupted = false;

void nxpSerial_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(nxpIntExtiConfig.exti_line) == RESET) {
        return;
    }

    // Debugging
    digitalHi(GPIOB, Pin_5);

    EXTI_ClearITPendingBit(nxpIntExtiConfig.exti_line);

    nxpInterrupted = true;

    // Debugging
    digitalLo(GPIOB, Pin_5);
}

void nxpSerialConfigureEXTI(void)
{
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(nxpIntExtiConfig.exti_port_source, nxpIntExtiConfig.exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(nxpIntExtiConfig.exti_port_source, nxpIntExtiConfig.exti_pin_source);
#endif

#if 0
    // Test GPIO status
    uint8_t status = GPIO_ReadInputDataBit(nxpIntExtiConfig.gpioPort, nxpIntExtiConfig.gpioPin);
    if (status) {
        return;
    }
#endif

    registerExtiCallbackHandler(nxpIntExtiConfig.exti_irqn, nxpSerial_EXTI_Handler);

    EXTI_ClearITPendingBit(nxpIntExtiConfig.exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = nxpIntExtiConfig.exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = nxpIntExtiConfig.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_SERIAL);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_SERIAL);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void nxpSerialIntExtiInit(void)
{
    gpio_config_t gpio;

    static bool nxpExtiInitDone = false;

    //if (nxpExtiInitDone || !nxpIntExtiConfig) {
    if (nxpExtiInitDone) {
        return;
    }

#ifdef STM32F10X
        if (nxpIntExtiConfig.gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(nxpIntExtiConfig.gpioAPB2Peripherals, ENABLE);
        }
#endif
#ifdef STM32F303
        if (nxpIntExtiConfig.gpioAHBPeripherals) {
            RCC_AHBPeriphClockCmd(nxpIntExtiConfig.gpioAHBPeripherals, ENABLE);
        }
#endif
// Interrupt monitoring RC5 (PB4)
{
gpio_config_t LEDgpio;
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
LEDgpio.pin = Pin_4;
LEDgpio.speed = Speed_2MHz;
LEDgpio.mode = Mode_Out_PP;
gpioInit(GPIOB, &LEDgpio);
digitalLo(GPIOB, Pin_4);

gpio_config_t ZEDgpio;
ZEDgpio.pin = Pin_5;
ZEDgpio.speed = Speed_2MHz;
ZEDgpio.mode = Mode_Out_PP;
gpioInit(GPIOB, &ZEDgpio);
digitalLo(GPIOB, Pin_5);
}

    gpio.pin = nxpIntExtiConfig.gpioPin;
    gpio.speed = Speed_2MHz;
    //gpio.mode = Mode_IN_FLOATING;
    //gpio.mode = Mode_Out_PP; // For port connectivity testing
    gpio.mode = Mode_IPU; // Input with pull-up
    gpioInit(nxpIntExtiConfig.gpioPort, &gpio);

    nxpSerialConfigureEXTI();

    nxpExtiInitDone = true;
}

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
    uint8_t lcr, lsr;
    uint8_t txlvl;

    if (!nxpi2cRead(addr, chan, IS7x0_REG_LCR, 1, &lcr)
     || !nxpi2cRead(addr, chan, IS7x0_REG_LSR, 1, &lsr))
        return false;

    if (lcr == 0x1D && (lsr & 0x60) == 0x60)
        return true;

    // Not after reset or unknown chip.
    // Try identifying the chip non-intrusively.

    // Wait until THR and TSR to go empty.

    if (!(lsr & IS7x0_LSR_TXEMPTY)) {
            delay(10);
            nxpi2cRead(addr, chan, IS7x0_REG_LSR, 1, &lsr);
            if (!(lsr & IS7x0_LSR_TXEMPTY))
                return false;
    }

    // TX is all empty, TX FIFO Level should be 0x40 (64)

    nxpi2cRead(addr, chan, IS7x0_REG_TXLVL, 1, &txlvl);

    if (txlvl == 64)
        return true;

    return false;
}

#define MAX_BAUDRATE 57600

static void nxpSetSpeed(nxpSerial_t *nxp)
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

    if (prescaler == 1 && (mcr & IS7x0_MCR_CLKDIV)) {
        mcr &= ~(1 << 7);
        nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    }

    if (prescaler == 4 && !(mcr & IS7x0_MCR_CLKDIV)) {
        mcr |= (1 << 7);
        nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    }

    nxpRead(nxp, IS7x0_REG_LCR, 1, &lcr);
    nxpWrite(nxp, IS7x0_REG_LCR, lcr|IS7x0_LCR_DIVLATEN);

    nxpWrite(nxp, IS7x0_REG_DLL, divisor & 0xff);
    nxpWrite(nxp, IS7x0_REG_DLH, (divisor >> 8) & 0xff);

    nxpWrite(nxp, IS7x0_REG_LCR, lcr);
}

static void nxpSetLine(nxpSerial_t *nxp)
{
    portOptions_t options = nxp->port.options; 
    uint8_t lcr = IS7x0_LCR_WLEN8;

    if (options & SERIAL_STOPBITS_2)
        lcr |= IS7x0_LCR_STOP2;

    if (options & SERIAL_PARITY_EVEN)
        lcr |= IS7x0_LCR_PAREVEN;

    nxpWrite(nxp, IS7x0_REG_LCR, lcr);
}

static void nxpTransmitterControl(nxpSerial_t *nxp, bool enable)
{
    if (enable)
        nxp->efcr &= ~IS7x0_EFCR_TXDISABLE;
    else
        nxp->efcr |= IS7x0_EFCR_TXDISABLE;

    nxpWrite(nxp, IS7x0_REG_EFCR, nxp->efcr);
}

static void nxpReceiverControl(nxpSerial_t *nxp, bool enable)
{
    if (enable)
        nxp->efcr &= ~IS7x0_EFCR_RXDISABLE;
    else
        nxp->efcr |= IS7x0_EFCR_RXDISABLE;

    nxpWrite(nxp, IS7x0_REG_EFCR, nxp->efcr);
}

static void nxpEnableEnhancedFunctions(nxpSerial_t *nxp)
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

static void nxpFIFOEnable(nxpSerial_t *nxp)
{
    uint8_t fcr;

    nxpRead(nxp, IS7x0_REG_FCR, 1, &fcr);

    fcr |= IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST|IS7x0_FCR_FIFO_EN;

    nxpWrite(nxp, IS7x0_REG_FCR, fcr);

    nxp->fcr = fcr;
}

static void nxpSetTriggerLevel(nxpSerial_t *nxp)
{
    uint8_t fcr;
    uint8_t mcr;
    uint8_t tlr;

    nxpRead(nxp, IS7x0_REG_FCR, 1, &fcr);
    fcr = IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST; // RX and TX triggers to zero, reset both fifos, disable FIFO.
    nxpWrite(nxp, IS7x0_REG_FCR, fcr);

    nxpRead(nxp, IS7x0_REG_MCR, 1, &mcr);
    mcr |= IS7x0_MCR_TCRTLR_EN;
    nxpWrite(nxp, IS7x0_REG_MCR, mcr);

    tlr = (1 << IS7x0_TLR_RX_SFT)|(((NXPSERIAL_MAX_TXFRAG + 1) / 4) << IS7x0_TLR_TX_SFT);
    nxpWrite(nxp, IS7x0_REG_TLR, tlr);

#if 0 // Done in nxpFIFOEnable()
    fcr = IS7x0_FCR_FIFO_EN;
    nxpWrite(nxp, IS7x0_REG_FCR, fcr);
#endif
}

static void nxpEnableInterrupt(nxpSerial_t *nxp)
{
    uint8_t ier;

    ier = IS7x0_IER_RHR;
    //ier |= IS7x0_IER_THR;
    //ier |= IS7x0_IER_CTS;
    //ier |= IS7x0_IER_RTS;
    //ier |= IS7x0_IER_MODEM;
    //ier |= IS7x0_IER_LINE;

    nxpWrite(nxp, IS7x0_REG_IER, ier);
}

#if 0
// Experimenting exti (RC2 = PA1)
EXTI_INIT()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    // EXTI line
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    registerExtiCallbackHandler(EXTI1_IRQn, nxpSerial_EXTI_Handler);

    // Set priotity
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}
#endif

serialPort_t *openNXPSerial(
	nxpSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options)
{
    nxpSerial_t *nxp = &nxpSerialPorts[portIndex];

    // nxp->active = false; // It's in the initializer

    // Device tree examples:
    //   twserial0 at i2c1 addr 0x4c chan 0 type nxp750 irq 4
    //   twserial1 at i2c1 addr 0x4d chan 0 type nxp750 irq 4
    //   twserial2 at i2c1 addr 0x4e chan 0 type nxp762 irq 4
    //   twserial3 at i2c1 addr 0x4e chan 1 type nxp762 irq 4
    //   twserial4 at i2c1 addr 0x4f chan 0 type twub irq 10
    //   twserial4 at i2c1 addr 0x50 chan 0 type twub irq 10

    if (portIndex == 1) {
        // Switch Science BOB
        // Should obtain from cli variables
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->addr = 0x4c;
        nxp->chan = 0;
        nxp->freq = 12000000;
    } else if (portIndex == 0) {
        // Sparkfun BOB
        // Should obtain from cli variables
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->addr = 0x4d;
        nxp->chan = 0;
        nxp->freq = 14745600;
    }

    if (!nxpProbe(nxp->addr, nxp->chan) || !nxpReset(nxp->addr, nxp->chan))
        return NULL;

    nxp->port.vTable = nxpSerialVTable;
    nxp->port.baudRate = baud;
    nxp->port.mode = MODE_RXTX;
    nxp->port.options = options;
    nxp->port.callback = callback;
    nxp->nxpSerialPortIndex = portIndex;

    resetBuffers(nxp);

    nxp->rxlvl = 0;
    nxp->txlvl = 0; // Actual value will be read for the 1st TX data

    nxpWrite(nxp, IS7x0_REG_IOCONTROL, IS7x0_IOC_RESET);

    nxpEnableEnhancedFunctions(nxp);
    nxpSetSpeed(nxp);
    nxpSetLine(nxp);
    nxpSetTriggerLevel(nxp);
    nxpFIFOEnable(nxp);

    nxpRead(nxp, IS7x0_REG_EFCR, 1, &nxp->efcr);

    //EXTI_INIT();
    nxpSerialIntExtiInit();

    nxpEnableInterrupt(nxp);

    nxp->active = true;

    return &nxp->port;
}

void nxpSerialWriteByte(serialPort_t *instance, uint8_t ch)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    if (!nxp->active)
        return;

    if (txBufferRoom(nxp->port)) {
        nxp->port.txBuffer[nxp->port.txBufferHead] = ch;
        nxp->port.txBufferHead = (nxp->port.txBufferHead + 1)
                % nxp->port.txBufferSize;
    }
}

uint8_t nxpSerialRxBytesWaiting(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    if (!nxp->active)
        return 0;

    return rxBufferLen(nxp->port);
}

uint8_t nxpSerialTxBytesFree(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    if (!nxp->active)
        return 0;

    return txBufferRoom(nxp->port);
}

uint8_t nxpSerialReadByte(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;
    int rxavail;
    uint8_t ch;

    if (!nxp->active)
        return 0;

    rxavail = rxBufferLen(nxp->port);

    if (rxavail) {
        ch = nxp->port.rxBuffer[nxp->port.rxBufferTail];
        nxp->port.rxBufferTail = (nxp->port.rxBufferTail + 1)
                % nxp->port.rxBufferSize;
        return ch;
    }
    return 0;
}

void nxpSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    nxp->port.baudRate = baudrate;
    nxpSetSpeed(nxp);
}

bool nxpSerialTransmitBufferEmpty(serialPort_t *instance)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    if (!nxp->active)
        return false;

    return txBufferLen(nxp->port) == 0;
}

void nxpSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    nxpSerial_t *nxp = (nxpSerial_t *)instance;

    if (!nxp->active)
        return;

    nxp->port.mode = mode;

    nxpTransmitterControl(nxp, ((nxp->port.mode & MODE_TX) == MODE_TX));
    nxpReceiverControl(nxp, ((nxp->port.mode & MODE_RX) == MODE_RX));
}

void nxpSerialPoller(void)
{
    // static int scanport = 0; // Should service all ports: MAX_NXPSERIAL_PORTS
    nxpSerial_t *nxp;

    int rxroom;
    int rxburst;
    static int rxlen;

    int txavail;
    int txburst;
    int txlen;

    digitalHi(GPIOB, Pin_4);

    // Quickly scan the slaves for interrupts
    __disable_irq();
    if (nxpInterrupted) {
        nxpInterrupted = false;
        __enable_irq();
        for (int index = 0 ; index < MAX_NXPSERIAL_PORTS ; index++) {
            nxp = &nxpSerialPorts[index];

            if (!nxp->active)
                continue;

            nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);
        }
    } else {
        __enable_irq();
    }

    for (int index = 0 ; index < MAX_NXPSERIAL_PORTS ; index++) {

        nxp = &nxpSerialPorts[index];

        if (!nxp->active)
            continue;

        // Interrupt processing policy:
        // The NXP7x0's interrupt is used to reduce the number of polls:
        // the slave need not to be polled unless interrupted.
        // Once interrupted, it will be completely serviced until
        // IIR[0] (INTSTAT) goes high
        // (i.e. not interrupting anymore == INT signal is high).

//again:;
        if ((nxp->iir & IS7x0_IIR_INTSTAT)
         && (nxp->rxlvl == 0)
         && (txBufferLen(nxp->port) == 0))
            goto out;

        switch (nxp->iir & IS7x0_IIR_INTMSK) {
        case IS7x0_IIR_RXTIMO:   // RX chars below trigger are available
        case IS7x0_IIR_RHR:      // RX chars above trigger are available
            // Update the rxlvl
            nxpRead(nxp, IS7x0_REG_RXLVL, 1, &nxp->rxlvl);
            break;

        case IS7x0_IIR_THR:      // TX fifo ready for more queuing
            // Update the txlvl
            nxpRead(nxp, IS7x0_REG_TXLVL, 1, &nxp->txlvl);
            break;

        case IS7x0_IIR_LINESTAT:
            // Hard case here: there is at least one char
            // with line error, but we don't know which.
            // To salvage valid chars, we have to read chars in the
            // fifo one by one with associated LSR, which is a pain.
            // If rxlvl is non-zero (left over from last service),
            // we can assume #rxlvl chars are error free.

            // For now, just reset the RX FIFO, for KISS.
            // We can handle the safe #rxlvl chars when the code is mature.
            nxpWrite(nxp, IS7x0_REG_FCR, nxp->fcr & ~IS7x0_FCR_TXFIFO_RST);
            nxpRead(nxp, IS7x0_REG_RXLVL, 1, &nxp->rxlvl);
            nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);
            // goto again;
            break;

        default: 
            break;
        }

#if 0
        if (nxp->rxlvl) {
            rxlen = (nxp->rxlvl > 8) ? 8 : nxp->rxlvl;
	    nxpRead(nxp, IS7x0_REG_RHR, rxlen, dummyBuffer);
            nxp->rxlvl -= rxlen;

            if (nxp->port.callback) {
                int i;
                for (i = 0 ; i < rxlen ; i++) {
                    (*nxp->port.callback)(dummyBuffer[i]);
                }
            }
        }
#endif

#if 0
        // Debugging receiver: Discard RX FIFO.
        uint8_t dummyBuffer[256];

        if (nxp->rxlvl) {
            rxlen = nxp->rxlvl;
            if (rxlen > NXPSERIAL_MAX_RXFRAG)
                rxlen = NXPSERIAL_MAX_RXFRAG;
            nxpRead(nxp, IS7x0_REG_RHR, rxlen, dummyBuffer);
            nxp->rxlvl -= rxlen;
        }
#else
        // If there's a room in rxBuf, try to read from RX FIFO.
        // Limit single transfer to MIN(rxroom, rxfifolevel, maxfrag).
        // Actual transfer size will further be limited by the end
        // of the physical buffer (can't wrap around in a burst transfer).

        rxroom = rxBufferRoom(nxp->port);

        if (rxroom) {
            if (nxp->rxlvl) {
                rxlen = (rxroom < nxp->rxlvl) ? rxroom : nxp->rxlvl;

                if (rxlen > NXPSERIAL_MAX_RXFRAG)
                    rxlen = NXPSERIAL_MAX_RXFRAG;

                rxburst = rxBufferBurstLimit(nxp->port);

                if (rxlen > rxburst)
                    rxlen = rxburst;

                // Debuggin' (Shouldn't happen)
                if (rxlen <= 0 || rxlen > NXPSERIAL_MAX_RXFRAG) {
                    rxlen = 1;
                }

                nxpRead(nxp, IS7x0_REG_RHR, rxlen,
                        &(nxp->port.rxBuffer[nxp->port.rxBufferHead]));

                nxp->port.rxBufferHead = (nxp->port.rxBufferHead + rxlen)
                        % nxp->port.rxBufferSize;

                nxp->rxlvl -= rxlen;
            }
        }
#endif

        int rxavail;

        if ((rxavail = rxBufferLen(nxp->port)) && nxp->port.callback) {
            uint8_t ch;
            while (rxavail--) {
                ch = nxp->port.rxBuffer[nxp->port.rxBufferTail];
                nxp->port.rxBufferTail = (nxp->port.rxBufferTail + 1)
                        % nxp->port.rxBufferSize;
                (*nxp->port.callback)(ch);
            }
        }

        // XXX Take rxlen into account for txlen calculation.

        if ((txavail = txBufferLen(nxp->port)) != 0) {

            // Can pump out without retrieving a new txlvl value,
            // knowing there is at least old txlvl bytes of space
            // available for queuing.

            if (nxp->txlvl < NXPSERIAL_MAX_TXFRAG)
                nxpRead(nxp, IS7x0_REG_TXLVL, 1, &nxp->txlvl);

            txburst = txBufferBurstLimit(nxp->port);

            if (nxp->txlvl) {
                txlen = (txavail < nxp->txlvl) ? txavail : nxp->txlvl;

                if (txlen > NXPSERIAL_MAX_TXFRAG)
                    txlen = NXPSERIAL_MAX_TXFRAG;

                if (txlen > txburst)
                    txlen = txburst;

	    if (txlen <= 0 || txlen > NXPSERIAL_MAX_TXFRAG) {
                    debug[3] = txlen;
                    txlen = 1;
                }

                nxpWriteBuffer(nxp, IS7x0_REG_THR, txlen,
                        &(nxp->port.txBuffer[nxp->port.txBufferTail]));

                nxp->port.txBufferTail = (nxp->port.txBufferTail + txlen)
                        % nxp->port.txBufferSize;

                nxp->txlvl -= txlen;
            }
        }

        nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);

out:;
        if (index == 1) {
            debug[0] = nxp->iir;
            debug[1] = nxp->rxlvl;
            debug[2] = rxBufferRoom(nxp->port);
            debug[3] = rxBufferLen(nxp->port);
        }
    }

    digitalLo(GPIOB, Pin_4);
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
