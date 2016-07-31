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

#include "target.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/system.h"

#include "serial.h"
#include "serial_nxp7x0.h"

#include "sc16is7x0.h"    // NXP SC16IS7xx registers
#include "ubreg.h"        // I2C/UART Bridge specific registers

#if 1
#include "common/printf.h"
#define dprintf(x) printf x
#else
#define dprintf(x)
#endif

//
// Watch out!!!
// Slaves with ATmega328 running at 16MHz can't handle back-to-back WRITEs.
// @800KHz, 45usec delay is required.
// 
#define UBdelayMicroseconds(n) delayMicroseconds(n)


#ifdef NAZE
#define NXPSERIAL_SPI_INSTANCE NAZE_SPI_INSTANCE
#define NXPSERIAL_SPI_CS_GPIO  NAZE_SPI_CS_GPIO
#define NXPSERIAL_SPI_CS_PIN   NAZE_SPI_CS_PIN
#endif

#define DISABLE_NXP(nxp)  GPIO_SetBits((nxp)->spicsgpio, (nxp)->spicspin)
#define ENABLE_NXP(nxp)   GPIO_ResetBits((nxp)->spicsgpio, (nxp)->spicspin)

//#define NXPSERIAL_MAX_RXFRAG 16
//#define NXPSERIAL_MAX_TXFRAG 16 
#define NXPSERIAL_MAX_RXFRAG 12
#define NXPSERIAL_MAX_TXFRAG 12 

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
    // I2c
    I2C_TypeDef      *i2cbus;
    uint8_t          addr;
    // SPI
    SPI_TypeDef      *spibus;
    GPIO_TypeDef     *spicsgpio;
    uint16_t         spicspin;

    uint8_t          chan;

    uint8_t          devtype;
#define NXPSERIAL_DEVTYPE_NONE         0
#define NXPSERIAL_DEVTYPE_NXP          1 // NXP SC16IS74{0,1},7{5,6}{0,2}
#define NXPSERIAL_DEVTYPE_MINIMAL      2 // UART Bridge
#define NXPSERIAL_DEVTYPE_UBLOX        3 // u-blox DDC

#define UBX_REG_LENHI    0xfd
#define UBX_REG_LENLO    0xfe
#define UBX_REG_DATA     0xff

    uint16_t         apiver;

    int32_t         freq;      // -1:don't care, 0:baudrate/150, otherwise::crystal
#define UB_FREQ_DONTCARE   -1
#define UB_FREQ_BAUDx150    0

    int8_t           polled;

    //uint8_t          cycletime; // Average time per byte

    uint8_t          rxlvl;
    uint8_t          txlvl;

    uint8_t          iir;       // Last IIR read
    uint8_t          fcr;       // FCR soft copy
    uint8_t          efcr;      // EFCR soft copy

    int              bcycle;
} nxpSerial_t;

extern nxpSerial_t nxpSerialPorts[];
extern const struct serialPortVTable nxpSerialVTable[];

#ifdef USE_NXPSERIAL1
#define MAX_NXPSERIAL_PORTS 1
#endif
#ifdef USE_NXPSERIAL2
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 2
#endif
#ifdef USE_NXPSERIAL3
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 3
#endif
#ifdef USE_NXPSERIAL4
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 4
#endif
#ifdef USE_NXPSERIAL5
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 5
#endif
#ifdef USE_NXPSERIAL6
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 6
#endif
#ifdef USE_NXPSERIAL7
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 7
#endif
#ifdef USE_NXPSERIAL8
#undef MAX_NXPSERIAL_PORTS
#define MAX_NXPSERIAL_PORTS 8
#endif

static bool nxpSerialPortsInit = false;
nxpSerial_t nxpSerialPorts[MAX_NXPSERIAL_PORTS];

/*
 * Buffer management
 */
#define rxBufferLen(port) ((((port).rxBufferHead - (port).rxBufferTail)) & ((port).rxBufferSize - 1))
#define txBufferLen(port) ((((port).txBufferHead - (port).txBufferTail)) & ((port).txBufferSize - 1))

#define rxBufferRoom(port) ((port).rxBufferSize - rxBufferLen(port) - 1)
#define txBufferRoom(port) ((port).txBufferSize - txBufferLen(port) - 1)

#define rxBufferBurstLimit(port) ((port).rxBufferSize - (port).rxBufferHead)
#define txBufferBurstLimit(port) ((port).txBufferSize - (port).txBufferTail)

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

/*
 * Interrupt related
 */
volatile bool nxpInterrupted = false;

// Some experimental pin assignments

#ifdef SPRACINGF3
// RC2 (BLUE) = PA1 : IRQ
// (GREEN) = PB5
// (YELLOW) = PB4
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
#endif

#ifdef NAZE
#define I2CSERIAL_INT_PERIPH          RCC_APB2Periph_GPIOA
#define I2CSERIAL_INT_PIN             Pin_1
#define I2CSERIAL_INT_GPIO            GPIOA
#define I2CSERIAL_INT_EXTI_LINE       EXTI_Line1
#define I2CSERIAL_INT_EXTI_PIN_SOURCE GPIO_PinSource1
#define I2CSERIAL_INT_IRQN            EXTI1_IRQn
#endif

extiConfig_t nxpIntExtiConfig = {
#ifdef STM32F10X
    .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
#endif
#ifdef STM32F303xC
    .gpioAHBPeripherals	 = RCC_AHBPeriph_GPIOB,
#endif

    .gpioPort            = I2CSERIAL_INT_GPIO,
    .gpioPin             = I2CSERIAL_INT_PIN,

    .exti_port_source    = I2CSERIAL_INT_GPIO,
    .exti_pin_source     = I2CSERIAL_INT_EXTI_PIN_SOURCE,
    .exti_line           = I2CSERIAL_INT_EXTI_LINE,
    .exti_irqn           = I2CSERIAL_INT_IRQN,
};

void nxpSerial_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(nxpIntExtiConfig.exti_line) == RESET) {
        return;
    }

    // digitalHi(GPIOB, Pin_5); // Debugging

    EXTI_ClearITPendingBit(nxpIntExtiConfig.exti_line);

    nxpInterrupted = true;

    // digitalLo(GPIOB, Pin_5); // Debugging
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

    gpio.pin = nxpIntExtiConfig.gpioPin;
    gpio.speed = Speed_2MHz;
    //gpio.mode = Mode_IN_FLOATING;
    //gpio.mode = Mode_Out_PP; // For port connectivity testing
    gpio.mode = Mode_IPU; // Input with pull-up
    gpioInit(nxpIntExtiConfig.gpioPort, &gpio);

    nxpSerialConfigureEXTI();

    nxpExtiInitDone = true;
}

// Litte tools for debugging/monitoring

static void nxpDebugSetup(void)
{
    gpio_config_t LEDgpio;

#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif
#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif

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

static void nxpDebugOFF(void)
{
    digitalLo(GPIOB, Pin_4);
}

static void nxpDebugON(void)
{
    digitalHi(GPIOB, Pin_4);
}

static void nxpZEDOFF(void)
{
    digitalLo(GPIOB, Pin_5);
}

static void nxpZEDON(void)
{
    digitalHi(GPIOB, Pin_5);
}

/*
 * Port level
 */
static void nxpInitSPI(nxpSerial_t *nxp)
{
}

/*
 * Register level access
 */
#define NXP_CHANREG(chan,reg) (((reg) << 3)|((chan)<< 1))

#define nxpi2cRead(addr, chan, reg, n, buf) \
            i2cRead(addr, NXP_CHANREG(chan, reg), n, buf)
#define nxpi2cWrite(addr, chan, reg, data) \
            i2cWrite(addr, NXP_CHANREG(chan, reg), data)
#define nxpi2cWriteBuffer(addr, chan, reg, n, buf) \
            i2cWriteBuffer(addr, NXP_CHANREG(chan, reg), n, buf)

#if 0
#define nxpRead(nxp, reg, n, buf) (\
            nxpi2cRead((nxp)->addr, (nxp)->chan, reg, n, buf),\
            (nxp)->bcycle += ((n) + 5))
#define nxpWrite(nxp, reg, data) (\
            nxpi2cWrite((nxp)->addr, (nxp)->chan, reg, data),\
            (nxp)->bcycle += 3)
#define nxpWriteBuffer(nxp, reg, n, buf) (\
            nxpi2cWriteBuffer((nxp)->addr, (nxp)->chan, reg, n, buf),\
            (nxp)->bcycle += ((n) + 2))
#endif

static
void
nxpReadSPI(nxpSerial_t *nxp, uint8_t reg, uint8_t *buf)
{
    ENABLE_NXP(nxp);
    spiTransferByte(nxp->spibus, 0x80|NXP_CHANREG(nxp->chan, reg));
    *buf = spiTransferByte(nxp->spibus, 0);
    DISABLE_NXP(nxp);
}

static
void
nxpWriteSPI(nxpSerial_t *nxp, uint8_t reg, uint8_t val)
{
    uint8_t wbuf[2];

    wbuf[0] = NXP_CHANREG(nxp->chan, reg);
    wbuf[1] = val;
    ENABLE_NXP(nxp);
    spiTransfer(nxp->spibus, NULL, wbuf, 2);
    DISABLE_NXP(nxp);
}

static
void
nxpWriteBufferSPI(nxpSerial_t *nxp, uint8_t reg, int len, uint8_t *buf)
{
    ENABLE_NXP(nxp);
    spiTransferByte(nxp->spibus, reg);
    spiTransfer(nxp->spibus, NULL, buf, len);
    DISABLE_NXP(nxp);
}

static
void
nxpReadBufferSPI(nxpSerial_t *nxp, uint8_t reg, int len, uint8_t *buf)
{
    ENABLE_NXP(nxp);
    spiTransferByte(nxp->spibus, 0x80|(reg << 3));
    spiTransfer(nxp->spibus, buf, NULL, len);
    DISABLE_NXP(nxp);
}

static bool
nxpRead(nxpSerial_t *nxp, uint8_t reg, int n, uint8_t *buf)
{
    if (nxp->bustype == NXPSERIAL_BUSTYPE_I2C) {
        nxp->bcycle += n + 5;
    	return nxpi2cRead(nxp->addr, nxp->chan, reg, n, buf);
    } else {
        if (n == 1)
            nxpReadSPI(nxp, reg, buf);
        else
            nxpReadBufferSPI(nxp, reg, n, buf);

        return true;
    }
}

static bool
nxpWrite(nxpSerial_t *nxp, uint8_t reg, uint8_t data)
{
    if (nxp->bustype == NXPSERIAL_BUSTYPE_I2C) {
        nxp->bcycle += 3;
        return nxpi2cWrite(nxp->addr, nxp->chan, reg, data);
    } else {
        nxpWriteSPI(nxp, reg, data);
        return true;
    }
}

static bool
nxpWriteBuffer(nxpSerial_t *nxp, uint8_t reg, int n, uint8_t *buf)
{
    if (nxp->bustype == NXPSERIAL_BUSTYPE_I2C) {
        nxp->bcycle += n + 2;
        return nxpi2cWriteBuffer(nxp->addr, nxp->chan, reg, n, buf);
    } else {
        nxpWriteBufferSPI(nxp, reg, n, buf);
        return true;
    }
}

static
bool
nxpResetNXP(nxpSerial_t *nxp)
{
    uint8_t ioc, lcr, lsr, efcr;

    if (nxp->bustype == NXPSERIAL_BUSTYPE_SPI) {
        // Software reset in SPI mode is broken.
        dprintf(("nxpResetNXP: SPI, assume successful reset\r\n"));
        goto resetok;
    }

#if 1
uint8_t spr;
nxpRead(nxp, IS7x0_REG_SPR, 1, &spr);
dprintf(("nxpResetNXP: pre reset spr 0x%x\r\n", spr));
#endif

    nxpRead(nxp, IS7x0_REG_IOCONTROL, 1, &ioc);
    nxpWrite(nxp, IS7x0_REG_IOCONTROL, ioc|IS7x0_IOC_SRESET);

    for (int retry = 0 ; retry < 10 ; retry++) {
        delay(5);  // This is a software reset, so delay can be much shorter???.
        nxpRead(nxp, IS7x0_REG_IOCONTROL, 1, &ioc);
        if (ioc & IS7x0_IOC_SRESET) {
            dprintf(("nxpResetNXP: ioc 0x%x\r\n", ioc));
        }
    }

    if (ioc & IS7x0_IOC_SRESET) {
        dprintf(("nxpResetNXP: SRESET didn't go down ioc 0x%x\r\n", ioc));
        return false;
    }

#if 1
nxpRead(nxp, IS7x0_REG_SPR, 1, &spr);
dprintf(("nxpResetNXP: post reset spr 0x%x\r\n", spr));
#endif

    if (!nxpRead(nxp, IS7x0_REG_LCR, 1, &lcr)
     || !nxpRead(nxp, IS7x0_REG_LSR, 1, &lsr)) {
        dprintf(("nxpResetNXP: lcr or lcr read failed\r\n"));
        return false;
    }

    dprintf(("nxpResetNXP: lcr 0x%x lsr 0x%x\r\n", lcr, lsr));

    if (lcr == 0x1D && (lsr & 0x60) == 0x60) {
        goto resetok;
    }

    return false;

resetok:;
    // Disable transmitter & receiver
    nxpRead(nxp,IS7x0_REG_EFCR, 1, &efcr);
    nxpWrite(nxp, IS7x0_REG_EFCR, efcr|IS7x0_EFCR_TXDISABLE|IS7x0_EFCR_RXDISABLE);
}

static
bool
nxpResetUB(nxpSerial_t *nxp)
{
    return true;
}

static
bool
nxpResetUBLOX(nxpSerial_t *nxp)
{
    return true;
}

static
bool
nxpReset(nxpSerial_t *nxp)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_NXP)
        return nxpResetNXP(nxp);
    else if (nxp->devtype == NXPSERIAL_DEVTYPE_MINIMAL)
        return nxpResetUB(nxp);
    else if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return nxpResetUBLOX(nxp);

    return false;
}

static
bool
nxpProbeNXP(nxpSerial_t *nxp)
{
    uint8_t lcr, lsr;
    uint8_t txlvl;

    if (!nxpRead(nxp, IS7x0_REG_LCR, 1, &lcr)
     || !nxpRead(nxp, IS7x0_REG_LSR, 1, &lsr)) {
        dprintf(("nxpProbeNXP: lcr or lsr read failed\r\n"));
        return false;
    }

    dprintf(("nxpProbeNXP: lcr 0x%x lsr 0x%x\r\n", lcr, lsr));

    uint8_t iir;
    nxpRead(nxp, IS7x0_REG_IIR, 1, &iir);
    dprintf(("nxpProbeNXP: iir 0x%x\r\n", iir));

    // Reset state: LCR = 0x1D, LSR = 0x60
    // "7.4.1 Hardware reset, Power-On Reset (POR) and software reset"
    if (lcr == 0x1D && (lsr & 0x60) == 0x60) {
        dprintf(("nxpProbeNXP: lcr and lsr reset signature match\r\n"));
        return true;
    }

    // Not after reset or unknown chip.
    // Try reading the SPR for our signature.
    uint8_t spr;
    nxpRead(nxp, IS7x0_REG_SPR, 1, &spr);
    dprintf(("nxpProbeNXP: spr 0x%x\r\n", spr));

    if (spr == 0xAA) {
        // It looks like we have seen this chip before.
        dprintf(("nxpProbeNXP: spr signature match\r\n"));
        return true;
    }

    // Try identifying the chip non-intrusively.

    // Wait until THR and TSR to go empty.

    // o If TXLVL is not zero, see if TXLVL is decreasing.
    //   (Must wait for longest 1 char time = 1/15 sec)

    if (!(lsr & IS7x0_LSR_TXEMPTY)) {
            // XXX Should be long enough for 64 bytes to go out @150bps
            // XXX 150/10 = 15 bytes/sec, 64/15 = 4.267secs!!!
            // 100msec is good for 1char@9600bps
            delay(100);

            nxpRead(nxp, IS7x0_REG_LSR, 1, &lsr);
            if (!(lsr & IS7x0_LSR_TXEMPTY))
                return false;
    }

    // TX is all empty, TX FIFO Level should be 0x40 (64)

    nxpRead(nxp, IS7x0_REG_TXLVL, 1, &txlvl);

    if (txlvl == 64) {
        dprintf(("nxpProbeNXP: txlvl signature match\r\n"));
        return true;
    }

    return false;
}

static
bool
nxpProbeUB(nxpSerial_t *nxp)
{
    uint8_t id[4];

    if (!nxpRead(nxp, IS7x0_REG_SPR, 4, id))
        return false;

    if (id[0] == 'U' && id[1] == 'B') {
        nxp->apiver = (id[2] << 8)|id[3];
        return true;
    } else if (id[0] == 0xAA && id[1] == 0xAA) {
        // PIC temporary, should implement "UBxx".
        nxp->apiver = 99;
        return true;
    }

    return false;
}

static
bool
nxpProbeUBLOX(nxpSerial_t *nxp)
{
    // Observations and assumptions:
    // (1) All data registers except 0xfd (length high), 0xfe (length low) and
    // 0xff (stream outlet) returns their address.
    // (2) Length is limited to relatively small number and does not extend to
    // 0xfd00~ range (On MAX-M8Q, only goes up to 0x19xx.)
    //
    // Probe strategy:
    // (1) Read reg 0xf0 to 0xff.
    // (2) Check if 0xf0 to 0xfc are read as their address, and
    // (3) Check if 0xfd != 0xfd

    uint8_t ubuf[14];

    if (!i2cRead(nxp->addr, 0xf0, 14, ubuf))
        return false;

    for (int i = 0 ; i <= 0xc ; i++) {
        if (ubuf[i] != 0xf0 + i)
            return false;
    }

    if (ubuf[0xd] >= 0xfd)
        return false;

#if 0
// A little test for MAX-M8Q
int retry = 0;
int mlen;
uint8_t lbuf[2];
uint8_t dbuf[16];
int tlen;
bool ok;

do {
  if (retry) delay(1000);
  do {
    ok = i2cRead(nxp->addr, 0xfd, 2, lbuf);
  } while (!ok);
  mlen = (lbuf[0] << 0)|lbuf[1];
} while (mlen == 0 && ++retry < 10);

// Dump out the garbage
while (mlen) {
  tlen = (mlen > 16) ? 16 : mlen;
  i2cRead(nxp->addr, 0xff, tlen, dbuf);
}

// Wait for a new message
retry = 0;

do {
  if (retry) delay(1000);
  do {
    ok = i2cRead(nxp->addr, 0xfd, 2, lbuf);
  } while (!ok);
  mlen = (lbuf[0] << 0)|lbuf[1];
} while (mlen == 0 && ++retry < 10);

// Read it
while (mlen) {
  tlen = (mlen > 16) ? 16 : mlen;
  i2cRead(nxp->addr, 0xff, tlen, dbuf);
}
#endif

    return true;
}

static
bool
nxpProbe(nxpSerial_t *nxp)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_NXP)
        return nxpProbeNXP(nxp);
    else if (nxp->devtype == NXPSERIAL_DEVTYPE_MINIMAL)
        return nxpProbeUB(nxp);
    else if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return nxpProbeUBLOX(nxp);

    return false;
}

static void nxpSetSpeed(nxpSerial_t *nxp)
{
    uint32_t divisor;
    uint32_t prescaler;
    uint32_t baudrate;

    if (nxp->freq == UB_FREQ_DONTCARE)
        return;  // Fixed or don't care

    baudrate = nxp->port.baudRate;

    if (nxp->freq == UB_FREQ_BAUDx150) {
        uint8_t brreg;
        baudrate /= 150;
        brreg = baudrate;
        nxpWrite(nxp, UB_REG_BRL, brreg);
	UBdelayMicroseconds(100);
        brreg = baudrate >> 8;
        nxpWrite(nxp, UB_REG_BRH, brreg);
        return;
    }

#if 0
#define MAX_BAUDRATE 57600
    if (baudrate > MAX_BAUDRATE)
        baudrate = MAX_BAUDRATE;
#endif

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
    if (nxp->devtype == NXPSERIAL_DEVTYPE_MINIMAL
     || nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

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
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

    if (enable)
        nxp->efcr &= ~IS7x0_EFCR_TXDISABLE;
    else
        nxp->efcr |= IS7x0_EFCR_TXDISABLE;

    nxpWrite(nxp, IS7x0_REG_EFCR, nxp->efcr);
    UBdelayMicroseconds(50);
}

static void nxpReceiverControl(nxpSerial_t *nxp, bool enable)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

    if (enable)
        nxp->efcr &= ~IS7x0_EFCR_RXDISABLE;
    else
        nxp->efcr |= IS7x0_EFCR_RXDISABLE;

    nxpWrite(nxp, IS7x0_REG_EFCR, nxp->efcr);
    UBdelayMicroseconds(50);
}

static void nxpEnableEnhancedFunctions(nxpSerial_t *nxp)
{
    uint8_t lcr;
    uint8_t efr;

    if (nxp->devtype == NXPSERIAL_DEVTYPE_MINIMAL
     || nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

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

    if (nxp->devtype == NXPSERIAL_DEVTYPE_MINIMAL
     || nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

    //nxpRead(nxp, IS7x0_REG_FCR, 1, &fcr);

    fcr = IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST|IS7x0_FCR_FIFO_EN;

    nxpWrite(nxp, IS7x0_REG_FCR, fcr);

    //nxp->fcr = fcr;
}

static void nxpSetTriggerLevel(nxpSerial_t *nxp)
{
    uint8_t fcr;
    uint8_t mcr;
    uint8_t tlr;

    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX)
        return;

    nxpRead(nxp, IS7x0_REG_FCR, 1, &fcr);
    fcr = IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST; // RX and TX triggers to zero, reset both fifos, disable FIFO.
    nxpWrite(nxp, IS7x0_REG_FCR, fcr);
    UBdelayMicroseconds(50);

#if 0
uint8_t spr;
nxpRead(nxp, IS7x0_REG_SPR, 1, &spr);
printf("nxpSetTriggerLevel: pre spr 0x%x\r\n", spr);
#endif

    // Enable TCR/TLR access
    nxpRead(nxp, IS7x0_REG_MCR, 1, &mcr);
    mcr |= IS7x0_MCR_TCRTLR_EN;
    nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    UBdelayMicroseconds(50);

    tlr = (1 << IS7x0_TLR_RX_SFT)|(((NXPSERIAL_MAX_TXFRAG + 1) / 4) << IS7x0_TLR_TX_SFT);
    nxpWrite(nxp, IS7x0_REG_TLR, tlr);

    // Disable TCR/TLR access
    mcr &= ~IS7x0_MCR_TCRTLR_EN;
    nxpWrite(nxp, IS7x0_REG_MCR, mcr);
    UBdelayMicroseconds(50);

#if 0
nxpRead(nxp, IS7x0_REG_SPR, 1, &spr);
printf("nxpSetTriggerLevel: post spr 0x%x\r\n", spr);
#endif

#if 0 // Done in nxpFIFOEnable()
    fcr = IS7x0_FCR_FIFO_EN;
    nxpWrite(nxp, IS7x0_REG_FCR, fcr);
#endif
}

static void nxpEnableInterrupt(nxpSerial_t *nxp)
{
    uint8_t ier;

    if (nxp->polled)
        return;

    ier = IS7x0_IER_RHR;
    //ier |= IS7x0_IER_THR;
    //ier |= IS7x0_IER_CTS;
    //ier |= IS7x0_IER_RTS;
    //ier |= IS7x0_IER_MODEM;
    //ier |= IS7x0_IER_LINE;

    nxpWrite(nxp, IS7x0_REG_IER, ier);
}

serialPort_t *openNXPSerial(
	nxpSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options)
{
    nxpSerial_t *nxp;

    if (nxpSerialPortsInit) {
        for (int i = 0 ; i < MAX_NXPSERIAL_PORTS; i++)
            nxpSerialPorts[i].active = false;
        nxpSerialPortsInit = true;
    }

    nxp = &nxpSerialPorts[portIndex];

    switch (portIndex) {
    case 7:
        // Sparkfun BOB on SPI
        nxp->bustype = NXPSERIAL_BUSTYPE_SPI;
        nxp->spibus = NXPSERIAL_SPI_INSTANCE;
        nxp->spicsgpio = NXPSERIAL_SPI_CS_GPIO;
        nxp->spicspin = NXPSERIAL_SPI_CS_PIN;
        nxp->devtype = NXPSERIAL_DEVTYPE_NXP;
        nxp->freq = 14745600;
        nxp->polled = 1;
        break;

    case 0:
        // Sparkfun BOB
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_NXP;
        nxp->addr = 0x4d;
        nxp->chan = 0;
        nxp->freq = 14745600;
        nxp->polled = 1;

        //setupDebugPins();

        break;

    case 1:
        // Switch Science BOB
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_NXP;
        nxp->addr = 0x4c;
        nxp->chan = 0;
        nxp->freq = 12000000;
        nxp->polled = 1;
        break;

    case 2:
        // pic-twub (1825)
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_MINIMAL; // MINIMAL, MINIMAL_IMUX
        nxp->addr = 0x36;
        nxp->chan = 0;
        nxp->freq = UB_FREQ_BAUDx150;
        nxp->polled = 1;
        break;

    case 3:
        // pic-twub (1822)
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_MINIMAL;
        nxp->addr = 0x38;
        nxp->chan = 0;
        nxp->freq = UB_FREQ_BAUDx150;
        nxp->polled = 1;
        break;

    case 4:
        // MWOSD
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_MINIMAL;
        nxp->addr = 0x19;
        nxp->chan = 0;
        nxp->freq = UB_FREQ_BAUDx150;
        nxp->polled = 1;
        break;

    case 5:
        // Arduino Pro Mini UB
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_MINIMAL;
        nxp->addr = 0x18;
        nxp->chan = 0;
        nxp->freq = UB_FREQ_BAUDx150;
        nxp->polled = 1;
        break;

    case 6:
        // u-blox DDC
        nxp->bustype = NXPSERIAL_BUSTYPE_I2C;
        nxp->devtype = NXPSERIAL_DEVTYPE_UBLOX;
        nxp->addr = 0x42;
        nxp->freq = -1;
        nxp->polled = 1;
        break;


    default:
        return NULL;
    }

    dprintf(("openNXPSerial: probing and resetting portIndex %d\r\n", portIndex));

    if (!nxpProbe(nxp)) {
        dprintf(("openNXPSerial: probe failed\r\n"));
    }

    // If this is a NXP chip, we write a signature into SPR for hot start.
    if (nxp->devtype == NXPSERIAL_DEVTYPE_NXP)
        nxpWrite(nxp, IS7x0_REG_SPR, 0xAA);

    if (!nxpReset(nxp)) {
        dprintf(("openNXPSerial: reset failed\r\n"));
        return NULL;
    }

    nxp->port.vTable = nxpSerialVTable;
    nxp->port.baudRate = baud;
    nxp->port.mode = MODE_RXTX;
    nxp->port.options = options;
    nxp->port.callback = callback;
    nxp->nxpSerialPortIndex = portIndex;

    resetBuffers(nxp);

    nxp->rxlvl = 0;
    nxp->txlvl = 0; // Actual value will be read for the 1st TX data

    nxp->bcycle = 0;

    if (nxp->devtype != NXPSERIAL_DEVTYPE_UBLOX) {
        // Superfluious reset???
        nxpWrite(nxp, IS7x0_REG_IOCONTROL, IS7x0_IOC_SRESET);
        UBdelayMicroseconds(50);

        nxpEnableEnhancedFunctions(nxp);
        nxpSetSpeed(nxp);
        nxpSetLine(nxp);
        nxpSetTriggerLevel(nxp);
        nxpFIFOEnable(nxp);
        nxpRead(nxp, IS7x0_REG_EFCR, 1, &nxp->efcr);
    }

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

static void nxpUpdateRXLVL(nxpSerial_t *nxp)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX) {
        // u-blox's data queue is big, but data can be read in small chunks.
        uint8_t lbuf[2];
        uint16_t len; 
        if (!i2cRead(nxp->addr, UBX_REG_LENHI, 2, lbuf)) {
            // Failed to read RXLVL... just return?
            return;
        }
        len = (lbuf[0] << 8)|lbuf[1];
        nxp->rxlvl = (len > 255) ? 255 : len;
    } else {
        nxpRead(nxp, IS7x0_REG_RXLVL, 1, &nxp->rxlvl);
    }
}

static void nxpUpdateTXLVL(nxpSerial_t *nxp)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX) {
        // u-blox doesn't provide any mechanism for write pacing,
        // But since messages can be written in small chunks,
        // pretend it has a reasonable amount of buffer.
        nxp->txlvl = 64;
    } else {
        nxpRead(nxp, IS7x0_REG_TXLVL, 1, &nxp->txlvl);
    }
}

static void nxpReadRHR(nxpSerial_t *nxp, int len, uint8_t *buf)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX) {
        i2cRead(nxp->addr, UBX_REG_DATA, len, buf);
    } else {
        nxpRead(nxp, IS7x0_REG_RHR, len, buf);
    }
}

static void nxpWriteTHR(nxpSerial_t *nxp, int len, uint8_t *buf)
{
    if (nxp->devtype == NXPSERIAL_DEVTYPE_UBLOX) {
        i2cWriteBuffer(nxp->addr, -1, len, buf);
    } else {
        nxpWriteBuffer(nxp, IS7x0_REG_THR, len, buf);
    }
}

void nullFunction(void)
{
}

static uint8_t dummytbuf[16];
static uint8_t dummyrbuf[128];
static int seq = 0;
static int dummycount = 0;
#include "common/printf.h"

void nxpSerialPoller(void)
{
    int bcycle;

    bool interrupted;
    static int scanport = 0;
    nxpSerial_t *nxp;

    int rxroom;
    int rxburst;
    static int rxlen;

    int txavail;
    int txburst;
    int txlen;

    uint8_t rxqlen;

    digitalHi(GPIOB, Pin_4);

#if 0
  // Arduino-UB
  if ((++dummycount % 10) == 0) {
    tfp_sprintf(dummytbuf, "%d\r\n", seq++);
    if (seq > 9999) seq = 0;
    i2cWriteBuffer(0x19, IS7x0_REG_THR, 6, dummytbuf);

    delayMicroseconds(40);

    i2cRead(0x19, IS7x0_REG_RXLVL, 1, &rxqlen);

    if (rxqlen > 0) {
        rxlen = rxqlen > 16 ? 16 : rxqlen;
        i2cRead(0x19, IS7x0_REG_RHR, rxlen, dummyrbuf);
    }
  }
#endif

    __disable_irq();
    interrupted = nxpInterrupted;
    nxpInterrupted = false;
    __enable_irq();

    // Quickly scan the slaves for interrupts

    for (int i = 0 ; i < MAX_NXPSERIAL_PORTS ; i++) {
        nxp = &nxpSerialPorts[i];

        if (!nxp->active)
            continue;

        nxp->bcycle = 0;

        if (interrupted)
            nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);
        else if (nxp->polled)
            // Simulate RX time-out
            nxp->iir = IS7x0_IIR_RXTIMO;
    }

    bcycle = 0;

    for (int i = 0 ; i < MAX_NXPSERIAL_PORTS ; i++) {

        if (bcycle > 32)
            break;

        nxp = &nxpSerialPorts[scanport];
        scanport = (scanport + 1) % MAX_NXPSERIAL_PORTS;

        if (!nxp->active)
            continue;

        if ((nxp->iir & IS7x0_IIR_INTSTAT)
         && (nxp->rxlvl == 0)
         && (txBufferLen(nxp->port) == 0))
            goto out;

        switch (nxp->iir & IS7x0_IIR_INTMSK) {
        case IS7x0_IIR_RXTIMO:   // RX chars below trigger are available
        case IS7x0_IIR_RHR:      // RX chars above trigger are available
            // Update the rxlvl
            nxpUpdateRXLVL(nxp);
            break;

        case IS7x0_IIR_THR:      // TX fifo ready for more queuing
            // Update the txlvl
            nxpUpdateTXLVL(nxp);
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
            UBdelayMicroseconds(50); // XXX
            nxpRead(nxp, IS7x0_REG_RXLVL, 1, &nxp->rxlvl);
            nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);
            break;

        default: 
            break;
        }

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

                nxpReadRHR(nxp, rxlen, 
                        &(nxp->port.rxBuffer[nxp->port.rxBufferHead]));

                nxp->port.rxBufferHead = (nxp->port.rxBufferHead + rxlen)
                        % nxp->port.rxBufferSize;

                nxp->rxlvl -= rxlen;
            }
        }

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

        if (((txavail = txBufferLen(nxp->port)) != 0) && bcycle < 32) {

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

                nxpWriteTHR(nxp, txlen, 
                        &(nxp->port.txBuffer[nxp->port.txBufferTail]));

                nxp->port.txBufferTail = (nxp->port.txBufferTail + txlen)
                        % nxp->port.txBufferSize;

                nxp->txlvl -= txlen;
            }
        }

out:;
        if (!nxp->polled)
            nxpRead(nxp, IS7x0_REG_IIR, 1, &nxp->iir);

        int index = nxp - nxpSerialPorts;
        if (index < 2) {
        	if (nxp->bcycle > debug[index])
			debug[index] = nxp->bcycle;
        }

        bcycle += nxp->bcycle;
    }

    if (bcycle > debug[2])
        debug[2] = bcycle;

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
