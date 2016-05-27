// NXP SC16IS7{4,5,6}0

#define     IS7x0_REG_RHR        0x00
#define     IS7x0_REG_THR        0X00
#define     IS7x0_REG_IER        0X01
#define     IS7x0_REG_FCR        0X02
#define     IS7x0_REG_IIR        0X02
#define     IS7x0_REG_LCR        0X03
#define     IS7x0_REG_MCR        0X04
#define     IS7x0_REG_LSR        0X05
#define     IS7x0_REG_MSR        0X06
#define     IS7x0_REG_SPR        0X07
#define     IS7x0_REG_TCR        0X06
#define     IS7x0_REG_TLR        0X07
#define     IS7x0_REG_TXLVL      0X08
#define     IS7x0_REG_RXLVL      0X09
#define     IS7x0_REG_IODIR      0X0A
#define     IS7x0_REG_IOSTATE    0X0B
#define     IS7x0_REG_IOINTENA   0X0C
#define     IS7x0_REG_IOCONTROL  0X0E
#define     IS7x0_REG_EFCR       0X0F

#define     IS7x0_REG_DLL        0x00
#define     IS7x0_REG_DLH        0X01

#define     IS7x0_REG_EFR        0X02
#define     IS7x0_REG_XON1       0X04
#define     IS7x0_REG_XON2       0X05
#define     IS7x0_REG_XOFF1      0X06
#define     IS7x0_REG_XOFF2      0X07

// Bits in IER
#define     IS7x0_IER_CTS        0X80
#define     IS7x0_IER_RTS        0X40
#define     IS7x0_IER_XOFF       0X20
#define     IS7x0_IER_SLEEP      0X10
#define     IS7x0_IER_MODEM      0X08
#define     IS7x0_IER_LINE       0X04
#define     IS7x0_IER_THR        0X02
#define     IS7x0_IER_RHR        0X01

// Bits in FCR
#define     IS7x0_FCR_RXTRG_SFT  6
#define     IS7x0_FCR_TXTRG_SFT  4
#define     IS7x0_FCR_TXFRST     0x04
#define     IS7x0_FCR_RXFRST     0x02
#define     IS7x0_FCR_FIFOEN     0x01

// Bits in IIR
#define     IS7x0_IIR_FIFOEN7    0x80
#define     IS7x0_IIR_FIFOEN6    0x40
#define     IS7x0_IIR_INTPRIMSK  0x3E
#define     IS7x0_IIR_INTPRISFT  1
#define     IS7x0_IIR_INTSTAT    0x01

// Bits in LCR
#define     IS7x0_LCR_DIVLATEN   0x80
#define     IS7x0_LCR_SETBRK     0x40
#define     IS7x0_LCR_PARMSK     0x38
#define       IS7x0_LCR_PARNONE    0x00
#define       IS7x0_LCR_PARODD     0x08
#define       IS7x0_LCR_PAREVEN    0x18
#define       IS7x0_LCR_PARONE     0x28
#define       IS7x0_LCR_PARZERO    0x38
#define     IS7x0_LCR_STOPBIT    0x04
#define     IS7x0_LCR_WLENMSK    0x03
#define       IS7x0_LCR_WLEN5      0x00
#define       IS7x0_LCR_WLEN6      0x01
#define       IS7x0_LCR_WLEN7      0x02
#define       IS7x0_LCR_WLEN8      0x03

// Bits in MCR
#define     IS7x0_MCR_CLKDIV     0x80
#define     IS7x0_MCR_IRDAEN     0x40
#define     IS7x0_MCR_XONANY     0x20
#define     IS7x0_MCR_LOOPBACK   0x10
#define     IS7x0_MCR_TCRTLREN   0x04
#define     IS7x0_MCR_RTS        0x02
#define     IS7x0_MCR_DTR        0x01

// Bits in LSR
#define     IS7x0_LSR_FIFOERR    0x80
#define     IS7x0_LSR_THRTSREMPTY 0x40
#define     IS7x0_LSR_THREMPTY   0x20
#define     IS7x0_LSR_BRKINT     0x10
#define     IS7x0_LSR_FERR       0x08
#define     IS7x0_LSR_PERR       0x04
#define     IS7x0_LSR_OERR       0x02
#define     IS7x0_LSR_DATARCV    0x01
