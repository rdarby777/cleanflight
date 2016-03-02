typedef struct bbspiHardware_s {
	GPIO_TypeDef *ss_gpio;
	uint16_t ss_pin;
	GPIO_TypeDef *sck_gpio;
	uint16_t sck_pin;
	GPIO_TypeDef *mosi_gpio;
	uint16_t mosi_pin;
} bbspiHardware_t;

typedef struct bbspi_s {
    bool active;
    GPIO_TypeDef *ss_gpio;
    gpio_config_t ss_cfg;
    GPIO_TypeDef *sck_gpio;
    gpio_config_t sck_cfg;
    GPIO_TypeDef *mosi_gpio;
    gpio_config_t mosi_cfg;
} bbspi_t;

bbspiHardware_t *bbspiGetHardwareConfig();

#define    BBSPI_SS_LO(bbspi) \
    digitalLo(bbspi->ss_gpio, bbspi->ss_cfg.pin)
#define    BBSPI_SS_HI(bbspi) \
    digitalHi(bbspi->ss_gpio, bbspi->ss_cfg.pin)
#define    BBSPI_SCK_LO(bbspi) \
    digitalLo(bbspi->sck_gpio, bbspi->sck_cfg.pin)
#define    BBSPI_SCK_HI(bbspi) \
    digitalHi(bbspi->sck_gpio, bbspi->sck_cfg.pin)
#define BBSPI_MOSI_LO(bbspi) \
    digitalLo(bbspi->mosi_gpio, bbspi->mosi_cfg.pin)
#define BBSPI_MOSI_HI(bbspi) \
    digitalHi(bbspi->mosi_gpio, bbspi->mosi_cfg.pin)
