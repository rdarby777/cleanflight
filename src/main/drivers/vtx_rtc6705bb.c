/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNECS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Bit banging SPI version of RTC6705 VTX
 *
 * Compatible with vtx_rtc6705.c by sblakemore for ImpulseRC Singularity FCs,
 * originated from Giles Burgess (giles@multiflite.co.uk) for MultiFlite FCs.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <strings.h>

#include <platform.h>
#include <config/config.h>

#include "build_config.h"
#include "gpio.h"
#include "system.h"

#include "drivers/bbspi.h"
#include "drivers/vtx.h"
#include "drivers/vtx_rtc6705.h"

#include "common/printf.h"
//#define dprintf(x) printf x
#define dprintf(x)

#if defined(VTX) && defined(BBSPI)
//
// Bit-banging SPI layer
// Ideally, it would be a separate service with a separate file,
// e.g., drivers/bus_spi_soft.c or something, but since there is no other
// device using the service at the moment, it is kept here.
//

// Requires 3 plain GPIO ports for SS, SCK and MOSI.
// Find out the ports to use from port code variables:
// bbspi_ss
// bbspi_sck
// bbspi_mosi
// 100th is GPIO (100 = GPIOA, 200=GPIOB, ... 600=GPIOB), and 10th & 1th are pin number, for example,
//     101 = GPIOA, Pin 1 (PA1)
//     204 = GPIOB, Pin 4 (PB4)
//     313 = GPIOC, Pin 13 (PC13)
//     600 = GPIOF, Pin 0 (PF0)
//     0xx = Invalid
// etc. Zero in 100th represents an invalid assignment, and is an initial value
// for the variables.
//
// Some valid codes for ss,sck,mosi
// SPRF3
//	101 (PA1), 204 (PB4), 205 (PB5)
//	101 (PA1), 102 (PWM7/PA2), 103 (PWM8/PA3)

// Things for config_master.h

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "blackbox/blackbox_io.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

// End of things for config_master.h

static bool getGPIOSpecFromCode(int portcode, GPIO_TypeDef **pGPIO, uint16_t *pPin)
{
    int gpionum;
    int pinnum;

    if (portcode < 100)
        return false;

    gpionum = portcode / 100;
    pinnum = portcode % 100;

    switch(gpionum) {
    case 1:
        *pGPIO = GPIOA;
        break;
    case 2:
        *pGPIO = GPIOB;
        break;
    case 3:
        *pGPIO = GPIOC;
        break;
    case 4:
        *pGPIO = GPIOD;
        break;
    case 5:
        *pGPIO = GPIOE;
        break;
    case 6:
        *pGPIO = GPIOF;
        break;
    case 0: // Fall through
    default:
        return false;
    }

    if (pinnum >= 16)
        return false;

    *pPin = 1 << pinnum;

    return true;
}

static bool bbspihwChecked = false;
static bbspiHardware_t bbspihw;
static bool bbspihwValid = false;

static bool getGPIOSpec()
{
    if (bbspihwChecked)
        return bbspihwValid;

    if (getGPIOSpecFromCode(masterConfig.bbspi_ss_code,
            &bbspihw.ss_gpio, &bbspihw.ss_pin)
        && getGPIOSpecFromCode(masterConfig.bbspi_sck_code,
            &bbspihw.sck_gpio, &bbspihw.sck_pin)
        && getGPIOSpecFromCode(masterConfig.bbspi_mosi_code,
            &bbspihw.mosi_gpio, &bbspihw.mosi_pin)) {
	bbspihwValid = true;
    } else
        bbspihwValid = false;

    bbspihwChecked = true;

    return bbspihwValid;
}

bbspiHardware_t *bbspiGetHardwareConfig()
{
#if 0
    static bbspiHardware_t bbspihardware = {
        .ss_gpio = BBSPI_SS_GPIO,
        .ss_pin = BBSPI_SS_PIN,
        .sck_gpio = BBSPI_SCK_GPIO,
        .sck_pin = BBSPI_SCK_PIN,
        .mosi_gpio = BBSPI_MOSI_GPIO,
        .mosi_pin = BBSPI_MOSI_PIN
    };

    return &bbspihardware;
#else
    if (getGPIOSpec())
    	return &bbspihw;

    return NULL;
#endif
}

#define	GPIONAME(x) \
	((x) == GPIOA ? "GPIOA" \
	:(x) == GPIOB ? "GPIOB" \
	:(x) == GPIOC ? "GPIOC" \
	: "UNKNOWN")
#define	GPIOPIN(x) \
	(ffs(x) - 1)

bool bbspiInit(bbspi_t *bbspi)
{
    dprintf(("bbspiInit: ss %s:%d sck %s:%d mosi %s:%d\r\n",
	GPIONAME(bbspi->ss_gpio), GPIOPIN(bbspi->ss_cfg.pin),
	GPIONAME(bbspi->sck_gpio), GPIOPIN(bbspi->sck_cfg.pin),
	GPIONAME(bbspi->mosi_gpio), GPIOPIN(bbspi->mosi_cfg.pin)));

    gpioInit(bbspi->ss_gpio, &bbspi->ss_cfg);
    gpioInit(bbspi->sck_gpio, &bbspi->sck_cfg);
    gpioInit(bbspi->mosi_gpio, &bbspi->mosi_cfg);
    BBSPI_SS_HI(bbspi);

    bbspi->active = true;

    return true;
}

void bbspiWrite(bbspi_t *bbspi, uint32_t data, int bits)
{
    int i;

    BBSPI_SS_LO(bbspi);

    delayMicroseconds(5);

    for (i = 0; i < bits; i++) {

        if (data & 1)
            BBSPI_MOSI_HI(bbspi);
        else
            BBSPI_MOSI_LO(bbspi);

		delayMicroseconds(1);	// Wait for data to settle(!!!)

        BBSPI_SCK_HI(bbspi);

        delayMicroseconds(1);	// Paranoia

        BBSPI_SCK_LO(bbspi);

            data >>= 1;
    }

    BBSPI_SS_HI(bbspi);
}

//
// RTC6705 driver layer
//
#define Fosc    8       // Oscillator frequency in MHz
#define RREG    400     // Fixed R divisor

uint16_t freqTab[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
    { 5865, 5846, 5825, 5805, 5785, 5765, 5746, 5725 },
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 },
    { 5705, 5685, 5665, 5665, 5885, 5905, 5905, 5905 },
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 },
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }
};

#define RTC6705WDATA(addr, rw, data) \
    ((addr & 0xf) | (rw << 4) | (data << 5))

static void rtc6705_SetFreq(bbspi_t *bbspi, int freq)
{
    int g = freq * RREG / (2 * Fosc);
    int ndiv = g / 64;
    int adiv = g % 64;

    dprintf(("rtc6705_SetFreq: freq %d, R 0x%x N 0x%x A 0x%x\r\n", freq, RREG, ndiv, adiv));

    dprintf(("rtc6705_SetFreq: addr 0x%x data 0x%x -> 0x%x\r\n", 1, ((ndiv << 7)|adiv), RTC6705WDATA(1, 1, ((ndiv << 7)|adiv))));

    bbspiWrite(bbspi, RTC6705WDATA(0, 1, RREG), 25);

    delayMicroseconds(20);

    bbspiWrite(bbspi, RTC6705WDATA(1, 1, ((ndiv << 7)|adiv)), 25);
}

static void rtc6705_SetChan(bbspi_t *bbspi, int band, int chan)
{
    rtc6705_SetFreq(bbspi, freqTab[band][chan]);
}

//
// VTX instance and API
//

static bbspi_t vtxbbspi = {
#if 0
    .active = false,
    .ss_gpio = BBSPI_SS_GPIO,
    .ss_cfg = { BBSPI_SS_PIN, Mode_Out_PP, Speed_2MHz },
    .sck_gpio = BBSPI_SCK_GPIO,
    .sck_cfg = { BBSPI_SCK_PIN, Mode_Out_PP, Speed_2MHz },
    .mosi_gpio = BBSPI_MOSI_GPIO,
    .mosi_cfg = { BBSPI_MOSI_PIN, Mode_Out_PP, Speed_2MHz } 
#else
    .active = false,
    .ss_cfg = { .mode = Mode_Out_PP, .speed = Speed_2MHz },
    .sck_cfg = { .mode = Mode_Out_PP, .speed = Speed_2MHz },
    .mosi_cfg = { .mode = Mode_Out_PP, .speed = Speed_2MHz } 
#endif
};

bool vtxInit()
{
    dprintf(("vtxInit:\r\n"));

    if (!feature(FEATURE_BBSPI) || !getGPIOSpec())
        return false;

    vtxbbspi.ss_gpio = bbspihw.ss_gpio;
    vtxbbspi.ss_cfg.pin = bbspihw.ss_pin;
    vtxbbspi.sck_gpio = bbspihw.sck_gpio;
    vtxbbspi.sck_cfg.pin = bbspihw.sck_pin;
    vtxbbspi.mosi_gpio = bbspihw.mosi_gpio;
    vtxbbspi.mosi_cfg.pin = bbspihw.mosi_pin;

    if (!bbspiInit(&vtxbbspi)) {
        dprintf(("vtxInit: bbspiInit() failed\r\n"));
	return false;
    }

    delayMicroseconds(10000);

    return true;
}

void vtxSetFreq(int mhz)
{
    dprintf(("vtxSetFreq: mhz %d\r\n", mhz));
    if (vtxbbspi.active)
    	rtc6705_SetFreq(&vtxbbspi, mhz);
}

void vtxSetChan(int band, int chan)
{
    dprintf(("vtxSetChan: band %d chan %d\r\n", band, chan));
    if (vtxbbspi.active)
    	rtc6705_SetChan(&vtxbbspi, band - 1, chan - 1);
}
#endif
