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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Derived from: */
/*
 * Author: Sean Blakemore (sean@impulserc.com)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build_config.h"
#include "debug.h"
#include <platform.h>
#include "scheduler.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/sdcard.h"
#include "drivers/buf_writer.h"
#include "rx/rx.h"
#include "rx/msp.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/transponder_ir.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/vtxrc.h"

#include "drivers/vtx.h"

#include "telemetry/telemetry.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "mw.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "version.h"

<<<<<<< HEAD
=======
#include "io/rc_controls.h"
bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range); // XXX BAD Should be declared and exported in rc_controls.h

>>>>>>> 2c95c0e... Initial commit for VTXRC integration
#ifdef VTX
void vtxRcInit()
{
    vtxInit();

#ifndef VTXRC
    if (!(masterConfig.vtx_mode == 0 || masterConfig.vtx_mode == 1))
	masterConfig.vtx_mode = 0;
#endif

    if (masterConfig.vtx_mode == 0) {
        vtxSetChan(masterConfig.vtx_band, masterConfig.vtx_channel);
    } else if (masterConfig.vtx_mode == 1) {
        vtxSetFreq(masterConfig.vtx_mhz);
    }
}
#endif
<<<<<<< HEAD
=======

#ifdef VTXRC

static uint8_t locked = 0;

static void setChannelSaveAndNotify(uint8_t *bandOrChannel, uint8_t step, int32_t min, int32_t max)
{
    if (ARMING_FLAG(ARMED))
        locked = 1;

    if (masterConfig.vtx_mode == 0 && !locked) {
        uint8_t temp = (*bandOrChannel) + step;
        temp = constrain(temp, min, max);
        *bandOrChannel = temp;

        vtxSetChan(masterConfig.vtx_band, masterConfig.vtx_channel);
        writeEEPROM();
        readEEPROM();
        beeperConfirmationBeeps(temp);
    }
}

void vtxRcIncrementBand()
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), 1, VTX_BAND_MIN, VTX_BAND_MAX);
}

void vtxRcDecrementBand()
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), -1, VTX_BAND_MIN, VTX_BAND_MAX);
}

void vtxRcIncrementChannel()
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), 1, VTX_CHANNEL_MIN, VTX_CHANNEL_MAX);
}

void vtxRcDecrementChannel()
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), -1, VTX_CHANNEL_MIN, VTX_CHANNEL_MAX);
}

void vtxRcUpdateActivatedChannel(vtxRcChannelActivationCondition_t *vtxRcChannelActivationConditions)
{
    if (ARMING_FLAG(ARMED))
        locked = 1;

    if (masterConfig.vtx_mode == 2 && !locked) {
        static uint8_t lastIndex = -1;
        uint8_t index;

        for (index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            vtxRcChannelActivationCondition_t *vtxRcChannelActivationCondition = &vtxRcChannelActivationConditions[index];

            if (isRangeActive(vtxRcChannelActivationCondition->auxChannelIndex, &vtxRcChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;
                vtxSetChan(vtxRcChannelActivationCondition->band, vtxRcChannelActivationCondition->channel);
                break;
            }
        }
    }
}
#endif
>>>>>>> 2c95c0e... Initial commit for VTXRC integration
