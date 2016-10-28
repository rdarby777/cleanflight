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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#include "hardware_revision.h"

void targetConfiguration(master_t *config)
{
#ifdef BEEBRAIN
    // alternative defaults settings for Beebrain target
    config->motorConfig.motorPwmRate = 4000;
    config->failsafeConfig.failsafe_delay = 2;
    config->failsafeConfig.failsafe_off_delay = 0;

    config->motorConfig.minthrottle = 1049;

    config->gyro_lpf = 1;
    config->gyro_soft_lpf_hz = 100;
    config->gyro_soft_notch_hz_1 = 0;
    config->gyro_soft_notch_hz_2 = 0;

    /*for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        config->rxConfig.channelRanges[channel].min = 1180;
        config->rxConfig.channelRanges[channel].max = 1860;
    }*/

    for (int profileId = 0; profileId < 2; profileId++) {
        config->profile[profileId].pidProfile.P8[ROLL] = 70;
        config->profile[profileId].pidProfile.I8[ROLL] = 70;
        config->profile[profileId].pidProfile.D8[ROLL] = 30;
        config->profile[profileId].pidProfile.P8[PITCH] = 80;
        config->profile[profileId].pidProfile.I8[PITCH] = 80;
        config->profile[profileId].pidProfile.D8[PITCH] = 30;
        config->profile[profileId].pidProfile.P8[YAW] = 180;
        config->profile[profileId].pidProfile.I8[YAW] = 45;
        config->profile[profileId].pidProfile.P8[PIDLEVEL] = 50;
        config->profile[profileId].pidProfile.D8[PIDLEVEL] = 50;
        config->profile[profileId].pidProfile.levelSensitivity = 1.0f;

        for (int rateProfileId = 0; rateProfileId < MAX_RATEPROFILES; rateProfileId++) {
            config->profile[profileId].controlRateProfile[rateProfileId].rcRate8 = 100;
            config->profile[profileId].controlRateProfile[rateProfileId].rcYawRate8 = 110;
            config->profile[profileId].controlRateProfile[rateProfileId].rcExpo8 = 20;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[ROLL] = 80;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[PITCH] = 80;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[YAW] = 80;

            config->profile[profileId].pidProfile.dtermSetpointWeight = 254;
            config->profile[profileId].pidProfile.setpointRelaxRatio = 100;
        }
    }
#endif
        
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        config->beeperConfig.isOD = false;
        config->beeperConfig.isInverted = true;
    }
}

