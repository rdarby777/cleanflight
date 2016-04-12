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

/*
 * Author: Sean Blakemore (sean@impulserc.com)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#pragma once

// XXX Do these belong here? Variable-based to support 1.3 and 2.4G?
#define VTX_BAND_MIN    1
#define VTX_BAND_MAX    5
#define VTX_CHANNEL_MIN	1
#define VTX_CHANNEL_MAX 8

#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT     10
#define MAX_VTX_CUSTOM_CHANNEL 8

typedef struct vtxRcChannelActivationCondition_s {
    uint8_t auxChannelIndex;
    uint8_t band;
    uint8_t channel;
    channelRange_t range;
} vtxRcChannelActivationCondition_t;

void vtxRcInit();
void vtxRcIncrementBand();
void vtxRcDecrementBand();
void vtxRcIncrementChannel();
void vtxRcDecrementChannel();
void vtxRcUpdateActivatedChannel();
