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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/system.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/io.h"
#include "drivers/flash.h"
#include "drivers/sdcard.h"
#include "drivers/max7456.h"
#include "drivers/vtx_soft_spi_rtc6705.h"
#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/mw.h"
#include "fc/fc_msp.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/transponder_ir.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/serial_4way.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif


#include "io/serial_4way.h"

extern uint16_t cycleTime; // FIXME dependency on mw.c
extern void resetProfile(profile_t *profile);

static const char * const flightControllerIdentifier = BETAFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

typedef struct box_e {
    const uint8_t boxId;         // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} box_t;

// FIXME remove ;'s
static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    //{ BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXGTUNE, "GTUNE;", 21 },
    { BOXSONAR, "SONAR;", 22 },
    { BOXSERVO1, "SERVO1;", 23 },
    { BOXSERVO2, "SERVO2;", 24 },
    { BOXSERVO3, "SERVO3;", 25 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { BOXAIRMODE, "AIR MODE;", 28 },
    { BOX3DDISABLESWITCH, "DISABLE 3D SWITCH;", 29},
    { BOXFPVANGLEMIX, "FPV ANGLE MIX;", 30},
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;


#define RATEPROFILE_MASK (1 << 7)

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
static void msp4WayIfFn(serialPort_t *serialPort)
{
    // rem: App: Wait at least appx. 500 ms for BLHeli to jump into
    // bootloader mode before try to connect any ESC
    // Start to activate here
    esc4wayProcess(serialPort);
    // former used MSP uart is still active
    // proceed as usual with MSP commands
}
#endif

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    stopPwmAllMotors();
    systemReset();

    // control should never return here.
    while (true) ;
}

static void serializeNames(sbuf_t *dst, const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        sbufWriteU8(dst, *c);
}

static const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permenantId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(sbuf_t *dst)
{
    int i, activeBoxId, j, flag = 1, count = 0, len;
    const box_t *box;

reset:
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < activeBoxIdCount; i++) {
        activeBoxId = activeBoxIds[i];

        box = findBoxByActiveBoxId(activeBoxId);
        if (!box) {
            continue;
        }

        len = strlen(box->boxName);
        if (flag) {
            count += len;
        } else {
            for (j = 0; j < len; j++)
                sbufWriteU8(dst, box->boxName[j]);
        }
    }

    if (flag) {
        flag = 0;
        goto reset;
    }
}

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
#ifdef USE_SDCARD
    uint8_t flags = 1 /* SD card supported */ ;
    uint8_t state = 0;

    sbufWriteU8(dst, flags);

    // Merge the card and filesystem states together
    if (!sdcard_isInserted()) {
        state = MSP_SDCARD_STATE_NOT_PRESENT;
    } else if (!sdcard_isFunctional()) {
        state = MSP_SDCARD_STATE_FATAL;
    } else {
        switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
            break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                if (sdcard_isInitialized()) {
                    state = MSP_SDCARD_STATE_FS_INIT;
                } else {
                    state = MSP_SDCARD_STATE_CARD_INIT;
                }
            break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
                state = MSP_SDCARD_STATE_FATAL;
            break;
        }
    }

    sbufWriteU8(dst, state);
    sbufWriteU8(dst, afatfs_getLastError());
    // Write free space and total space in kilobytes
    sbufWriteU32(dst, afatfs_getContiguousFreeSpace() / 1024);
    sbufWriteU32(dst, sdcard_getMetadata()->numBlocks / 2); // Block size is half a kilobyte
#else
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    uint8_t flags = (flashfsIsReady() ? 1 : 0) | 2 /* FlashFS is supported */;

    sbufWriteU8(dst, flags);
    sbufWriteU32(dst, geometry->sectors);
    sbufWriteU32(dst, geometry->totalSize);
    sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    sbufWriteU8(dst, 0); // FlashFS is neither ready nor supported
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, const uint16_t size, bool useLegacyFormat)
{
    BUILD_BUG_ON(MSP_PORT_DATAFLASH_INFO_SIZE < 16);

    uint16_t readLen = size;
    const int bytesRemainingInBuf = sbufBytesRemaining(dst) - MSP_PORT_DATAFLASH_INFO_SIZE;
    if (readLen > bytesRemainingInBuf) {
        readLen = bytesRemainingInBuf;
    }
    // size will be lower than that requested if we reach end of volume
    if (readLen > flashfsGetSize() - address) {
        // truncate the request
        readLen = flashfsGetSize() - address;
    }
    sbufWriteU32(dst, address);
    if (!useLegacyFormat) {
        // new format supports variable read lengths
        sbufWriteU16(dst, readLen);
        sbufWriteU8(dst, 0); // placeholder for compression format
    }

    // bytesRead will equal readLen
    const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);
    sbufAdvance(dst, bytesRead);

    if (useLegacyFormat) {
        // pad the buffer with zeros
        for (int i = bytesRead; i < size; i++) {
            sbufWriteU8(dst, 0);
        }
    }
}
#endif

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    activeBoxIdCount = 0;
    activeBoxIds[activeBoxIdCount++] = BOXARM;

    if (!feature(FEATURE_AIRMODE)) {
	activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;
    }

    if (sensors(SENSOR_ACC)) {
        activeBoxIds[activeBoxIdCount++] = BOXANGLE;
        activeBoxIds[activeBoxIdCount++] = BOXHORIZON;
    }

    if (sensors(SENSOR_BARO)) {
        activeBoxIds[activeBoxIdCount++] = BOXBARO;
    }

    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        activeBoxIds[activeBoxIdCount++] = BOXMAG;
        activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
        activeBoxIds[activeBoxIdCount++] = BOXHEADADJ;
    }

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        activeBoxIds[activeBoxIdCount++] = BOXGPSHOME;
        activeBoxIds[activeBoxIdCount++] = BOXGPSHOLD;
    }
#endif

#ifdef SONAR
    if (feature(FEATURE_SONAR)) {
        activeBoxIds[activeBoxIdCount++] = BOXSONAR;
    }
#endif

    if (feature(FEATURE_FAILSAFE)) {
        activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;
    }

    if (masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE) {
        activeBoxIds[activeBoxIdCount++] = BOXPASSTHRU;
    }

    activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        activeBoxIds[activeBoxIdCount++] = BOXLEDLOW;
    }
#endif

#ifdef BLACKBOX
    if (feature(FEATURE_BLACKBOX)) {
        activeBoxIds[activeBoxIdCount++] = BOXBLACKBOX;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXFPVANGLEMIX;

    if (feature(FEATURE_3D)) {
    	activeBoxIds[activeBoxIdCount++] = BOX3DDISABLESWITCH;
    }

    if (feature(FEATURE_SERVO_TILT)) {
        activeBoxIds[activeBoxIdCount++] = BOXCAMSTAB;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        activeBoxIds[activeBoxIdCount++] = BOXCALIB;
    }

    if (feature(FEATURE_OSD)) {
	activeBoxIds[activeBoxIdCount++] = BOXOSD;
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY) && masterConfig.telemetryConfig.telemetry_switch) {
        activeBoxIds[activeBoxIdCount++] = BOXTELEMETRY;
    }
#endif

#ifdef GTUNE
    activeBoxIds[activeBoxIdCount++] = BOXGTUNE;
#endif

#ifdef USE_SERVOS
    if (masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE) {
        activeBoxIds[activeBoxIdCount++] = BOXSERVO1;
        activeBoxIds[activeBoxIdCount++] = BOXSERVO2;
        activeBoxIds[activeBoxIdCount++] = BOXSERVO3;
    }
#endif
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

static uint32_t packFlightModeFlags(void)
{
    uint32_t i, junk, tmp;

    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    junk = 0;
    tmp = IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << BOXANGLE |
        IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << BOXHORIZON |
        IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << BOXBARO |
        IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << BOXMAG |
        IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)) << BOXHEADFREE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)) << BOXHEADADJ |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)) << BOXCAMSTAB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMTRIG)) << BOXCAMTRIG |
        IS_ENABLED(FLIGHT_MODE(GPS_HOME_MODE)) << BOXGPSHOME |
        IS_ENABLED(FLIGHT_MODE(GPS_HOLD_MODE)) << BOXGPSHOLD |
        IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)) << BOXPASSTHRU |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)) << BOXBEEPERON |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDMAX)) << BOXLEDMAX |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)) << BOXLEDLOW |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)) << BOXLLIGHTS |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCALIB)) << BOXCALIB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGOV)) << BOXGOV |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)) << BOXOSD |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)) << BOXTELEMETRY |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGTUNE)) << BOXGTUNE |
        IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << BOXSONAR |
        IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
        IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << BOXAIRMODE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX)) << BOXFPVANGLEMIX;

    for (i = 0; i < activeBoxIdCount; i++) {
        int flag = (tmp & (1 << activeBoxIds[i]));
        if (flag)
            junk |= 1 << i;
    }

    return junk;
}

static bool mspFcProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    uint32_t i;

    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        for (i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
            sbufWriteU8(dst, flightControllerIdentifier[i]);
        }
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
        for (i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
            sbufWriteU8(dst, boardIdentifier[i]);
        }
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
        break;

    case MSP_BUILD_INFO:
        for (i = 0; i < BUILD_DATE_LENGTH; i++) {
            sbufWriteU8(dst, buildDate[i]);
        }
        for (i = 0; i < BUILD_TIME_LENGTH; i++) {
            sbufWriteU8(dst, buildTime[i]);
        }

        for (i = 0; i < GIT_SHORT_REVISION_LENGTH; i++) {
            sbufWriteU8(dst, shortGitRevision[i]);
        }
        break;

    // DEPRECATED - Use MSP_API_VERSION
    case MSP_IDENT:
        sbufWriteU8(dst, MW_VERSION);
        sbufWriteU8(dst, masterConfig.mixerMode);
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU32(dst, CAP_DYNBALANCE); // "capability"
        break;

    case MSP_STATUS_EX:
        sbufWriteU16(dst, cycleTime);
#ifdef USE_I2C
        sbufWriteU16(dst, i2cGetErrorCounter());
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        sbufWriteU32(dst, packFlightModeFlags());
        sbufWriteU8(dst, getCurrentProfile());
        sbufWriteU16(dst, constrain(averageSystemLoadPercent, 0, 100));
        sbufWriteU8(dst, MAX_PROFILE_COUNT);
        sbufWriteU8(dst, getCurrentControlRateProfile());
        break;

    case MSP_NAME:
        {
            const unsigned int nameLen = strlen(masterConfig.name);
            for (i = 0; i < nameLen; i++) {
                sbufWriteU8(dst, masterConfig.name[i]);
            }
        }
        break;

    case MSP_STATUS:
        sbufWriteU16(dst, cycleTime);
#ifdef USE_I2C
        sbufWriteU16(dst, i2cGetErrorCounter());
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        sbufWriteU32(dst, packFlightModeFlags());
        sbufWriteU8(dst, masterConfig.current_profile_index);
        break;
    case MSP_RAW_IMU:
        {
            // Hack scale due to choice of units for sensor data in multiwii
            const uint8_t scale = (acc.acc_1G > 512) ? 4 : 1;
            for (i = 0; i < 3; i++)
                sbufWriteU16(dst, accSmooth[i] / scale);
            for (i = 0; i < 3; i++)
                sbufWriteU16(dst, gyroADC[i]);
            for (i = 0; i < 3; i++)
                sbufWriteU16(dst, magADC[i]);
        }
        break;
#ifdef USE_SERVOS
    case MSP_SERVO:
        sbufWriteData(dst, &servo, MAX_SUPPORTED_SERVOS * 2);
        break;
    case MSP_SERVO_CONFIGURATIONS:
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, masterConfig.servoConf[i].min);
            sbufWriteU16(dst, masterConfig.servoConf[i].max);
            sbufWriteU16(dst, masterConfig.servoConf[i].middle);
            sbufWriteU8(dst, masterConfig.servoConf[i].rate);
            sbufWriteU8(dst, masterConfig.servoConf[i].angleAtMin);
            sbufWriteU8(dst, masterConfig.servoConf[i].angleAtMax);
            sbufWriteU8(dst, masterConfig.servoConf[i].forwardFromChannel);
            sbufWriteU32(dst, masterConfig.servoConf[i].reversedSources);
        }
        break;
    case MSP_SERVO_MIX_RULES:
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            sbufWriteU8(dst, masterConfig.customServoMixer[i].targetChannel);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].inputSource);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].rate);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].speed);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].min);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].max);
            sbufWriteU8(dst, masterConfig.customServoMixer[i].box);
        }
        break;
#endif
    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++) {
            if (i >= MAX_SUPPORTED_MOTORS || !pwmGetMotors()[i].enabled) {
                sbufWriteU16(dst, 0);
                continue;
            }

            sbufWriteU16(dst, convertMotorToExternal(motor[i]));
        }
        break;
    case MSP_RC:
        for (i = 0; i < rxRuntimeConfig.channelCount; i++)
            sbufWriteU16(dst, rcData[i]);
        break;
    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;
    case MSP_ALTITUDE:
#if defined(BARO) || defined(SONAR)
        sbufWriteU32(dst, altitudeHoldGetEstimatedAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        sbufWriteU16(dst, vario);
        break;
    case MSP_SONAR_ALTITUDE:
#if defined(SONAR)
        sbufWriteU32(dst, sonarGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;
    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, rssi);
        if(masterConfig.batteryConfig.multiwiiCurrentMeterOutput) {
            sbufWriteU16(dst, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
        } else
            sbufWriteU16(dst, (int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
        break;
    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, masterConfig.auto_disarm_delay);
        sbufWriteU8(dst, masterConfig.disarm_kill_switch);
        break;
    case MSP_LOOP_TIME:
        sbufWriteU16(dst, (uint16_t)gyro.targetLooptime);
        break;
    case MSP_RC_TUNING:
        sbufWriteU8(dst, currentControlRateProfile->rcRate8);
        sbufWriteU8(dst, currentControlRateProfile->rcExpo8);
        for (i = 0 ; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        sbufWriteU8(dst, currentControlRateProfile->dynThrPID);
        sbufWriteU8(dst, currentControlRateProfile->thrMid8);
        sbufWriteU8(dst, currentControlRateProfile->thrExpo8);
        sbufWriteU16(dst, currentControlRateProfile->tpa_breakpoint);
        sbufWriteU8(dst, currentControlRateProfile->rcYawExpo8);
        sbufWriteU8(dst, currentControlRateProfile->rcYawRate8);
        break;
    case MSP_PID:
        for (i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU8(dst, currentProfile->pidProfile.P8[i]);
            sbufWriteU8(dst, currentProfile->pidProfile.I8[i]);
            sbufWriteU8(dst, currentProfile->pidProfile.D8[i]);
        }
        break;
    case MSP_PIDNAMES:
        serializeNames(dst, pidnames);
        break;
    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, PID_CONTROLLER_BETAFLIGHT);
        break;
    case MSP_MODE_RANGES:
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = &masterConfig.modeActivationConditions[i];
            const box_t *box = &boxes[mac->modeId];
            sbufWriteU8(dst, box->permanentId);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;
    case MSP_ADJUSTMENT_RANGES:
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *adjRange = &masterConfig.adjustmentRanges[i];
            sbufWriteU8(dst, adjRange->adjustmentIndex);
            sbufWriteU8(dst, adjRange->auxChannelIndex);
            sbufWriteU8(dst, adjRange->range.startStep);
            sbufWriteU8(dst, adjRange->range.endStep);
            sbufWriteU8(dst, adjRange->adjustmentFunction);
            sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
        }
        break;
    case MSP_BOXNAMES:
        serializeBoxNamesReply(dst);
        break;
    case MSP_BOXIDS:
        for (i = 0; i < activeBoxIdCount; i++) {
            const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
            if (!box) {
                continue;
            }
            sbufWriteU8(dst, box->permanentId);
        }
        break;
    case MSP_MISC:
        sbufWriteU16(dst, masterConfig.rxConfig.midrc);

        sbufWriteU16(dst, masterConfig.motorConfig.minthrottle);
        sbufWriteU16(dst, masterConfig.motorConfig.maxthrottle);
        sbufWriteU16(dst, masterConfig.motorConfig.mincommand);

        sbufWriteU16(dst, masterConfig.failsafeConfig.failsafe_throttle);

#ifdef GPS
        sbufWriteU8(dst, masterConfig.gpsConfig.provider); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, masterConfig.gpsConfig.sbasMode); // gps_ubx_sbas
#else
        sbufWriteU8(dst, 0); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, 0); // gps_ubx_sbas
#endif
        sbufWriteU8(dst, masterConfig.batteryConfig.multiwiiCurrentMeterOutput);
        sbufWriteU8(dst, masterConfig.rxConfig.rssi_channel);
        sbufWriteU8(dst, 0);

        sbufWriteU16(dst, masterConfig.mag_declination / 10);

        sbufWriteU8(dst, masterConfig.batteryConfig.vbatscale);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatmincellvoltage);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatmaxcellvoltage);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatwarningcellvoltage);
        break;

    case MSP_MOTOR_PINS:
        // FIXME This is hardcoded and should not be.
        for (i = 0; i < 8; i++)
            sbufWriteU8(dst, i + 1);
        break;
#ifdef GPS
    case MSP_RAW_GPS:
        sbufWriteU8(dst, STATE(GPS_FIX));
        sbufWriteU8(dst, GPS_numSat);
        sbufWriteU32(dst, GPS_coord[LAT]);
        sbufWriteU32(dst, GPS_coord[LON]);
        sbufWriteU16(dst, GPS_altitude);
        sbufWriteU16(dst, GPS_speed);
        sbufWriteU16(dst, GPS_ground_course);
        break;
    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, GPS_update & 1);
        break;
    case MSP_GPSSVINFO:
        sbufWriteU8(dst, GPS_numCh);
           for (i = 0; i < GPS_numCh; i++){
               sbufWriteU8(dst, GPS_svinfo_chn[i]);
               sbufWriteU8(dst, GPS_svinfo_svid[i]);
               sbufWriteU8(dst, GPS_svinfo_quality[i]);
               sbufWriteU8(dst, GPS_svinfo_cno[i]);
           }
        break;
#endif
    case MSP_DEBUG:
        // output some useful QA statistics
        // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

        for (i = 0; i < DEBUG16_VALUE_COUNT; i++)
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, masterConfig.accelerometerTrims.values.pitch);
        sbufWriteU16(dst, masterConfig.accelerometerTrims.values.roll);
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

    case MSP_FEATURE:
        sbufWriteU32(dst, featureMask());
        break;

    case MSP_BOARD_ALIGNMENT:
        sbufWriteU16(dst, masterConfig.boardAlignment.rollDegrees);
        sbufWriteU16(dst, masterConfig.boardAlignment.pitchDegrees);
        sbufWriteU16(dst, masterConfig.boardAlignment.yawDegrees);
        break;

    case MSP_VOLTAGE_METER_CONFIG:
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatscale);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatmincellvoltage);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatmaxcellvoltage);
        sbufWriteU8(dst, masterConfig.batteryConfig.vbatwarningcellvoltage);
        break;

    case MSP_CURRENT_METER_CONFIG:
        sbufWriteU16(dst, masterConfig.batteryConfig.currentMeterScale);
        sbufWriteU16(dst, masterConfig.batteryConfig.currentMeterOffset);
        sbufWriteU8(dst, masterConfig.batteryConfig.currentMeterType);
        sbufWriteU16(dst, masterConfig.batteryConfig.batteryCapacity);
        break;

    case MSP_MIXER:
        sbufWriteU8(dst, masterConfig.mixerMode);
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, masterConfig.rxConfig.serialrx_provider);
        sbufWriteU16(dst, masterConfig.rxConfig.maxcheck);
        sbufWriteU16(dst, masterConfig.rxConfig.midrc);
        sbufWriteU16(dst, masterConfig.rxConfig.mincheck);
        sbufWriteU8(dst, masterConfig.rxConfig.spektrum_sat_bind);
        sbufWriteU16(dst, masterConfig.rxConfig.rx_min_usec);
        sbufWriteU16(dst, masterConfig.rxConfig.rx_max_usec);
        sbufWriteU8(dst, masterConfig.rxConfig.rcInterpolation);
        sbufWriteU8(dst, masterConfig.rxConfig.rcInterpolationInterval);
        sbufWriteU16(dst, masterConfig.rxConfig.airModeActivateThreshold);
        sbufWriteU8(dst, masterConfig.rxConfig.rx_spi_protocol);
        sbufWriteU32(dst, masterConfig.rxConfig.rx_spi_id);
        sbufWriteU8(dst, masterConfig.rxConfig.rx_spi_rf_channel_count);
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, masterConfig.failsafeConfig.failsafe_delay);
        sbufWriteU8(dst, masterConfig.failsafeConfig.failsafe_off_delay);
        sbufWriteU16(dst, masterConfig.failsafeConfig.failsafe_throttle);
        sbufWriteU8(dst, masterConfig.failsafeConfig.failsafe_kill_switch);
        sbufWriteU16(dst, masterConfig.failsafeConfig.failsafe_throttle_low_delay);
        sbufWriteU8(dst, masterConfig.failsafeConfig.failsafe_procedure);
        break;

    case MSP_RXFAIL_CONFIG:
        for (i = 0; i < rxRuntimeConfig.channelCount; i++) {
            sbufWriteU8(dst, masterConfig.rxConfig.failsafe_channel_configurations[i].mode);
            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(masterConfig.rxConfig.failsafe_channel_configurations[i].step));
        }
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, masterConfig.rxConfig.rssi_channel);
        break;

    case MSP_RX_MAP:
        for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++)
            sbufWriteU8(dst, masterConfig.rxConfig.rcmap[i]);
        break;

    case MSP_BF_CONFIG:
        sbufWriteU8(dst, masterConfig.mixerMode);

        sbufWriteU32(dst, featureMask());

        sbufWriteU8(dst, masterConfig.rxConfig.serialrx_provider);

        sbufWriteU16(dst, masterConfig.boardAlignment.rollDegrees);
        sbufWriteU16(dst, masterConfig.boardAlignment.pitchDegrees);
        sbufWriteU16(dst, masterConfig.boardAlignment.yawDegrees);

        sbufWriteU16(dst, masterConfig.batteryConfig.currentMeterScale);
        sbufWriteU16(dst, masterConfig.batteryConfig.currentMeterOffset);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, masterConfig.serialConfig.portConfigs[i].identifier);
            sbufWriteU16(dst, masterConfig.serialConfig.portConfigs[i].functionMask);
            sbufWriteU8(dst, masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex);
        }
        break;

#ifdef LED_STRIP
    case MSP_LED_COLORS:
        for (i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &masterConfig.colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;

    case MSP_LED_STRIP_CONFIG:
        for (i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
            ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
            sbufWriteU32(dst, *ledConfig);
        }
        break;

    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, masterConfig.modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, masterConfig.specialColors.color[j]);
        }
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_BLACKBOX_CONFIG:
#ifdef BLACKBOX
        sbufWriteU8(dst, 1); //Blackbox supported
        sbufWriteU8(dst, masterConfig.blackbox_device);
        sbufWriteU8(dst, masterConfig.blackbox_rate_num);
        sbufWriteU8(dst, masterConfig.blackbox_rate_denom);
#else
        sbufWriteU8(dst, 0); // Blackbox not supported
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_TRANSPONDER_CONFIG:
#ifdef TRANSPONDER
        sbufWriteU8(dst, 1); //Transponder supported
        for (i = 0; i < sizeof(masterConfig.transponderData); i++) {
            sbufWriteU8(dst, masterConfig.transponderData[i]);
        }
#else
        sbufWriteU8(dst, 0); // Transponder not supported
#endif
        break;

    case MSP_OSD_CONFIG:
#ifdef OSD
        sbufWriteU8(dst, 1); // OSD supported
        // send video system (AUTO/PAL/NTSC)
        sbufWriteU8(dst, masterConfig.osdProfile.video_system);
        sbufWriteU8(dst, masterConfig.osdProfile.units);
        sbufWriteU8(dst, masterConfig.osdProfile.rssi_alarm);
        sbufWriteU16(dst, masterConfig.osdProfile.cap_alarm);
        sbufWriteU16(dst, masterConfig.osdProfile.time_alarm);
        sbufWriteU16(dst, masterConfig.osdProfile.alt_alarm);

        for (i = 0; i < OSD_MAX_ITEMS; i++) {
            sbufWriteU16(dst, masterConfig.osdProfile.item_pos[i]);
        }
#else
        sbufWriteU8(dst, 0); // OSD not supported
#endif
        break;

    case MSP_BF_BUILD_INFO:
        for (i = 0; i < 11; i++) {
            sbufWriteU8(dst, buildDate[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        }
        sbufWriteU32(dst, 0); // future exp
        sbufWriteU32(dst, 0); // future exp
        break;

    case MSP_3D:
        sbufWriteU16(dst, masterConfig.flight3DConfig.deadband3d_low);
        sbufWriteU16(dst, masterConfig.flight3DConfig.deadband3d_high);
        sbufWriteU16(dst, masterConfig.flight3DConfig.neutral3d);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, masterConfig.rcControlsConfig.deadband);
        sbufWriteU8(dst, masterConfig.rcControlsConfig.yaw_deadband);
        sbufWriteU8(dst, masterConfig.rcControlsConfig.alt_hold_deadband);
        sbufWriteU16(dst, masterConfig.flight3DConfig.deadband3d_throttle);
        break;
    case MSP_SENSOR_ALIGNMENT:
        sbufWriteU8(dst, masterConfig.sensorAlignmentConfig.gyro_align);
        sbufWriteU8(dst, masterConfig.sensorAlignmentConfig.acc_align);
        sbufWriteU8(dst, masterConfig.sensorAlignmentConfig.mag_align);
        break;
    case MSP_ADVANCED_CONFIG :
        if (masterConfig.gyro_lpf) {
            sbufWriteU8(dst, 8); // If gyro_lpf != OFF then looptime is set to 1000
            sbufWriteU8(dst, 1);
        } else {
            sbufWriteU8(dst, masterConfig.gyro_sync_denom);
            sbufWriteU8(dst, masterConfig.pid_process_denom);
        }
        sbufWriteU8(dst, masterConfig.motorConfig.useUnsyncedPwm);
        sbufWriteU8(dst, masterConfig.motorConfig.motorPwmProtocol);
        sbufWriteU16(dst, masterConfig.motorConfig.motorPwmRate);
        break;
    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, masterConfig.gyro_soft_lpf_hz);
        sbufWriteU16(dst, currentProfile->pidProfile.dterm_lpf_hz);
        sbufWriteU16(dst, currentProfile->pidProfile.yaw_lpf_hz);
        sbufWriteU16(dst, masterConfig.gyro_soft_notch_hz_1);
        sbufWriteU16(dst, masterConfig.gyro_soft_notch_cutoff_1);
        sbufWriteU16(dst, currentProfile->pidProfile.dterm_notch_hz);
        sbufWriteU16(dst, currentProfile->pidProfile.dterm_notch_cutoff);
        sbufWriteU16(dst, masterConfig.gyro_soft_notch_hz_2);
        sbufWriteU16(dst, masterConfig.gyro_soft_notch_cutoff_2);
        break;
    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, currentProfile->pidProfile.rollPitchItermIgnoreRate);
        sbufWriteU16(dst, currentProfile->pidProfile.yawItermIgnoreRate);
        sbufWriteU16(dst, currentProfile->pidProfile.yaw_p_limit);
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, currentProfile->pidProfile.vbatPidCompensation);
        sbufWriteU8(dst, currentProfile->pidProfile.setpointRelaxRatio);
        sbufWriteU8(dst, currentProfile->pidProfile.dtermSetpointWeight);
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, currentProfile->pidProfile.itermThrottleGain);
        sbufWriteU16(dst, currentProfile->pidProfile.rateAccelLimit);
        sbufWriteU16(dst, currentProfile->pidProfile.yawRateAccelLimit);
        break;
    case MSP_SENSOR_CONFIG:
        sbufWriteU8(dst, masterConfig.acc_hardware);
        sbufWriteU8(dst, masterConfig.baro_hardware);
        sbufWriteU8(dst, masterConfig.mag_hardware);
        break;

    case MSP_REBOOT:
        if (mspPostProcessFn) {
            *mspPostProcessFn = mspRebootFn;
        }
        break;

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_SET_4WAY_IF:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());
        if (mspPostProcessFn) {
            *mspPostProcessFn = msp4WayIfFn;
        }
        break;
#endif

    default:
        return false;
    }
    return true;
}

#ifdef GPS
static void mspFcWpCommand(sbuf_t *dst, sbuf_t *src)
{
    uint8_t wp_no;
    int32_t lat = 0, lon = 0;
    wp_no = sbufReadU8(src);    // get the wp number
    if (wp_no == 0) {
        lat = GPS_home[LAT];
        lon = GPS_home[LON];
    } else if (wp_no == 16) {
        lat = GPS_hold[LAT];
        lon = GPS_hold[LON];
    }
    sbufWriteU8(dst, wp_no);
    sbufWriteU32(dst, lat);
    sbufWriteU32(dst, lon);
    sbufWriteU32(dst, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
    sbufWriteU16(dst, 0);                 // heading  will come here (deg)
    sbufWriteU16(dst, 0);                 // time to stay (ms) will come here
    sbufWriteU8(dst, 0);                  // nav flag will come here
}
#endif

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    const uint32_t readAddress = sbufReadU32(src);
    uint16_t readLength;
    bool useLegacyFormat;
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
        useLegacyFormat = false;
    } else {
        readLength = 128;
        useLegacyFormat = true;
    }

    serializeDataflashReadReply(dst, readAddress, readLength, useLegacyFormat);
}
#endif

static mspResult_e mspFcProcessInCommand(uint8_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint16_t tmp;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif
#ifdef OSD
    uint8_t addr, font_data[64];
#endif
    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        value = sbufReadU8(src);
        if ((value & RATEPROFILE_MASK) == 0) {
            if (!ARMING_FLAG(ARMED)) {
                if (value >= MAX_PROFILE_COUNT) {
                    value = 0;
                }
                changeProfile(value);
            }
        } else {
            value = value & ~RATEPROFILE_MASK;

            if (value >= MAX_RATEPROFILES) {
                value = 0;
            }
            changeControlRateProfile(value);
        }

        break;
    case MSP_SET_HEAD:
        magHold = sbufReadU16(src);
        break;
    case MSP_SET_RAW_RC:
#ifndef SKIP_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];

                for (i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }

                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;
    case MSP_SET_ACC_TRIM:
        masterConfig.accelerometerTrims.values.pitch = sbufReadU16(src);
        masterConfig.accelerometerTrims.values.roll  = sbufReadU16(src);
        break;
    case MSP_SET_ARMING_CONFIG:
        masterConfig.auto_disarm_delay = sbufReadU8(src);
        masterConfig.disarm_kill_switch = sbufReadU8(src);
        break;
    case MSP_SET_LOOP_TIME:
        sbufReadU16(src);
        break;
    case MSP_SET_PID_CONTROLLER:
        break;
    case MSP_SET_PID:
        for (i = 0; i < PID_ITEM_COUNT; i++) {
            currentProfile->pidProfile.P8[i] = sbufReadU8(src);
            currentProfile->pidProfile.I8[i] = sbufReadU8(src);
            currentProfile->pidProfile.D8[i] = sbufReadU8(src);
        }
        break;
    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &masterConfig.modeActivationConditions[i];
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermenantId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);

                useRcControlsConfig(masterConfig.modeActivationConditions, &masterConfig.motorConfig, &currentProfile->pidProfile);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;
    case MSP_SET_ADJUSTMENT_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = &masterConfig.adjustmentRanges[i];
            i = sbufReadU8(src);
            if (i < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                adjRange->adjustmentIndex = i;
                adjRange->auxChannelIndex = sbufReadU8(src);
                adjRange->range.startStep = sbufReadU8(src);
                adjRange->range.endStep = sbufReadU8(src);
                adjRange->adjustmentFunction = sbufReadU8(src);
                adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        if (dataSize >= 10) {
            currentControlRateProfile->rcRate8 = sbufReadU8(src);
            currentControlRateProfile->rcExpo8 = sbufReadU8(src);
            for (i = 0; i < 3; i++) {
                value = sbufReadU8(src);
                currentControlRateProfile->rates[i] = MIN(value, i == FD_YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            }
            value = sbufReadU8(src);
            currentControlRateProfile->dynThrPID = MIN(value, CONTROL_RATE_CONFIG_TPA_MAX);
            currentControlRateProfile->thrMid8 = sbufReadU8(src);
            currentControlRateProfile->thrExpo8 = sbufReadU8(src);
            currentControlRateProfile->tpa_breakpoint = sbufReadU16(src);
            if (dataSize >= 11) {
                currentControlRateProfile->rcYawExpo8 = sbufReadU8(src);
            }
            if (dataSize >= 12) {
                currentControlRateProfile->rcYawRate8 = sbufReadU8(src);
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;
    case MSP_SET_MISC:
        tmp = sbufReadU16(src);
        if (tmp < 1600 && tmp > 1400)
            masterConfig.rxConfig.midrc = tmp;

        masterConfig.motorConfig.minthrottle = sbufReadU16(src);
        masterConfig.motorConfig.maxthrottle = sbufReadU16(src);
        masterConfig.motorConfig.mincommand = sbufReadU16(src);

        masterConfig.failsafeConfig.failsafe_throttle = sbufReadU16(src);

#ifdef GPS
        masterConfig.gpsConfig.provider = sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        masterConfig.gpsConfig.sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
        sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        sbufReadU8(src); // gps_ubx_sbas
#endif
        masterConfig.batteryConfig.multiwiiCurrentMeterOutput = sbufReadU8(src);
        masterConfig.rxConfig.rssi_channel = sbufReadU8(src);
        sbufReadU8(src);

        masterConfig.mag_declination = sbufReadU16(src) * 10;

        masterConfig.batteryConfig.vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        masterConfig.batteryConfig.vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        break;
    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++) { // FIXME should this use MAX_MOTORS or MAX_SUPPORTED_MOTORS instead of 8
            motor_disarmed[i] = convertExternalToMotor(sbufReadU16(src));
        }
        break;
    case MSP_SET_SERVO_CONFIGURATION:
#ifdef USE_SERVOS
        if (dataSize != 1 + sizeof(servoParam_t)) {
            return MSP_RESULT_ERROR;
            break;
        }
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        } else {
            masterConfig.servoConf[i].min = sbufReadU16(src);
            masterConfig.servoConf[i].max = sbufReadU16(src);
            masterConfig.servoConf[i].middle = sbufReadU16(src);
            masterConfig.servoConf[i].rate = sbufReadU8(src);
            masterConfig.servoConf[i].angleAtMin = sbufReadU8(src);
            masterConfig.servoConf[i].angleAtMax = sbufReadU8(src);
            masterConfig.servoConf[i].forwardFromChannel = sbufReadU8(src);
            masterConfig.servoConf[i].reversedSources = sbufReadU32(src);
        }
#endif
        break;

    case MSP_SET_SERVO_MIX_RULE:
#ifdef USE_SERVOS
        i = sbufReadU8(src);
        if (i >= MAX_SERVO_RULES) {
            return MSP_RESULT_ERROR;
        } else {
            masterConfig.customServoMixer[i].targetChannel = sbufReadU8(src);
            masterConfig.customServoMixer[i].inputSource = sbufReadU8(src);
            masterConfig.customServoMixer[i].rate = sbufReadU8(src);
            masterConfig.customServoMixer[i].speed = sbufReadU8(src);
            masterConfig.customServoMixer[i].min = sbufReadU8(src);
            masterConfig.customServoMixer[i].max = sbufReadU8(src);
            masterConfig.customServoMixer[i].box = sbufReadU8(src);
            loadCustomServoMixer();
        }
#endif
        break;

    case MSP_SET_3D:
        masterConfig.flight3DConfig.deadband3d_low = sbufReadU16(src);
        masterConfig.flight3DConfig.deadband3d_high = sbufReadU16(src);
        masterConfig.flight3DConfig.neutral3d = sbufReadU16(src);
        masterConfig.flight3DConfig.deadband3d_throttle = sbufReadU16(src);
        break;

    case MSP_SET_RC_DEADBAND:
        masterConfig.rcControlsConfig.deadband = sbufReadU8(src);
        masterConfig.rcControlsConfig.yaw_deadband = sbufReadU8(src);
        masterConfig.rcControlsConfig.alt_hold_deadband = sbufReadU8(src);
        break;

    case MSP_SET_RESET_CURR_PID:
        resetProfile(currentProfile);
        break;

    case MSP_SET_SENSOR_ALIGNMENT:
        masterConfig.sensorAlignmentConfig.gyro_align = sbufReadU8(src);
        masterConfig.sensorAlignmentConfig.acc_align = sbufReadU8(src);
        masterConfig.sensorAlignmentConfig.mag_align = sbufReadU8(src);
        break;

    case MSP_RESET_CONF:
        if (!ARMING_FLAG(ARMED)) {
            resetEEPROM();
            readEEPROM();
        }
        break;
    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        break;
    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            ENABLE_STATE(CALIBRATE_MAG);
        break;
    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
            return true;
        }
        writeEEPROM();
        readEEPROM();
        break;

#ifdef BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            masterConfig.blackbox_device = sbufReadU8(src);
            masterConfig.blackbox_rate_num = sbufReadU8(src);
            masterConfig.blackbox_rate_denom = sbufReadU8(src);
        }
        break;
#endif

#ifdef TRANSPONDER
    case MSP_SET_TRANSPONDER_CONFIG:
        if (dataSize != sizeof(masterConfig.transponderData)) {
            return MSP_RESULT_ERROR;
            break;
        }

        for (i = 0; i < sizeof(masterConfig.transponderData); i++) {
            masterConfig.transponderData[i] = sbufReadU8(src);
        }

        transponderUpdateData(masterConfig.transponderData);
        break;
#endif
#ifdef OSD
    case MSP_SET_OSD_CONFIG:
        addr = sbufReadU8(src);
        // set all the other settings
        if ((int8_t)addr == -1) {
            masterConfig.osdProfile.video_system = sbufReadU8(src);
            masterConfig.osdProfile.units = sbufReadU8(src);
            masterConfig.osdProfile.rssi_alarm = sbufReadU8(src);
            masterConfig.osdProfile.cap_alarm = sbufReadU16(src);
            masterConfig.osdProfile.time_alarm = sbufReadU16(src);
            masterConfig.osdProfile.alt_alarm = sbufReadU16(src);
        }
        // set a position setting
        else {
            masterConfig.osdProfile.item_pos[addr] = sbufReadU16(src);
        }
        break;
    case MSP_OSD_CHAR_WRITE:
        addr = sbufReadU8(src);
        for (i = 0; i < 54; i++) {
            font_data[i] = sbufReadU8(src);
        }
        max7456WriteNvm(addr, font_data);
        break;
#endif

#ifdef USE_RTC6705
    case MSP_SET_VTX_CONFIG:
        tmp = sbufReadU16(src);
        if  (tmp < 40)
            masterConfig.vtx_channel = tmp;
        if (current_vtx_channel != masterConfig.vtx_channel) {
            current_vtx_channel = masterConfig.vtx_channel;
            rtc6705_soft_spi_set_channel(vtx_freq[current_vtx_channel]);
        }
        break;
#endif

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_ERASE:
        flashfsEraseCompletely();
        break;
#endif

#ifdef GPS
    case MSP_SET_RAW_GPS:
        if (sbufReadU8(src)) {
            ENABLE_STATE(GPS_FIX);
        } else {
            DISABLE_STATE(GPS_FIX);
        }
        GPS_numSat = sbufReadU8(src);
        GPS_coord[LAT] = sbufReadU32(src);
        GPS_coord[LON] = sbufReadU32(src);
        GPS_altitude = sbufReadU16(src);
        GPS_speed = sbufReadU16(src);
        GPS_update |= 2;        // New data signalisation to GPS functions // FIXME Magic Numbers
        break;
    case MSP_SET_WP:
        wp_no = sbufReadU8(src);    //get the wp number
        lat = sbufReadU32(src);
        lon = sbufReadU32(src);
        alt = sbufReadU32(src);     // to set altitude (cm)
        sbufReadU16(src);           // future: to set heading (deg)
        sbufReadU16(src);           // future: to set time to stay (ms)
        sbufReadU8(src);            // future: to set nav flag
        if (wp_no == 0) {
            GPS_home[LAT] = lat;
            GPS_home[LON] = lon;
            DISABLE_FLIGHT_MODE(GPS_HOME_MODE);        // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
            ENABLE_STATE(GPS_FIX_HOME);
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
        } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
            GPS_hold[LAT] = lat;
            GPS_hold[LON] = lon;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
            nav_mode = NAV_MODE_WP;
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
        }
        break;
#endif
    case MSP_SET_FEATURE:
        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap
        break;

    case MSP_SET_BOARD_ALIGNMENT:
        masterConfig.boardAlignment.rollDegrees = sbufReadU16(src);
        masterConfig.boardAlignment.pitchDegrees = sbufReadU16(src);
        masterConfig.boardAlignment.yawDegrees = sbufReadU16(src);
        break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
        masterConfig.batteryConfig.vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        masterConfig.batteryConfig.vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_CURRENT_METER_CONFIG:
        masterConfig.batteryConfig.currentMeterScale = sbufReadU16(src);
        masterConfig.batteryConfig.currentMeterOffset = sbufReadU16(src);
        masterConfig.batteryConfig.currentMeterType = sbufReadU8(src);
        masterConfig.batteryConfig.batteryCapacity = sbufReadU16(src);
        break;

#ifndef USE_QUAD_MIXER_ONLY
    case MSP_SET_MIXER:
        masterConfig.mixerMode = sbufReadU8(src);
        break;
#endif

    case MSP_SET_RX_CONFIG:
        masterConfig.rxConfig.serialrx_provider = sbufReadU8(src);
        masterConfig.rxConfig.maxcheck = sbufReadU16(src);
        masterConfig.rxConfig.midrc = sbufReadU16(src);
        masterConfig.rxConfig.mincheck = sbufReadU16(src);
        masterConfig.rxConfig.spektrum_sat_bind = sbufReadU8(src);
        if (dataSize > 8) {
            masterConfig.rxConfig.rx_min_usec = sbufReadU16(src);
            masterConfig.rxConfig.rx_max_usec = sbufReadU16(src);
        }
        if (dataSize > 12) {
            masterConfig.rxConfig.rcInterpolation = sbufReadU8(src);
            masterConfig.rxConfig.rcInterpolationInterval = sbufReadU8(src);
            masterConfig.rxConfig.airModeActivateThreshold = sbufReadU16(src);
        }
        if (dataSize > 16) {
            masterConfig.rxConfig.rx_spi_protocol = sbufReadU8(src);
            masterConfig.rxConfig.rx_spi_id = sbufReadU32(src);
            masterConfig.rxConfig.rx_spi_rf_channel_count = sbufReadU8(src);
        }
        break;
    case MSP_SET_FAILSAFE_CONFIG:
        masterConfig.failsafeConfig.failsafe_delay = sbufReadU8(src);
        masterConfig.failsafeConfig.failsafe_off_delay = sbufReadU8(src);
        masterConfig.failsafeConfig.failsafe_throttle = sbufReadU16(src);
        masterConfig.failsafeConfig.failsafe_kill_switch = sbufReadU8(src);
        masterConfig.failsafeConfig.failsafe_throttle_low_delay = sbufReadU16(src);
        masterConfig.failsafeConfig.failsafe_procedure = sbufReadU8(src);
        break;

    case MSP_SET_RXFAIL_CONFIG:
        i = sbufReadU8(src);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            masterConfig.rxConfig.failsafe_channel_configurations[i].mode = sbufReadU8(src);
            masterConfig.rxConfig.failsafe_channel_configurations[i].step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        masterConfig.rxConfig.rssi_channel = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
            masterConfig.rxConfig.rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_BF_CONFIG:

#ifdef USE_QUAD_MIXER_ONLY
        sbufReadU8(src); // mixerMode ignored
#else
        masterConfig.mixerMode = sbufReadU8(src); // mixerMode
#endif

        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap

        masterConfig.rxConfig.serialrx_provider = sbufReadU8(src); // serialrx_type

        masterConfig.boardAlignment.rollDegrees = sbufReadU16(src); // board_align_roll
        masterConfig.boardAlignment.pitchDegrees = sbufReadU16(src); // board_align_pitch
        masterConfig.boardAlignment.yawDegrees = sbufReadU16(src); // board_align_yaw

        masterConfig.batteryConfig.currentMeterScale = sbufReadU16(src);
        masterConfig.batteryConfig.currentMeterOffset = sbufReadU16(src);
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
                break;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                    break;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            }
        }
        break;

#ifdef LED_STRIP
    case MSP_SET_LED_COLORS:
        for (i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &masterConfig.colors[i];
            color->h = sbufReadU16(src);
            color->s = sbufReadU8(src);
            color->v = sbufReadU8(src);
        }
        break;

    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = sbufReadU8(src);
            if (i >= LED_MAX_STRIP_LENGTH || dataSize != (1 + 4)) {
                return MSP_RESULT_ERROR;
                break;
            }
            ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
        }
        break;

    case MSP_SET_LED_STRIP_MODECOLOR:
        {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color))
                return MSP_RESULT_ERROR;
        }
        break;
#endif

    case MSP_SET_ADVANCED_CONFIG :
        masterConfig.gyro_sync_denom = sbufReadU8(src);
        masterConfig.pid_process_denom = sbufReadU8(src);
        masterConfig.motorConfig.useUnsyncedPwm = sbufReadU8(src);
#ifdef USE_DSHOT
        masterConfig.motorConfig.motorPwmProtocol = constrain(sbufReadU8(src), 0, PWM_TYPE_MAX - 1);
#else
        masterConfig.motorConfig.motorPwmProtocol = constrain(sbufReadU8(src), 0, PWM_TYPE_BRUSHED);
#endif
        masterConfig.motorConfig.motorPwmRate = sbufReadU16(src);
        break;
    case MSP_SET_FILTER_CONFIG :
        masterConfig.gyro_soft_lpf_hz = sbufReadU8(src);
        currentProfile->pidProfile.dterm_lpf_hz = sbufReadU16(src);
        currentProfile->pidProfile.yaw_lpf_hz = sbufReadU16(src);
        if (dataSize > 5) {
            masterConfig.gyro_soft_notch_hz_1 = sbufReadU16(src);
            masterConfig.gyro_soft_notch_cutoff_1 = sbufReadU16(src);
            currentProfile->pidProfile.dterm_notch_hz = sbufReadU16(src);
            currentProfile->pidProfile.dterm_notch_cutoff = sbufReadU16(src);
        }
        if (dataSize > 13) {
            masterConfig.gyro_soft_notch_hz_2 = sbufReadU16(src);
            masterConfig.gyro_soft_notch_cutoff_2 = sbufReadU16(src);
        }
        break;
    case MSP_SET_PID_ADVANCED:
        currentProfile->pidProfile.rollPitchItermIgnoreRate = sbufReadU16(src);
        currentProfile->pidProfile.yawItermIgnoreRate = sbufReadU16(src);
        currentProfile->pidProfile.yaw_p_limit = sbufReadU16(src);
        sbufReadU8(src); // reserved
        currentProfile->pidProfile.vbatPidCompensation = sbufReadU8(src);
        currentProfile->pidProfile.setpointRelaxRatio = sbufReadU8(src);
        currentProfile->pidProfile.dtermSetpointWeight = sbufReadU8(src);
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        currentProfile->pidProfile.itermThrottleGain = sbufReadU8(src);
        currentProfile->pidProfile.rateAccelLimit = sbufReadU16(src);
        currentProfile->pidProfile.yawRateAccelLimit = sbufReadU16(src);
        break;
    case MSP_SET_SENSOR_CONFIG:
        masterConfig.acc_hardware = sbufReadU8(src);
        masterConfig.baro_hardware = sbufReadU8(src);
        masterConfig.mag_hardware = sbufReadU8(src);
        break;

    case MSP_SET_NAME:
        memset(masterConfig.name, 0, ARRAYLEN(masterConfig.name));
        for (i = 0; i < MIN(MAX_NAME_LENGTH, dataSize); i++) {
            masterConfig.name[i] = sbufReadU8(src);
        }
        break;
    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspFcProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
#ifdef GPS
    } else if (cmdMSP == MSP_WP) {
        mspFcWpCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
#ifdef USE_FLASHFS
    } else if (cmdMSP == MSP_DATAFLASH_READ) {
        mspFcDataFlashReadCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspFcProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}

/*
 * Return a pointer to the process command function
 */
void mspFcInit(void)
{
    initActiveBoxIds();
}

void mspServerPush(mspPacket_t *push, uint8_t *data, int len)
{
    sbuf_t *dst = &push->buf;

    while (len--) {
        sbufWriteU8(dst, *data++);
    }
}

mspPushCommandFnPtr mspFcPushInit(void)
{
    return mspServerPush;
}
