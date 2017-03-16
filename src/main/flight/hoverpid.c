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

#define SRC_MAIN_FLIGHT_PID_LUXFLOAT_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rate_profile.h"

#include "flight/pid.h"
#include "config/config_unittest.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"
#include "flight/hover.h"

extern float dT;
extern uint8_t PIDweight[3];
extern float lastITermf[3], ITermLimitf[3];

extern pt1Filter_t deltaFilter[3];
extern pt1Filter_t yawFilter;

extern biquadFilter_t dtermFilterNotch[3];
extern biquadFilter_t dtermFilterLpf[3];

extern float accelAngle[2];


// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale

STATIC_UNIT_TESTED int16_t pidHoverCore(int axis, const pidProfile_t *pidProfile, float gyroRate, float angleRate)
{
    static float lastRateForDelta[3];

    SET_PID_LUX_FLOAT_CORE_LOCALS(axis);

    const float rateError = angleRate - gyroRate;

    // -----calculate P component
    float PTerm = luxPTermScale * rateError * pidProfile->P8[axis] * PIDweight[axis] / 100;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW) {
        if (pidProfile->yaw_lpf_hz) {
            PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());
        }
        if (pidProfile->yaw_p_limit) {
            PTerm = constrainf(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }
    }

    // -----calculate I component
    float ITerm = lastITermf[axis] + luxITermScale * rateError * getdT() * pidProfile->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrainf(ITerm, -PID_MAX_I, PID_MAX_I);
    lastITermf[axis] = ITerm;

    // -----calculate D component
    float DTerm;
    if (pidProfile->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        float delta;
        delta = rateError - lastRateForDelta[axis];
        lastRateForDelta[axis] = rateError;

        // Divide delta by targetLooptime to get differential (ie dr/dt)
        delta /= getdT();

        // Filter delta
        if (pidProfile->dterm_notch_hz) {
            delta = biquadFilterApply(&dtermFilterNotch[axis], delta);
        }

        if (pidProfile->dterm_lpf_hz) {
            if (pidProfile->dterm_filter_type == FILTER_BIQUAD) {
                delta = biquadFilterApply(&dtermFilterLpf[axis], delta);
            } else {
                // DTerm delta low pass filter
                delta = pt1FilterApply4(&deltaFilter[axis], delta, pidProfile->dterm_lpf_hz, getdT());
            }
        }

        DTerm = luxDTermScale * delta * pidProfile->D8[axis] * PIDweight[axis] / 100;
        DTerm = constrainf(DTerm, -PID_MAX_D, PID_MAX_D);
    }
    GET_PID_LUX_FLOAT_CORE_LOCALS(axis);
    // -----calculate total PID output
    return lrintf(PTerm + ITerm + DTerm);
}

void pidHover(const pidProfile_t *pidProfile,uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim)
{
    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        float angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            angleRate = 27.0f * rcCommand[YAW] / 32.0f;
        } else {
        //added accelAngle[axis] to change error angle
            const float errorAngle = constrain(2 * rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                        - attitude.raw[axis] + angleTrim->raw[axis] + accelAngle[axis];

            angleRate = errorAngle * pidProfile->P8[PIDLEVEL] / 16.0f;

        }
        
        // --------low-level gyro-based PID. ----------
        const float gyroRate = luxGyroScale * gyroADCf[axis] * gyro.scale;
        axisPID[axis] = pidHoverCore(axis, pidProfile, gyroRate, angleRate);
    }
}

