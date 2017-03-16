#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "common/axis.h"
#include "common/maths.h"


#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

//#include "drivers/sensor.h"
//#include "drivers/accgyro.h"

#include "fc/rate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"
#include "fc/config.h"

#include "flight/pid.h"
//#include "flight/imu.h"

#include "rx/rx.h"

//#include "sensors/sensors.h"
//#include "sensors/acceleration.h"

#include "hover.h"
#define VEL_CTRL_GAIN 10

extern uint8_t PIDweight[3];

static float vel[3] = {0.0f,0.0f,0.0f};
static float pos[3] = {0.0f,0.0f,0.0f};
float accelAngle[2] = {0.0f,0.0f};
//float throCorrect = 0.0f;

void accelReset (int axis){
//reset for when mode goes inactive
    pos[axis] = 0.0f;
    vel[axis] = 0.0f;
    accelAngle[axis] = 0.0f;
}


void accelCorrection(int axis,float accTimeSum,float accSum){

//Logic:
//If the control stick is in the deadband integrate accelerations (pts) to get velocity
//Each acceleration (rectangular) integral vel_acc is progressively summed to get vel.
//Position is obtained by doing further integration on velocity.

//If the control is outside the deadband keep tracking velocity. Reset position tracking 
//and output angle correction.

    extern float accVelScale;   
    float dt;
    float vel_acc;

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec

    vel_acc = accSum * accVelScale * (float)accTimeSum;

    // Integrator - Position in cm
    if ((ABS(rcData[axis] - rxConfig()->midrc)) < (rcControlsConfig()->deadband)){
        pos[axis] += vel_acc * 0.5f * dt + vel[axis] * dt;   // integrate velocity to get distance (x= a/2 * t^2)
    } else {
        pos[axis] = 0;
    }

    vel[axis] += vel_acc;

    //For an increase in X, pitch craft +ve
    //For an increase in Y, roll craft +ve
  //  if (axis < 2){ 
        accelAngle[!(bool)axis] = vel[axis]*VEL_CTRL_GAIN;
   // }

}


void rxHover(void){
    int32_t prop2;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    // 100 corresponds to 100%, any value after that sees the PID gains linearly decreased
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop2 = 100 - currentControlRateProfile->dynThrPID;
        }
    }

    //Using the Data (stick input) a value is set for rcCommand as according to the RC Control rc_curves
    //These curves are constructed by 7 points and have a linear interpolation betweeen these points

    for (int axis = 0; axis < 3; axis++) {
        int32_t prop1;
        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (rcControlsConfig()->deadband) {
                if (tmp > rcControlsConfig()->deadband) {
                    tmp -= rcControlsConfig()->deadband;
                } else {
                    tmp = 0;
                }
            }

            rcCommand[axis] = rcLookupPitchRoll(tmp);
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
            // non coupled PID reduction scaler used in PID controller 1 and PID controller 2. 100 means 100% of the pids
            PIDweight[axis] = prop2;
        } else {
            if (rcControlsConfig()->yaw_deadband) {
                if (tmp > rcControlsConfig()->yaw_deadband) {
                    tmp -= rcControlsConfig()->yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = rcLookupYaw(tmp) * -rcControlsConfig()->yaw_control_direction;
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * ABS(tmp) / 500;
            // YAW TPA disabled.
            PIDweight[axis] = 100;
        }

        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    int32_t tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);       // [MINCHECK;2000] -> [0;1000]
    rcCommand[THROTTLE] = rcLookupThrottle(tmp);
   }
