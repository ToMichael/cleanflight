#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>


#include "common/axis.h"
#include "common/maths.h"


#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/profile.h"


#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"


#include "fc/rate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"
#include "fc/config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "rx/rx.h"

#include "io/motors.h"

#include "hover.h"

#define SCLENGTH 1000
#define PITCHROLL_CTRL_GAIN 0.5
#define HOVER_P 45
#define HOVER_I 20
#define HOVER_D 0.5
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)

extern uint8_t PIDweight[3];

static float vel[3] = {0.0f,0.0f,0.0f};
//static float pos[3] = {0.0f,0.0f,0.0f};
float accelAngle[2] = {0.0f,0.0f};
int32_t throCorrect = 0;
static float accZ_old = 0;
int32_t errorVelI = 0;
static float initialRawThrottleHold = 0;
static float initialThrottleHold = 0;
float storeParams[SCLENGTH][3];
uint16_t storeCount = 0;

void altHold(void)
{

        //angleCorrect variable set to change the throttleCorrect variable more quickly than via PID loop 

        /*float angleCorrect=1.0f;
        angleCorrect = 1.0f/(cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll))*cos_approx(DECIDEGREES_TO_DEGREES(attitude.values.pitch)));
        throCorrect = throCorrect * angleCorrect;*/

        if (ABS(rcData[THROTTLE] - initialRawThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
            errorVelI = 0;
            vel[2]=0.0f;
            rcCommand[THROTTLE] = initialThrottleHold + rcData[THROTTLE] - initialRawThrottleHold;

            if (rcData[THROTTLE] > initialRawThrottleHold){
                rcCommand[THROTTLE] -= rcControlsConfig()->alt_hold_deadband;    
            } else {
                rcCommand[THROTTLE] += rcControlsConfig()->alt_hold_deadband; 
            }
        } else {
            rcCommand[THROTTLE] = constrain(initialThrottleHold + throCorrect, motorConfig()->minthrottle, motorConfig()->maxthrottle);
        }
      
}


int32_t pidThrottle(int32_t setVel, float accZ, float accZ_old){
    int32_t result = 0;
    int32_t error;

    // P
    error = setVel - (int32_t)vel[2];
    result = constrain((HOVER_P * error), -300, +300);
    //result = HOVER_P*error;
    // I
    errorVelI += (HOVER_I * error);
    errorVelI = constrain(errorVelI, -(8192 * 200), (8192 * 200));
    result += errorVelI / 8192;     // I in range +/-400

    // D
    result -= constrain(HOVER_D * ((int32_t)accZ + (int32_t)accZ_old) / 512, -100, 100);
    //result -= HOVER_D * ((int32_t)accZ + (int32_t)accZ_old) / 512;

    return result;
   

}

void accelReset (int axis){
//reset for when mode goes inactive
    //pos[axis] = 0.0f;
    vel[axis] = 0.0f;
    if (axis<2){
        accelAngle[axis] = 0.0f;
    } else {
        throCorrect = 0;
   }
}


//enabling mode and gathering initialisation data
void updateHoverMode (void){
    
    if (!rcModeIsActive(BOXHOVER)) {
        DISABLE_FLIGHT_MODE(HOVER_MODE);
        rcControlsConfig()->deadband = 0;
        rcControlsConfig()->alt_hold_deadband = 40;
        return;
    }

    if (!FLIGHT_MODE(HOVER_MODE)) {
        DISABLE_FLIGHT_MODE(ANGLE_MODE);
        ENABLE_FLIGHT_MODE(HOVER_MODE);
        rcControlsConfig()->deadband = 40;
        rcControlsConfig()->alt_hold_deadband = 200;
        initialRawThrottleHold = rcData[THROTTLE];
        initialThrottleHold = rcCommand[THROTTLE];
        errorVelI = 0;
        accelReset(0);
        accelReset(1);
        accelReset(2);
        imuResetAccelerationSum();
    }

}

void accelCorrection(int axis,float accTimeSum,float acc){

//Logic:
//If the control stick is in the deadband integrate accelerations (pts) to get velocity
//Each acceleration (rectangular) integral vel_acc is progressively summed to get vel.
//Position is obtained by doing further integration on velocity.

//If the control is outside the deadband keep tracking velocity. Reset position tracking 
//and output angle correction.

    extern float accVelScale;   
  //  float dt;
    float vel_acc;


    //  dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec

    vel_acc = acc * accVelScale * (float)accTimeSum;

    // Integrator - Position in cm
    /*if ((ABS(rcData[axis] - rxConfig()->midrc)) < (rcControlsConfig()->deadband)){
        pos[axis] += vel_acc * 0.5f * dt + vel[axis] * dt;   // integrate velocity to get distance (x= a/2 * t^2)
    } else {
        pos[axis] = 0;
    }*/

    vel[axis] += vel_acc;

    //For an increase in X, pitch craft -ve
    //For an increase in Y, roll craft +ve
    if (axis == 0){ 
       
        //Dont perform angle correction if we are outside the deadband

        if ((ABS(rcData[1] - rxConfig()->midrc)) < rcControlsConfig()->deadband){
            accelAngle[1] = -vel[0]*PITCHROLL_CTRL_GAIN;
        } else {
             accelAngle[1]=0.0f;
        }
    } else if(axis == 1) {
        
        if ((ABS(rcData[0] - rxConfig()->midrc)) < rcControlsConfig()->deadband){
            accelAngle[0] = vel[1]*PITCHROLL_CTRL_GAIN;
        } else {
            accelAngle[0]=0.0f;
        }

    } else {
        //Throttle Correct calls in the last two accelerations
         if ((ABS(rcData[0] - rxConfig()->midrc)) < rcControlsConfig()->alt_hold_deadband){
            throCorrect = pidThrottle(0,acc,accZ_old);
            //stored for D component of PID controller
            accZ_old = acc;
            
            storeParams[storeCount][0]=acc;
            storeParams[storeCount][1]=vel[2];
            storeParams[storeCount][2]=(float)throCorrect;
            
            storeCount++;

            if (storeCount == SCLENGTH){
                storeCount = 0;
            }

         } else {
             throCorrect = 0;
         } 

    }
}


void rxHover(void){
    int32_t prop2 = 100;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    // 100 corresponds to 100%, any value after that sees the PID gains linearly decreased

   /* if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop2 = 100 - currentControlRateProfile->dynThrPID;
        }
    }*/

    //Using the Data (stick input) a value is set for rcCommand as according to the RC Control rc_curves
    //These curves are constructed by 7 points and have a linear interpolation betweeen these points

    for (int axis = 0; axis < 3; axis++) {
        int32_t prop1;
        //tmp set by data
        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            //deadband logic
            if (rcControlsConfig()->deadband) {
                if (tmp > rcControlsConfig()->deadband) {
                    tmp -= rcControlsConfig()->deadband;
                } else {
                    tmp = 0;
                }
            }
            //lookup data curves
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


    altHold();

   /* int32_t tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);       // [MINCHECK;2000] -> [0;1000]
    rcCommand[THROTTLE] = rcLookupThrottle(tmp);*/
   }
