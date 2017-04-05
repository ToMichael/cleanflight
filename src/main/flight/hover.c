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
#include "flight/kalman.h"
#include "rx/rx.h"

#include "io/motors.h"

#include "hover.h"


#define PITCHROLL_CTRL_GAIN 0.5
#define TEMPDB 10
#define THRLIMIT 750
#define HOVER_P 10
#define HOVER_I 0.008
#define HOVER_D 0.004
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)

extern uint8_t PIDweight[3];

static float vel[3] = {0.0f,0.0f,0.0f};
//static float pos[3] = {0.0f,0.0f,0.0f};
float accelAngle[2] = {0.0f,0.0f};
int32_t throCorrect = 0;
static float accZ_old = 0;
static float setVel=0;
int32_t errorVelI = 0;
static float prevThroData = 0;
int32_t centre = 0;
static float initialThro = 0;
float storeParams[SCLENGTH][3];
uint16_t storeCount = 0;
bool inHover = 0;


void velCurve(int32_t centre, int32_t data){
    
    if(data<1100){
        setVel -= (1100 - data)/250.0f;
    } else if(data>1900){
        setVel += (data - 1900)/250.0f;
    } else {
        if (ABS(data-centre)<100){
            setVel += ((float)data - prevThroData)*0.2f;
        } else if (ABS(data-centre)<300){
            setVel += ((float)data - prevThroData)*0.6f;
        } else {
            setVel += ((float)data - prevThroData)*2;
        }
        prevThroData = data;
    }
}



void altHold(void)
{

        //angleCorrect variable set to change the throttleCorrect variable more quickly than via PID loop 

        /*float angleCorrect=1.0f;
        angleCorrect = 1.0f/(cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll))*cos_approx(DECIDEGREES_TO_DEGREES(attitude.values.pitch)));
        throCorrect = throCorrect * angleCorrect;*/

        /*if (ABS(rcData[THROTTLE] - prevThroData) > rcControlsConfig()->alt_hold_deadband) {
            errorVelI=0;
            vel[Z]= 0.0f;
            rcCommand[THROTTLE] = initialThro + rcData[THROTTLE] - prevThroData;

            if (rcData[THROTTLE] > prevThroData){
                rcCommand[THROTTLE] -= rcControlsConfig()->alt_hold_deadband;    
            } else {
                rcCommand[THROTTLE] += rcControlsConfig()->alt_hold_deadband; 
            }
        } else {
            rcCommand[THROTTLE] = constrain(initialThro + throCorrect, motorConfig()->minthrottle, motorConfig()->maxthrottle);
        }*/

    //if inside deadband just do control 
    //if outside deadband change the set point for velocity according to S curve
    //if data value unchanged (within 10) for 100 runs renter deadband and reconstruct curve


    if (ABS(rcData[THROTTLE] - prevThroData) > TEMPDB){
        velCurve(centre,rcData[THROTTLE]);
        errorVelI=0;    
    }

  /*  if (ABS(rcData[THROTTLE] - prevThroData))>40){
        setVel += (rcData[THROTTLE] - prevThroData)/10.0f;
        prevThroData = rcData[THROTTLE];
        errorVelI=0;
        
    }*/

    rcCommand[THROTTLE] = constrain(initialThro + throCorrect, motorConfig()->minthrottle, motorConfig()->maxthrottle);
   
    return;   
}


int32_t pidThrottle(float setVel, float accZ, float accZ_old, float accTimeSum){
    int32_t result = 0;
    
    //int32_t error;
/*
    // P
    error = setVel - (int32_t)(vel[2]);
    //result = constrain((HOVER_P * error), -300, +300);
    result = HOVER_P * error;
    // I
    errorVelI += (HOVER_I * error);
    //errorVelI = constrain(errorVelI, -(8192 * 200), (8192 * 200));
    result += errorVelI / 8192;     

    // D
    //result -= constrain(HOVER_D * ((int32_t)accZ + (int32_t)accZ_old) / 512, -100, 100);
    result -= HOVER_D * ((int32_t)accZ + (int32_t)accZ_old) / 512;*/

    float resfloat = 0.0f;
    float errfloat = 0.0f;
    errfloat = setVel - vel[Z];
    resfloat = HOVER_P * errfloat;


    errorVelI += (HOVER_I * errfloat);
    resfloat += errorVelI *accTimeSum /1000000.0f;   

    resfloat -= HOVER_D * (accZ + (accZ_old)) / 2;

    result = (int32_t)constrain(resfloat,-THRLIMIT,THRLIMIT);

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
   return;
}


//enabling mode and gathering initialisation data
void updateHoverMode (void){
//If the stick is not in the Hover designated position but we are in Hover mode:
//Disable Hover mode and reset the deadband parameters

    if (!rcModeIsActive(BOXHOVER)) {
        if(FLIGHT_MODE(HOVER_MODE)){
            DISABLE_FLIGHT_MODE(HOVER_MODE);
            rcControlsConfig()->deadband = 0;
            rcControlsConfig()->alt_hold_deadband = 40;
            inHover = false;
        }
    } else {
        if (!FLIGHT_MODE(HOVER_MODE)) {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HOVERFREE_MODE);
            ENABLE_FLIGHT_MODE(HOVER_MODE);
            rcControlsConfig()->deadband = 40;
            rcControlsConfig()->alt_hold_deadband = 200;
            prevThroData = rcData[THROTTLE];
            centre = rcData[THROTTLE];
            initialThro = rcCommand[THROTTLE];
            errorVelI = 0;
            setVel = 0;
            for (int axis = 0; axis < 3; axis++) {
                accelReset(axis);
               // kalmanInit((float)accSum[axis]/(float)accSumCount,axis);
            }

            imuResetAccelerationSum();
            inHover = true;
        }
    }


    if (!rcModeIsActive(BOXHOVERFREE)) {
        if(FLIGHT_MODE(HOVERFREE_MODE)){
            DISABLE_FLIGHT_MODE(HOVERFREE_MODE);
            rcControlsConfig()->deadband = 0;
            rcControlsConfig()->alt_hold_deadband = 40;
            imuConfig()->max_angle_inclination = 500;
            inHover = false;
        }
    } else {
        if (!FLIGHT_MODE(HOVERFREE_MODE)) {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HOVER_MODE);
            ENABLE_FLIGHT_MODE(HOVERFREE_MODE);
            rcControlsConfig()->deadband = 40;
            rcControlsConfig()->alt_hold_deadband = 200;
            imuConfig()->max_angle_inclination = 800;
            prevThroData = rcData[THROTTLE];
            centre = rcData[THROTTLE];
            initialThro = rcCommand[THROTTLE];
            errorVelI = 0;
            setVel = 0;
            for (int axis = 0; axis < 3; axis++) {
                accelReset(axis);
               // kalmanInit((float)accSum[axis]/(float)accSumCount,axis);
            }

            imuResetAccelerationSum();
            inHover = true;
        }
    }



    return;
}





void accelCorrection(int axis,float accTimeSum,float acc){

    //Logic:
    //If the control stick is in the deadband integrate accelerations (pts) to get velocity
    //Each acceleration (trapezoidal) integral vel_acc is progressively summed to get vel.
    //Position is obtained by doing further integration on velocity.

    //If the control is outside the deadband keep tracking velocity. Reset position tracking 
    //and output angle correction.

    extern float accVelScale;   
  //  float dt;
    float vel_acc;
  // float accKalman;

    //  dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    //accKalman = kalmanUpdate(acc,axis);
    
    vel_acc = (acc + accZ_old) / 2 * accVelScale * (float)accTimeSum;

    // Integrator - Position in cm
    /*if ((ABS(rcData[axis] - rxConfig()->midrc)) < (rcControlsConfig()->deadband)){
        pos[axis] += vel_acc * 0.5f * dt + vel[axis] * dt;   // integrate velocity to get distance (x= a/2 * t^2)
    } else {
        pos[axis] = 0;
    }*/
    if (ABS(rcData[THROTTLE] - prevThroData) < TEMPDB){
        vel[axis] += vel_acc;
    }

    //For an increase in X, pitch craft -ve
    //For an increase in Y, roll craft +ve
    /*if (axis == X){ 
       
        //Dont perform angle correction if outside the deadband

        if ((ABS(rcData[PITCH] - rxConfig()->midrc)) < rcControlsConfig()->deadband){
            accelAngle[PITCH] = -vel[X]*PITCHROLL_CTRL_GAIN;
        } else {
             accelAngle[PITCH]=0.0f;
        }
    } else if (axis == Y) {
        
        if ((ABS(rcData[ROLL] - rxConfig()->midrc)) < rcControlsConfig()->deadband){
            accelAngle[ROLL] = vel[Y]*PITCHROLL_CTRL_GAIN;
        } else {
            accelAngle[ROLL]=0.0f;
        }

    } else {*/
        //Throttle Correct calls in the last two accelerations
         //if ((ABS(rcData[THROTTLE] - prevThroData)) < rcControlsConfig()->alt_hold_deadband){
         //   throCorrect = pidThrottle(setVel,acc,accZ_old,accTimeSum);         
         //} else {
           // throCorrect = 0;
         //} 


       //if (ABS(rcData[THROTTLE] - prevThroData) < TEMPDB){
            throCorrect = pidThrottle(setVel,acc,accZ_old,accTimeSum);         
       // } else {
         //   throCorrect = 0;
        // } 

            storeCount++;
            storeParams[storeCount][0]=setVel;
            storeParams[storeCount][1]=vel[Z];
            storeParams[storeCount][2]=(float)throCorrect;
            
            

            if (storeCount == SCLENGTH){
                storeCount = 0;
            }

   //}

    accZ_old = acc;

    return;
}


void rxHover(void){
    //int32_t prop2 = 100;

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
        //int32_t prop1;
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
            //prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * tmp / 500;
            //prop1 = (uint16_t)prop1 * prop2 / 100;
            // non coupled PID reduction scaler used in PID controller 1 and PID controller 2. 100 means 100% of the pids
            //PIDweight[axis] = prop2;
        } else {
            if (rcControlsConfig()->yaw_deadband) {
                if (tmp > rcControlsConfig()->yaw_deadband) {
                    tmp -= rcControlsConfig()->yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = rcLookupYaw(tmp) * -rcControlsConfig()->yaw_control_direction;
            //prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * ABS(tmp) / 500;
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
    return;

   }
