#include <stdint.h>
#include <math.h>
#include <platform.h>

#include "kalman.h"

//the noise in the system
#define  QKALMAN  0.00005
#define  RKALMAN  1.5

// no G, A or H necessary (all 1)
float kkalman;
static float pkalman[3];
static float xkalman[3];

void kalmanInit(float Z, int i) {

//initialize with a measurement
    xkalman[i] = 0;
    pkalman[i] = (xkalman[i]-Z)*(xkalman[i]-Z);
}


float kalmanUpdate(float Z, int i) {
        //Predict Step
        float p_pred;
        float x_pred;
        x_pred = xkalman[i];
        p_pred = pkalman[i] + QKALMAN;
        //calculate the Kalman gain
        kkalman = p_pred * (1.0/(p_pred + RKALMAN));
        //Update Step
        xkalman[i] = x_pred + kkalman * (Z - x_pred); 
        pkalman[i] = (1- kkalman) * (1-kkalman) * p_pred + kkalman*RKALMAN*kkalman;       
        return xkalman[i];
}
