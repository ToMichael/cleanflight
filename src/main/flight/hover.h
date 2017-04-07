#pragma once


void velCurve(int32_t centre, int32_t data);

void imuCalculatePIDCorrectionValue(float accZ, float accZ_old,float accTimeSum);
void accelReset (int axis);

void imuCalculateVelocityIntegration(int axis,float accTimeSum,float acc, float acc_old);

void rxHover(void);

void calculatePitchRollYawRCCommands(void);
void initialiseHoverMode(int32_t pitchLimits);
int32_t leaveHoverMode(void);
char getHoverMode (void);



#define SCLENGTH 1000