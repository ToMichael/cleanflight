#pragma once

void velCurve(int32_t centre, int32_t data);
void altHold(void);
int32_t pidThrottle(float setVel, float accZ, float accZ_old,float accTimeSum);
void accelReset (int axis);
void updateHoverMode (void);
void accelCorrection(int axis,float accTimeSum,float accSum);
void rxHover(void);

#define SCLENGTH 1000