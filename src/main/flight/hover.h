#pragma once

void altHold(void);
int32_t pidThrottle(int32_t setVel, float accZ, float accZ_old);
void accelReset (int axis);
void updateHoverMode (void);
void accelCorrection(int axis,float accTimeSum,float accSum);
void rxHover(void);