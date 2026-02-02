#pragma once

#include <Arduino.h>

extern int gpLed;

extern volatile int DistFront, DistBack, DistRight, DistLeft, LumMoy;
extern volatile int Temp, Hum, Ubat;
extern volatile int32_t latitude, longitude;

extern volatile int ModMove;
extern volatile bool robot_fwd_val;

extern const char page[];
