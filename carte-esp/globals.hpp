#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

extern HardwareSerial SerialMega;

extern int gpLed;

extern int DistFront, DistBack, DistRight, DistLeft, LumMoy, Temp, Hum;
extern char latitude[20];
extern char longitude[20];

extern int ModMove;
extern bool robot_fwd_val;

extern const char page[];
