#pragma once

extern void startCameraServer();

extern void robot_stop();
extern void robot_setup();
extern void camera_setup();
extern void wifi_setup();
extern void ota_setup();

extern int gpLed;

extern int ps_ram;

extern volatile int DistFront, DistBack, DistRight, DistLeft, LumMoy;

extern volatile int Temp, Hum, Ubat, Sat;
extern volatile int32_t latitude, longitude, altitude, speedGPS;

extern volatile int ModMove;
extern volatile bool robot_fwd_val;

extern const char page[];
