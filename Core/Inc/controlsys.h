#ifndef CONTROLSYS_H
#define CONTROLSYS_H

typedef struct {

	float Kp;
	float Ki;
	float Kd;
	float tau;
	float a1;
	float a2;
	float b1;
	float b2;
	float T;
	float area;
	float prevdif;
	float rate;
	float prevmea;		
	float out;

} controlsys;

void  controlsys_Init(controlsys *cs);
float controlsys_Update(controlsys *cs, float set, float mea);

#endif
