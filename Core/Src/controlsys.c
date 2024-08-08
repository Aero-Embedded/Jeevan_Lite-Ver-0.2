#include "controlsys.h"

void controlsys_Init(controlsys *cs) {


	cs->area = 0.0;
	cs->prevdif = 0.0;

	cs->rate  = 0.0;
	cs->prevmea = 0.0;

	cs->out = 0.0;

}

float controlsys_Update(controlsys *cs, float set, float mea) {

	
    float error = set - mea;
    float pro = cs->Kp * error;
    float dInput = (mea - cs->prevmea);
    cs->area  = cs->area +(cs->Ki * error*0.002);
   // cs->area  = cs->area +(cs->Ki * dInput);

    if (cs->area > cs->b2) {

        cs->area = cs->b2;

    } else if (cs->area < cs->b1) {

        cs->area = cs->b1;

    }

    cs->rate = -(cs->Kd *(dInput/0.002));


    cs->out = (pro + cs->area + cs->rate);

    if (cs->out > cs->a2) {

        cs->out = cs->a2;

    } else if (cs->out < cs->a1) {

        cs->out = cs->a1;

    }
    cs->prevmea = mea;
    return (cs->out);


}
