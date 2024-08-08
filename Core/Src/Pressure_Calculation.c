/*
 * Pressure_Calculation.c
 *
 *  Created on: May 1, 2021
 *      Author: krish
 */
#include "main.h"


float _Pressure_Offset ;
float _Pressure_Val ;

float _Flow_Val  ,_Flow_Offset ;

long adj(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float _Differential_Pressure ,_Volume_Val, _Total_Volume,_max_volume_val ,_max_volume_val_ml,_max_Tidal_volume_val ;
/* Volume Variables */
float Volume,Flow_Volume;
uint16_t RR_I_TIME=1000,_I_TIMER_ACHEIVED,RR_E_TIME=1000,_E_TIMER_ACHEIVED;
/* To Convert Kilo Pascal to Bar */
//#define CONVERT_KPAtoBar(a,offset) ((a-offset)/1000)*10
float _CONVERT_KPAtoBar(float a, float offset)
{
	return(((a-offset)/1000)*10);
}

///* To Convert Differential Pressure to Flow litre per minute */
//#define CONVERT_dPAtoFlow(dp) (0.1512*(dp*dp*dp))-(3.3224*(dp*dp))+(41.657*dp)
float _CONVERT_dPAtoFlow(float dp)
{

	float f=0;
	if(dp>0)
		{
		f=0.1512*dp*dp*dp-3.3424*dp*dp+41.657*dp;
		f=adj(f,0,160,0,185);
		}
		else if(dp<0)
		{
		dp*=-1;
		f=0.1512*dp*dp*dp-3.3424*dp*dp+41.657*dp;
		f=adj(f,0,160,0,260);
		f*=-1;
		}

	return f;
}


/* To Convert PRESSURE value to CM of H20 Volt */
//#define CONVERT_PRESStoCMH2O(a,b) ((a-b)/44.13)

float _CONVERT_PRESStoCMH2O(float a, float b)
{
	return((float)((a-b)/44.13));
}
