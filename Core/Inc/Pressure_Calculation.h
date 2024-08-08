/*
 * Pressure_Calculation.h
 *
 *  Created on: May 1, 2021
 *      Author: krish
 */

#ifndef INC_PRESSURE_CALCULATION_H_
#define INC_PRESSURE_CALCULATION_H_

/*Pressure Realated Externs*/
extern float _Pressure_Val,  _Total_Volume,_max_volume_val ,_max_volume_val_ml,_max_Tidal_volume_val;
extern float  _Pressure_Offset;
/*Flow Realated Externs*/
extern float _Flow_Val  ,_Flow_Offset ;
extern float _Differential_Pressure ,_Volume_Val  ;
/* Volume Variables */
extern float Volume,Flow_Volume;
extern uint16_t RR_I_TIME,_I_TIMER_ACHEIVED,RR_E_TIME,_E_TIMER_ACHEIVED;

/* To Convert PRESSURE value to CM of H20 Volt */
#define CONVERT_PRESStoCMH2O(a,b) ((a-b)/44.13)


/* To Convert Kilo Pascal to Bar */
#define CONVERT_KPAtoBar(a,offset) ((a-offset)/1000)*10

/* To Convert Differential Pressure to Flow litre per minute */
#define CONVERT_dPAtoFlow(dp) (0.1512*(dp*dp*dp))-(3.3224*(dp*dp))+(41.657*dp)


extern float _CONVERT_PRESStoCMH2O(float a, float b);
extern float _CONVERT_dPAtoFlow(float dp) ;
extern float _CONVERT_KPAtoBar(float a, float offset) ;

#endif /* INC_PRESSURE_CALCULATION_H_ */
