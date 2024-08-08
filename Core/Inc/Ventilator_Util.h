/*
 * Ventilator_Util.h
 *
 *  Created on: May 1, 2021
 *      Author: krish
 */

#ifndef INC_VENTILATOR_UTIL_H_
#define INC_VENTILATOR_UTIL_H_
/*Manual Include */
#include "Venitilator_Cfg.h"

/* To Convert RAW ADC value to Milli Volt */
#define CONVERT_ADCtoMilliVot(a) (float)((((a*REFERENCE_VOLT_Inmv)/ADC_12BITRESOLUTION)))
#define CONVERT_ADCtoMilliVot_3V3(a) (float)((((a*REFERENCE_VOLT_Inmv_3V3)/ADC_12BITRESOLUTION)))
#define CONVERT_ADS115_ADCtoMilliVot(a) (float)((a*REFERENCE_VOLT_ADS1115_Inmv)/ADC_15BITRESOLUTION)


#endif /* INC_VENTILATOR_UTIL_H_ */
