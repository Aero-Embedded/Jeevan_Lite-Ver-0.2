/*
 * Venitilator_Cfg.h
 *
 *  Created on: Apr 28, 2021
 *      Author: krish
 */

#ifndef INC_VENITILATOR_CFG_H_
#define INC_VENITILATOR_CFG_H_

/*
 Pressure Offset Value
*/
#define PRESSURE_OFFSET 244.00f

/*
 Pressure PRESSURE_CALIBRATION_SAMPLES
*/
#define PRESSURE_CALIBRATION_SAMPLES 100u

/*
 Pressure FLOW_CALIBRATION_SAMPLES
*/
#define FLOW_CALIBRATION_SAMPLES 100u


/*
 Pressure PRESSURE_CALIBRATION_SAMPLES
*/
#define gP_SENSITIVITY 44.13f

/*
 Pressure delP_sensitivity
*/
#define delP_SENSITIVITY 1000u


/*
  REFERENCE_VOLT_Inmv
*/
#define REFERENCE_VOLT_Inmv 5000u
#define REFERENCE_VOLT_Inmv_3V3 3300u
#define REFERENCE_VOLT_ADS1115_Inmv 4096u
/*
  REFERENCE_VOLT_Inmv
*/
#define ADC_12BITRESOLUTION 4096u
#define ADC_15BITRESOLUTION 32768u

/*
  Minimum Time needed for sensor to convert data */
#define SENSOR_CONVERSION_TIME 1u // In Milli Second

#define ENABLE 	1
#define DISABLE 0

#define DEBUG_GPIO DISABLE

#define CYCLIC_TRANSMIT ENABLE


#endif /* INC_VENITILATOR_CFG_H_ */
