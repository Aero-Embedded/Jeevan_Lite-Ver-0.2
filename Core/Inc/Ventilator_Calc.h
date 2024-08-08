/*
 * Ventilator_Calc.h
 *
 *  Created on: Apr 28, 2021
 *      Author: krish
 */

#ifndef INC_VENTILATOR_CALC_H_
#define INC_VENTILATOR_CALC_H_

/* Pressure Variables */
extern float pressure,P_cmh2o,P_cmh2o1;
extern uint16_t delPOffset,delPOnline;



/**
  * @brief  Convert ADC into Avgerage millivolt of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */

extern float getAvgMilliVot(uint16_t *sample_buf, int avgCount);

extern float getAvgMilliVot_3V3(uint16_t *sample_buf, int avgCount);


extern float getAvg(uint16_t *sample_buf, int avgCount);

/**
  * @brief  Convert ADC into Avgerage millivolt of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */
extern uint16_t getAvgMilliVot_MovingAverage(uint16_t *sample_buf, int avgCount, uint16_t ma);

/**
  * @brief  Set Predefined Calibration value
  * @param  argument: none
  * @retval none
  */
extern uint8_t Calibration_Process(void);

/**
  * @brief  Calculate G Pressure
  * @param  argument: adc value
  * @retval calculate voltage gP_volt
  */

extern uint16_t F_gPCalc(uint16_t value);


extern uint8_t chksum8(const unsigned char *buff, size_t len);


extern double MOVING_AVERAGE_PRESSURE(double Inval);

/**
  * @brief  MOVING_AVERAGE PRESSURE
  * @param  argument: none
  * @retval none
  */

extern double MOVING_AVERAGE_FLOW(double Inval);

#endif /* INC_VENTILATOR_CALC_H_ */
