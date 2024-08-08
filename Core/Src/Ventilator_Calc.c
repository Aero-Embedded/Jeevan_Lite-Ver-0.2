/*
 * Ventilator_Calc.c
 *
 *  Created on: Apr 28, 2021
 *      Author: krish
 */

/* Includes*/
#include "stm32f4xx_hal.h"
#include "Venitilator_Cfg.h"
#include "ventilator_types.h"
#include "main.h"
#include "Ventilator_Util.h"
#include "Pressure_Calculation.h"



/*Calibration Variables*/
uint16_t _Pressure_CalibAVGBuf[PRESSURE_CALIBRATION_SAMPLES], _Flow_CalibAVGBuf[FLOW_CALIBRATION_SAMPLES] ;

/*static Variables */
static uint16_t _Pressure_SampleCount ;
static int MA_readings [10];
const  int MA_numReadings  = 10;
static int MA_readIndex  = 0;
static long MA_total  = 0;
static long Mov_average;

static int MA_Flow_readings [10];
const  int MA_Flow_numReadings  = 10;
static int MA_Flow_readIndex  = 0;
static long MA_Flow_total  = 0;
static long Mov_Flow_average;


/**
  * @brief  Convert ADC into Avgerage millivolt of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */
float millivot1 = 0 ;
float getAvgMilliVot(uint16_t *sample_buf, int avgCount){
    float rmSum = 0;
    float Avg_Value = 0 ;


    for(int i = 0; i < avgCount; i++){
        rmSum += sample_buf[i];
    }

    Avg_Value = (float) (rmSum / avgCount);

    millivot1 = (float) CONVERT_ADCtoMilliVot(Avg_Value) ;

    return millivot1;
}

/**
  * @brief  Convert ADC into Avgerage millivolt of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */
float millivot2 = 0 ;
float getAvgMilliVot_3V3(uint16_t *sample_buf, int avgCount){
    float rmSum = 0;
    float Avg_Value = 0 ;


    for(int i = 0; i < avgCount; i++){
        rmSum += sample_buf[i];
    }

    Avg_Value = (float) (rmSum / avgCount);

    millivot2 = (float) CONVERT_ADCtoMilliVot_3V3(Avg_Value) ;

    return millivot2;
}

/**
  * @brief  Convert ADC into Avgerage of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */

float getAvg(uint16_t *sample_buf, int avgCount){
    float rmSum = 0;
    float Avg_Value = 0 ;


    for(int i = 0; i < avgCount; i++){
        rmSum += sample_buf[i];
    }

    Avg_Value = (float) (rmSum / avgCount);

    return Avg_Value;
}

/**
  * @brief  Convert ADC into Avgerage millivolt of n number of samples
  * @param  argument: sample_buf (in) avgCount(in)
  * @retval average value
  */

uint16_t getAvgMilliVot_MovingAverage(uint16_t *sample_buf, int avgCount, uint16_t ma){
    uint16_t Avg_Value = 0 ;
    uint16_t millivot,old_value = 0 ;

    for(int i = 0; i < avgCount; i++){
        Avg_Value = ((sample_buf[i] - old_value)/ma )+ old_value;
        old_value = Avg_Value ;
    }

    millivot = CONVERT_ADCtoMilliVot(Avg_Value) ;

    return millivot;
}

uint16_t getAvgMilliVot_MovingAverage_ADS1115(uint16_t *sample_buf, int avgCount, uint16_t ma){
    uint16_t Avg_Value = 0 ;
    uint16_t millivot,old_value = 0 ;

    for(int i = 0; i < avgCount; i++){
        Avg_Value = ((sample_buf[i] - old_value)/ma )+ old_value;
        old_value = Avg_Value ;
    }

    millivot = CONVERT_ADS115_ADCtoMilliVot(Avg_Value) ;

    return millivot;
}

/**
  * @brief  Set Predefined Calibration value
  * @param  argument: none
  * @retval none
  */
uint8_t Calibration_Process(void)
{
	/* Set Calibration value from configuration*/
	uint8_t _retVal = E_NOK ;

	if(_Pressure_SampleCount < PRESSURE_CALIBRATION_SAMPLES)
	{
		_Pressure_CalibAVGBuf[_Pressure_SampleCount]= ADC_RESULT_BUF[0];
		_Flow_CalibAVGBuf[_Pressure_SampleCount]= ADS1115_raw ;
		_retVal = E_PENDING ;
		_Pressure_SampleCount++;
	}
	else
	{
		_Pressure_Offset = getAvgMilliVot_MovingAverage(_Pressure_CalibAVGBuf,_Pressure_SampleCount,8);
		_Flow_Offset = getAvgMilliVot_MovingAverage_ADS1115(_Flow_CalibAVGBuf,_Pressure_SampleCount,8);
		_Pressure_SampleCount = 0 ;
		_retVal = E_OK ;
	}
	return(_retVal);
}

/**
  * @brief  CRC8 calculation
  * @param  argument: none
  * @retval none
  */

uint8_t chksum8(const unsigned char *buff, size_t len)
{
    unsigned int sum;       // nothing gained in using smaller types!
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);   // parenthesis not required!
    return (uint8_t)sum;
}

/**
  * @brief  MOVING_AVERAGE PRESSURE
  * @param  argument: Inval
  * @retval Mov_average
  */

double MOVING_AVERAGE_PRESSURE(double Inval)
{
	MA_total = MA_total - MA_readings[MA_readIndex];
	MA_readings[MA_readIndex] = Inval;
	MA_total = MA_total + MA_readings[MA_readIndex];
	MA_readIndex = MA_readIndex + 1;
	if (MA_readIndex >= MA_numReadings) {
		MA_readIndex = 0;
	}
	Mov_average = MA_total / MA_numReadings;
	return(Mov_average);
}


/**
  * @brief  MOVING_AVERAGE PRESSURE
  * @param  argument: Inval
  * @retval Mov_Flow_average
  */

double MOVING_AVERAGE_FLOW(double Inval)
{
	MA_Flow_total = MA_Flow_total - MA_Flow_readings[MA_Flow_readIndex];
	MA_Flow_readings[MA_Flow_readIndex] = Inval;
	MA_Flow_total = MA_Flow_total + MA_Flow_readings[MA_Flow_readIndex];
	MA_Flow_readIndex = MA_Flow_readIndex + 1;
	if (MA_Flow_readIndex >= MA_Flow_numReadings) {
		MA_Flow_readIndex = 0;
	}
	Mov_Flow_average = MA_Flow_total / MA_Flow_numReadings;
	return(Mov_Flow_average);
}

