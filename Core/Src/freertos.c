/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "usbd_cdc_if.h"
#include "stdlib.h"
#include "string.h"
#include "adc.h"
#include "dma.h"
#include "dac.h"
//#include "ventilator_types.h"
#include "Venitilator_Cfg.h"
#include "Ventilator_Calc.h"
#include "math.h"
#include "stdio.h"
#include "Pressure_Calculation.h"
#include "Ventilator_Util.h"
#include "arm_math.h"
#include "usart.h"
#include "controlsys.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
WaveFormState _CurrentWaveFormState = NoWaveFormState ;
BackupModes _CurrentBackupMode = IdleState ;
ComputationState _CurrentComputationState = NoComputeState ;
ADCOperationState _CurrentADCOperationState = CalibrationState ;
CurrentMode _CurrentMode = NoMode, _RequestedMode = NoMode ;
PWM_DUTY_State _PWM_DUTY_State = NO_DUTY ;

ALERT_RESPONSE_PACKET _ALERT_RESPONSE_PKT ;
RESPOND_SERVICE_PACKET_tst _RESPOND_SERVICE_PACKET ;

#define SET_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 1 )
#define CLEAR_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 0 )
#define _ALERT_RESPONSE_PKT_length 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int varsize;


float pressure_max=0;
float pressure_max_acheived=0;
uint16_t pressure_errror_count;
float check_pressure;
FIRST_FRAME FIRST_FRAME_name;

#define E_TIME_TOLERANCE 30u
#define BIPAP_E_TIME_TOLERANCE 30u
uint16_t _E_TIMER = 0 , _I_TIMER,_RT_TIMER;
controlsys cs_PIP = { 150,170, 0, 0.02, 300, 4095, 300, 4095, 0.002 };
controlsys cs_PEEP = { 100,150, 0, 0.02, 300, 4095, 300, 4095, 0.002 };
//controlsys cs = { 145,95, 0, 0.02, 300, 4095, 300, 4095, 0.002 };
controlsys cs_flow = { 30, 200, 0.01, 0.02, 300, 4095, 300, 4095, 0.002 };

uint16_t _APNEA_COUNTER = 0 ,_PSV_IGNORE=1000;
uint8_t _cPAP_Trigger = pdFALSE, _PSV_Trigger = pdFALSE, _PSV_REACH =pdFALSE ;
uint32_t RT_Wave = 0 , I_Wave = 0, E_Wave = 0 , RR_Value = 0 , RTI_Wave = 0 ;
uint32_t _Req_RT_Wave = 1000 , _Req_I_Wave = 1250, _Req_E_Wave = 3750 , _Req_RR_Value = 12 , _Req_RTI_Wave = 0 ;
uint8_t  _Received_PIP_Val =0 , _Received_PEEP_Val = 0 ,_Received_VT_Val = 0 ;
uint8_t _PIP_Val = 0 , _PEEP_Val = 0 ,  _Mode_Val =0 , _PIO2_Val = 0 ,_Reserved_Mode = 0 ,_Control_Byte = 0,_Flow_Rate=0,_CPAP_Val = 0,_IPAP_Val=0,_EPAP_Val=0 ;
uint16_t _VT_Val= 0 ;
uint8_t _TX_DEBUG_DATA_BUF[3] ;
uint8_t _TX_DATA_BUF[6] ;
uint16_t _DAC_Val = 0 ;
uint8_t _IN_TRIG_PERCENT = 0 ;
uint16_t _CALC_TRIG_VAL = 0 ;
uint16_t _TRIG_WINDOW = 0 ,_TOLERANCE_EWAVE = 0;

//////////////////
uint16_t _BIPAP_CALC_TRIG_VAL = 0 ;
uint16_t _BIPAP_TRIG_WINDOW = 0 ,_BIPAP_TOLERANCE_EWAVE = 0;
///////////////////

//RANGE
uint8_t _RANGE_MODE_Val=0;
uint16_t _RANGE_VT_MIN_Val=0;
uint16_t _RANGE_VT_MAX_Val=0;
uint8_t _RANGE_PIP_MIN_Val=0;
uint8_t _RANGE_PIP_MAX_Val=0;
uint8_t _RANGE_RR_MIN_Val=0;
uint8_t _RANGE_RR_MAX_Val=0;
uint16_t _RANGE_MINT_VOL_MIN_Val=0;
uint16_t _RANGE_MINT_VOL_MAX_Val=0;
uint8_t _RANGE_SPO2_MIN_Val=0;
uint8_t _RANGE_SPO2_MAX_Val=0;
uint8_t _RANGE_PULSE_MIN_Val=0;
uint8_t _RANGE_PULSE_MAX_Val=0;

uint8_t _SERVICE_DATA0_Val=0;
uint8_t _SERVICE_DATA1_Val=0;

float _AVG_Pressure,_AVG_CirusO2Sensor,_AVG_O2FlowFeedback,CirusO2Sensor_Val, O2FlowFeedback_Val ;
float  _AVG_Flow ;
uint16_t _ADC_CHANNEL0[4096],_ADC_CHANNEL1[4096], _ADC_CHANNEL2[4096];
uint16_t _DAC_VAL0 , _DAC_VAL1 ;
uint8_t _MODE_DATA_SWITCH_FLAG = pdFALSE ;
float _Peep_Avg=0,_Peep_Avg_count=0,_Peep_Avg_val=0,_Set_Peep=0,_Set_Peep_BIPAP_APR_VC=0;
float _Pip_Avg=0,_Pip_Avg_count=0,_Pip_Avg_val=0;
float _Vol_Avg=0,_Vol_Avg_count=0,_Vol_Avg_val=0;


uint16_t  _APNEA_TIME,_TRIG_TIME, _TRIG_PER;
uint16_t  _TRIG_TYPE,_TRIG_LMT;
uint16_t  _T_HIGH,_T_LOW;

float simv_trigger_offset=0,_Wait_Flow_offset=1;

long _DROPPED_PACKET_COUNT = 0 , _UART_OVER_ERROR = 0 ;

/*PIP & PEEP Reach  Flag*/
uint8_t _PIP_REACHED_FLAG = pdFALSE , _PEEP_REACHED_FLAG = pdFALSE ;

long _ADC_SampleCount ;
float _Adc_Channel0 = 0 ,_Adc_Channel1 = 0 ;

uint8_t TransmitFlag = 0 ;

uint16_t _OldPressure_Val,_OldFlow_Val ;
uint8_t RR;
float Acheived_RR;

SET_PARAM_CMD_PACKET *_RX_PARAM_PKT ;
SET_CYCLIC_PACKET _CYCLIC_TRANSMIT_PKT ;
SET_MONITORING_PACKET _MONITORING_TRANSMIT_PKT ;
DATA_REQUEST_PACKET _DATA_REQ_PKT ;


__attribute__((section(".ccmram")))
SENSOR_DATA_BANK _SENSOR_DATA_BANK_CCM ;

uint16_t _BUFFER_INDEX = 0 ;


/*********  FOR FIO2 ***********/
uint16_t FiO2_old=21;
extern int k;
uint16_t O2_DAC=1870;
uint16_t fio2count=0;
uint16_t fio2count2=0;
uint16_t O2_typecast;
uint16_t cirus_Online;
float cirus_volt;
uint16_t cirus_volt2;
uint16_t cirus_volt_new;
uint16_t cirus_volt_old;
uint16_t O2_percentage;
float O2_percentage_80;
int O2_process=1;
int O2_read=1;

int apnea_mode;
uint16_t CHECK_O2_APNEA_COUNTER;

int _Pressure_Base;
int _Flow_Base;

/***********  END  ************/

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PulseGenerator */
osThreadId_t PulseGeneratorHandle;
const osThreadAttr_t PulseGenerator_attributes = {
  .name = "PulseGenerator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CMD_Process_Tas */
osThreadId_t CMD_Process_TasHandle;
const osThreadAttr_t CMD_Process_Tas_attributes = {
  .name = "CMD_Process_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TransmitTask */
osThreadId_t TransmitTaskHandle;
const osThreadAttr_t TransmitTask_attributes = {
  .name = "TransmitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DataComputation */
osThreadId_t DataComputationHandle;
const osThreadAttr_t DataComputation_attributes = {
  .name = "DataComputation",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Conversion */
osThreadId_t ADC_ConversionHandle;
const osThreadAttr_t ADC_Conversion_attributes = {
  .name = "ADC_Conversion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Set_DAC_Value */
osThreadId_t Set_DAC_ValueHandle;
const osThreadAttr_t Set_DAC_Value_attributes = {
  .name = "Set_DAC_Value",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CMVPC */
osThreadId_t CMVPCHandle;
const osThreadAttr_t CMVPC_attributes = {
  .name = "CMVPC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CMVVC */
osThreadId_t CMVVCHandle;
const osThreadAttr_t CMVVC_attributes = {
  .name = "CMVVC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SIMVPC */
osThreadId_t SIMVPCHandle;
const osThreadAttr_t SIMVPC_attributes = {
  .name = "SIMVPC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SIMVVC */
osThreadId_t SIMVVCHandle;
const osThreadAttr_t SIMVVC_attributes = {
  .name = "SIMVVC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PS_V */
osThreadId_t PS_VHandle;
const osThreadAttr_t PS_V_attributes = {
  .name = "PS_V",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for cPAP_V */
osThreadId_t cPAP_VHandle;
const osThreadAttr_t cPAP_V_attributes = {
  .name = "cPAP_V",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BiPAP_V */
osThreadId_t BiPAP_VHandle;
const osThreadAttr_t BiPAP_V_attributes = {
  .name = "BiPAP_V",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for APR_VC */
osThreadId_t APR_VCHandle;
const osThreadAttr_t APR_VC_attributes = {
  .name = "APR_VC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Debug */
osThreadId_t DebugHandle;
const osThreadAttr_t Debug_attributes = {
  .name = "Debug",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_CMD_Handle */
osThreadId_t UART_CMD_HandleHandle;
const osThreadAttr_t UART_CMD_Handle_attributes = {
  .name = "UART_CMD_Handle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for oneMilliSecond */
osThreadId_t oneMilliSecondHandle;
const osThreadAttr_t oneMilliSecond_attributes = {
  .name = "oneMilliSecond",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControlSystem */
osThreadId_t ControlSystemHandle;
const osThreadAttr_t ControlSystem_attributes = {
  .name = "ControlSystem",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PSV_Mode */
osThreadId_t PSV_ModeHandle;
const osThreadAttr_t PSV_Mode_attributes = {
  .name = "PSV_Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PCCMV_Backup */
osThreadId_t PCCMV_BackupHandle;
const osThreadAttr_t PCCMV_Backup_attributes = {
  .name = "PCCMV_Backup",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VCCMV_Backup */
osThreadId_t VCCMV_BackupHandle;
const osThreadAttr_t VCCMV_Backup_attributes = {
  .name = "VCCMV_Backup",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OxygenTask */
osThreadId_t OxygenTaskHandle;
const osThreadAttr_t OxygenTask_attributes = {
  .name = "OxygenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Mode_Analysis */
osThreadId_t Mode_AnalysisHandle;
const osThreadAttr_t Mode_Analysis_attributes = {
  .name = "Mode_Analysis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_ble */
osThreadId_t service_bleHandle;
const osThreadAttr_t service_ble_attributes = {
  .name = "service_ble",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_bldc */
osThreadId_t service_bldcHandle;
const osThreadAttr_t service_bldc_attributes = {
  .name = "service_bldc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_valve */
osThreadId_t service_valveHandle;
const osThreadAttr_t service_valve_attributes = {
  .name = "service_valve",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_proxima */
osThreadId_t service_proximaHandle;
const osThreadAttr_t service_proxima_attributes = {
  .name = "service_proxima",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_leak */
osThreadId_t service_leakHandle;
const osThreadAttr_t service_leak_attributes = {
  .name = "service_leak",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_battery */
osThreadId_t service_batteryHandle;
const osThreadAttr_t service_battery_attributes = {
  .name = "service_battery",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_led */
osThreadId_t service_ledHandle;
const osThreadAttr_t service_led_attributes = {
  .name = "service_led",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_ads1115 */
osThreadId_t service_ads1115Handle;
const osThreadAttr_t service_ads1115_attributes = {
  .name = "service_ads1115",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_servo */
osThreadId_t service_servoHandle;
const osThreadAttr_t service_servo_attributes = {
  .name = "service_servo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_sensor */
osThreadId_t service_sensorHandle;
const osThreadAttr_t service_sensor_attributes = {
  .name = "service_sensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_nebulis */
osThreadId_t service_nebulisHandle;
const osThreadAttr_t service_nebulis_attributes = {
  .name = "service_nebulis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_oxygen */
osThreadId_t service_oxygenHandle;
const osThreadAttr_t service_oxygen_attributes = {
  .name = "service_oxygen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Receive_Buffer */
osSemaphoreId_t Receive_BufferHandle;
const osSemaphoreAttr_t Receive_Buffer_attributes = {
  .name = "Receive_Buffer"
};
/* Definitions for ADC_Buffer */
osSemaphoreId_t ADC_BufferHandle;
const osSemaphoreAttr_t ADC_Buffer_attributes = {
  .name = "ADC_Buffer"
};
/* Definitions for UART_RX_Buffer */
osSemaphoreId_t UART_RX_BufferHandle;
const osSemaphoreAttr_t UART_RX_Buffer_attributes = {
  .name = "UART_RX_Buffer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void PulseGeneratorTask(void *argument);
void CMD_Handler(void *argument);
void TransmitDataTask(void *argument);
void DataComputation_Task(void *argument);
void ADC_Conversion_Task(void *argument);
void Set_DAC_Value_Task(void *argument);
void CMVPC_Mode_Task(void *argument);
void CMVVC_Mode_Task(void *argument);
void SIMVPC_Mode_Task(void *argument);
void SIMVVC_Mode_Task(void *argument);
void PS_V_Mode_Task(void *argument);
void cPAP_V_Mode_Task(void *argument);
void BiPAP_V_Mode_Task(void *argument);
void APR_VC_Mode_Task(void *argument);
void Debug_Task(void *argument);
void UART_CMD_Handler_Task(void *argument);
void oneMilliSecondTask(void *argument);
void ControlSystemTask(void *argument);
void PSV_ModeTask(void *argument);
void PCCMV_BackupTask(void *argument);
void VCCMV_BackupTask(void *argument);
void OxyTask(void *argument);
void Mode_Analysis_Task(void *argument);
void SERVICE_Communication(void *argument);
void SERVICE_Blower(void *argument);
void SERVICE_ExpValve(void *argument);
void SERVICE_Proximal(void *argument);
void SERVICE_Leak(void *argument);
void SERVICE_Battery(void *argument);
void SERVICE_Led(void *argument);
void SERVICE_ADS1115(void *argument);
void SERVICE_Servo(void *argument);
void SERVICE_Sensor(void *argument);
void SERVICE_Nebuliser(void *argument);
void SERVICE_Oxygen(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Receive_Buffer */
  Receive_BufferHandle = osSemaphoreNew(1, 1, &Receive_Buffer_attributes);

  /* creation of ADC_Buffer */
  ADC_BufferHandle = osSemaphoreNew(1, 1, &ADC_Buffer_attributes);

  /* creation of UART_RX_Buffer */
  UART_RX_BufferHandle = osSemaphoreNew(1, 1, &UART_RX_Buffer_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PulseGenerator */
  PulseGeneratorHandle = osThreadNew(PulseGeneratorTask, NULL, &PulseGenerator_attributes);

  /* creation of CMD_Process_Tas */
  CMD_Process_TasHandle = osThreadNew(CMD_Handler, NULL, &CMD_Process_Tas_attributes);

  /* creation of TransmitTask */
  TransmitTaskHandle = osThreadNew(TransmitDataTask, NULL, &TransmitTask_attributes);

  /* creation of DataComputation */
  DataComputationHandle = osThreadNew(DataComputation_Task, NULL, &DataComputation_attributes);

  /* creation of ADC_Conversion */
  ADC_ConversionHandle = osThreadNew(ADC_Conversion_Task, NULL, &ADC_Conversion_attributes);

  /* creation of Set_DAC_Value */
  Set_DAC_ValueHandle = osThreadNew(Set_DAC_Value_Task, NULL, &Set_DAC_Value_attributes);

  /* creation of CMVPC */
  CMVPCHandle = osThreadNew(CMVPC_Mode_Task, NULL, &CMVPC_attributes);

  /* creation of CMVVC */
  CMVVCHandle = osThreadNew(CMVVC_Mode_Task, NULL, &CMVVC_attributes);

  /* creation of SIMVPC */
  SIMVPCHandle = osThreadNew(SIMVPC_Mode_Task, NULL, &SIMVPC_attributes);

  /* creation of SIMVVC */
  SIMVVCHandle = osThreadNew(SIMVVC_Mode_Task, NULL, &SIMVVC_attributes);

  /* creation of PS_V */
  PS_VHandle = osThreadNew(PS_V_Mode_Task, NULL, &PS_V_attributes);

  /* creation of cPAP_V */
  cPAP_VHandle = osThreadNew(cPAP_V_Mode_Task, NULL, &cPAP_V_attributes);

  /* creation of BiPAP_V */
  BiPAP_VHandle = osThreadNew(BiPAP_V_Mode_Task, NULL, &BiPAP_V_attributes);

  /* creation of APR_VC */
  APR_VCHandle = osThreadNew(APR_VC_Mode_Task, NULL, &APR_VC_attributes);

  /* creation of Debug */
  DebugHandle = osThreadNew(Debug_Task, NULL, &Debug_attributes);

  /* creation of UART_CMD_Handle */
  UART_CMD_HandleHandle = osThreadNew(UART_CMD_Handler_Task, NULL, &UART_CMD_Handle_attributes);

  /* creation of oneMilliSecond */
  oneMilliSecondHandle = osThreadNew(oneMilliSecondTask, NULL, &oneMilliSecond_attributes);

  /* creation of ControlSystem */
  ControlSystemHandle = osThreadNew(ControlSystemTask, NULL, &ControlSystem_attributes);

  /* creation of PSV_Mode */
  PSV_ModeHandle = osThreadNew(PSV_ModeTask, NULL, &PSV_Mode_attributes);

  /* creation of PCCMV_Backup */
  PCCMV_BackupHandle = osThreadNew(PCCMV_BackupTask, NULL, &PCCMV_Backup_attributes);

  /* creation of VCCMV_Backup */
  VCCMV_BackupHandle = osThreadNew(VCCMV_BackupTask, NULL, &VCCMV_Backup_attributes);

  /* creation of OxygenTask */
  OxygenTaskHandle = osThreadNew(OxyTask, NULL, &OxygenTask_attributes);

  /* creation of Mode_Analysis */
  Mode_AnalysisHandle = osThreadNew(Mode_Analysis_Task, NULL, &Mode_Analysis_attributes);

  /* creation of service_ble */
  service_bleHandle = osThreadNew(SERVICE_Communication, NULL, &service_ble_attributes);

  /* creation of service_bldc */
  service_bldcHandle = osThreadNew(SERVICE_Blower, NULL, &service_bldc_attributes);

  /* creation of service_valve */
  service_valveHandle = osThreadNew(SERVICE_ExpValve, NULL, &service_valve_attributes);

  /* creation of service_proxima */
  service_proximaHandle = osThreadNew(SERVICE_Proximal, NULL, &service_proxima_attributes);

  /* creation of service_leak */
  service_leakHandle = osThreadNew(SERVICE_Leak, NULL, &service_leak_attributes);

  /* creation of service_battery */
  service_batteryHandle = osThreadNew(SERVICE_Battery, NULL, &service_battery_attributes);

  /* creation of service_led */
  service_ledHandle = osThreadNew(SERVICE_Led, NULL, &service_led_attributes);

  /* creation of service_ads1115 */
  service_ads1115Handle = osThreadNew(SERVICE_ADS1115, NULL, &service_ads1115_attributes);

  /* creation of service_servo */
  service_servoHandle = osThreadNew(SERVICE_Servo, NULL, &service_servo_attributes);

  /* creation of service_sensor */
  service_sensorHandle = osThreadNew(SERVICE_Sensor, NULL, &service_sensor_attributes);

  /* creation of service_nebulis */
  service_nebulisHandle = osThreadNew(SERVICE_Nebuliser, NULL, &service_nebulis_attributes);

  /* creation of service_oxygen */
  service_oxygenHandle = osThreadNew(SERVICE_Oxygen, NULL, &service_oxygen_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	vTaskSuspend(CMVPCHandle);
	vTaskSuspend(CMVVCHandle);
	vTaskSuspend(SIMVPCHandle);
	vTaskSuspend(SIMVVCHandle);
	vTaskSuspend(PS_VHandle);
	vTaskSuspend(PSV_ModeHandle);
	vTaskSuspend(cPAP_VHandle);
	vTaskSuspend(BiPAP_VHandle);
	vTaskSuspend(APR_VCHandle);
	vTaskSuspend(Set_DAC_ValueHandle);
	vTaskResume(TransmitTaskHandle);
	vTaskSuspend(DebugHandle);
	vTaskSuspend(PCCMV_BackupHandle);
	vTaskSuspend(VCCMV_BackupHandle);
	vTaskSuspend(service_bleHandle);
	vTaskSuspend(service_bldcHandle);
	vTaskSuspend(service_valveHandle);
	vTaskSuspend(service_proximaHandle);
	vTaskSuspend(service_leakHandle);
	vTaskSuspend(service_nebulisHandle);
	vTaskSuspend(service_servoHandle);
	vTaskSuspend(service_ledHandle);
	vTaskSuspend(service_sensorHandle);

	vTaskSuspend(service_ads1115Handle);
	vTaskSuspend(service_oxygenHandle);
	vTaskSuspend(service_batteryHandle);
	//vTaskSuspend(OxygenTaskHandle);
	_SENSOR_DATA_BANK_CCM._ACTIVE_BANK = 1 ;
	controlsys_Init(&cs_PIP);
	controlsys_Init(&cs_PEEP);
	controlsys_Init(&cs_flow);

	//controlsys_Update(&cs, need, sense);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;

	//_ALERT_RESPONSE_PKT.FIRST_FRAME_UN.FIRST_BYTES = 0x81 ;

	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PulseGeneratorTask */
/**
 * @brief Function implementing the PulseGenerator thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PulseGeneratorTask */
void PulseGeneratorTask(void *argument)
{
  /* USER CODE BEGIN PulseGeneratorTask */
	TickType_t xDelay = 0 ; // portTICK_PERIOD_MS
	/* Infinite loop */
	for(;;)
	{

		switch (_CurrentWaveFormState) {
		case Generate_I_Wave:

			RR_E_TIME=_E_TIMER_ACHEIVED;
			_I_TIMER_ACHEIVED=0;
			fio2count++;

			varsize=sizeof(_ALERT_RESPONSE_PKT);
			SEND_ALERT_PACKET();


			_Peep_Avg_val=round(_Peep_Avg/_Peep_Avg_count);
			_Pip_Avg_val=_Pip_Avg/_Pip_Avg_count;
			_Vol_Avg_val=_Vol_Avg/_Vol_Avg_count;

			if(_Peep_Avg_val>_PEEP_Val)
			{
				_Set_Peep=_Set_Peep-0.1f;
				if(_Set_Peep<=3)
				{
					_Set_Peep=3;
				}

			}
			else
			{
				_Set_Peep=_Set_Peep+0.1f;
				if(_Set_Peep>_PEEP_Val+2)
				{
					_Set_Peep=_PEEP_Val/2;
				}
			}
			//_Set_Peep=4.01;
			/////////////////////////
			if(_Mode_Val==8)
			{
			if(_Peep_Avg_val>_EPAP_Val)
					{
				_Set_Peep_BIPAP_APR_VC=_Set_Peep_BIPAP_APR_VC-0.5;
					}
					else
					{
						_Set_Peep_BIPAP_APR_VC=_Set_Peep_BIPAP_APR_VC+0.5;
						if(_Set_Peep_BIPAP_APR_VC>_EPAP_Val+2)
						{
							_Set_Peep_BIPAP_APR_VC=_EPAP_Val;
						}
					}
			}
			/////////////////////////

			_Peep_Avg=0;
			_Peep_Avg_count=0;

			_Pip_Avg=0;
			_Pip_Avg_count=0;

			_Total_Volume=0;
			controlsys_Init(&cs_PIP);
			controlsys_Init(&cs_PEEP);
			controlsys_Init(&cs_flow);
			//  controlsys cs = { 40,5, 0, 0.02, 300, 4095, 300, 4095, 0.002 };
			_Control_Byte&=(uint8_t) (~(0x80));
			HAL_GPIO_WritePin(Wave_GPIO_Port, Wave_Pin, GPIO_PIN_SET);
			_CurrentWaveFormState = Generate_E_Wave ;
			_CurrentComputationState = Compute_I_Wave ;
			xDelay = (RTI_Wave) ;
			_I_TIMER = RTI_Wave ;
			_E_TIMER = 0 ;
			if(_Mode_Val==8)
			{
				xDelay=_T_HIGH*1000;
				_I_TIMER=_T_HIGH*1000;
				_E_TIMER=_T_LOW*1000;
			}
			if(_Mode_Val==5)
					{
						xDelay=_T_HIGH*1000;
						_I_TIMER=_T_HIGH*1000;
						_E_TIMER=_T_LOW*1000;
					}
			vTaskDelay( xDelay );
			break;
		case Generate_E_Wave:

			_E_TIMER_ACHEIVED=0;
			RR_I_TIME=_I_TIMER_ACHEIVED;
			_max_volume_val=(_Total_Volume/100);
			_max_Tidal_volume_val=_Total_Volume;

			_Control_Byte|=(uint8_t) 0x80;
			HAL_GPIO_WritePin(Wave_GPIO_Port, Wave_Pin, GPIO_PIN_RESET);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
			xDelay = (E_Wave) ;
			_I_TIMER = 0 ;
			_E_TIMER = E_Wave ;

			if(_MODE_DATA_SWITCH_FLAG == pdTRUE )
			{
				if(_CurrentMode !=NoMode){
					RT_Wave = _Req_RT_Wave ;
					I_Wave = _Req_I_Wave ;
					E_Wave = _Req_E_Wave ;
					RTI_Wave = _Req_RTI_Wave ;
				}
				if(_CurrentMode != _RequestedMode  && _RequestedMode == PCCMV)
				{
					vTaskResume(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = PCCMV ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == VCCMV)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskResume(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = VCCMV ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == SIMVPC)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskResume(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = SIMVPC ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == SIMVVC)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskResume(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = SIMVVC ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == cPAP)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskResume(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = cPAP ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == BiPAP)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskResume(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = BiPAP ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == APR_VC)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskResume(APR_VCHandle);
					vTaskSuspend(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = APR_VC ;
				}
				else if(_CurrentMode != _RequestedMode  && _RequestedMode == PSV)
				{
					vTaskSuspend(CMVPCHandle);
					vTaskSuspend(CMVVCHandle);
					vTaskSuspend(SIMVPCHandle);
					vTaskSuspend(SIMVVCHandle);
					vTaskSuspend(cPAP_VHandle);
					vTaskSuspend(BiPAP_VHandle);
					vTaskSuspend(APR_VCHandle);
					vTaskResume(PSV_ModeHandle);
					vTaskSuspend(PCCMV_BackupHandle);
					vTaskSuspend(VCCMV_BackupHandle);
					_CurrentMode = PSV ;
				}
				else
				{

				}

				_MODE_DATA_SWITCH_FLAG = pdFALSE ;
			}

			if(cPAP == _CurrentMode && _cPAP_Trigger == pdFALSE )
			{
				_CurrentWaveFormState = Generate_E_Wave ;
				_CurrentComputationState = Compute_E_Wave ;
			}
			else if(PSV == _CurrentMode && _cPAP_Trigger == pdFALSE )  //_PSV_Trigger
			{
				_CurrentWaveFormState = Generate_E_Wave ;
				_CurrentComputationState = Compute_E_Wave ;
			}
			else
			{
				_CurrentWaveFormState = Generate_I_Wave ;
				_CurrentComputationState = Compute_E_Wave ;
			}

			_MONITORING_TRANSMIT_PKT._header         = 0x504d ;
			_MONITORING_TRANSMIT_PKT._length         = sizeof(_MONITORING_TRANSMIT_PKT)-4 ;
			//_MONITORING_TRANSMIT_PKT._R_PIP          = 0 ;
			//_MONITORING_TRANSMIT_PKT._R_PEEP         = 0 ;
			//_MONITORING_TRANSMIT_PKT._R_VTI          = 0 ;
			//_MONITORING_TRANSMIT_PKT._R_VTE          = 0 ;
			_MONITORING_TRANSMIT_PKT._R_TI           = 0 ;
			_MONITORING_TRANSMIT_PKT._R_TE           = 0 ;
			_MONITORING_TRANSMIT_PKT._R_FIO2         = 0 ;
			_MONITORING_TRANSMIT_PKT._RiseTime       = 0 ;
			_MONITORING_TRANSMIT_PKT._R_P_PLATEAU    = 0 ;
			_MONITORING_TRANSMIT_PKT._CRC8           = chksum8(&_MONITORING_TRANSMIT_PKT._R_PIP,_MONITORING_TRANSMIT_PKT._length);
			if(_Mode_Val==8)
			{
			 xDelay=_T_LOW*1000;
			 _E_TIMER = _T_LOW*1000;
			}
			if(_Mode_Val==5)
						{
						 xDelay=_T_LOW*1000;
						 _E_TIMER = _T_LOW*1000;
						}
			vTaskDelay( xDelay );

			break;
		case NoWaveFormState:
			break;
		default:
			break;

		}
	}
  /* USER CODE END PulseGeneratorTask */
}

/* USER CODE BEGIN Header_CMD_Handler */
/**
 * @brief Function implementing the CMD_Process_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CMD_Handler */
void CMD_Handler(void *argument)
{
  /* USER CODE BEGIN CMD_Handler */
	xSemaphoreTake(Receive_BufferHandle, portMAX_DELAY);
	uint8_t _RX_CRC8 ;
	uint32_t _TX_LENGTH = 0 ;
	/* Infinite loop */
	for(;;)
	{

		/* Lock Task here until unlocked by ISR*/
		xSemaphoreTake(Receive_BufferHandle, portMAX_DELAY);

		//Order of Reception RR_Value/I_Wave/E_Wave/RT_Wave/_PIP_Val/_PEEP_Val/_VT_Val/_Mode_Val/_PIO2_Val
		_RX_CRC8 = chksum8(&received_data[3],((SET_PARAM_CMD_PACKET*) (received_data))->_length);

		if(_RX_CRC8 == ((SET_PARAM_CMD_PACKET*) (received_data))->_CRC8)
		{
#if CYCLIC_TRANSMIT == DISABLE
			if((0x5052 == ((SET_PARAM_CMD_PACKET*) (received_data))->_header) && (0x01 == ((DATA_REQUEST_PACKET*) (received_data))->_data))
			{
				_CYCLIC_TRANSMIT_PKT._header          = 0x5052 ;
				_CYCLIC_TRANSMIT_PKT._length          = sizeof(_CYCLIC_TRANSMIT_PKT)-4 ;
				_CYCLIC_TRANSMIT_PKT._Pressure_Val    = _Pressure_Val ;
				_CYCLIC_TRANSMIT_PKT._Flow_Val        = _Flow_Val ;
				_CYCLIC_TRANSMIT_PKT._Volume_Val      = _Volume_Val ;
				_CYCLIC_TRANSMIT_PKT._CRC8            = chksum8(&_CYCLIC_TRANSMIT_PKT._Pressure_Val,_CYCLIC_TRANSMIT_PKT._length); ;

				CDC_Transmit_FS((uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT)) ;

				if(TransmitFlag == 0)
				{
					HAL_UART_Transmit_IT(&huart3,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));
					TransmitFlag = 1 ;
				}
			}
			else if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header) && (0x02 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
#else
				if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header) && (0x02 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
#endif
				{	CDC_Transmit_FS((uint8_t*)&_MONITORING_TRANSMIT_PKT,sizeof(_MONITORING_TRANSMIT_PKT)-1) ;
				if(TransmitFlag == 0)
				{
					HAL_UART_Transmit_IT(&huart3,(uint8_t*)&_MONITORING_TRANSMIT_PKT,sizeof(_MONITORING_TRANSMIT_PKT));
					TransmitFlag = 1 ;
				}
				}
				else if((0x5052 == ((SET_PARAM_CMD_PACKET*) (received_data))->_header) && (0x03 == ((DATA_REQUEST_PACKET*) (received_data))->_data))
				{
					if(_SENSOR_DATA_BANK_CCM._HISTORY_AVAILBLE == 1 )
					{
						_TX_LENGTH = (I_Wave+E_Wave)*3 ;

						if(_SENSOR_DATA_BANK_CCM._ACTIVE_BANK == 1)
						{
							CDC_Transmit_FS((uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK2,_TX_LENGTH) ;
							if(TransmitFlag == 0)
							{
								HAL_UART_Transmit_IT(&huart3,(uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK2,_TX_LENGTH) ;
								TransmitFlag = 1 ;
							}
						}
						else
						{
							CDC_Transmit_FS((uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK1,_TX_LENGTH) ;
							if(TransmitFlag == 0)
							{
								HAL_UART_Transmit_IT(&huart3,(uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK1,_TX_LENGTH) ;
								TransmitFlag = 1 ;
							}
						}
					}
				}
				else if((0x5053 == ((SET_PARAM_CMD_PACKET*) (received_data))->_header))
								{
									COMMAND_HANDLER((SET_PARAM_CMD_PACKET*) (received_data));
								}
								else if((0x5054 == ((SET_PARAM_CMD_PACKET*) (received_data))->_header))
								{
									ALERT_COMMAND_HANDLER((ALERT_RANGE_PACKET*) (received_data));
								}
								else if((0x5055 == ((SET_PARAM_CMD_PACKET*) (received_data))->_header))
								{
									SERVICE_COMMAND_HANDLER((REQUEST_SERVICE_PACKET_tst*) (received_data));
								//	vTaskSuspend(TransmitTaskHandle);
								}
								else
								{
									//_DROPPED_PACKET_COUNT++;
								}
		}

	}
  /* USER CODE END CMD_Handler */
}

/* USER CODE BEGIN Header_TransmitDataTask */
/**
 * @brief Function implementing the TransmitTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TransmitDataTask */
void TransmitDataTask(void *argument)
{
  /* USER CODE BEGIN TransmitDataTask */
	TickType_t xDelay = 20 ; // portTICK_PERIOD_MS
	/* Infinite loop */
	for(;;)
	{
		_CYCLIC_TRANSMIT_PKT._header          = 0x5052 ;
		_CYCLIC_TRANSMIT_PKT._length          = sizeof(_CYCLIC_TRANSMIT_PKT)-4 ;
		_CYCLIC_TRANSMIT_PKT._Pressure_Val    = _Pressure_Val ;
		_CYCLIC_TRANSMIT_PKT._Flow_Val        = _Flow_Val ;
		//_CYCLIC_TRANSMIT_PKT._Flow_Val        = -100 ;
		//_CYCLIC_TRANSMIT_PKT._Volume_Val      = -100 ;
		_CYCLIC_TRANSMIT_PKT._Volume_Val      = _Total_Volume ;
		_CYCLIC_TRANSMIT_PKT._Control_Byte    = _Control_Byte ;
		_CYCLIC_TRANSMIT_PKT._SPO2            = 0 ;
		_CYCLIC_TRANSMIT_PKT._Heart_BPM       = 0 ;
		_CYCLIC_TRANSMIT_PKT._CRC8            = chksum8(&_CYCLIC_TRANSMIT_PKT._Pressure_Val,_CYCLIC_TRANSMIT_PKT._length); ;

		CDC_Transmit_FS((uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT)) ;

		HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));

		vTaskDelay( xDelay );
	}
  /* USER CODE END TransmitDataTask */
}

/* USER CODE BEGIN Header_DataComputation_Task */
/**
 * @brief Function implementing the DataComputation thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DataComputation_Task */
void DataComputation_Task(void *argument)
{
  /* USER CODE BEGIN DataComputation_Task */
	TickType_t xDelay = 1 ; // portTICK_PERIOD_MS
	/* Infinite loop */
	for(;;)
	{
		if(_CurrentMode != NoMode)
		{

			if(_SENSOR_DATA_BANK_CCM._ACTIVE_BANK == 1 && _BUFFER_INDEX < (I_Wave+E_Wave) )
			{
				_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._FLOW_VAL_BUF[_BUFFER_INDEX] = _Flow_Val ;
				_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._PRESSURE_VAL_BUF[_BUFFER_INDEX] = _Pressure_Val ;
				_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._VOLUME_VAL_BUF[_BUFFER_INDEX] = _VT_Val ;
				_BUFFER_INDEX++ ;
			}
			else
			{
				/*At least one Bank is full set the flag */
				_SENSOR_DATA_BANK_CCM._HISTORY_AVAILBLE = 1 ;

				if(_SENSOR_DATA_BANK_CCM._ACTIVE_BANK == 1 )
				{
					_BUFFER_INDEX = 0 ;
					_SENSOR_DATA_BANK_CCM._ACTIVE_BANK = 2 ;
					_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._FLOW_VAL_BUF[_BUFFER_INDEX] = _Flow_Val ;
					_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._PRESSURE_VAL_BUF[_BUFFER_INDEX] = _Pressure_Val ;
					_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._VOLUME_VAL_BUF[_BUFFER_INDEX] = _VT_Val ;
					_BUFFER_INDEX++ ;
				}
				else
				{
					if(_SENSOR_DATA_BANK_CCM._ACTIVE_BANK == 2 && _BUFFER_INDEX < (I_Wave+E_Wave))
					{
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._FLOW_VAL_BUF[_BUFFER_INDEX] = _Flow_Val ;
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._PRESSURE_VAL_BUF[_BUFFER_INDEX] = _Pressure_Val ;
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK2._VOLUME_VAL_BUF[_BUFFER_INDEX] = _VT_Val ;
						_BUFFER_INDEX++ ;
					}
					else
					{
						_BUFFER_INDEX = 0 ;
						_SENSOR_DATA_BANK_CCM._ACTIVE_BANK = 1 ;
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._FLOW_VAL_BUF[_BUFFER_INDEX] = _Flow_Val ;
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._PRESSURE_VAL_BUF[_BUFFER_INDEX] = _Pressure_Val ;
						_SENSOR_DATA_BANK_CCM._HISTORY_BANK1._VOLUME_VAL_BUF[_BUFFER_INDEX] = _VT_Val ;
						_BUFFER_INDEX++ ;
					}
				}

			}
		}
		vTaskDelay( xDelay );
	}
  /* USER CODE END DataComputation_Task */
}

/* USER CODE BEGIN Header_ADC_Conversion_Task */
/**
 * @brief Function implementing the ADC_Conversion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ADC_Conversion_Task */
void ADC_Conversion_Task(void *argument)
{
  /* USER CODE BEGIN ADC_Conversion_Task */
	xSemaphoreTake(ADC_BufferHandle, portMAX_DELAY);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_RESULT_BUF, 3);

	ExpValve_Open();
	/* Infinite loop */
	for(;;)
	{
		uint8_t _retVal = E_NOK ;
		xSemaphoreTake(ADC_BufferHandle, portMAX_DELAY);
		switch (_CurrentADCOperationState) {
		case CalibrationState:
			/* Close the Inspiratory Valve */

			/* Perform Calibration process for PRESSURE and FLOW Sensor */
			_retVal = Calibration_Process();

			if(_retVal == E_PENDING)
			{

			}
			else if(_retVal == E_OK)
			{
				_CurrentADCOperationState = ConversationState ;
			}
			else
			{
				/*Ideally Should not Reach this State */
			}

			break;
		case ConversationState:
			//float _AVG_Pressure,_AVG_CirusO2Sensor,_AVG_O2FlowFeedback,CirusO2Sensor_Val, O2FlowFeedback_Val ;
			_AVG_Pressure = getAvgMilliVot(_ADC_CHANNEL0, _ADC_SampleCount) ;
			_AVG_Flow = CONVERT_ADS115_ADCtoMilliVot(ADS1115_raw) ;

			//_AVG_CirusO2Sensor = getAvg(_ADC_CHANNEL1, _ADC_SampleCount) ;
			_AVG_O2FlowFeedback = getAvg(_ADC_CHANNEL2, _ADC_SampleCount) ;

			//_AVG_Pressure = MOVING_AVERAGE_PRESSURE(_AVG_Pressure);
			//_AVG_Flow = MOVING_AVERAGE_FLOW(_AVG_Flow);

			_Pressure_Val = _CONVERT_PRESStoCMH2O ((float)_AVG_Pressure,_Pressure_Offset) ;
			_Pressure_Val = MOVING_AVERAGE_PRESSURE(_Pressure_Val);
			_Differential_Pressure = _CONVERT_KPAtoBar(_AVG_Flow,_Flow_Offset) ;
			_Flow_Val = _CONVERT_dPAtoFlow(_Differential_Pressure) ;
			_Flow_Val = MOVING_AVERAGE_FLOW(_Flow_Val);
			_Volume_Val = _Flow_Val /60 ; //mL
			_Total_Volume=_Total_Volume+_Volume_Val;

			_ADC_SampleCount = 0 ;

			break;
		default:
			break;
		}
		/* Conversion TIME needed for Sensor 1ms */
		vTaskDelay(SENSOR_CONVERSION_TIME);
	}
  /* USER CODE END ADC_Conversion_Task */
}

/* USER CODE BEGIN Header_Set_DAC_Value_Task */
/**
 * @brief Function implementing the Set_DAC_Value thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Set_DAC_Value_Task */
void Set_DAC_Value_Task(void *argument)
{
  /* USER CODE BEGIN Set_DAC_Value_Task */
	TickType_t xDelay = 100 ; // portTICK_PERIOD_MS
	/* Infinite loop */
	for(;;)
	{
		if(_DAC_Val < 4096 )
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_Val) ;
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, _DAC_Val) ;
			_DAC_Val++;
		}
		else
			_DAC_Val = 0 ;
		/*if(O2_DAC>4095)
			O2_DAC=1850;*/

		vTaskDelay( xDelay );
	}
  /* USER CODE END Set_DAC_Value_Task */
}

/* USER CODE BEGIN Header_CMVPC_Mode_Task */
/**
 * @brief Function implementing the CMVPC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CMVPC_Mode_Task */
void CMVPC_Mode_Task(void *argument)
{
  /* USER CODE BEGIN CMVPC_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
#if DEBUG_GPIO == ENABLE
		HAL_GPIO_TogglePin(GPIOE, MODE_FLAG_Pin);
#endif

		switch (_CurrentComputationState) {
		case Compute_I_Wave:
			_Pressure_Base=1;
			_Flow_Base=0;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();

			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_PEEP_Val)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}
			break;
		case NoComputeState:
			break;
		default:
			break;

		}
	}
  /* USER CODE END CMVPC_Mode_Task */
}

/* USER CODE BEGIN Header_CMVVC_Mode_Task */
/**
 * @brief Function implementing the CMVVC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CMVVC_Mode_Task */
void CMVVC_Mode_Task(void *argument)
{
  /* USER CODE BEGIN CMVVC_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:
			_Pressure_Base=0;
			_Flow_Base=1;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();
			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_PEEP_Val)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}

			break;
		case NoComputeState:
			break;
		default:
			break;

		}
	}
  /* USER CODE END CMVVC_Mode_Task */
}

/* USER CODE BEGIN Header_SIMVPC_Mode_Task */
/**
 * @brief Function implementing the SIMVPC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SIMVPC_Mode_Task */
void SIMVPC_Mode_Task(void *argument)
{
  /* USER CODE BEGIN SIMVPC_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:
			_Pressure_Base=1;
			_Flow_Base=0;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();

			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_PEEP_Val)
				{

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}


			if(_E_TIMER <= (_TRIG_WINDOW) &&  _E_TIMER != 0)
			{

				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+_TRIG_LMT))||(_Pressure_Val<(simv_trigger_offset-_TRIG_LMT)))
					{
						Switch_TASK_I_CYCLE();
					}
				}
				else
				{

					if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
					{
						Switch_TASK_I_CYCLE();
					}
				}
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==0)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}

			break;

		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END SIMVPC_Mode_Task */
}

/* USER CODE BEGIN Header_SIMVVC_Mode_Task */
/**
 * @brief Function implementing the SIMVVC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SIMVVC_Mode_Task */
void SIMVVC_Mode_Task(void *argument)
{
  /* USER CODE BEGIN SIMVVC_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:

			_Pressure_Base=0;
			_Flow_Base=1;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();
			break;
		case Compute_E_Wave:

			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_PEEP_Val)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}
			if(_E_TIMER <= (_TRIG_WINDOW) &&  _E_TIMER != 0)
			{

				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+_TRIG_LMT))||(_Pressure_Val<(simv_trigger_offset-_TRIG_LMT)))
					{
						Switch_TASK_I_CYCLE();
					}
				}
				else
				{

					if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
					{
						Switch_TASK_I_CYCLE();
					}
				}
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==0)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}


			break;
		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END SIMVVC_Mode_Task */
}

/* USER CODE BEGIN Header_PS_V_Mode_Task */
/**
 * @brief Function implementing the PS_V thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PS_V_Mode_Task */
void PS_V_Mode_Task(void *argument)
{
  /* USER CODE BEGIN PS_V_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END PS_V_Mode_Task */
}

/* USER CODE BEGIN Header_cPAP_V_Mode_Task */
/**
 * @brief Function implementing the cPAP_V thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_cPAP_V_Mode_Task */
void cPAP_V_Mode_Task(void *argument)
{
  /* USER CODE BEGIN cPAP_V_Mode_Task */
	/* Infinite loop */
	for (;;) {
		switch (_CurrentComputationState) {

		case Compute_E_Wave:

			///////////////////////////////////////////////////
			if (_cPAP_Trigger == pdFALSE) {

				///////////////////////////
				ExpValve_Close();
				BLOWER_ON();
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,_DAC_VAL0);



				///////////////////////////
				//									if(_TRIG_TYPE==1)
				//									{
				//									if((_Pressure_Val>(simv_trigger_offset+_TRIG_LMT))||(_Pressure_Val<(simv_trigger_offset-_TRIG_LMT)))
				//									{
				//										_APNEA_COUNTER = _APNEA_TIME * 1000 ;
				//													_cPAP_Trigger = FALSE ;
				//									}
				//								    }
				//									else
				//{
				if (_APNEA_COUNTER == 0) {
					/* NO Trigger arrived during Apnea time switch to I time */
					//_CurrentBackupMode = PCCMV_BACKUP;
					_E_TIMER = E_Wave ;
					if(_CurrentBackupMode == PCCMV_BACKUP )
					{

						CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_APNEA);
						 apnea_mode=0;
						_Set_Peep=_PEEP_Val/2;
						_cPAP_Trigger = pdTRUE;
						vTaskResume(PCCMV_BackupHandle);
						vTaskSuspend(VCCMV_BackupHandle);
						vTaskSuspend(cPAP_VHandle);
					}
					else if(_CurrentBackupMode == VCCMV_BACKUP)
					//else if(0)
					{
						CLEAR_ALERT_BIT(SECOND_FRAME_UN,_ALERT_APNEA);
						apnea_mode=0;
						_Set_Peep=_PEEP_Val/2;
						_cPAP_Trigger = pdTRUE;
						vTaskResume(VCCMV_BackupHandle);
						vTaskSuspend(PCCMV_BackupHandle);
						vTaskSuspend(cPAP_VHandle);
					}
					else if(_CurrentBackupMode == IdleState)
					{

						apnea_mode=1;
						vTaskSuspend(PCCMV_BackupHandle);
						vTaskSuspend(VCCMV_BackupHandle);
					}


				}
				if((_Pressure_Val>(_CPAP_Val+3))||(_Pressure_Val<(_CPAP_Val-3)))
				{
					 apnea_mode=1;
					_APNEA_COUNTER = _APNEA_TIME * 1000;
					_cPAP_Trigger = pdFALSE;
					SET_ALERT_BIT(SECOND_FRAME_UN,_ALERT_APNEA);


				}
				/*	if ((_Flow_Val > (_TRIG_LMT + 50))
						|| (_Flow_Val < ((_TRIG_LMT + 50) * -1))) {
					_APNEA_COUNTER = _APNEA_TIME * 1000;
					_cPAP_Trigger = pdFALSE;
				}
				 */

				//}
				//////////////////////////
			}
#if 0
			else {
				if (_PEEP_REACHED_FLAG == pdTRUE) {
					//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
					//if(_Pressure_Val<_PEEP_Val)
					if (_Pressure_Val < _PEEP_Val) {
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
								_DAC_VAL0);
					} else {
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
								0);
						if(_Flow_Val==0)
						{
							_Wait_Flow_offset=0;
						}
						if(_Wait_Flow_offset==0)
						{
							if ((_Flow_Val > (_TRIG_LMT + 50))
									|| (_Flow_Val < ((_TRIG_LMT + 50) * -1))) {
								_APNEA_COUNTER = _APNEA_TIME * 1000;
								_cPAP_Trigger = pdFALSE;
								_CurrentWaveFormState = Generate_E_Wave;
							}
						}
					}
				}
				if (_Pressure_Val < _Set_Peep) {
					_PEEP_REACHED_FLAG = pdTRUE;
					ExpValve_Close();
				} else if (_PEEP_REACHED_FLAG == pdFALSE) {
					ExpValve_Open();
				}


			}
#endif
			//////////////////////////////////
			/*else if (Trigger Reach Condition)
			 {
			 // Reload Counter
			 _APNEA_COUNTER = _APNEA_TIME * 1000 ;
			 _cPAP_Trigger = FALSE ;
			 }
			 */

			break;
		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END cPAP_V_Mode_Task */
}

/* USER CODE BEGIN Header_BiPAP_V_Mode_Task */
/**
 * @brief Function implementing the BiPAP_V thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BiPAP_V_Mode_Task */
void BiPAP_V_Mode_Task(void *argument)
{
  /* USER CODE BEGIN BiPAP_V_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();

			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_EPAP_Val)
				{

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep_BIPAP_APR_VC)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}


			if(_E_TIMER <= (_BIPAP_TRIG_WINDOW) &&  _E_TIMER != 0 && simv_trigger_offset != 0)
			{

				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+3))||(_Pressure_Val<(simv_trigger_offset-3)))
					{
						Switch_TASK_E_CYCLE();
					_APNEA_COUNTER = _APNEA_TIME * 1000;
					}
				}
				else
				{

				/*	if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
					{
						Switch_TASK_E_CYCLE();
					}*/
				}
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==0)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}
/////////////////////////
			if (_APNEA_COUNTER == 0) {
									/* NO Trigger arrived during Apnea time switch to I time */
									//_CurrentBackupMode = PCCMV_BACKUP;
									_E_TIMER = E_Wave ;
									if(_CurrentBackupMode == PCCMV_BACKUP )
									{
										_Mode_Val=9;
										_cPAP_Trigger = pdTRUE;
										vTaskResume(PCCMV_BackupHandle);
										vTaskSuspend(VCCMV_BackupHandle);
										vTaskSuspend(BiPAP_VHandle);
									}
									else if(_CurrentBackupMode == VCCMV_BACKUP)
									{
										_Mode_Val=10;
										_cPAP_Trigger = pdTRUE;
										vTaskResume(VCCMV_BackupHandle);
										vTaskSuspend(PCCMV_BackupHandle);
										vTaskSuspend(BiPAP_VHandle);
									}
									else if(_CurrentBackupMode == IdleState)
									{
										vTaskSuspend(PCCMV_BackupHandle);
										vTaskSuspend(VCCMV_BackupHandle);
									}
								}


			///////////////
			break;

		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END BiPAP_V_Mode_Task */
}

/* USER CODE BEGIN Header_APR_VC_Mode_Task */
/**
 * @brief Function implementing the APR_VC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APR_VC_Mode_Task */
void APR_VC_Mode_Task(void *argument)
{
  /* USER CODE BEGIN APR_VC_Mode_Task */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();

			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				//_DAC_VAL0=controlsys_Update(&cs, _PEEP_Val , _Pressure_Val);
				//if(_Pressure_Val<_PEEP_Val)
				if(_Pressure_Val<_EPAP_Val)
				{

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep_BIPAP_APR_VC)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}


			if(_E_TIMER <= (_BIPAP_TRIG_WINDOW) &&  _E_TIMER != 0 && simv_trigger_offset != 0)
			{

				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+3))||(_Pressure_Val<(simv_trigger_offset-3)))
					{
					//	Switch_TASK_E_CYCLE();
					_APNEA_COUNTER = _APNEA_TIME * 1000;
					}
				}
				else
				{

				/*	if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
					{
						Switch_TASK_E_CYCLE();
					}*/
				}
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==0)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}
/////////////////////////
			if (_APNEA_COUNTER == 0) {
									/* NO Trigger arrived during Apnea time switch to I time */
									//_CurrentBackupMode = PCCMV_BACKUP;
									_E_TIMER = E_Wave ;
									if(_CurrentBackupMode == PCCMV_BACKUP )
									{
										_Mode_Val=9;
										_cPAP_Trigger = pdTRUE;
										vTaskResume(PCCMV_BackupHandle);
										vTaskSuspend(VCCMV_BackupHandle);
										vTaskSuspend(APR_VCHandle);
									}
									else if(_CurrentBackupMode == VCCMV_BACKUP)
									{
										_Mode_Val=10;
										_cPAP_Trigger = pdTRUE;
										vTaskResume(VCCMV_BackupHandle);
										vTaskSuspend(PCCMV_BackupHandle);
										vTaskSuspend(APR_VCHandle);
									}
									else if(_CurrentBackupMode == IdleState)
									{
										vTaskSuspend(PCCMV_BackupHandle);
										vTaskSuspend(VCCMV_BackupHandle);
									}
								}


			///////////////
			break;

		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END APR_VC_Mode_Task */
}

/* USER CODE BEGIN Header_Debug_Task */
/**
 * @brief Function implementing the Debug thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Debug_Task */
void Debug_Task(void *argument)
{
  /* USER CODE BEGIN Debug_Task */
	TickType_t xDelay = 20 ; // portTICK_PERIOD_MS
	/* Infinite loop */
	for(;;)
	{
		_CYCLIC_TRANSMIT_PKT._header          = 0x5052 ;
		_CYCLIC_TRANSMIT_PKT._length          = sizeof(_CYCLIC_TRANSMIT_PKT)-4 ;
		_CYCLIC_TRANSMIT_PKT._Pressure_Val    = _Pressure_Val ;
		//_CYCLIC_TRANSMIT_PKT._Flow_Val        = _Flow_Val ;
		_CYCLIC_TRANSMIT_PKT._Flow_Val        = -100 ;
		_CYCLIC_TRANSMIT_PKT._Volume_Val      = -101 ;
		//_CYCLIC_TRANSMIT_PKT._Volume_Val      = _Volume_Val ;
		_CYCLIC_TRANSMIT_PKT._Control_Byte    = 0 ;
		_CYCLIC_TRANSMIT_PKT._SPO2            = 0 ;
		_CYCLIC_TRANSMIT_PKT._Heart_BPM       = 0 ;
		_CYCLIC_TRANSMIT_PKT._CRC8            = chksum8(&_CYCLIC_TRANSMIT_PKT._Pressure_Val,_CYCLIC_TRANSMIT_PKT._length); ;

		CDC_Transmit_FS((uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT)) ;

		HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));

		vTaskDelay( xDelay );
	}
  /* USER CODE END Debug_Task */
}

/* USER CODE BEGIN Header_UART_CMD_Handler_Task */
/**
 * @brief Function implementing the UART_CMD_Handle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_CMD_Handler_Task */
void UART_CMD_Handler_Task(void *argument)
{
  /* USER CODE BEGIN UART_CMD_Handler_Task */
	xSemaphoreTake(UART_RX_BufferHandle, portMAX_DELAY);
	HAL_UART_Receive_DMA(&huart3,(uint8_t*)UART_RX_BUF,sizeof(UART_RX_BUF));
	uint8_t _RX_CRC8 = 0 ;
	uint32_t _TX_LENGTH = 0 ;
	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(UART_RX_BufferHandle, portMAX_DELAY);

#if DEBUG_GPIO == ENABLE
		HAL_GPIO_WritePin(Wave_GPIO_Port, RX_NOTIFY_Pin, GPIO_PIN_SET);
#endif

		_RX_CRC8 = chksum8(&UART_RX_BUF[3],((((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_length)));
		if(_RX_CRC8 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_CRC8)
		{
#if CYCLIC_TRANSMIT == DISABLE
			if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header) && (0x01 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
			{
				_CYCLIC_TRANSMIT_PKT._header          = 0x5052 ;
				_CYCLIC_TRANSMIT_PKT._length          = sizeof(_CYCLIC_TRANSMIT_PKT)-4 ;
				_CYCLIC_TRANSMIT_PKT._Pressure_Val    = _Pressure_Val ;
				_CYCLIC_TRANSMIT_PKT._Flow_Val        = _Flow_Val ;
				_CYCLIC_TRANSMIT_PKT._Volume_Val      = _Volume_Val ;
				_CYCLIC_TRANSMIT_PKT._CRC8            = chksum8(&_CYCLIC_TRANSMIT_PKT._Pressure_Val,_CYCLIC_TRANSMIT_PKT._length); ;

				CDC_Transmit_FS((uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT)) ;

				HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_CYCLIC_TRANSMIT_PKT,sizeof(_CYCLIC_TRANSMIT_PKT));

			}
			else if((0x5052 == ((SET_PARAM_CMD_PACKET*                                                                                                                                                                                                                                            ) (UART_RX_BUF))->_header) && (0x02 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
#else
				if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header) && (0x02 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
#endif
				{
					CDC_Transmit_FS((uint8_t*)&_MONITORING_TRANSMIT_PKT,sizeof(_MONITORING_TRANSMIT_PKT)-1) ;
					HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_MONITORING_TRANSMIT_PKT,sizeof(_MONITORING_TRANSMIT_PKT));
				}
				else if((0x5052 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header) && (0x03 == ((DATA_REQUEST_PACKET*) (UART_RX_BUF))->_data))
				{
					if(_SENSOR_DATA_BANK_CCM._HISTORY_AVAILBLE == 1 )
					{
						_TX_LENGTH = (I_Wave+E_Wave)*3 ;

						if(_SENSOR_DATA_BANK_CCM._ACTIVE_BANK == 1)
						{
							CDC_Transmit_FS((uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK2,_TX_LENGTH) ;
							HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK2,_TX_LENGTH) ;
						}
						else
						{
							CDC_Transmit_FS((uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK1,_TX_LENGTH) ;
							HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_SENSOR_DATA_BANK_CCM._HISTORY_BANK1,_TX_LENGTH) ;
						}
					}
				}
				else if((0x5053 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					COMMAND_HANDLER((SET_PARAM_CMD_PACKET*) (UART_RX_BUF));
				}
				else if((0x5054 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					ALERT_COMMAND_HANDLER((ALERT_RANGE_PACKET*) (UART_RX_BUF));
				}
				else if((0x5055 == ((SET_PARAM_CMD_PACKET*) (UART_RX_BUF))->_header))
				{
					SERVICE_COMMAND_HANDLER((REQUEST_SERVICE_PACKET_tst*) (UART_RX_BUF));
				//	vTaskSuspend(TransmitTaskHandle);
				}
				else
				{
					_DROPPED_PACKET_COUNT++;
				}
		}
		else
		{
			_DROPPED_PACKET_COUNT++;
		}

		HAL_UART_Receive_DMA(&huart3,(uint8_t*)UART_RX_BUF,sizeof(UART_RX_BUF));
	}
  /* USER CODE END UART_CMD_Handler_Task */
}

/* USER CODE BEGIN Header_oneMilliSecondTask */
/**
 * @brief Function implementing the oneMilliSecond thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oneMilliSecondTask */
void oneMilliSecondTask(void *argument)
{
  /* USER CODE BEGIN oneMilliSecondTask */
	TickType_t xDelay = 1 ;
	/* Infinite loop */
	for(;;)
	{
		if(_PSV_IGNORE>0)
		{
			_PSV_IGNORE-- ;
		}
		if(_E_TIMER>0)
		{
			_E_TIMER-- ;
			_E_TIMER_ACHEIVED++;

		}
		if(_I_TIMER>0)
		{
			_I_TIMER-- ;
			_I_TIMER_ACHEIVED++;

		}
		if(_APNEA_COUNTER > 0)
		{
			_APNEA_COUNTER-- ;
		}
		if(apnea_mode==1)
		{
			CHECK_O2_APNEA_COUNTER++;
		}


		vTaskDelay( xDelay );
	}
  /* USER CODE END oneMilliSecondTask */
}

/* USER CODE BEGIN Header_ControlSystemTask */
/**
 * @brief Function implementing the ControlSystem thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ControlSystemTask */
void ControlSystemTask(void *argument)
{
  /* USER CODE BEGIN ControlSystemTask */
	TickType_t xDelay = 2 ; //2 ms
	/* Infinite loop */
	for(;;)
	{
		if(_CurrentMode == VCCMV)
		{
			if(_CurrentComputationState == Compute_I_Wave){

				if(_Total_Volume<_VT_Val)
				{
					_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);

				}
				else
				{
					_DAC_VAL0=0;
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 1850);
				}

			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{
				if(_PEEP_REACHED_FLAG==pdTRUE)
				{
					//if(_Pressure_Val<_PEEP_Val)
					//	{
					_Peep_Avg+=_Pressure_Val;
					_Peep_Avg_count++;
					_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
					//}
					//else
					//	{
					//	_DAC_VAL0=0;
					//}
				}
				else
				{
					_DAC_VAL0=0;
				}
			}
		}
		if(_CurrentMode == PCCMV)
		{
			if(_CurrentComputationState == Compute_I_Wave){
				_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);



				if(_I_TIMER<200)
				{
					_Pip_Avg+=_Pressure_Val;
					_Pip_Avg_count++;

					_Vol_Avg+=_Total_Volume;
					_Vol_Avg_count++;
				}

			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{
				if(_PEEP_REACHED_FLAG==pdTRUE)
				{
					//if(_Pressure_Val<_PEEP_Val)
					//	{
					_Peep_Avg+=_Pressure_Val;
					_Peep_Avg_count++;
					_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
					//}
					//else
					//	{
					//	_DAC_VAL0=0;
					//}
				}
				else
				{
					_DAC_VAL0=0;
				}
			}
		}
		if(_CurrentMode == SIMVPC)
		{
			if(_CurrentComputationState == Compute_I_Wave){
				_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);

			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{
				if(_PEEP_REACHED_FLAG==pdTRUE)
				{
					//if(_Pressure_Val<_PEEP_Val)
					//	{
					_Peep_Avg+=_Pressure_Val;
					_Peep_Avg_count++;
					_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
					//}
					//else
					//	{
					//	_DAC_VAL0=0;
					//}
				}
				else
				{
					_DAC_VAL0=0;
				}
			}
		}

		if(_CurrentMode == SIMVVC)
		{
			if(_CurrentComputationState == Compute_I_Wave){
				if(_Total_Volume<_VT_Val)
				{
					_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);
				}
				else
				{
					_DAC_VAL0=0;
				}
			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{
				if(_PEEP_REACHED_FLAG==pdTRUE)
				{
					//if(_Pressure_Val<_PEEP_Val)
					//	{
					_Peep_Avg+=_Pressure_Val;
					_Peep_Avg_count++;
					_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
					//}
					//else
					//	{
					//	_DAC_VAL0=0;
					//}
				}
				else
				{
					_DAC_VAL0=0;
				}
			}
		}
		//////////////////
		if(_CurrentMode == cPAP)
		{
			if(_CurrentComputationState == Compute_I_Wave){
				if(_CurrentBackupMode == PCCMV_BACKUP)
				{
					_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);
				}
				else if(_CurrentBackupMode == VCCMV_BACKUP)
				{
					if(_Total_Volume<_VT_Val)
					{
						_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);
					}
					else
					{
						_DAC_VAL0=0;
					}
				}

			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{

				if(_cPAP_Trigger == pdFALSE)
				{

					_DAC_VAL0=controlsys_Update(&cs_PIP, _CPAP_Val , _Pressure_Val);
				}
				else
				{
					if(_PEEP_REACHED_FLAG==pdTRUE)
					{
						//if(_Pressure_Val<_PEEP_Val)
						//	{
						_Peep_Avg+=_Pressure_Val;
						_Peep_Avg_count++;
						_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
						//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
						//}
						//else
						//	{
						//	_DAC_VAL0=0;
						//}
					}
					else
					{
						_DAC_VAL0=0;
					}
				}
			}}
		//////////////////
		//////////////////
				if(_CurrentMode == PSV)
				{
					if(_CurrentComputationState == Compute_I_Wave){
						if(_cPAP_Trigger == pdFALSE)
												{
						if(_CurrentBackupMode == PCCMV_BACKUP)
						{
							_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);
						}
						else if(_CurrentBackupMode == VCCMV_BACKUP)
						{
							if(_Total_Volume<_VT_Val)
							{
								_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);
							}
							else
							{
								_DAC_VAL0=0;
							}
						}
												}
						else
						{
							_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);
						}

					}
					else if(_CurrentComputationState == Compute_E_Wave)
					{

						if(_cPAP_Trigger == pdFALSE)
						{

							_DAC_VAL0=controlsys_Update(&cs_PIP, _CPAP_Val , _Pressure_Val);
						}
						else
						{
							if(_PEEP_REACHED_FLAG==pdTRUE)
							{
								//if(_Pressure_Val<_PEEP_Val)
								//	{
								_Peep_Avg+=_Pressure_Val;
								_Peep_Avg_count++;
								_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
								//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
								//}
								//else
								//	{
								//	_DAC_VAL0=0;
								//}
							}
							else
							{
								_DAC_VAL0=0;
							}
						}
					}}
				//////////////////
				if(_CurrentMode == BiPAP)
				{
					if(_CurrentComputationState == Compute_I_Wave){
						if(_APNEA_COUNTER == 0)
																	{
											if(_CurrentBackupMode == PCCMV_BACKUP)
											{
												_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);
											}
											else if(_CurrentBackupMode == VCCMV_BACKUP)
											{
												if(_Total_Volume<_VT_Val)
												{
													_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);
												}
												else
												{
													_DAC_VAL0=0;
												}
											}
																	}
											else
											{
												_DAC_VAL0=controlsys_Update(&cs_PIP , _IPAP_Val , _Pressure_Val);
											}
					}
					else if(_CurrentComputationState == Compute_E_Wave)
					{
						if(_APNEA_COUNTER > 0)
								{

									_DAC_VAL0=controlsys_Update(&cs_PIP, _EPAP_Val , _Pressure_Val);
								}
								else
								{
									if(_PEEP_REACHED_FLAG==pdTRUE)
									{
										//if(_Pressure_Val<_PEEP_Val)
										//	{
										_Peep_Avg+=_Pressure_Val;
										_Peep_Avg_count++;
										_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
										//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
										//}
										//else
										//	{
										//	_DAC_VAL0=0;
										//}
									}
									else
									{
										_DAC_VAL0=0;
									}
								}

					}
				}
				////////////////////////////
				//////////////////
							if(_CurrentMode == APR_VC)
							{
								if(_CurrentComputationState == Compute_I_Wave){
									if(_APNEA_COUNTER == 0)
																				{
														if(_CurrentBackupMode == PCCMV_BACKUP)
														{
															_DAC_VAL0=controlsys_Update(&cs_PIP , _PIP_Val , _Pressure_Val);
														}
														else if(_CurrentBackupMode == VCCMV_BACKUP)
														{
															if(_Total_Volume<_VT_Val)
															{
																_DAC_VAL0=controlsys_Update(&cs_flow, _Flow_Rate , _Flow_Val);
															}
															else
															{
																_DAC_VAL0=0;
															}
														}
																				}
														else
														{
															_DAC_VAL0=controlsys_Update(&cs_PIP , _IPAP_Val , _Pressure_Val);
														}
								}
								else if(_CurrentComputationState == Compute_E_Wave)
								{
									if(_APNEA_COUNTER > 0)
											{

												_DAC_VAL0=controlsys_Update(&cs_PIP, _EPAP_Val , _Pressure_Val);
											}
											else
											{
												if(_PEEP_REACHED_FLAG==pdTRUE)
												{
													//if(_Pressure_Val<_PEEP_Val)
													//	{
													_Peep_Avg+=_Pressure_Val;
													_Peep_Avg_count++;
													_DAC_VAL0=controlsys_Update(&cs_PEEP, _PEEP_Val , _Pressure_Val);
													//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
													//}
													//else
													//	{
													//	_DAC_VAL0=0;
													//}
												}
												else
												{
													_DAC_VAL0=0;
												}
											}

								}
							}
							////////////////////////////

		vTaskDelay(xDelay);
	}
  /* USER CODE END ControlSystemTask */
}

/* USER CODE BEGIN Header_PSV_ModeTask */
/**
 * @brief Function implementing the PSV_Mode thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PSV_ModeTask */
void PSV_ModeTask(void *argument)
{
  /* USER CODE BEGIN PSV_ModeTask */
	/* Infinite loop */
	for(;;)
	{
		for (;;) {
			switch (_CurrentComputationState) {
			case Compute_I_Wave:
				_PEEP_REACHED_FLAG=pdFALSE;
						BLOWER_ON();
						if(_I_TIMER>0)
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
						ExpValve_Close();
						_PSV_IGNORE=1000;
			break;
			case Compute_E_Wave:
				if (_cPAP_Trigger == pdFALSE) {
					//ExpValve_Close();
					BLOWER_ON();
					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,_DAC_VAL0);
					/////////////////////////////////////
					if(_PEEP_REACHED_FLAG==pdTRUE)
							{
								if(_Pressure_Val<_CPAP_Val)
								{
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
								}
								else
								{
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
								}
							}
							if(_Pressure_Val<3) //_Set_Peep
							{
								_PEEP_REACHED_FLAG=pdTRUE;
								ExpValve_Close();
							}
							else if(_PEEP_REACHED_FLAG==pdFALSE)
							{
								ExpValve_Open();
							}
					/////////////////////////////////////

					if(_Pressure_Val==_CPAP_Val)
					{
						_PSV_REACH =pdTRUE;
					}
					if(_PSV_IGNORE==0)
					{
					if(_PSV_REACH==pdTRUE)
					{
					if((_Pressure_Val>(_CPAP_Val+3))||(_Pressure_Val<(_CPAP_Val-3)))
									{
						_APNEA_COUNTER = _APNEA_TIME * 1000;
																_cPAP_Trigger = pdFALSE;
																_PSV_REACH =pdFALSE;
																_Set_Peep=_PEEP_Val/2;
										Switch_TASK_I_CYCLE();


									}
					}
					}
					if (_APNEA_COUNTER == 0) {
						/* NO Trigger arrived during Apnea time switch to I time */
						//_CurrentBackupMode = PCCMV_BACKUP;
						_E_TIMER = E_Wave ;
						if(_CurrentBackupMode == PCCMV_BACKUP )
						{
							_Set_Peep=_PEEP_Val/2;
							_cPAP_Trigger = pdTRUE;
							vTaskResume(PCCMV_BackupHandle);
							vTaskSuspend(VCCMV_BackupHandle);
							vTaskSuspend(PSV_ModeHandle);
						}
						else if(_CurrentBackupMode == VCCMV_BACKUP)
						{
							_Set_Peep=_PEEP_Val/2;
							_cPAP_Trigger = pdTRUE;
							vTaskResume(VCCMV_BackupHandle);
							vTaskSuspend(PCCMV_BackupHandle);
							vTaskSuspend(PSV_ModeHandle);
						}
						else if(_CurrentBackupMode == IdleState)
						{
							vTaskSuspend(PCCMV_BackupHandle);
							vTaskSuspend(VCCMV_BackupHandle);
						}
					}

				}


				break;
			case NoComputeState:
				break;
			default:
				break;
			}
		}
	}
  /* USER CODE END PSV_ModeTask */
}

/* USER CODE BEGIN Header_PCCMV_BackupTask */
/**
 * @brief Function implementing the PCCMV_Backup thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PCCMV_BackupTask */
void PCCMV_BackupTask(void *argument)
{
  /* USER CODE BEGIN PCCMV_BackupTask */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:
			_Pressure_Base=1;
			_Flow_Base=0;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();

			break;
		case Compute_E_Wave:
			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				if(_Pressure_Val<_PEEP_Val)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}
			//if(_E_TIMER <= (_TRIG_TIME*100) &&  _E_TIMER != 0)
			//	_TRIG_PER = (((float)(E_Wave -_E_TIMER)/E_Wave)*100);

			if(_E_TIMER <= (_TRIG_WINDOW) &&  _E_TIMER != 0)
			{
				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+_TRIG_LMT))||(_Pressure_Val<(simv_trigger_offset-_TRIG_LMT)))
					{
						controlsys_Init(&cs_PIP);
						controlsys_Init(&cs_PEEP);
						controlsys_Init(&cs_flow);
						_CurrentWaveFormState = Generate_E_Wave ;
						_CurrentComputationState = Compute_E_Wave ;
						_PSV_IGNORE=1000;
						_APNEA_COUNTER = _APNEA_TIME * 1000;
						_PSV_REACH =pdFALSE;
						_cPAP_Trigger = pdFAIL;
						if(_CurrentMode == cPAP)
						vTaskResume(cPAP_VHandle);

						if(_CurrentMode == PSV)
						vTaskResume(PSV_ModeHandle);

						if(_CurrentMode == BiPAP)
						{
						vTaskResume(BiPAP_VHandle);
						_Mode_Val=8;
						}
						if (_CurrentMode == APR_VC) {
							vTaskResume(APR_VCHandle);
							_Mode_Val = 5;
						}
						//ExpValve_Open();
						_PEEP_REACHED_FLAG=pdFALSE;
						vTaskSuspend(PCCMV_BackupHandle);
						vTaskSuspend(VCCMV_BackupHandle);

					}
				}
				else
				{

					if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
					{
						_cPAP_Trigger = pdFAIL;
						vTaskResume(cPAP_VHandle);
						vTaskSuspend(PCCMV_BackupHandle);
						vTaskSuspend(VCCMV_BackupHandle);
					}
				}
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==0)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}

			break;

		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END PCCMV_BackupTask */
}

/* USER CODE BEGIN Header_VCCMV_BackupTask */
/**
 * @brief Function implementing the VCCMV_Backup thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_VCCMV_BackupTask */
void VCCMV_BackupTask(void *argument)
{
  /* USER CODE BEGIN VCCMV_BackupTask */
	/* Infinite loop */
	for(;;)
	{
		switch (_CurrentComputationState) {

		case Compute_I_Wave:

			_Pressure_Base=0;
			_Flow_Base=1;
			_PEEP_REACHED_FLAG=pdFALSE;
			BLOWER_ON();
			//_DAC_VAL0=controlsys_Update(&cs, _PIP_Val , _Pressure_Val);
			if(_I_TIMER>0)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0);
			ExpValve_Close();
			break;
		case Compute_E_Wave:

			if(_PEEP_REACHED_FLAG==pdTRUE)
			{
				if(_Pressure_Val<_PEEP_Val)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, _DAC_VAL0) ;
				}
				else
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
				}
			}
			if(_Pressure_Val<_Set_Peep)
			{
				_PEEP_REACHED_FLAG=pdTRUE;
				ExpValve_Close();
			}
			else if(_PEEP_REACHED_FLAG==pdFALSE)
			{
				ExpValve_Open();
			}
			//if(_E_TIMER <= (_TRIG_TIME*100) &&  _E_TIMER != 0)
			//_TRIG_PER=(((float)(E_Wave -_E_TIMER)/E_Wave)*100);
			if(_E_TIMER <= (_TRIG_WINDOW) &&  _E_TIMER != 0)
			{
				if(_TRIG_TYPE==1)
				{
					if((_Pressure_Val>(simv_trigger_offset+_TRIG_LMT))||(_Pressure_Val<(simv_trigger_offset-_TRIG_LMT)))
					{
						_APNEA_COUNTER = _APNEA_TIME * 1000;
						_PSV_REACH =pdFALSE;
						_cPAP_Trigger = pdFAIL;
						if (_CurrentMode == cPAP)
							vTaskResume(cPAP_VHandle);

						if (_CurrentMode == PSV)
							vTaskResume(PSV_ModeHandle);

						if(_CurrentMode == BiPAP)
						{
						vTaskResume(BiPAP_VHandle);
						_Mode_Val=8;
						}
						if (_CurrentMode == APR_VC) {
												vTaskResume(APR_VCHandle);
												_Mode_Val = 5;
						}
						vTaskSuspend(PCCMV_BackupHandle);
						vTaskSuspend(VCCMV_BackupHandle);

					}
				}
				/*else
						{

							if((_Flow_Val>(_TRIG_LMT))||(_Flow_Val<(_TRIG_LMT*-1)))
							{
								_cPAP_Trigger = pdFAIL;
								vTaskResume(cPAP_VHandle);
								vTaskSuspend(PCCMV_BackupHandle);
								vTaskSuspend(VCCMV_BackupHandle);
							}
						}*/
			}
			else
			{
				if(_TRIG_TYPE==1)
				{
					if(_Flow_Val==1)
					{

						simv_trigger_offset=_Pressure_Val;
					}
				}
			}


			break;
		case NoComputeState:
			break;
		default:
			break;
		}
	}
  /* USER CODE END VCCMV_BackupTask */
}

/* USER CODE BEGIN Header_OxyTask */
/**
* @brief Function implementing the OxygenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OxyTask */
void OxyTask(void *argument)
{
  /* USER CODE BEGIN OxyTask */
  /* Infinite loop */
  for(;;)
  {
	  if(_CurrentMode == PCCMV || VCCMV || SIMVPC || SIMVVC)
	  {
	  		if(_CurrentComputationState == Compute_I_Wave)
	  		{

	  			O2_read=1;
	            /*****   DEFAULT DAC2 VALUE & SERVO POSITION    *********/
	  			if(FiO2_old!=_PIO2_Val)
	  			{
	  				if(_PIO2_Val<=30&&_PIO2_Val>=20)
	  				{
	  					    k=(int)(0.7*1000)/20;
	  						//TIM12->CCR1=k;
	  						TIM12->CCR1=60;
	  						O2_DAC=1900;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=40&&_PIO2_Val>30)
	  				{
	  					    k=(int)(0.7*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=60;
	  						O2_DAC=1950;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=50&&_PIO2_Val>=40)
	  				{
	  					    k=(int)(0.7*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=60;
	  						O2_DAC=2100;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=60&&_PIO2_Val>50)
	  				{
	  					    k=(int)(0.7*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=75;
	  						O2_DAC=2150;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=70&&_PIO2_Val>60)
	  				{
	  					    k=(int)(0.9*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=75;
	  						O2_DAC=2200;
	  						FiO2_old=_PIO2_Val;
	  				}

	  				else if(_PIO2_Val<=80&&_PIO2_Val>70)
	  				{
	  					    k=(int)(0.9*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=80;
	  						O2_DAC=2240;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=90&&_PIO2_Val>80)
	  				{
	  					    k=(int)(0.9*1000)/20;
	  						//TIM12->CCR1=k;
	  					    TIM12->CCR1=80;
	  						O2_DAC=2300;
	  						FiO2_old=_PIO2_Val;
	  				}
	  				else if(_PIO2_Val<=100&&_PIO2_Val>90)
	  				{
	  					     k=(int)(1.5*1000)/20;
	  						 //TIM12->CCR1=k;
	  					     TIM12->CCR1=90;
	  						 O2_DAC=2650;
	  						 FiO2_old=_PIO2_Val;
	  				}



	  			}


               /*****  DAC2 VALUE CHANGE WITH RUN TIME     *******/

	  			if(O2_process==1)
		        {

	  				if(_PIO2_Val>21)
	  				{
	  					if(O2_percentage<_PIO2_Val)
	  					{
	  						if(fio2count>=3)
	  						{
	  							O2_DAC=O2_DAC+20;
	  							fio2count=0;
	  						}
	  						HAL_DAC_SetValue( & hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, O2_DAC);
	  						O2_process=0;
	  					}
	  					else if(O2_percentage>_PIO2_Val)
	  					{
	  						if(fio2count>=3)
	  						{
	  							O2_DAC=O2_DAC-20;
	  							fio2count=0;

	  						}
	  						HAL_DAC_SetValue( & hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, O2_DAC);
	  						O2_process=0;

	  					}
	  					else
	  					{
	  						HAL_DAC_SetValue( & hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, O2_DAC);
	  						O2_process=0;
	  						fio2count=0;
	  					}
	  				}

	  				if(_PIO2_Val==21)
	  				{
	  					O2_DAC=1860;
	  					HAL_DAC_SetValue( & hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, O2_DAC);
	  					O2_process=0;
	  					TIM12->CCR1=40;
	  					if(fio2count>=3)
	  					{
	  						fio2count=0;

	  					}
	  					}

	  				}


 /***********   WHEN PIP ACHEIVED FIO2 VALUE TO BE ZERO   **************/
	  			if(_Pressure_Base==1)
	  			{
	  				if(_Pressure_Val>=(_PIP_Val))
	  				{
	  					  	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 1850);

	  				}
	  			}
	  			if(_Flow_Base==1)
	  			{
	  				if(_Total_Volume>_VT_Val)
				    {

					    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 1850);
				    }
	  			}


	  			}
	  		else if(_CurrentComputationState == Compute_E_Wave)
	  		{
	  			O2_process=1;													 //Reset a O2 process
	  			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 1850);	 //close the proportional valve
	  			if(O2_read==1)
	  			{
	  				if(_E_TIMER<100)
	  				{
	  					fio2count2=0;
	  					_AVG_CirusO2Sensor=(ADC_RESULT_BUF[1]*3300)/4095;
	  					O2_read=0;
	  				}
	  			}

	  			O2_typecast=(int)_AVG_CirusO2Sensor;
	  		    cirus_volt=O2_typecast-150;
	  		    cirus_volt2=cirus_volt;
	  			cirus_volt_new=cirus_volt2;
	  			cirus_volt_new=((cirus_volt_new-cirus_volt_old)/10.00)+cirus_volt_old;
	  		    cirus_volt_old=cirus_volt_new;
	  		    O2_percentage_80=(cirus_volt_new*80)/640;
	  			O2_percentage=((int)O2_percentage_80)+21;



	  			/*if(_CurrentMode == PSV)
	  			{
	  				if(_APNEA_COUNTER>0)
	  				{
	  					 TIM12->CCR1=60;
	  					 O2_DAC=1900;
	  					 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, O2_DAC);
	  				}
	  			}*/

	  		}

	  }







    osDelay(1);
  }
  /* USER CODE END OxyTask */
}

/* USER CODE BEGIN Header_Mode_Analysis_Task */
/**
* @brief Function implementing the Mode_Analysis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mode_Analysis_Task */
void Mode_Analysis_Task(void *argument)
{
  /* USER CODE BEGIN Mode_Analysis_Task */
  /* Infinite loop */
  for(;;)
  {
	  /*************  for vti calculation     **********/

		if(_CurrentMode == PCCMV || VCCMV)
		{
			if(_CurrentComputationState == Compute_I_Wave)
			{

				/****************  for pip  ***************/
				if(_Pip_Avg_val >_RANGE_PIP_MIN_Val&&_Pip_Avg_val<_RANGE_PIP_MAX_Val)
				{
					CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_YN);
				}
				else
				{
					SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_YN);
					if(_RANGE_PIP_MIN_Val>_Pressure_Val)
					{

						CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_HL);

					}
					else if(_RANGE_PIP_MAX_Val<_Pressure_Val)
					{
						SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_HL);
					}

				}

				/****************  for minite volume  ***************/

								if(_max_volume_val >_RANGE_MINT_VOL_MIN_Val&&_max_volume_val<_RANGE_MINT_VOL_MAX_Val)
								{
									CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_MINT_VOLUME_YN);
								}
								else
								{
									SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_MINT_VOLUME_YN);

									RR=(((SET_PARAM_CMD_PACKET*) ((UART_RX_BUF)))->_RR);

									_max_volume_val_ml=((_max_volume_val*RR)/10);
									if(_RANGE_MINT_VOL_MIN_Val>_max_volume_val)
									{

										CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_MINT_VOLUME_HL);

									}
									else if(_RANGE_MINT_VOL_MAX_Val<_max_volume_val)
									{
										SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_MINT_VOLUME_HL);
									}

								}


			  /****************  for tidal volume  ***************/

				               if(_max_Tidal_volume_val >_RANGE_VT_MIN_Val&&_max_Tidal_volume_val<_RANGE_VT_MAX_Val)
				               {
				               		CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_YN);
				               }
				               	else
				               {
				               		SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_YN);


				               		if(_RANGE_VT_MIN_Val>_max_Tidal_volume_val)
				               		{

				               				CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);

				               		}
				               		else if(_RANGE_VT_MAX_Val<_max_Tidal_volume_val)
				               		{
				               					SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);
				               		}

				               	}


		  /****************  for Respiratory rate  ***************/
				       Acheived_RR=(60000/(RR_E_TIME+RR_I_TIME));


				      if(Acheived_RR >_RANGE_RR_MIN_Val&&Acheived_RR<_RANGE_RR_MAX_Val)
				      {
				          CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_YN);
				      }
				      else
				      {
				           SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_YN);


				           if(_RANGE_RR_MIN_Val>Acheived_RR)
				           {

				               CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);

				            }
				            else if(_RANGE_RR_MAX_Val<Acheived_RR)
				            {
				               	SET_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);
				             }

				        }



			}
			else if(_CurrentComputationState == Compute_E_Wave)
			{

				CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_HL);
				CLEAR_ALERT_BIT(FIRST_FRAME_UN,_ALERT_MINT_VOLUME_HL);
				CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_VT_HL);
				CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_T_RR_HL);

			}
		}

    osDelay(1);
  }
  /* USER CODE END Mode_Analysis_Task */
}

/* USER CODE BEGIN Header_SERVICE_Communication */
/**
* @brief Function implementing the service_ble thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Communication */
void SERVICE_Communication(void *argument)
{
  /* USER CODE BEGIN SERVICE_Communication */
  /* Infinite loop */
  for(;;)
  {

	  	_RESPOND_SERVICE_PACKET._REPORT0 = 0x80 ;
	  	_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	osDelay(1);
	  	SEND_REPORT_PACKET();
	  	osDelay(1);
	  	vTaskSuspend(service_bleHandle);



  }
  /* USER CODE END SERVICE_Communication */
}

/* USER CODE BEGIN Header_SERVICE_Blower */
/**
* @brief Function implementing the service_bldc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Blower */
void SERVICE_Blower(void *argument)
{
  /* USER CODE BEGIN SERVICE_Blower */
  /* Infinite loop */
  for(;;)
  {



	  	  	BLOWER_ON();
	  	  	ExpValve_Open();
	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 4095) ;


	  	  	osDelay(1000);

	  	  	if(_Flow_Val>40)
	  	  	{
	  	  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x40 ;
	  	  		  	  	_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  		SEND_REPORT_PACKET();
	  	  	}
	  	  	else
	  	  	{
	  	  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  		SEND_REPORT_PACKET();
	  	  	}

	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	  	   // BLOWER_OFF();
	  	  	osDelay(1);
	  	  	vTaskSuspend(service_bldcHandle);


    osDelay(1);
  }
  /* USER CODE END SERVICE_Blower */
}

/* USER CODE BEGIN Header_SERVICE_ExpValve */
/**
* @brief Function implementing the service_valve thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_ExpValve */
void SERVICE_ExpValve(void *argument)
{
  /* USER CODE BEGIN SERVICE_ExpValve */
	float temp_Pressure_Val1;
	float temp_Pressure_Val2;
	float total_temp_Pressure_Val;
  /* Infinite loop */
  for(;;)
  {



	 	  	  	BLOWER_ON();
	 	  	  	ExpValve_Close();
	 	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 2500) ;


	 	  	  	osDelay(2000);
	 	    	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	 	    	osDelay(1000);

	 	  	    temp_Pressure_Val1=_Pressure_Val;


	 	    	osDelay(1000);
	 	    	temp_Pressure_Val2=_Pressure_Val;

	 	    	total_temp_Pressure_Val=temp_Pressure_Val1-temp_Pressure_Val2;

	 	  	  	if(total_temp_Pressure_Val<=1)
	 	  	  	{
	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x32 ;
	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	 	  	  		SEND_REPORT_PACKET();
	 	  	  	}
	 	  	  	else
	 	  	  	{
	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	 	  	  		SEND_REPORT_PACKET();
	 	  	  	}

	 	  	    ExpValve_Open();
	 	  	    BLOWER_OFF();
	 	  	  	osDelay(1);
	 	  	  	vTaskSuspend(service_valveHandle);



    osDelay(1);
  }
  /* USER CODE END SERVICE_ExpValve */
}

/* USER CODE BEGIN Header_SERVICE_Proximal */
/**
* @brief Function implementing the service_proxima thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Proximal */
void SERVICE_Proximal(void *argument)
{
  /* USER CODE BEGIN SERVICE_Proximal */
	    float temp_Pressure_Val1;
		float temp_Pressure_Val2;
		float total_temp_Pressure_Val;
  /* Infinite loop */
  for(;;)
  {
	  	  	  	  	BLOWER_ON();
	  	 	  	  	ExpValve_Close();
	  	 	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 2500) ;


	  	 	  	  	osDelay(2000);
	  	 	    	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	  	 	    	osDelay(1000);

	  	 	  	    temp_Pressure_Val1=_Pressure_Val;


	  	 	    	osDelay(1000);
	  	 	    	temp_Pressure_Val2=_Pressure_Val;

	  	 	    	total_temp_Pressure_Val=temp_Pressure_Val1-temp_Pressure_Val2;

	  	 	  	  	if(total_temp_Pressure_Val<=1)
	  	 	  	  	{
	  	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x08 ;
	  	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	 	  	  		SEND_REPORT_PACKET();
	  	 	  	  	}
	  	 	  	  	else
	  	 	  	  	{
	  	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	 	  	  		SEND_REPORT_PACKET();
	  	 	  	  	}

	  	 	  	    ExpValve_Open();
	  	 	  	    BLOWER_OFF();
	  	 	  	  	osDelay(1);
	  	 	  	  	vTaskSuspend(service_proximaHandle);

  }
  /* USER CODE END SERVICE_Proximal */
}

/* USER CODE BEGIN Header_SERVICE_Leak */
/**
* @brief Function implementing the service_leak thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Leak */
void SERVICE_Leak(void *argument)
{
  /* USER CODE BEGIN SERVICE_Leak */
	        float temp_Pressure_Val1;
			float temp_Pressure_Val2;
			float total_temp_Pressure_Val;
  /* Infinite loop */
  for(;;)
  {
	  	  	  	  	    BLOWER_ON();
	  	  	 	  	  	ExpValve_Close();
	  	  	 	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 2500) ;


	  	  	 	  	  	osDelay(2000);
	  	  	 	    	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	  	  	 	    	osDelay(1000);

	  	  	 	  	    temp_Pressure_Val1=_Pressure_Val;


	  	  	 	    	osDelay(1000);
	  	  	 	    	temp_Pressure_Val2=_Pressure_Val;

	  	  	 	    	total_temp_Pressure_Val=temp_Pressure_Val1-temp_Pressure_Val2;

	  	  	 	  	  	if(total_temp_Pressure_Val<=1)
	  	  	 	  	  	{
	  	  	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x02 ;
	  	  	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  	 	  	  		SEND_REPORT_PACKET();
	  	  	 	  	  	}
	  	  	 	  	  	else
	  	  	 	  	  	{
	  	  	 	  	    	_RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  	 	  	  		_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  	 	  	  		SEND_REPORT_PACKET();
	  	  	 	  	  	}

	  	  	 	  	    ExpValve_Open();
	  	  	 	  	    BLOWER_OFF();
	  	  	 	  	  	osDelay(1);
	  	  	 	  	  	vTaskSuspend(service_leakHandle);
  }
  /* USER CODE END SERVICE_Leak */
}

/* USER CODE BEGIN Header_SERVICE_Battery */
/**
* @brief Function implementing the service_battery thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Battery */
void SERVICE_Battery(void *argument)
{
  /* USER CODE BEGIN SERVICE_Battery */
  /* Infinite loop */
  for(;;)
  {
	  	  BLOWER_ON();
	  	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 4095) ;
	  	  osDelay(2000);
	  	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	  	  osDelay(2000);
	  	  BLOWER_OFF();

	  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x20 ;
	  	  SEND_REPORT_PACKET();

	  	  vTaskSuspend(service_batteryHandle);
  }
  /* USER CODE END SERVICE_Battery */
}

/* USER CODE BEGIN Header_SERVICE_Led */
/**
* @brief Function implementing the service_led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Led */
void SERVICE_Led(void *argument)
{
  /* USER CODE BEGIN SERVICE_Led */
  /* Infinite loop */
  for(;;)
  {
	      /* HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
	       osDelay(2000);
	       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);
	       osDelay(2000);
	       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 1);
	       osDelay(2000);
	               HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
	       	       osDelay(100);
	       	       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 0);
	       	       osDelay(210);
	       	       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 0);
	       	       osDelay(100);*/

	  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x80 ;
	  	  SEND_REPORT_PACKET();

	  	  vTaskSuspend(service_ledHandle);
  }
  /* USER CODE END SERVICE_Led */
}

/* USER CODE BEGIN Header_SERVICE_ADS1115 */
/**
* @brief Function implementing the service_ads1115 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_ADS1115 */
void SERVICE_ADS1115(void *argument)
{
  /* USER CODE BEGIN SERVICE_ADS1115 */\
  //int I2C_Module=2;
  //unsigned char ADSwrite[3];
  float I2C_check;

  /* Infinite loop */
  for(;;)
  {
	    /*I2C_Module=2;

	    ADSwrite[0] = 0x01;
	    ADSwrite[1] = 0x42;
	    ADSwrite[2] = 0xE3;


	  I2C_Module=(HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS<<1, ADSwrite, 3, 10)!=HAL_OK);

	  if((!(I2C_Module))==1)
	  {

	  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x10 ;
	  	  SEND_REPORT_PACKET();
	  }
	  else
	  {
		    _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
		    _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
		 	 SEND_REPORT_PACKET();

	  }*/
	    I2C_check= _AVG_Flow;

	      if(I2C_check>2400)
		  {

		  	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
		  	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x10 ;
		  	  SEND_REPORT_PACKET();
		  }
		  else
		  {
			    _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
			    _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
			 	 SEND_REPORT_PACKET();

		  }
	  	  vTaskSuspend(service_ads1115Handle);
  }
  /* USER CODE END SERVICE_ADS1115 */
}

/* USER CODE BEGIN Header_SERVICE_Servo */
/**
* @brief Function implementing the service_servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Servo */
void SERVICE_Servo(void *argument)
{
  /* USER CODE BEGIN SERVICE_Servo */
	float pressure_check;
  /* Infinite loop */
  for(;;)
  {
	  	  ExpValve_Close();
	  	  TIM12->CCR1=90;
	      BLOWER_ON();
	  	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 4095) ;
	  	  osDelay(5000);
	  	  pressure_check=_Pressure_Val;
	  	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;
	  	  osDelay(500);
	  	  BLOWER_OFF();

	  	  if(pressure_check>3 && pressure_check<10)
	  	  {
	  		  _RESPOND_SERVICE_PACKET._REPORT0 = 0x04 ;
	  		  _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  }

	  	  else
	      {
	  		   _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  		   _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  }
	  	  SEND_REPORT_PACKET();

	      TIM12->CCR1=45;
	  	  vTaskSuspend(service_servoHandle);

  }
  /* USER CODE END SERVICE_Servo */
}

/* USER CODE BEGIN Header_SERVICE_Sensor */
/**
* @brief Function implementing the service_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Sensor */
void SERVICE_Sensor(void *argument)
{
  /* USER CODE BEGIN SERVICE_Sensor */
  /* Infinite loop */
  for(;;)
  {

	            BLOWER_ON();
	  	  	  	ExpValve_Open();
	  	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 4095) ;


	  	  	  	osDelay(2000);

	  	  	  	if(_Flow_Val>100)
	  	  	  	{
	  	  	  	        _RESPOND_SERVICE_PACKET._REPORT0 = 0x10 ;
	  	  	  		  	_RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  	  		    SEND_REPORT_PACKET();
	  	  	  	}
	  	  	  	else
	  	  	  	{
	  	  	  	      _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	  	  	  		  _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  	  	  		  SEND_REPORT_PACKET();
	  	  	  	}

	  	  	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) ;

	  	  	  	osDelay(1);
	  	  	  	vTaskSuspend(service_bldcHandle);



	  /*_RESPOND_SERVICE_PACKET._REPORT0 = 0x10 ;
	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  SEND_REPORT_PACKET();*/

	  vTaskSuspend(service_sensorHandle);
	  osDelay(1);
  }
  /* USER CODE END SERVICE_Sensor */
}

/* USER CODE BEGIN Header_SERVICE_Nebuliser */
/**
* @brief Function implementing the service_nebulis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Nebuliser */
void SERVICE_Nebuliser(void *argument)
{
  /* USER CODE BEGIN SERVICE_Nebuliser */
  /* Infinite loop */
  for(;;)
  {
	          /* HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
	  	       osDelay(2000);
	  	       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);

	  	               HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
	  	       	       osDelay(100);*/

	  _RESPOND_SERVICE_PACKET._REPORT0 = 0x01 ;
	  _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	  SEND_REPORT_PACKET();

	  vTaskSuspend(service_nebulisHandle);






    osDelay(1);
  }
  /* USER CODE END SERVICE_Nebuliser */
}

/* USER CODE BEGIN Header_SERVICE_Oxygen */
/**
* @brief Function implementing the service_oxygen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERVICE_Oxygen */
void SERVICE_Oxygen(void *argument)
{
  /* USER CODE BEGIN SERVICE_Oxygen */
	float temp_AVG_O2FlowFeedback;
  /* Infinite loop */
  for(;;)
  {

	       HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 2500) ;
	       osDelay(2000);

	       _AVG_O2FlowFeedback=(ADC_RESULT_BUF[2]*3300)/4095;

	       temp_AVG_O2FlowFeedback=_AVG_O2FlowFeedback;
	       HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 1800) ;
	       osDelay(2000);
	       if(temp_AVG_O2FlowFeedback>3100)
	       {

	    	   _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	    	   _RESPOND_SERVICE_PACKET._REPORT1 = 0x40 ;
	    	   SEND_REPORT_PACKET();
	       }
	       else
	       {
	    	     _RESPOND_SERVICE_PACKET._REPORT0 = 0x00 ;
	    	  	 _RESPOND_SERVICE_PACKET._REPORT1 = 0x00 ;
	    	  	  SEND_REPORT_PACKET();

	       }

	  	  vTaskSuspend(service_oxygenHandle);
  }
  /* USER CODE END SERVICE_Oxygen */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( ADC_BufferHandle, &xHigherPriorityTaskWoken );
	_ADC_CHANNEL0[_ADC_SampleCount] = ADC_RESULT_BUF[0];
	_ADC_CHANNEL1[_ADC_SampleCount] = ADC_RESULT_BUF[1];
	_ADC_CHANNEL2[_ADC_SampleCount] = ADC_RESULT_BUF[2];
	_ADC_SampleCount++;
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(UART_RX_BufferHandle, &xHigherPriorityTaskWoken);
	/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
	 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
	 then calling portEND_SWITCHING_ISR() will request a context switch. If
	 xHigherPriorityTaskWoken is still pdFALSE then calling
	 portEND_SWITCHING_ISR() will have no effect */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	TransmitFlag = 0 ;
	HAL_UART_DMA_Tx_Stop(&huart3);

}

void Switch_TASK_I_CYCLE(void)
{

	vTaskDelete(PulseGeneratorHandle);
	_I_TIMER = 0 ;
	_E_TIMER = 0 ;
	_CurrentWaveFormState = Generate_I_Wave ;
	_CurrentComputationState = NoComputeState ;
	/* creation of PulseGenerator */
	PulseGeneratorHandle = osThreadNew(PulseGeneratorTask, NULL, &PulseGenerator_attributes);
	//vTaskResume(PulseGeneratorHandle);
}


void Switch_TASK_E_CYCLE(void)
{

	vTaskDelete(PulseGeneratorHandle);
	_I_TIMER = 0 ;
	_E_TIMER = 0 ;
	_CurrentWaveFormState = Generate_E_Wave ;
	_CurrentComputationState = NoComputeState ;
	/* creation of PulseGenerator */
	PulseGeneratorHandle = osThreadNew(PulseGeneratorTask, NULL, &PulseGenerator_attributes);
	//vTaskResume(PulseGeneratorHandle);
}


void COMMAND_HANDLER(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET)
{
	uint32_t CycleTime = 0 , I_Time = 0 , E_Time = 0 ,RT_Value = 0 ;


	_Mode_Val = 0x0F & (RX_PARAM_CMD_PACKET->_mode) ;

	if((_Mode_Val>0&&_Mode_Val<=4)||(_Mode_Val==9)||(_Mode_Val==10))
	{
		if(_Mode_Val==9)
		{
			_CurrentBackupMode = PCCMV_BACKUP;
		}
		else if(_Mode_Val==10)
		{
			_CurrentBackupMode = VCCMV_BACKUP;
		}
		CycleTime = 60000/RX_PARAM_CMD_PACKET->_RR ;

		I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E)>>4;
		E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
		RT_Value =(0xF0 & RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME)>>4;
		_PIP_Val = RX_PARAM_CMD_PACKET->_PIP;
		_PEEP_Val = RX_PARAM_CMD_PACKET->_PEEP;
		_VT_Val = RX_PARAM_CMD_PACKET->_VTI;
		_Reserved_Mode = (0xF0 & RX_PARAM_CMD_PACKET->_mode)>>4 ;
		_PIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
		_Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;
		_APNEA_TIME = RX_PARAM_CMD_PACKET->_APNEA;
		//_APNEA_TIME = (0xF0 & RX_PARAM_CMD_PACKET->_APNEA_TRIG_TIME)>>4;
		_TRIG_TIME = 0x0F & (RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME);

		_TRIG_TYPE = (0xF0 & RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT)>>4;
		_TRIG_LMT = 0x0F & (RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT);
		_T_HIGH = (0xF0 & RX_PARAM_CMD_PACKET->_T_HIGH_LOW)>>4;
		_T_LOW = 0x0F & (RX_PARAM_CMD_PACKET->_T_HIGH_LOW);

		_Set_Peep=_PEEP_Val/2;
	}
	if(_Mode_Val==7||_Mode_Val==6||_Mode_Val==8||_Mode_Val==5)
	{
		_CPAP_Val= RX_PARAM_CMD_PACKET->_PEEP;
		CycleTime = 60000/RX_PARAM_CMD_PACKET->_RR ;
		I_Time = (0xF0 & RX_PARAM_CMD_PACKET->_I_E)>>4;
		E_Time = 0x0F & (RX_PARAM_CMD_PACKET->_I_E);
		RT_Value =(0xF0 & RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME)>>4;
		_IPAP_Val = RX_PARAM_CMD_PACKET->_PIP;
		_EPAP_Val = RX_PARAM_CMD_PACKET->_PEEP;
		//_VT_Val = RX_PARAM_CMD_PACKET->_VTI;
		_Reserved_Mode = (0xF0 & RX_PARAM_CMD_PACKET->_mode)>>4 ;
		_PIO2_Val = RX_PARAM_CMD_PACKET->_FIO2;
		_Flow_Rate=RX_PARAM_CMD_PACKET->_FlowRate;
		_APNEA_TIME = RX_PARAM_CMD_PACKET->_APNEA;
		//_APNEA_TIME = (0xF0 & RX_PARAM_CMD_PACKET->_APNEA_TRIG_TIME)>>4;
		_TRIG_TIME = 0x0F & (RX_PARAM_CMD_PACKET->_RiseTime_TRIG_TIME);

		_TRIG_TYPE = (0xF0 & RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT)>>4;
		_TRIG_LMT = 0x0F & (RX_PARAM_CMD_PACKET->_TRIG_TYPE_TRIG_LMT);
		_T_HIGH = (0xF0 & RX_PARAM_CMD_PACKET->_T_HIGH_LOW)>>4;
		_T_LOW = 0x0F & (RX_PARAM_CMD_PACKET->_T_HIGH_LOW);
		_cPAP_Trigger = pdFALSE;
		//_Set_Peep=_PEEP_Val;
		_CurrentWaveFormState = Generate_E_Wave ;
		_CurrentComputationState = Compute_E_Wave ;
		_Set_Peep=_EPAP_Val;
		_Set_Peep_BIPAP_APR_VC=_EPAP_Val;
	}

	_APNEA_COUNTER = (1000*_APNEA_TIME) ;

	if(_CurrentWaveFormState == NoWaveFormState)
	{
		/*Calculation for the Pulse Width */
		I_Wave = (I_Time * (CycleTime/(I_Time+E_Time))) ;
		E_Wave = (E_Time * (CycleTime/(I_Time+E_Time))) ;
		RT_Value = RT_Value  ;
		RTI_Wave = I_Wave - RT_Value ;

		_RequestedMode = _Mode_Val ;
		RTI_Wave = I_Wave ;

		if(_RequestedMode == cPAP || _RequestedMode == PSV )
		{
			_CurrentWaveFormState = Generate_E_Wave ;
		}
		else
		{
			_CurrentWaveFormState = Generate_I_Wave ;
		}

		_MODE_DATA_SWITCH_FLAG = pdTRUE ;
	}
	else
	{
		/*Calculation for the Pulse Width */
		_Req_I_Wave = (I_Time * (CycleTime/(I_Time+E_Time))) ;
		_Req_E_Wave = (E_Time * (CycleTime/(I_Time+E_Time))) ;
		RT_Value = RT_Value ;
		_Req_RTI_Wave = _Req_I_Wave - RT_Value ;

		_RequestedMode = _Mode_Val ;
		_MODE_DATA_SWITCH_FLAG = pdTRUE ;
	}

	_CALC_TRIG_VAL = ((float)E_TIME_TOLERANCE/100.00)*(E_Wave) ;

	_TOLERANCE_EWAVE = E_Wave - _CALC_TRIG_VAL ;

	_TRIG_WINDOW = _TOLERANCE_EWAVE * (((float)_TRIG_TIME*10.00)/100.00) ;
	//////////BIPAP//////////

_BIPAP_CALC_TRIG_VAL = ((float)BIPAP_E_TIME_TOLERANCE/100.00)*(_T_LOW*1000) ;

_BIPAP_TOLERANCE_EWAVE = (_T_LOW*1000) - _BIPAP_CALC_TRIG_VAL ;

_BIPAP_TRIG_WINDOW = _BIPAP_TOLERANCE_EWAVE * (((float)80)/100.00) ;

	//////////////////
/*if(_Mode_Val==8)
{
	_CALC_TRIG_VAL = ((float)E_TIME_TOLERANCE/100.00)*(_T_HIGH*1000) ;

	_TOLERANCE_EWAVE = (_T_HIGH*1000) - _CALC_TRIG_VAL ;

	_TRIG_WINDOW = _TOLERANCE_EWAVE * (((float)_TRIG_TIME*10.00)/100.00) ;

}
*/
}


void ALERT_COMMAND_HANDLER(ALERT_RANGE_PACKET *RX_ALERT_RANGE_PACKET)
{

	/* Do Processing of ALERT COMMAND*/
	_RANGE_MODE_Val = RX_ALERT_RANGE_PACKET->_RANGE_MODE;
	_RANGE_VT_MIN_Val= (RX_ALERT_RANGE_PACKET->_RANGE_VT_MIN)*10;
	_RANGE_VT_MAX_Val= (RX_ALERT_RANGE_PACKET->_RANGE_VT_MAX)*10;
    _RANGE_PIP_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_PIP_MIN;
	_RANGE_PIP_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_PIP_MAX;
	_RANGE_RR_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_RR_MIN;
	_RANGE_RR_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_RR_MAX;
	_RANGE_MINT_VOL_MIN_Val=(RX_ALERT_RANGE_PACKET->_RANGE_MINT_VOL_MIN);
	_RANGE_MINT_VOL_MAX_Val=(RX_ALERT_RANGE_PACKET->_RANGE_MINT_VOL_MAX);
	_RANGE_SPO2_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_SPO2_MIN;
	_RANGE_SPO2_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_SPO2_MAX;
	_RANGE_PULSE_MIN_Val=RX_ALERT_RANGE_PACKET->_RANGE_PULSE_MIN;
	_RANGE_PULSE_MAX_Val=RX_ALERT_RANGE_PACKET->_RANGE_PULSE_MAX;

}

void SERVICE_COMMAND_HANDLER(REQUEST_SERVICE_PACKET_tst *RX_SERVICE_RANGE_PACKET)
{

	/* Do Processing of SERVICE COMMAND*/
	_SERVICE_DATA0_Val = RX_SERVICE_RANGE_PACKET->_SERVICE_DATA0;
	_SERVICE_DATA1_Val= (RX_SERVICE_RANGE_PACKET->_SERVICE_DATA1);

	if(_SERVICE_DATA0_Val!=0)
	{
	 switch(_SERVICE_DATA0_Val)
	 {
	 	 case 128:
	 		vTaskResume(service_bleHandle);
		 break;
	 	case 64:
	 		 vTaskResume(service_bldcHandle);
	 	break;
	 	case 32:
	 		 vTaskResume(service_valveHandle);
	 	break;
	 	case 16:
	 		 vTaskResume(service_sensorHandle);
	 	break;
	 	case 8:
	 		 vTaskResume(service_proximaHandle);
	 	break;
	 	case 4:
	 		 vTaskResume(service_servoHandle);
	 	break;

	 	case 2:
	 		 vTaskResume(service_leakHandle);
	 	break;

	 	case 1:
	 		 vTaskResume(service_nebulisHandle);
	 	break;
	 	default:
		break;
	 }


	 }

	if(_SERVICE_DATA1_Val!=0)
	{
		 switch(_SERVICE_DATA1_Val)
		 {
		 	 case 128:
		 		vTaskResume(service_ledHandle);
			 break;
		 	case 64:
		 		vTaskResume(service_oxygenHandle);
		    break;
		 	case 32:
		 		vTaskResume(service_batteryHandle);
		    break;
		 	case 16:
		 		vTaskResume(service_ads1115Handle);
		 	break;
		 	default:
		 	break;
		 }
	}


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart->ErrorCode == HAL_UART_ERROR_ORE) {
		__HAL_UART_FLUSH_DRREGISTER(huart);
		__HAL_UART_CLEAR_OREFLAG(huart);
		_UART_OVER_ERROR++;
		HAL_UART_Receive_DMA(&huart3,(uint8_t*)UART_RX_BUF,sizeof(UART_RX_BUF));
	}
}

HAL_StatusTypeDef HAL_UART_DMA_Tx_Stop(UART_HandleTypeDef *huart)
{
	uint32_t dmarequest = 0x00U;
	dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT);
	if((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
	{
		CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

		/* Abort the UART DMA Tx channel */
		if(huart->hdmatx != NULL)
		{
			HAL_DMA_Abort(huart->hdmatx);
		}
		/* Disable TXEIE and TCIE interrupts */
		CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
		huart->gState = HAL_UART_STATE_READY;

		return HAL_OK;
	}
	else return HAL_ERROR;

}
void SEND_REPORT_PACKET()
{

//#define SET_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 1 )
//#define CLEAR_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 0 )

	//SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_HL);
	//SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_OXY_YN);
	//SET_ALERT_BIT(FOURTH_FRAME_UN,_ALERT_LEAK_YN);
	//CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_PULSET_HL);
	_RESPOND_SERVICE_PACKET._header = 0x5055 ;
	_RESPOND_SERVICE_PACKET._length = 0x08 ;
	_RESPOND_SERVICE_PACKET._CRC8   = chksum8((unsigned char*)&_RESPOND_SERVICE_PACKET._REPORT0,_RESPOND_SERVICE_PACKET._length);
	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_RESPOND_SERVICE_PACKET,sizeof(_RESPOND_SERVICE_PACKET)) ;
	CDC_Transmit_FS((uint8_t*)&_RESPOND_SERVICE_PACKET,sizeof(_RESPOND_SERVICE_PACKET)) ;

}



void SEND_ALERT_PACKET()
{

	//#define SET_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 1 )
	//#define CLEAR_ALERT_BIT(frame,bitname) (_ALERT_RESPONSE_PKT.frame.FRAMEBits.bitname = 0 )

		//SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_HL);
		//SET_ALERT_BIT(FIRST_FRAME_UN,_ALERT_PIP_YN);
		//SET_ALERT_BIT(FOURTH_FRAME_UN,_ALERT_LEAK_YN);
		//CLEAR_ALERT_BIT(THIRD_FRAME_UN,_ALERT_PULSET_HL);
		_ALERT_RESPONSE_PKT._header=0x5054;
		_ALERT_RESPONSE_PKT._length=8;
		_ALERT_RESPONSE_PKT._CRC8   = chksum8((unsigned char*)&_ALERT_RESPONSE_PKT.FIRST_FRAME_UN.FIRST_BYTES,_ALERT_RESPONSE_PKT_length);

		CDC_Transmit_FS((uint8_t*)&_ALERT_RESPONSE_PKT,sizeof(_ALERT_RESPONSE_PKT)) ;

		HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&_ALERT_RESPONSE_PKT,sizeof(_ALERT_RESPONSE_PKT));

}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
