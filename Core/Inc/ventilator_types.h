/*
 * ventilator_types.h
 *
 *  Created on: Apr 28, 2021
 *      Author: krish
 */

#ifndef INC_VENTILATOR_TYPES_H_
#define INC_VENTILATOR_TYPES_H_



typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _mode;
	uint8_t  _SERVICE_DATA0;
	uint8_t  _SERVICE_DATA1;
	uint16_t NC0 ;
	uint8_t  NC1 ;
	uint8_t  NC2;
	uint8_t  NC3;
	uint8_t  NC4 ;
	uint8_t  NC5;
	uint8_t  NC6;
	uint8_t  NC7;
	uint8_t  NC8;
	uint8_t  _CRC8;

} REQUEST_SERVICE_PACKET_tst;


typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _REPORT0;
	uint8_t  _REPORT1;
	uint8_t FLOW;
	uint8_t PRESSURE;
	uint8_t O2FLOW;
	uint8_t CIRUS;
	uint8_t BATTERY;
	uint8_t NCBYTE;
	uint8_t  _CRC8;

} RESPOND_SERVICE_PACKET_tst;

typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _Response;
} SET_RESPONSE_PACKET ;



typedef enum
{
	Generate_I_Wave = 0 ,
	Generate_E_Wave ,
	Generate_E_1msWaveSet ,
	Generate_E_1msWaveReSet ,
	NoWaveFormState
}WaveFormState;

typedef enum
{
	Compute_I_Wave = 0,
	Compute_E_Wave ,
	NoComputeState
}ComputationState;


typedef enum
{
	PCCMV_BACKUP = 0,
	VCCMV_BACKUP ,
	IdleState
}BackupModes;


typedef enum
{
	CalibrationState = 0,
	ConversationState
}ADCOperationState;


typedef enum
{
	PCCMV = 0x01,
	VCCMV = 0x02,
	SIMVPC = 0x03,
	SIMVVC = 0x04,
	APR_VC = 0x05,
	PSV = 0x06,
	cPAP = 0x07,
	BiPAP = 0x08,
	NoMode = 0xFE,
	DebugMode = 0xFF
}CurrentMode;

typedef enum
{
	DUTY_CYCLE_100 = 0 ,
	DUTY_CYCLE_20,
	NO_DUTY,
} PWM_DUTY_State;


typedef enum
{
	gPcalibrationState = 0,
	delPcalibration
}CalibrationPressureState;


typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _mode;
	uint8_t  _PIP;
	uint8_t  _PEEP;
	uint16_t _VTI ;
	uint8_t  _I_E ;
	uint8_t  _RR;
	uint8_t  _FIO2;
	uint8_t _RiseTime_TRIG_TIME ;
	uint8_t  _FlowRate;
	uint8_t  _APNEA;
	uint8_t  _TRIG_TYPE_TRIG_LMT;
	uint8_t  _T_HIGH_LOW;
	uint8_t  _CRC8;

} SET_PARAM_CMD_PACKET ;



typedef struct __attribute__((packed))
{
	uint16_t _header ;
	uint8_t  _length ;
	uint8_t  _R_PIP;
	uint8_t  _R_PEEP;
	uint16_t _R_VTI;
	uint16_t _R_VTE ;
	uint16_t _R_TI ;
	uint16_t _R_TE ;
	uint8_t  _R_FIO2 ;
	uint16_t _RiseTime ;
	uint8_t  _R_P_PLATEAU ;
	uint8_t  _CRC8;

} SET_MONITORING_PACKET ;

typedef struct  __attribute__((packed))
{
	uint16_t _header ; // 2
	uint8_t  _length ; // 1
	uint8_t  _Pressure_Val; // 1
	int16_t  _Flow_Val; //2
	int16_t _Volume_Val; //2
	uint8_t _Control_Byte; //2
	uint8_t _SPO2; //2
	uint8_t _Heart_BPM;
	uint8_t  _CRC8; // 1
} SET_CYCLIC_PACKET ;


typedef struct  __attribute__((packed))
{
	uint16_t  _header ; // 2
	uint8_t   _length ; // 1
	uint8_t   _data; // 1
	uint8_t   _Reserved0; //1
	uint16_t  _Reserved1; //2
	uint16_t  _Reserved2; //2
	uint16_t  _Reserved3; //2
	uint16_t  _Reserved4; //2
	uint16_t  _Reserved5; //2
	uint8_t   _CRC8; // 1
} DATA_REQUEST_PACKET ;


typedef struct  __attribute__((packed))
{
	uint8_t _PRESSURE_VAL_BUF[6000] ;
	uint8_t _FLOW_VAL_BUF[6000] ;
	uint8_t _VOLUME_VAL_BUF[6000] ;
}SENSOR_DATA_HISTORY ;

typedef struct __attribute__((packed))
{
	SENSOR_DATA_HISTORY _HISTORY_BANK1 ;
	SENSOR_DATA_HISTORY _HISTORY_BANK2 ;
	uint8_t _ACTIVE_BANK ;
	uint8_t _HISTORY_AVAILBLE ;
}SENSOR_DATA_BANK ;


typedef struct  __attribute__((packed))
{
	uint16_t  _header ; // 2
	uint8_t   _length ; // 1
	uint8_t _RANGE_MODE          ;
	uint8_t _RANGE_VT_MIN        ;
	uint8_t _RANGE_VT_MAX        ;
	uint8_t _RANGE_PIP_MIN       ;
	uint8_t _RANGE_PIP_MAX       ;
	uint8_t _RANGE_RR_MIN        ;
	uint8_t _RANGE_RR_MAX        ;
	uint8_t _RANGE_MINT_VOL_MIN  ;
	uint8_t _RANGE_MINT_VOL_MAX  ;
	uint8_t _RANGE_SPO2_MIN      ;
	uint8_t _RANGE_SPO2_MAX      ;
	uint8_t _RANGE_PULSE_MIN     ;
	uint8_t _RANGE_PULSE_MAX     ;
	uint8_t _CRC8;
} ALERT_RANGE_PACKET  ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_FLOW_ALERT_HL:1;
	volatile uint8_t _ALERT_FLOW_ALERT_YN:1;
	volatile uint8_t _ALERT_PIP_HL:1;
	volatile uint8_t _ALERT_PIP_YN:1;
	volatile uint8_t _ALERT_OXY_HL:1;
	volatile uint8_t _ALERT_OXY_YN:1;
	volatile uint8_t _ALERT_MINT_VOLUME_HL:1;
	volatile uint8_t _ALERT_MINT_VOLUME_YN:1;
} FIRST_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _Reserved1:4;
	volatile uint8_t _ALERT_BAT_DRAIN:1;
	volatile uint8_t _ALERT_OXYGEN_SUPPLY:1;
	volatile uint8_t _ALERT_PATIENT_CIRCUIT:1;
	volatile uint8_t _ALERT_APNEA:1;
} SECOND_FRAME ;


typedef struct __attribute__((packed)) {

	volatile uint8_t _ALERT_PULSET_HL:1;
	volatile uint8_t _ALERT_PULSE_YN:1;
	volatile uint8_t _ALERT_SPO2_HL:1;
	volatile uint8_t _ALERT_SPO2_YN:1;
	volatile uint8_t _ALERT_VT_HL:1;
	volatile uint8_t _ALERT_VT_YN:1;
	volatile uint8_t _ALERT_T_RR_HL:1;
	volatile uint8_t _ALERT_T_RR_YN:1;
} THIRD_FRAME ;


typedef struct __attribute__((packed)) {
	volatile uint8_t _Reserved2:4;
	volatile uint8_t _ALERT_LEAK_HL:1;
	volatile uint8_t _ALERT_LEAK_YN:1;
	volatile uint8_t _ALERT_PEEP_HL:1;
	volatile uint8_t _ALERT_PEEP_YN:1;

} FOURTH_FRAME ;

typedef struct __attribute__((packed)) {
	volatile uint8_t _Reserved3:4;
	volatile uint8_t _ALERT_NEBULIZER:1;
	volatile uint8_t _ALERT_POWER:1;
	volatile uint8_t _ALERT_MICRO_SD:1;
	volatile uint8_t _ALERT_BLE:1;
} FIFTH_FRAME ;

typedef struct __attribute__((packed)) {
	uint16_t _header; // 2
    uint8_t  _length; // 1
	union {
	 volatile unsigned char FIRST_BYTES;
	 FIRST_FRAME FRAMEBits ;
	}FIRST_FRAME_UN;

	union {
	 volatile unsigned char SECOND_BYTES;
	 SECOND_FRAME FRAMEBits ;
	}SECOND_FRAME_UN;

	union {
	 volatile unsigned char THIRD_BYTES;
	 THIRD_FRAME FRAMEBits ;
	}THIRD_FRAME_UN;

	union {
	 volatile unsigned char FOURTH_BYTES;
	 FOURTH_FRAME FRAMEBits ;
	}FOURTH_FRAME_UN;

	union {
	 volatile unsigned char FIFTH_BYTES;
	 FIFTH_FRAME FRAMEBits ;
	}FIFTH_FRAME_UN;

	//6th BYTE
	volatile uint8_t _Reserved4;

	//7th BYTE
	volatile uint8_t _Reserved5;

	//8th BYTE
	volatile uint8_t _Reserved6;

	volatile uint8_t _CRC8;

}ALERT_RESPONSE_PACKET;


/* Custom Macros */
#define E_OK      00
#define E_NOK     01
#define E_PENDING 10


#define MAX_DAC_VALUE 4096

#endif /* INC_VENTILATOR_TYPES_H_ */
