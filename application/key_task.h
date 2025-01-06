#ifndef __KEY_TASK_H
#define __KEY_TASK_H


#include "main.h"





typedef struct
{
	uint8_t W;
	uint8_t A;
	uint8_t S;
	uint8_t D;
	uint8_t SHIFT;
	uint8_t CTRL;
	uint8_t Q;
	uint8_t E;
	uint8_t r;
	uint8_t F;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t V;
	uint8_t B;
	uint8_t CTRL_F;
	uint8_t CTRL_B;
	uint8_t Z_F;
	uint8_t Z_B;
}Key_Value_t;

typedef struct
{
	Key_Value_t Key_Value;
	Key_Value_t Key_Value_Last;
}Key_Scan_t;




extern Key_Scan_t Key_ScanValue;


#endif
