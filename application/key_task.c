#include "key_task.h"
#include "remote_control.h"



void Key_Scan(void);
Key_Value_t Key_Value;


uint8_t Key_Function(uint16_t key);

void key_task(void const * argument)
{
	
	while(1)
	{
		Key_Scan();
    
	}
}

uint8_t Key_Function(uint16_t key)
{
	static uint16_t num = 0;
	if(key & rc_ctrl.key.v)
	{
		osDelay(10);
		while((key & rc_ctrl.key.v) == 1);
		return 1;
	}
	else return 0;
}


void Key_Scan(void)
{
	Key_Value.Q = Key_Function(KEY_PRESSED_OFFSET_Q);
	Key_Value.E = Key_Function(KEY_PRESSED_OFFSET_E);
	Key_Value.r = Key_Function(KEY_PRESSED_OFFSET_R);
	Key_Value.F = Key_Function(KEY_PRESSED_OFFSET_F);
	Key_Value.G = Key_Function(KEY_PRESSED_OFFSET_G);
	Key_Value.Z = Key_Function(KEY_PRESSED_OFFSET_Z);
	Key_Value.X = Key_Function(KEY_PRESSED_OFFSET_X);
	Key_Value.C = Key_Function(KEY_PRESSED_OFFSET_C);
	Key_Value.V = Key_Function(KEY_PRESSED_OFFSET_V);	
	Key_Value.B = Key_Function(KEY_PRESSED_OFFSET_B);
	Key_Value.SHIFT = Key_Function(KEY_PRESSED_OFFSET_SHIFT);
	Key_Value.CTRL = Key_Function(KEY_PRESSED_OFFSET_CTRL);
	Key_Value.CTRL_F = Key_Value.CTRL & Key_Value.F;
	Key_Value.CTRL_B = Key_Value.CTRL & Key_Value.B;
	Key_Value.Z_F = Key_Value.Z & Key_Value.F;
	Key_Value.Z_B = Key_Value.Z & Key_Value.B;

}

