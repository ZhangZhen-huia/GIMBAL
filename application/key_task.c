#include "key_task.h"
#include "remote_control.h"



void Key_Scan(void);
Key_Scan_t Key_ScanValue;


//uint8_t Key_Function(uint16_t key);

void key_task(void const * argument)
{
	
	while(1)
	{
		/*-- 10ms扫描一次按键 --*/
		Key_Scan();
		osDelay(10);
    
	}
}

//uint8_t Key_Function(uint16_t key)
//{
//	static uint16_t num = 0;
//	if(key & rc_ctrl.key.v)
//	{
//		osDelay(10);
//		while((key & rc_ctrl.key.v) == 1);
//		return 1;
//	}
//	else return 0;
//}


void Key_Scan(void)
{
	uint16_t Key_Temp,Key_Down,Key_Up;
	static uint16_t Key_Last;
	
	Key_Temp = rc_ctrl.key.v;
	Key_Down = Key_Temp & (Key_Last ^ Key_Temp);//异或运算，相同为0，不同为1     
	Key_Up 	= ~Key_Temp & (Key_Last ^ Key_Temp);//异或运算，相同为0，不同为1 
	Key_Last = Key_Temp;
	
	Key_ScanValue.Key_Value_Last = Key_ScanValue.Key_Value;
	
	if(Key_Up == KEY_PRESSED_OFFSET_Q)
	{
		Key_ScanValue.Key_Value.Q = 1;
	}
	else
		Key_ScanValue.Key_Value.Q = 0;

		if(Key_Up == KEY_PRESSED_OFFSET_E)
	{
		Key_ScanValue.Key_Value.E = 1;
	}
	else
		Key_ScanValue.Key_Value.E = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_R)
	{
		Key_ScanValue.Key_Value.r = 1;
	}
	else
		Key_ScanValue.Key_Value.r = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_F)
	{
		Key_ScanValue.Key_Value.F = 1;
	}
	else
		Key_ScanValue.Key_Value.F = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_G)
	{
		Key_ScanValue.Key_Value.G = 1;
	}
	else
		Key_ScanValue.Key_Value.G = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_Z)
	{
		Key_ScanValue.Key_Value.Z = 1;
	}
	else
		Key_ScanValue.Key_Value.Z = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_X)
	{
		Key_ScanValue.Key_Value.X = 1;
	}
	else
		Key_ScanValue.Key_Value.X = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_C)
	{
		Key_ScanValue.Key_Value.C = 1;
	}
	else
		Key_ScanValue.Key_Value.C = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_V)
	{
		Key_ScanValue.Key_Value.V = 1;
	}
	else
		Key_ScanValue.Key_Value.V = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_B)
	{
		Key_ScanValue.Key_Value.B = 1;
	}
	else
		Key_ScanValue.Key_Value.B = 0;
	
	if(Key_Up == KEY_PRESSED_OFFSET_SHIFT)
	{
		Key_ScanValue.Key_Value.SHIFT = 1;
	}
	else
		Key_ScanValue.Key_Value.SHIFT = 0;
	
	if(Key_Up == KEY_PRESSED_OFFSET_CTRL)
	{
		Key_ScanValue.Key_Value.CTRL = 1;
	}
	else
		Key_ScanValue.Key_Value.CTRL = 0;
	
	
	Key_ScanValue.Key_Value.CTRL_F = Key_ScanValue.Key_Value.CTRL & Key_ScanValue.Key_Value.F;
	Key_ScanValue.Key_Value.CTRL_B = Key_ScanValue.Key_Value.CTRL & Key_ScanValue.Key_Value.B;
	Key_ScanValue.Key_Value.Z_F = Key_ScanValue.Key_Value.Z & Key_ScanValue.Key_Value.F;
	Key_ScanValue.Key_Value.Z_B = Key_ScanValue.Key_Value.Z & Key_ScanValue.Key_Value.B;
	

}

//	Key_ScanValue.Key_Value.Q = Key_Function(KEY_PRESSED_OFFSET_Q);
//	Key_ScanValue.Key_Value.E = Key_Function(KEY_PRESSED_OFFSET_E);
//	Key_ScanValue.Key_Value.r = Key_Function(KEY_PRESSED_OFFSET_R);
//	Key_ScanValue.Key_Value.F = Key_Function(KEY_PRESSED_OFFSET_F);
//	Key_ScanValue.Key_Value.G = Key_Function(KEY_PRESSED_OFFSET_G);
//	Key_ScanValue.Key_Value.Z = Key_Function(KEY_PRESSED_OFFSET_Z);
//	Key_ScanValue.Key_Value.X = Key_Function(KEY_PRESSED_OFFSET_X);
//	Key_ScanValue.Key_Value.C = Key_Function(KEY_PRESSED_OFFSET_C);
//	Key_ScanValue.Key_Value.V = Key_Function(KEY_PRESSED_OFFSET_V);	
//	Key_ScanValue.Key_Value.B = Key_Function(KEY_PRESSED_OFFSET_B);
//	Key_ScanValue.Key_Value.SHIFT = Key_Function(KEY_PRESSED_OFFSET_SHIFT);
//	Key_ScanValue.Key_Value.CTRL = Key_Function(KEY_PRESSED_OFFSET_CTRL);