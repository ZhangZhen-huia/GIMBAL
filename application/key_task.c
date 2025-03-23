#include "key_task.h"
#include "remote_control.h"
#include "referee.h"
#include "detect_task.h"
#include "communicate_task.h"

Key_Scan_t Key_ScanValue;
Mouse_Data_t Mouse_Data;
ControlMode_e ControlMode;


static void MouseData_Combine(Mouse_Data_t * Data,ControlMode_e mode);
void Key_Scan(Key_Scan_t * Key,ControlMode_e mode);
void ControlMode_Get(void);
static EnemyColor_e Aimbot_KeyFunc(void);
//uint8_t Key_Function(uint16_t key);

void key_task(void const * argument)
{
	
	while(1)
	{
		
		/*-- 10ms扫描一次按键 --*/
		ControlMode_Get();
		Key_Scan(&Key_ScanValue,ControlMode);
		MouseData_Combine(&Mouse_Data,ControlMode);
		EnemyColor = Aimbot_KeyFunc();

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


void ControlMode_Get(void)
{
	if(toe_is_error(REFEREE_TOE))
		ControlMode = Rc;
	else
		ControlMode = ImageTransfer;
}

void Key_Scan(Key_Scan_t * Key,ControlMode_e mode)
{
	uint16_t Key_Temp,Key_Down,Key_Up;
	static uint16_t Key_Last;
	
	if(mode == Rc)
	Key_Temp = rc_ctrl.key.v;
	else
	Key_Temp = Referee_System.Image_trans_remote.keyboard_value;
	
	Key_Down = Key_Temp & (Key_Last ^ Key_Temp);//异或运算，相同为0，不同为1     
	Key_Up 	= ~Key_Temp & (Key_Last ^ Key_Temp);//异或运算，相同为0，不同为1 
	Key_Last = Key_Temp;
	
	Key->Key_Value_Last = Key->Key_Value;
	
	if(Key_Up == KEY_PRESSED_OFFSET_Q)
	{
		Key->Key_Value.Q = 1;
	}
	else
		Key->Key_Value.Q = 0;

		if(Key_Up == KEY_PRESSED_OFFSET_E)
	{
		Key->Key_Value.E = 1;
	}
	else
		Key->Key_Value.E = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_R)
	{
		Key->Key_Value.r = 1;
	}
	else
		Key->Key_Value.r = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_F)
	{
		Key->Key_Value.F = 1;
	}
	else
		Key_ScanValue.Key_Value.F = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_G)
	{
		Key->Key_Value.G = 1;
	}
	else
		Key->Key_Value.G = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_Z)
	{
		Key->Key_Value.Z = 1;
	}
	else
		Key->Key_Value.Z = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_X)
	{
		Key->Key_Value.X = 1;
	}
	else
		Key->Key_Value.X = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_C)
	{
		Key->Key_Value.C = 1;
	}
	else
		Key->Key_Value.C = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_V)
	{
		Key->Key_Value.V = 1;
	}
	else
		Key->Key_Value.V = 0;
	
		if(Key_Up == KEY_PRESSED_OFFSET_B)
	{
		Key->Key_Value.B = 1;
	}
	else
		Key->Key_Value.B = 0;
	
	if(Key_Up == KEY_PRESSED_OFFSET_SHIFT)
	{
		Key->Key_Value.SHIFT = 1;
	}
	else
		Key->Key_Value.SHIFT = 0;
	
	if(Key_Up == KEY_PRESSED_OFFSET_CTRL)
	{
		Key->Key_Value.CTRL = 1;
	}
	else
		Key->Key_Value.CTRL = 0;
	
//	
//	Key->Key_Value.CTRL_F = Key->Key_Value.CTRL & Key->Key_Value.F;
//	Key->Key_Value.CTRL_B = Key->Key_Value.CTRL & Key->Key_Value.B;
//	Key->Key_Value.Z_F = Key->Key_Value.Z & Key->Key_Value.F;
//	Key->Key_Value.Z_B = Key->Key_Value.Z & Key->Key_Value.B;
	
	
}


static void MouseData_Combine(Mouse_Data_t * Data,ControlMode_e mode)
{
	switch(mode)
	{
		case ImageTransfer:
					Data->mouse_x = Referee_System.Image_trans_remote.mouse_x;
					Data->mouse_y = Referee_System.Image_trans_remote.mouse_y;
					Data->mouse_z = Referee_System.Image_trans_remote.mouse_z;
					Data->mouse_l = Referee_System.Image_trans_remote.left_button_down;
					Data->mouse_r = Referee_System.Image_trans_remote.right_button_down;
					break;
		case Rc:
					Data->mouse_x = rc_ctrl.mouse.x;
					Data->mouse_y = rc_ctrl.mouse.y;
					Data->mouse_z = rc_ctrl.mouse.z;
					Data->mouse_l = rc_ctrl.mouse.press_l;
					Data->mouse_r = rc_ctrl.mouse.press_r;
					break;
		
		default:break;
	}


}

static EnemyColor_e Aimbot_KeyFunc(void)
{
	static  EnemyColor_e color = BLUE;
	if(Key_ScanValue.Key_Value.Z)
	{
		color++;
		color%=2;
	}
	return color;
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

