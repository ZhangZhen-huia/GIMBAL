#include "key_task.h"
#include "remote_control.h"
#include "referee.h"
#include "detect_task.h"
#include "communicate_task.h"
#include "can.h"
#include "Can_receive.h"
#include "queue.h"
#include "freertos.h"


Key_Scan_t Key_ScanValue;
Mouse_Data_t Mouse_Data;
ControlMode_e ControlMode;
ChassisMode_e ChassisMode;
uint32_t ImgTransferKey;

static void Gimbal_Reset(void);

void Key_Scan(Key_Scan_t * Key,ControlMode_e mode);
void ControlMode_Get(void);
static EnemyColor_e Aimbot_KeyFunc(void);
//uint8_t Key_Function(uint16_t key);
uint32_t ImgTransfer_KeyCombine(void);
void key_task(void const * argument)
{
	
	while(1)
	{	
		/*-- 10ms扫描一次按键 --*/
		ControlMode_Get();
		ImgTransferKey = ImgTransfer_KeyCombine();
		Key_Scan(&Key_ScanValue,ControlMode);
//		EnemyColor = Aimbot_KeyFunc();
		Gimbal_Reset();
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
	static uint8_t mode = 0;
	static uint8_t VPress_cnt;
	static uint8_t CtrlPress;
	if(Key_ScanValue.Key_Value.CTRL)
	{
		CtrlPress = 1;
		VPress_cnt = 20;
	}
	if(CtrlPress)
		VPress_cnt--;
	if(VPress_cnt <=0)
	{
		VPress_cnt = 0;
		CtrlPress = 0;
	}
	if( Key_ScanValue.Key_Value.V && VPress_cnt!=0)
	{
		mode++;
		mode%=2;
		VPress_cnt = 0;
		CtrlPress = 0;
	}
	if(mode == 0)
	{
		ControlMode = ImageTransfer;
	if( toe_is_error(REFEREE_TOE))
		ControlMode = Rc;
	else
		ControlMode = ImageTransfer;
	}
	else if(mode == 1)
	{
		ControlMode = Rc;
		if(toe_is_error(DBUS_TOE))
		ControlMode = ImageTransfer;
	}
}

void Key_Scan(Key_Scan_t * Key,ControlMode_e mode)
{
	uint32_t Key_Temp,Key_Down,Key_Up;
	static uint32_t Key_Last;
	
	if(mode == Rc)
	Key_Temp = rc_ctrl.key.v;
	else
	Key_Temp = ImgTransferKey;
	
//	Key_Down = Key_Temp & (Key_Last ^ Key_Temp);//异或运算，相同为0，不同为1     
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
	if(Key_Up == IMG_TRANSFER_KEY_FN1)
	{
		Key->Key_Value.FN_1 = 1;
	}
	else
		Key->Key_Value.FN_1 = 0;
	if(Key_Up == IMG_TRANSFER_KEY_FN2)
	{
		Key->Key_Value.FN_2 = 1;
	}
	else
		Key->Key_Value.FN_2 = 0;	
	if(Key_Up == IMG_TRANSFER_KEY_PAUSE)
	{
		Key->Key_Value.PAUSE = 1;
	}
	else
		Key->Key_Value.PAUSE = 0;	
	if(Key_Up == IMG_TRANSFER_KEY_TRIGGER)
	{
		Key->Key_Value.TRIGGER = 1;
	}
	else
		Key->Key_Value.TRIGGER = 0;		
	if(Key_Up == IMG_TRANSFER_KEY_MOUSE_MIDDLE)
	{
		Key->Key_Value.MIDDLE = 1;
	}
	else
		Key->Key_Value.MIDDLE = 0;		
//	
//	Key->Key_Value.CTRL_F = Key->Key_Value.CTRL & Key->Key_Value.F;
//	Key->Key_Value.CTRL_B = Key->Key_Value.CTRL & Key->Key_Value.B;
//	Key->Key_Value.Z_F = Key->Key_Value.Z & Key->Key_Value.F;
//	Key->Key_Value.Z_B = Key->Key_Value.Z & Key->Key_Value.B;
	
	
}


void MouseData_Combine(Mouse_Data_t * Data,ControlMode_e mode)
{
	switch(mode)
	{
		case ImageTransfer:
					Data->mouse_x = Referee_System.new_remote_data.mouse_x;
					Data->mouse_y = -Referee_System.new_remote_data.mouse_y;
					Data->mouse_z = Referee_System.new_remote_data.mouse_z;
					Data->mouse_l = Referee_System.new_remote_data.mouse_left;
					Data->mouse_r = Referee_System.new_remote_data.mouse_right;
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
	static  EnemyColor_e color = RED;
	if(Key_ScanValue.Key_Value.MIDDLE || (Key_ScanValue.Key_Value.PAUSE && Referee_System.new_remote_data.wheel == 1684))
	{
		color++;
		color%=2;
	}
	return color;
}



static void Gimbal_Reset(void)
{
	if(Key_ScanValue.Key_Value.X)
	{
		HAL_CAN_Stop(&hcan1);
		HAL_CAN_Stop(&hcan2);
		MX_CAN1_Init();
		MX_CAN2_Init();
		canfilter_init_start();
	}
}
uint32_t ImgTransfer_KeyCombine()
{
	uint32_t Key;
	Key = (uint32_t)Referee_System.new_remote_data.key;
	if(Referee_System.new_remote_data.fn_1)
		Key |= (uint32_t)(1<<16);
	else
		Key &= ~(uint32_t)(1<<16);

	if(Referee_System.new_remote_data.fn_2)
		Key |= (uint32_t)(1<<17);
	else
		Key &= ~(uint32_t)(1<<17);	
	
	if(Referee_System.new_remote_data.trigger)
		Key |= (uint32_t)(1<<18);
	else
		Key &= ~(uint32_t)(1<<18);	
	
	if(Referee_System.new_remote_data.pause)
		Key |= (uint32_t)(1<<19);
	else
		Key &= ~(uint32_t)(1<<19);	

	if(Referee_System.new_remote_data.mode_sw == 0)
		Key |= (uint32_t)(1<<20);
	else
		Key &= ~(uint32_t)(1<<20);
	
	if(Referee_System.new_remote_data.mode_sw == 1)
		Key |= (uint32_t)(1<<21);
	else
		Key &= ~(uint32_t)(1<<21);
	
	if(Referee_System.new_remote_data.mode_sw == 2)
		Key |= (uint32_t)(1<<22);
	else
		Key &= ~(uint32_t)(1<<22);
	
	if(Referee_System.new_remote_data.mouse_middle)
		Key |= (uint32_t)(1<<23);
	else
		Key &= ~(uint32_t)(1<<23);
	
	if(Referee_System.new_remote_data.wheel == IMG_SW_RIGHT)
		Key |= (uint32_t)(1<<24);
	else
		Key &= ~(uint32_t)(1<<24);

	if(Referee_System.new_remote_data.wheel == IMG_SW_LEFT)
		Key |= (uint32_t)(1<<25);
	else
		Key &= ~(uint32_t)(1<<25);
	return Key;
}

