#include "gimbal_behaviour.h"
#include "referee.h"
#include "detect_task.h"
#include "key_task.h"
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
//static void gimbal_follow_radar_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);


/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_gyro_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);


    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = -(pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);
//    {
//        static uint16_t last_turn_keyboard = 0;
//        static uint8_t gimbal_turn_flag = 0;
//        static fp32 gimbal_end_angle = 0.0f;

//        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
//        {
//            if (gimbal_turn_flag == 0)
//            {
//                gimbal_turn_flag = 1;
//                //保存掉头的目标值
//                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
//            }
//        }
//        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

//        if (gimbal_turn_flag)
//        {
//            //不断控制到掉头的目标值，正转，反装是随机
//            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
//            {
//                *yaw += TURN_SPEED;
//            }
//            else
//            {
//                *yaw -= TURN_SPEED;
//            }
//        }
//        //到达pi （180°）后停止
//        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
//        {
//            gimbal_turn_flag = 0;
//        }
//    }
}


/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);


    *yaw = yaw_channel * YAW_RC_SEN - Mouse_Data.mouse_x * YAW_MOUSE_SEN;
    *pitch = -(pitch_channel * PITCH_RC_SEN + Mouse_Data.mouse_y * PITCH_MOUSE_SEN);
	
		/*-- 一键掉头 --*/
		if(Key_ScanValue.Key_Value.r)
		{
			 *yaw = 180;
		}

}

/*-- 调试用 --*/
//fp32 aim_yaw_set;
//fp32 aim_pitch_set;
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_auto_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = auto_data.auto_yaw_set;//aim_yaw_set;//gimbal_control_set->gimbal_mini_data->auto_yaw_set;
    *pitch =auto_data.auto_pitch_set;//aim_pitch_set;//gimbal_control_set->gimbal_mini_data->auto_pitch_set;


}


/*-- 这一版本所有的雷达控制都取消了 --*/
///**
//  * @brief          雷达控制，半自动模式
//  * @param[in]      yaw: yaw轴角度控制，陀螺仪控制为角度的增量 单位 rad
//  * @param[in]      pitch: pitch轴角度控制，编码值控制，为角度的增量 单位 rad
//  * @param[in]      gimbal_control_set: 云台数据指针
//  * @retval         none
//  */
//static void gimbal_follow_radar_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
//{

//	static int8_t turn_flag = 1;
//	    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }


//		//*yaw = 0.3;
//		
//		if(turn_flag == 1)
//		{
//			*pitch = 0.1;
//			if(gimbal_control_set->gimbal_pitch_motor.relative_angle_set == gimbal_control_set->gimbal_pitch_motor.max_relative_angle)
//				turn_flag = -1;
//		}
//		
//		else if(turn_flag == -1)
//		{
//			*pitch = -0.1;
//			if(gimbal_control_set->gimbal_pitch_motor.relative_angle_set == gimbal_control_set->gimbal_pitch_motor.min_relative_angle)
//				turn_flag = 1;

//		}


//}








/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

		 /*-- 根据不同模式进行pitch和yaw增量获取 --*/
    if (gimbal_control_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE )
    {
        gimbal_gyro_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
    {
        gimbal_encode_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
		 else if (gimbal_control_set->gimbal_behaviour == GIMBAL_AUTO_ANGLE)
    {
        gimbal_auto_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }

	
			
			
}




/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //云台行为状态机设置（6种模式，但是只用的上陀螺仪绝对角度控制和编码器相对角度控制）
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    if (gimbal_mode_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_mode_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		else if (gimbal_mode_set->gimbal_behaviour == GIMBAL_AUTO_ANGLE)
		{
			  gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;

		}

}

/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
		
    if (gimbal_mode_set == NULL)
    {
        return;
    }

////		//右键按下进入自瞄
//	if(Mouse_Data.mouse_r == 1)
//	{
//		gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
//	}
//	else
//	{
//		gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
//	}	
//	
	
//	
//		if(Key_ScanValue.Key_Value.Q)// && Key_ScanValue.Key_Value_Last.Q !=1)
//		{
//			if( gimbal_mode_set->last_gimbal_behaviour == GIMBAL_AUTO_ANGLE)
//			{
//					gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
//			}
//			else if(gimbal_mode_set->last_gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
//			{
//				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
//			}
//		}
		
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
    }
		
		else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				//gimbal_mode_set->gimbal_behaviour = GIMBAL_GYRO_ANGLE;
			
				//		//右键按下进入自瞄
					if(Mouse_Data.mouse_r == 1)
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
	
		}
		else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
		}
}


