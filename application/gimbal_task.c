#include "gimbal_task.h"

#include "main.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "Can_receive.h"
#include "arm_math.h"
#include "gimbal_task.h"
#include "pid.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "user_lib.h"
#include "communicate_task.h"
#include "key_task.h"


//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }


static void gimbal_init(gimbal_control_t* init);
void gimbal_rc_to_control_vector(gimbal_control_t *gimbal_rc_control);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_motor_gyro_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_gyro_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_encode_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_motor_encode_angle_control(gimbal_motor_t *gimbal_motor);
float angle_to_180(float inf_yaw);//加减倍数，使角度到-180至180的范围
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor,fp32 aim_angle);
static void gimbal_motor_auto_angle_control(gimbal_motor_t *gimbal_motor);
		

/*云台任务总结构体*/
gimbal_control_t gimbal_control;	
fp32 yaw_relative_init;


//云台总任务
void gimbal_task(void const *pvParameters)
{
		gimbal_control.gimbal_behaviour = GIMBAL_INIT;
    //等待陀螺仪任务更新陀螺仪数据
		osDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init(&gimbal_control);


    while (1)
    {
        gimbal_set_mode(&gimbal_control);                       //设置云台控制模式（陀螺仪控制或者编码值控制）
				gimbal_mode_change_control_transit(&gimbal_control);    //模式切换保存数据
				gimbal_feedback_update(&gimbal_control);                //云台数据反馈
        gimbal_set_control(&gimbal_control);                    //设置云台控制量
        gimbal_control_loop(&gimbal_control);                   //云台控制PID计算



			if(POWER_OFF)//((ControlMode == Rc && toe_is_error(DBUS_TOE)) || (ControlMode == ImageTransfer && toe_is_error(REFEREE_TOE)))
			{
				CAN_cmd_gimbal_yaw(0);
				CAN_cmd_gimbal_pitch(0);
			}
      else
      {
				CAN_cmd_gimbal_yaw(gimbal_control.gimbal_yaw_motor.current_set);
        CAN_cmd_gimbal_pitch(gimbal_control.gimbal_pitch_motor.current_set);
      }
        
		osDelay(2);
    }
}

		
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
	uint16_t Pitch_offset_ecd = 4031;
	fp32 pitch_max_relative_angle=3380;
	fp32 pitch_min_relative_angle=4352;
	uint16_t Yaw_offset_ecd = 3473;
	
	static const fp32 Yaw_gyro_speed_pid[3] = {YAW_GYRO_SPEED_PID_KP, YAW_GYRO_SPEED_PID_KI, YAW_GYRO_SPEED_PID_KD};
	static const fp32 Yaw_gyro_angle_pid[3] = {YAW_GYRO_ANGLE_PID_KP, YAW_GYRO_ANGLE_PID_KI, YAW_GYRO_ANGLE_PID_KD};
	
	
	static const fp32 Yaw_auto_speed_pid[3] = {YAW_AUTO_SPEED_PID_KP, YAW_AUTO_SPEED_PID_KI, YAW_AUTO_SPEED_PID_KD};
	static const fp32 Yaw_auto_angle_pid[3] = {YAW_AUTO_ANGLE_PID_KP, YAW_AUTO_ANGLE_PID_KI, YAW_AUTO_ANGLE_PID_KD};
	
	
	static const fp32 Pitch_gyro_speed_pid[3] = {PITCH_GYRO_SPEED_PID_KP, PITCH_GYRO_SPEED_PID_KI, PITCH_GYRO_SPEED_PID_KD};
	static const fp32 Pitch_gyro_angle_pid[3] = {PITCH_GYRO_ANGLE_PID_KP, PITCH_GYRO_ANGLE_PID_KI, PITCH_GYRO_ANGLE_PID_KD};
	
	static const fp32 Pitch_encode_speed_pid[3] = {PITCH_ENCODE_SPEED_PID_KP, PITCH_ENCODE_SPEED_PID_KI, PITCH_ENCODE_SPEED_PID_KD};
	static const fp32 Pitch_encode_angle_pid[4] = {PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD,PITCH_ENCODE_ANGLE_PID_SKP};
	
	static const fp32 Pitch_auto_speed_pid[3] = {PITCH_AUTO_SPEED_PID_KP, PITCH_AUTO_SPEED_PID_KI, PITCH_AUTO_SPEED_PID_KD};
	static const fp32 Pitch_auto_angle_pid[4] = {PITCH_AUTO_ANGLE_PID_KP, PITCH_AUTO_ANGLE_PID_KI, PITCH_AUTO_ANGLE_PID_KD,PITCH_AUTO_ANGLE_PID_SKP};
	
//	static const fp32 Yaw_radar_speed_pid[3] = {YAW_RADAR_SPEED_PID_KP, YAW_RADAR_SPEED_PID_KI, YAW_RADAR_SPEED_PID_KD};

	//电机数据指针获取
	init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_yaw_motor_measure_point();
	init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_pitch_motor_measure_point();
	
    //陀螺仪数据指针获取
	init->gimbal_bmi088_data = get_INS_data_point();
	
    //遥控器数据指针获取
  init->gimbal_rc_ctrl = get_remote_control_point();
	init->gimbal_mini_data = get_mini_data_point();
	
	/* --- Pitch --- */
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_angle_pid,PID_POSITION,DATA_NORMAL,Pitch_gyro_angle_pid,PITCH_GYRO_ANGLE_PID_MAX_OUT,PITCH_GYRO_ANGLE_PID_MAX_IOUT,NONE);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_speed_pid,PID_POSITION,DATA_NORMAL,Pitch_gyro_speed_pid,PITCH_GYRO_SPEED_PID_MAX_OUT,PITCH_GYRO_SPEED_PID_MAX_IOUT,NONE);

	PID_init(&init->gimbal_pitch_motor.gimbal_motor_encode_angle_pid,PID_POSITION,DATA_NORMAL,Pitch_encode_angle_pid,PITCH_ENCODE_ANGLE_PID_MAX_OUT,PITCH_ENCODE_ANGLE_PID_MAX_IOUT,NONE);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_encode_speed_pid,PID_POSITION,DATA_NORMAL,Pitch_encode_speed_pid,PITCH_ENCODE_SPEED_PID_MAX_OUT,PITCH_ENCODE_SPEED_PID_MAX_IOUT,NONE);

	PID_init(&init->gimbal_pitch_motor.gimbal_motor_auto_angle_pid,PID_POSITION,DATA_NORMAL,Pitch_auto_angle_pid,PITCH_AUTO_ANGLE_PID_MAX_OUT,PITCH_AUTO_ANGLE_PID_MAX_IOUT,NONE);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_auto_speed_pid,PID_POSITION,DATA_NORMAL,Pitch_auto_speed_pid,PITCH_AUTO_SPEED_PID_MAX_OUT,PITCH_AUTO_SPEED_PID_MAX_IOUT,NONE);

	/* --- Yaw --- */
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_angle_pid,PID_POSITION,DATA_GYRO,Yaw_gyro_angle_pid,YAW_GYRO_ANGLE_PID_MAX_OUT,YAW_GYRO_ANGLE_PID_MAX_IOUT,NONE);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_speed_pid,PID_POSITION,DATA_NORMAL,Yaw_gyro_speed_pid,YAW_GYRO_SPEED_PID_MAX_OUT,YAW_GYRO_SPEED_PID_MAX_IOUT,NONE);

	PID_init(&init->gimbal_yaw_motor.gimbal_motor_auto_angle_pid,PID_POSITION,DATA_GYRO,Yaw_auto_angle_pid,YAW_AUTO_ANGLE_PID_MAX_OUT,YAW_AUTO_ANGLE_PID_MAX_IOUT,NONE);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_auto_speed_pid,PID_POSITION,DATA_NORMAL,Yaw_auto_speed_pid,YAW_AUTO_SPEED_PID_MAX_OUT,YAW_AUTO_SPEED_PID_MAX_IOUT,NONE);

//	PID_init(&init->gimbal_yaw_motor.gimbal_motor_radar_speed_pid,PID_POSITION,DATA_NORMAL,Yaw_radar_speed_pid,YAW_RADAR_SPEED_PID_MAX_OUT,YAW_RADAR_SPEED_PID_MAX_IOUT);
	init->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
	//pitch
	ecd_format(Pitch_offset_ecd);
	init->gimbal_pitch_motor.offset_ecd = Pitch_offset_ecd;
	ecd_format(pitch_max_relative_angle);
	ecd_format(pitch_min_relative_angle);
	
	init->gimbal_pitch_motor.max_relative_angle =  -motor_ecd_to_angle_change(pitch_max_relative_angle,Pitch_offset_ecd);
	init->gimbal_pitch_motor.min_relative_angle =  -motor_ecd_to_angle_change(pitch_min_relative_angle,Pitch_offset_ecd);
	
	ecd_format(Yaw_offset_ecd);
	init->gimbal_yaw_motor.offset_ecd = Yaw_offset_ecd;
	yaw_relative_init = -motor_ecd_to_angle_change(init->gimbal_yaw_motor.gimbal_motor_measure->ecd,init->gimbal_yaw_motor.offset_ecd);
	gimbal_feedback_update(init);

	init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
	init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;

}


//fp32 omeg;
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
//		static fp32 ecd,last_ecd,last_omg;
    if (feedback_update == NULL)
    {
        return;
    }
		
		//云台数据更新（更新Yaw和Pitch的绝对角度和相对角度、角速度）
    feedback_update->gimbal_pitch_motor.absolute_angle = -(feedback_update->gimbal_bmi088_data->INS_angle[INS_PITCH_ADDRESS_OFFSET]);//-chassis_data.pitch);
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
		feedback_update->gimbal_pitch_motor.motor_gyro = -feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Y_ADDRESS_OFFSET];

		
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_bmi088_data->INS_angle[INS_YAW_ADDRESS_OFFSET];
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
		feedback_update->gimbal_yaw_motor.motor_gyro  = feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Z_ADDRESS_OFFSET];
				
/*-- 尝试自己用ecd的差分来代替陀螺仪的角速度，但是效果不好，可能方法没有用对 --*/
//		last_ecd = ecd;
//		last_omg = omeg;
//		ecd = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;

//		
//		omeg = (ecd - last_ecd)/10.0f;
//    omeg = ((omeg)>409.6f)? omeg - 819.1f :omeg;
//		omeg = ((omeg)<-409.6f)? omeg + 819.1f :omeg;
//		
//		omeg = last_omg*0.5f + 0.5f*omeg ;
		
		
		

                                                       
}

/**
  * @brief          云台控制模式改变，保存相关数据
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
		if(gimbal_mode_change->last_gimbal_behaviour == gimbal_mode_change->gimbal_behaviour)
		{
				return;
		}
		
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
			gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

      gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
			gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;

		    
		gimbal_mode_change->last_gimbal_behaviour = gimbal_mode_change->gimbal_behaviour;
}



/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

		/*-- 根据不同模式获取pitch和yaw的增量 --*/
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);



		/*-- 根据不同模式进行yaw的限幅 --*/
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_gyro_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        gimbal_encode_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
		else if(set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
		{
				//自瞄模式下，陀螺仪角度控制
				gimbal_auto_angle_limit(&set_control->gimbal_yaw_motor,add_yaw_angle);
		}

		
		/*-- 根据不同模式进行pitch的限幅 --*/
   if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_gyro_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        gimbal_encode_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		else if(set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
		{
				//自瞄模式下，陀螺仪角度控制
				gimbal_auto_angle_limit(&set_control->gimbal_pitch_motor,add_pitch_angle);
		}


}



/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_gyro_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{     
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
		
		else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
		{
			
			gimbal_motor->absolute_angle_set += add;
			
			gimbal_motor->absolute_angle_set = gimbal_motor->absolute_angle_set >  180 ? gimbal_motor->absolute_angle_set-360:gimbal_motor->absolute_angle_set;
			gimbal_motor->absolute_angle_set = gimbal_motor->absolute_angle_set <	-180 ? gimbal_motor->absolute_angle_set+360:gimbal_motor->absolute_angle_set;

		}
		
		else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
		{
			//当前控制误差角度
			bias_angle = angle_to_180(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
			//云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
			if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
			{
					//如果是往最大机械角度控制方向
					if (add > 0.0f)
					{
							//计算出一个最大的添加角度，
							add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
					}
			}
			else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
			{
					if (add < 0.0f)
					{
							add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
					}
			}
			angle_set = gimbal_motor->absolute_angle_set;
			gimbal_motor->absolute_angle_set = angle_to_180(angle_set + add);
	}
}

extern uint8_t RotateMode[2];
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_AUTO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor,fp32 aim_angle)
{
	if(gimbal_motor == NULL)
	{
		return;
	}
	
	//yaw轴电机
	else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
	{
			aim_angle = aim_angle >  180 ? aim_angle-360:aim_angle;
			aim_angle = aim_angle <	-180 ? aim_angle+360:aim_angle;
		
	
		if(RotateMode[0])
		{
			//
			if(RotateMode[1] == 0)
					gimbal_motor->absolute_angle_set  = aim_angle - 2.2f;
			else
				gimbal_motor->absolute_angle_set  = aim_angle + 2.2f;
		}
		else
			gimbal_motor->absolute_angle_set  = aim_angle ;
	}
	
	//pitch轴电机
	else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{

		
    if (gimbal_motor->relative_angle >= gimbal_motor->max_relative_angle)
    {
			aim_angle = gimbal_motor->max_relative_angle;
		}
		
    else if (gimbal_motor->relative_angle<= gimbal_motor->min_relative_angle)
    {
			aim_angle = gimbal_motor->min_relative_angle;
    }
		
    gimbal_motor->relative_angle_set = angle_to_180(aim_angle);
	}
	
	
}




/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_encode_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
		
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
		
		/*-- 选择不同的PID --*/
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_gyro_angle_control(&control_loop->gimbal_yaw_motor);
    }
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_encode_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
       gimbal_motor_auto_angle_control(&control_loop->gimbal_yaw_motor);
    }
	
		
		/*-- 选择不同的PID --*/
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_gyro_angle_control(&control_loop->gimbal_pitch_motor);
    }
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_encode_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        gimbal_motor_auto_angle_control(&control_loop->gimbal_pitch_motor);
    }
	
}


/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_gyro_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_angle_pid,gimbal_motor->absolute_angle,gimbal_motor->absolute_angle_set);
		gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_speed_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
		
}


/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_encode_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_encode_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_encode_speed_pid, /*omeg*/gimbal_motor->motor_gyro,gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_AUTO，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_auto_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
		{
			//角度环，速度环串级pid调试
			gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_auto_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);
			gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_auto_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			//控制值赋值
			gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		}
		else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
		{
			//角度环，速度环串级pid调试
			gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_auto_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set);
			gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_auto_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			//控制值赋值
			gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		}
}




/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  * @attention      后面可以试着把这个写成函数指针？
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}


//加减倍数，使角度到-180至180的范围
float angle_to_180(float inf_yaw)
{	
	if(inf_yaw > 180.0f)
		inf_yaw -= 360.0f;
	else if(inf_yaw < -180.0f)
		inf_yaw += 360.0f;
	
	return inf_yaw;
}



/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

