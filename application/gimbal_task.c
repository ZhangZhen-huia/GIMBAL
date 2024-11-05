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
#include "aimbots_task.h"

fp32 final_yaw;
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
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
float angle_to_180(float inf_yaw);//加减倍数，使角度到-180至180的范围
static void gimbal_aimbot_angle_limit(gimbal_motor_t *gimbal_motor,fp32 aim_angle);


gimbal_control_t gimbal_control;	



//云台总任务
void gimbal_task(void const *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
	vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init(&gimbal_control);


    while (1)
    {
        gimbal_set_mode(&gimbal_control);                       //设置云台控制模式（陀螺仪控制或者编码值控制）
				gimbal_mode_change_control_transit(&gimbal_control);    //模式切换保存数据
				gimbal_feedback_update(&gimbal_control);                //云台数据反馈
        gimbal_set_control(&gimbal_control);                    //设置云台控制量
        gimbal_control_loop(&gimbal_control);                   //云台控制PID计算




            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_gimbal(0, 0);
            }
            else
            {
                CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.current_set,gimbal_control.gimbal_pitch_motor.current_set);
            }
        
		osDelay(1);
    }
}

		
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
	uint16_t Pitch_offset_ecd = 7325;
	fp32 pitch_max_relative_angle=6484;
	fp32 pitch_min_relative_angle=8166;
	
	static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	static const fp32 Yaw_GYRO_ABSOLUTE_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};
	static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	static const fp32 Pitch_GYRO_ABSOLUTE_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
   
	//电机数据指针获取
	init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_yaw_motor_measure_point();
	init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_pitch_motor_measure_point();
	
    //陀螺仪数据指针获取
	init->gimbal_bmi088_data = get_INS_data_point();
	
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
	//初始化pitch电机INS角度和速度pid
	K_FF_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,PID_POSITION,Pitch_GYRO_ABSOLUTE_pid,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT,PITCH_GYRO_ABSOLUTE_KF_STATIC,PITCH_GYRO_ABSOLUTE_KF_DYNAMIC);
	K_FF_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid,PID_POSITION,Pitch_speed_pid,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_KF_STATIC,PITCH_SPEED_KF_DYNAMIC);

    //初始化yaw电机pid
	K_FF_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,PID_POSITION,Yaw_GYRO_ABSOLUTE_pid,YAW_GYRO_ABSOLUTE_PID_MAX_OUT,YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,YAW_GYRO_ABSOLUTE_KF_STATIC,YAW_GYRO_ABSOLUTE_KF_DYNAMIC);
	K_FF_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,Yaw_speed_pid,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,YAW_SPEED_KF_STATIC,YAW_SPEED_KF_DYNAMIC);
	
	init->gimbal_behaviour = GIMBAL_ZERO_FORCE;
	//pitch
	ecd_format(Pitch_offset_ecd);
	init->gimbal_pitch_motor.offset_ecd = Pitch_offset_ecd;
	ecd_format(pitch_max_relative_angle);
	ecd_format(pitch_min_relative_angle);
	
	init->gimbal_pitch_motor.max_relative_angle =  -motor_ecd_to_angle_change(pitch_max_relative_angle,Pitch_offset_ecd);
	init->gimbal_pitch_motor.min_relative_angle =  -motor_ecd_to_angle_change(pitch_min_relative_angle,Pitch_offset_ecd);
	
  gimbal_feedback_update(init);

	
	init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
	init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;

}



/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
		static fp32 yaw_err,last_yaw;

    if (feedback_update == NULL)
    {
        return;
    }
		
		    //云台数据更新（更新Yaw和Pitch的绝对角度和相对角度、角速度）
    feedback_update->gimbal_pitch_motor.absolute_angle = feedback_update->gimbal_bmi088_data->INS_angle[INS_PITCH_ADDRESS_OFFSET];
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
		feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Y_ADDRESS_OFFSET];

		
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_bmi088_data->INS_angle[INS_YAW_ADDRESS_OFFSET];
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
		feedback_update->gimbal_yaw_motor.motor_gyro  = feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Z_ADDRESS_OFFSET];
		
		//yaw轴过零处理
		yaw_err=feedback_update->gimbal_yaw_motor.absolute_angle-last_yaw;
		final_yaw+=yaw_err;
		if(final_yaw>180){final_yaw-=360;}
		if(final_yaw<-180){final_yaw+=360;}
	  last_yaw=feedback_update->gimbal_yaw_motor.absolute_angle;
                                                       
}

/**
  * @brief          云台控制模式改变，保存相关数据
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
//		static fp32 yaw_err,last_yaw;
    if (gimbal_mode_change == NULL)
    {
        return;
    }
		if(gimbal_mode_change->last_gimbal_behaviour == gimbal_mode_change->gimbal_behaviour)
		{
				return;
		}

//			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
//			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
//			gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

//      gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
//			gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
//			gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
//		yaw_err=gimbal_mode_change->gimbal_yaw_motor.absolute_angle-last_yaw;
//		final_yaw+=yaw_err;
//		if(final_yaw>180){final_yaw-=360;}
//		if(final_yaw<-180){final_yaw+=360;}
//	  last_yaw=gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
		    
		
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

fp32 aim_yaw_set;
fp32 aim_pitch_set;
extern fp32 pitch_receive,yaw_receive;
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

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);



    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
		else if(set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
		{
				gimbal_aimbot_angle_limit(&set_control->gimbal_yaw_motor,yaw_receive);
		}
		
		
   if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		else if(set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
		{
				gimbal_aimbot_angle_limit(&set_control->gimbal_pitch_motor,pitch_receive);
		}

}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
/*限制最大角度放置云台旋转过度*/
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
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
			if(gimbal_motor->absolute_angle_set>180)
			{	
//				gimbal_motor->absolute_angle-=180;
				gimbal_motor->absolute_angle_set-=180;
			  final_yaw-=180;
				
			}
			else if(gimbal_motor->absolute_angle_set<-180)
			{
//				gimbal_motor->absolute_angle+=180;
				final_yaw+=180;
				gimbal_motor->absolute_angle_set+=180;

			}
		}
		else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
		{
    //now angle error
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

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
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
		
		
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
    {
       gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
		
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
}


/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
		{
			if(gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
			{
				//角度环，速度环串级pid调试
				gimbal_motor->motor_gyro_set = K_FF_Cal(&gimbal_motor->gimbal_motor_absolute_angle_pid,final_yaw,gimbal_motor->absolute_angle_set);
				gimbal_motor->current_set = K_FF_Cal(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			}
			else if(gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
			{
				//角度环，速度环串级pid调试
				gimbal_motor->motor_gyro_set = K_FF_Cal(&gimbal_motor->gimbal_motor_absolute_angle_pid,gimbal_motor->absolute_angle,gimbal_motor->absolute_angle_set);
				gimbal_motor->current_set = K_FF_Cal(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			}

		}
		else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
		{
			gimbal_motor->motor_gyro_set = K_FF_Cal(&gimbal_motor->gimbal_motor_absolute_angle_pid,gimbal_motor->absolute_angle,gimbal_motor->absolute_angle_set);
			gimbal_motor->current_set = K_FF_Cal(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
		}
}


/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

//    //角度环，速度环串级pid调试
//    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
//    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = 0;//(int16_t)(gimbal_motor->current_set);
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


float angle_to_180(float inf_yaw)//加减倍数，使角度到-180至180的范围
{	
	if(inf_yaw > 180.0f)
		inf_yaw -= 360.0f;
	else if(inf_yaw < -180.0f)
		inf_yaw += 360.0f;
	
	return inf_yaw;
}



static void gimbal_aimbot_angle_limit(gimbal_motor_t *gimbal_motor,fp32 aim_angle)
{
	if(gimbal_motor == NULL)
	{
		return;
	}
	else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
	{
		if(aim_angle>180.0f)
		{
			aim_angle = 180.0f;
		}
		else if(aim_angle < -180.0f)
		{
				aim_angle = -180.0f;
		}
		gimbal_motor->absolute_angle_set  = aim_angle;
	}
	
	else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{
		gimbal_motor->absolute_angle_set = aim_angle;
    if (gimbal_motor->relative_angle >= gimbal_motor->max_relative_angle)
    {
			gimbal_motor->absolute_angle_set = gimbal_motor->absolute_angle-1;
		}
    else if (gimbal_motor->relative_angle< gimbal_motor->min_relative_angle)
    {
			gimbal_motor->absolute_angle_set = gimbal_motor->absolute_angle+1;
    }
		
    gimbal_motor->absolute_angle_set = angle_to_180(gimbal_motor->absolute_angle_set);
	}
}

