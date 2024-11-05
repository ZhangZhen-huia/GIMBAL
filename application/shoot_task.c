#include "shoot_task.h"


shoot_control_t shoot_control;

//在当前比例下摩擦轮最大能到达转速29-->9300rpm差不多
int16_t trig=0,fric1=0,fric2=0;

static void shoot_init(shoot_control_t *shoot_init);
static void shoot_motor_speed_control(shoot_motor_t *shoot_motor);
static void gimbal_feedback_update(shoot_control_t *feedback_update);
static void gimbal_control_loop(shoot_control_t *control_loop);
static void Shoot_Debug_get_data(void);


void shoot_task(void const * argument)
{
	vTaskDelay(SHOOT_TASK_INIT_TIME);

    shoot_init(&shoot_control);


		while(1)
		{
		  gimbal_feedback_update(&shoot_control);
			gimbal_control_loop(&shoot_control);

			if (toe_is_error(DBUS_TOE))
      {
				CAN_cmd_trigger_firc(0,0,0);

      }
      else
      {
				CAN_cmd_trigger_firc(0,0,0);
				//CAN_cmd_trigger_firc(shoot_control.shoot_fric_L_motor.current_set,shoot_control.shoot_fric_R_motor.current_set,shoot_control.shoot_trig_motor.current_set);
      }
			osDelay(1);
		
		}



}







static void shoot_init(shoot_control_t *shoot_init)
{
	static const fp32 Shoot_trig_speed_pid[3] = {TRIG_SPEED_PID_KP,TRIG_SPEED_PID_KI,TRIG_SPEED_PID_KD};
	static const fp32 Shoot_fric_L_speed_pid[3] = {FRIC_L_SPEED_PID_KP,FRIC_L_SPEED_PID_KI,FRIC_L_SPEED_PID_KD};
	static const fp32 Shoot_fric_R_speed_pid[3] = {FRIC_L_SPEED_PID_KP,FRIC_L_SPEED_PID_KI,FRIC_L_SPEED_PID_KD};
	
	//电机数据指针获取
	shoot_init->shoot_fric_L_motor.shoot_motor_measure = get_gimbal_friction_motor_measure_point(Fric_L);
	shoot_init->shoot_fric_R_motor.shoot_motor_measure = get_gimbal_friction_motor_measure_point(Fric_R);
	shoot_init->shoot_trig_motor.shoot_motor_measure = get_gimbal_trigger_motor_measure_point();
	
	//遥控器数据指针获取
	shoot_init->shoot_rc_ctrl = get_remote_control_point();

	//初始化Trig电机速度pid
	K_FF_init(&shoot_init->shoot_trig_motor.shoot_speed_pid,PID_POSITION,Shoot_trig_speed_pid,TRIG_SPEEDD_PID_MAX_OUT,TRIG_SPEEDD_PID_MAX_IOUT,TRIG_SPEED_KF_STATIC,TRIG_SPEED_KF_DYNAMIC);

    //初始化firc电机速度pid
	K_FF_init(&shoot_init->shoot_fric_L_motor.shoot_speed_pid,PID_POSITION,Shoot_fric_L_speed_pid,FRIC_L_SPEEDD_PID_MAX_OUT,FRIC_L_SPEEDD_PID_MAX_IOUT,FRIC_L_SPEED_KF_STATIC,FRIC_L_SPEED_KF_DYNAMIC);
	K_FF_init(&shoot_init->shoot_fric_R_motor.shoot_speed_pid,PID_POSITION,Shoot_fric_R_speed_pid,FRIC_R_SPEEDD_PID_MAX_OUT,FRIC_R_SPEEDD_PID_MAX_IOUT,FRIC_R_SPEED_KF_STATIC,FRIC_R_SPEED_KF_DYNAMIC);

	gimbal_feedback_update(shoot_init);
}




/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(shoot_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
	
	feedback_update->shoot_fric_L_motor.motor_speed = feedback_update->shoot_fric_L_motor.shoot_motor_measure->rpm*FRIC_RPM_TO_SPEED_SEN;
	feedback_update->shoot_fric_R_motor.motor_speed = feedback_update->shoot_fric_R_motor.shoot_motor_measure->rpm*FRIC_RPM_TO_SPEED_SEN;
	feedback_update->shoot_trig_motor.motor_speed = feedback_update->shoot_trig_motor.shoot_motor_measure->rpm*TRIG_RPM_TO_SPEED_SEN;
#ifdef SHOOT_DEBUG
	Shoot_Debug_get_data();
#endif

		

}




/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(shoot_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
	if (toe_is_error(DBUS_TOE))
      {
				control_loop->shoot_fric_L_motor.motor_speed_set = 0;
				control_loop->shoot_fric_R_motor.motor_speed_set = 0;
				control_loop->shoot_trig_motor.motor_speed_set = 0;
      }
  else
	{
				control_loop->shoot_fric_L_motor.motor_speed_set = fric1;
				control_loop->shoot_fric_R_motor.motor_speed_set = fric2;
				control_loop->shoot_trig_motor.motor_speed_set = trig;

	}
		
  shoot_motor_speed_control(&control_loop->shoot_fric_L_motor);
	shoot_motor_speed_control(&control_loop->shoot_fric_R_motor);
	shoot_motor_speed_control(&control_loop->shoot_trig_motor);

}






static void shoot_motor_speed_control(shoot_motor_t *shoot_motor)
{
    if (shoot_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
	shoot_motor->current_set = K_FF_Cal_shoot(&shoot_motor->shoot_speed_pid,shoot_motor->motor_speed,shoot_motor->motor_speed_set,29,0);
}


/* -------------------------------vofa调参-------------------------------- */
static void Shoot_Debug_get_data(void)
{
	shoot_control.shoot_debug1.data1 = shoot_control.shoot_fric_L_motor.motor_speed;
	shoot_control.shoot_debug1.data2 = shoot_control.shoot_fric_R_motor.motor_speed;
	shoot_control.shoot_debug1.data3 = shoot_control.shoot_trig_motor.motor_speed;
	shoot_control.shoot_debug1.data4 = shoot_control.shoot_fric_L_motor.motor_speed_set;
	shoot_control.shoot_debug1.data5 = shoot_control.shoot_fric_R_motor.motor_speed_set;
	shoot_control.shoot_debug1.data6 = shoot_control.shoot_trig_motor.motor_speed_set;

}
const DebugData* get_shoot_PID_Debug(void)
{
	return &shoot_control.shoot_debug1;
}

