#include "shoot_task.h"
#include "tim.h"
#include "math.h"
#include "vofa_task.h"
#include "Can_receive.h"
#include "communicate_task.h"
#include "key_task.h"
shoot_control_t shoot_control;


//在当前比例下摩擦轮最大能到达转速29-->9300rpm差不多
fp32 fric1=-MAX_SPEED,fric2=MAX_SPEED;

static void shoot_init(shoot_control_t *shoot_init);
static void shoot_motor_control(shoot_motor_t *shoot_motor);
static void shoot_feedback_update(shoot_control_t *feedback_update);
static void shoot_control_loop(shoot_control_t *control_loop);
static void Shoot_Debug_get_data(void);
static void fric_motor_control(shoot_control_t * control_loop);




void shoot_task(void const * argument)
{
		vTaskDelay(SHOOT_TASK_INIT_TIME);

    shoot_init(&shoot_control);

		while(1)
		{
		  shoot_feedback_update(&shoot_control);			//发射电机数据更新
			shoot_motor_mode_set(&shoot_control);				//发射机构开关选择
			fric_motor_control(&shoot_control);					//摩擦轮控制量配置
			shoot_control_loop(&shoot_control);					//摩擦轮和拨弹盘输出值计算

			if(POWER_OFF)//((ControlMode == Rc && toe_is_error(DBUS_TOE)) || (ControlMode == ImageTransfer && toe_is_error(REFEREE_TOE)))
			{
				CAN_cmd_firc(0,0);

      }
      else
      {
				 //CAN_cmd_firc(0,0);
				CAN_cmd_firc(shoot_control.shoot_fric_L_motor.current_set,shoot_control.shoot_fric_R_motor.current_set);
      }
			osDelay(1);
		
		}



}








/**
	* @brief          发射机构初始化
  * @param[out]     shoot_init:"shoot_control"变量指针.
  * @retval         none
  */
static void shoot_init(shoot_control_t *shoot_init)
{

	static const fp32 Shoot_fric_L_speed_pid[3] = {FRIC_L_SPEED_PID_KP,FRIC_L_SPEED_PID_KI,FRIC_L_SPEED_PID_KD};
	static const fp32 Shoot_fric_R_speed_pid[3] = {FRIC_R_SPEED_PID_KP,FRIC_R_SPEED_PID_KI,FRIC_R_SPEED_PID_KD};
	static const fp32 Shoot_Speed_compensate_pid[3] = {190,0,0};
	//电机数据指针获取
	shoot_init->shoot_fric_L_motor.shoot_motor_measure = get_gimbal_friction_motor_measure_point(Fric_L);
	shoot_init->shoot_fric_R_motor.shoot_motor_measure = get_gimbal_friction_motor_measure_point(Fric_R);
	
	//遥控器数据指针获取
	shoot_init->shoot_rc_ctrl = get_remote_control_point();
	shoot_init->auto_fireFlag = get_autofire_flag_point();
  //初始化firc电机速度pid
	K_FF_init(&shoot_init->shoot_fric_L_motor.shoot_speed_pid,PID_POSITION,Shoot_fric_L_speed_pid,FRIC_L_SPEED_PID_MAX_OUT,FRIC_L_SPEED_PID_MAX_IOUT,FRIC_L_SPEED_KF_STATIC,FRIC_L_SPEED_KF_DYNAMIC);
	K_FF_init(&shoot_init->shoot_fric_R_motor.shoot_speed_pid,PID_POSITION,Shoot_fric_R_speed_pid,FRIC_R_SPEED_PID_MAX_OUT,FRIC_R_SPEED_PID_MAX_IOUT,FRIC_R_SPEED_KF_STATIC,FRIC_R_SPEED_KF_DYNAMIC);
	PID_init(&shoot_init->shoot_speed_compensate_pid,PID_POSITION,DATA_NORMAL,Shoot_Speed_compensate_pid,1,0,NONE);
	
	shoot_init->shoot_agency_state = SHOOT_OFF;
	shoot_init->trig_mode = Cease_fire;
	shoot_init->fric_mode = STOP;
	
	shoot_feedback_update(shoot_init);
}




/**
	* @brief          发射机构数据更新
  * @param[out]     feedback_update:"shoot_control"变量指针.
  * @retval         none
  */
static void shoot_feedback_update(shoot_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
	
	/*-- 左右摩擦轮速度反馈 --*/
	feedback_update->shoot_fric_L_motor.motor_speed = feedback_update->shoot_fric_L_motor.shoot_motor_measure->rpm*FRIC_RPM_TO_SPEED_SEN;
	feedback_update->shoot_fric_R_motor.motor_speed = feedback_update->shoot_fric_R_motor.shoot_motor_measure->rpm*FRIC_RPM_TO_SPEED_SEN;
		
	/*-- 弹速闭环 --*/
	feedback_update->shoot_cooling_heat_last = feedback_update->shoot_cooling_heat;	
	feedback_update->bullet_speed = chassis_data.bullet_speed / 65535.0f * 25.0f;
	feedback_update->shoot_cooling_heat = chassis_data.shoot_cooling_heat;

	/*-- 弹速Vofa调试 --*/
#ifdef SHOOT_DEBUG
	Shoot_Debug_get_data();
#endif

		

}




/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     control_loop:"shoot_control"变量指针.
  * @retval         none
  */
static void shoot_control_loop(shoot_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

	/*-- PID计算调用 --*/
  shoot_motor_control(&control_loop->shoot_fric_L_motor);
	shoot_motor_control(&control_loop->shoot_fric_R_motor);

}





/**
  * @brief          摩擦轮控制量设置
  * @param[out]     control_loop:"shoot_control"变量指针.
  * @retval         none
  */
static void fric_motor_control(shoot_control_t * control_loop)
{
	if (control_loop == NULL)
  {
      return;
  }

	if(control_loop->fric_mode == STOP)
	{
		control_loop->shoot_fric_L_motor.motor_speed_set = 0;
		control_loop->shoot_fric_R_motor.motor_speed_set = 0;

	}
	else if(control_loop->fric_mode == START)
	{
//		/*-- 弹速闭环，粗调了一下，效果不好 --*/
//		if(control_loop->shoot_cooling_heat > control_loop->shoot_cooling_heat_last)
//		{
//			control_loop->shoot_fric_L_motor.motor_speed_set = fric1+PID_calc(&control_loop->shoot_speed_compensate_pid,control_loop->bullet_speed,23);
//			control_loop->shoot_fric_R_motor.motor_speed_set = fric2-PID_calc(&control_loop->shoot_speed_compensate_pid,-control_loop->bullet_speed,23);
//		}
//		else
//		{
			control_loop->shoot_fric_L_motor.motor_speed_set = fric1;
			control_loop->shoot_fric_R_motor.motor_speed_set = fric2;
//		}
	}
}



/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     shoot_motor:"shoot_motor_t"变量指针.
  * @retval         none
  */
static void shoot_motor_control(shoot_motor_t *shoot_motor)
{
    if (shoot_motor == NULL)
    {
        return;
    }
	//摩擦轮速度环控制
	if((shoot_motor == &shoot_control.shoot_fric_L_motor) || (shoot_motor == &shoot_control.shoot_fric_R_motor))
	{
    
		shoot_motor->current_set = K_FF_Cal(&shoot_motor->shoot_speed_pid,shoot_motor->motor_speed,shoot_motor->motor_speed_set);
		
	}

}



/* ----------------------------------------------------------------------- */
/* -------------------------------vofa调参-------------------------------- */
/* ----------------------------------------------------------------------- */
static void Shoot_Debug_get_data(void)
{
	shoot_control.shoot_debug1.data1 = shoot_control.shoot_fric_L_motor.motor_speed;
	shoot_control.shoot_debug1.data2 = shoot_control.shoot_fric_R_motor.motor_speed;
	
	shoot_control.shoot_debug1.data4 = shoot_control.shoot_fric_L_motor.motor_speed_set;
	shoot_control.shoot_debug1.data5 = shoot_control.shoot_fric_R_motor.motor_speed_set;

}
const DebugData* get_shoot_PID_Debug(void)
{
	return &shoot_control.shoot_debug1;
}
/* ----------------------------------------------------------------------- */
/* ---------------------------------endif--------------------------------- */
/* ----------------------------------------------------------------------- */



