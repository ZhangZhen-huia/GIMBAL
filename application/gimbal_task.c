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
//�������ֵ���� 0��8191
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
float angle_to_180(float inf_yaw);//�Ӽ�������ʹ�Ƕȵ�-180��180�ķ�Χ
static void gimbal_aimbot_angle_limit(gimbal_motor_t *gimbal_motor,fp32 aim_angle);


gimbal_control_t gimbal_control;	



//��̨������
void gimbal_task(void const *pvParameters)
{
    //�ȴ������������������������
	vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //��̨��ʼ��
    gimbal_init(&gimbal_control);


    while (1)
    {
        gimbal_set_mode(&gimbal_control);                       //������̨����ģʽ�������ǿ��ƻ��߱���ֵ���ƣ�
				gimbal_mode_change_control_transit(&gimbal_control);    //ģʽ�л���������
				gimbal_feedback_update(&gimbal_control);                //��̨���ݷ���
        gimbal_set_control(&gimbal_control);                    //������̨������
        gimbal_control_loop(&gimbal_control);                   //��̨����PID����




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
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
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
   
	//�������ָ���ȡ
	init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_yaw_motor_measure_point();
	init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_pitch_motor_measure_point();
	
    //����������ָ���ȡ
	init->gimbal_bmi088_data = get_INS_data_point();
	
    //ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
	//��ʼ��pitch���INS�ǶȺ��ٶ�pid
	K_FF_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,PID_POSITION,Pitch_GYRO_ABSOLUTE_pid,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT,PITCH_GYRO_ABSOLUTE_KF_STATIC,PITCH_GYRO_ABSOLUTE_KF_DYNAMIC);
	K_FF_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid,PID_POSITION,Pitch_speed_pid,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_KF_STATIC,PITCH_SPEED_KF_DYNAMIC);

    //��ʼ��yaw���pid
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
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
		static fp32 yaw_err,last_yaw;

    if (feedback_update == NULL)
    {
        return;
    }
		
		    //��̨���ݸ��£�����Yaw��Pitch�ľ��ԽǶȺ���ԽǶȡ����ٶȣ�
    feedback_update->gimbal_pitch_motor.absolute_angle = feedback_update->gimbal_bmi088_data->INS_angle[INS_PITCH_ADDRESS_OFFSET];
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
		feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Y_ADDRESS_OFFSET];

		
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_bmi088_data->INS_angle[INS_YAW_ADDRESS_OFFSET];
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
		feedback_update->gimbal_yaw_motor.motor_gyro  = feedback_update->gimbal_bmi088_data->gyro[INS_GYRO_Z_ADDRESS_OFFSET];
		
		//yaw����㴦��
		yaw_err=feedback_update->gimbal_yaw_motor.absolute_angle-last_yaw;
		final_yaw+=yaw_err;
		if(final_yaw>180){final_yaw-=360;}
		if(final_yaw<-180){final_yaw+=360;}
	  last_yaw=feedback_update->gimbal_yaw_motor.absolute_angle;
                                                       
}

/**
  * @brief          ��̨����ģʽ�ı䣬�����������
  * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
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
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
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
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
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
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
		else if(set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
		{
				gimbal_aimbot_angle_limit(&set_control->gimbal_yaw_motor,yaw_receive);
		}
		
		
   if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		else if(set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
		{
				gimbal_aimbot_angle_limit(&set_control->gimbal_pitch_motor,pitch_receive);
		}

}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
/*�������Ƕȷ�����̨��ת����*/
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
    //��ǰ�������Ƕ�
    bias_angle = angle_to_180(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            //�����һ��������ӽǶȣ�
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
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
		
    //�Ƿ񳬹���� ��Сֵ
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
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
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
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
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
				//�ǶȻ����ٶȻ�����pid����
				gimbal_motor->motor_gyro_set = K_FF_Cal(&gimbal_motor->gimbal_motor_absolute_angle_pid,final_yaw,gimbal_motor->absolute_angle_set);
				gimbal_motor->current_set = K_FF_Cal(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			}
			else if(gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_AIMBOT)
			{
				//�ǶȻ����ٶȻ�����pid����
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
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

//    //�ǶȻ����ٶȻ�����pid����
//    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
//    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = 0;//(int16_t)(gimbal_motor->current_set);
}


/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  * @attention      ����������Ű����д�ɺ���ָ�룿
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}


float angle_to_180(float inf_yaw)//�Ӽ�������ʹ�Ƕȵ�-180��180�ķ�Χ
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

