/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "freertos.h"
#include "task.h"
#include "stdio.h"



/**
	*PID�޷�
	*�����޷���PID������޷�
**/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LIMIT_MAX_MIN(input, max,min) \
    {                          				\
        if (input > max)       				\
        {                      				\
            input = max;       				\
        }                      				\
        else if (input < min)  				\
        {                      				\
            input = min;       				\
        }                      				\
    }

float my_fabs(float input)
{
	if(input<0)
		input=-input;
	else 
		input=input;
	
	return input;
}

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		/*PIDģʽѡ����������*/
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		/*PID����޷��ͻ����޷�*/
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		/*�����PID*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
/*Kp*err+Ki*err�Ļ���+Kd*err��΢��*/
/*�������˵�ͨ�˲�*/
float PID_calc(pid_type_def *pid, float ref, float set)
{
	uint8_t a=0.5;
    if (pid == NULL)
    {
        return 0.0f;
    }
	if(pid->data_mode == DATA_GYRO)
	{
	  if( pid->error[0] >  180.0f)  pid->error[0] -= 360.0f; 
	  if( pid->error[0] < -180.0f)  pid->error[0] += 360.0f; 
	}
		
	if(pid->data_mode == DATA_NORMAL)
	{
       //��������
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//Ŀ��ֵ
    pid->fdb = ref;//����ֵ
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//�������
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//�����Kp*���
			
        pid->Iout += pid->Ki * pid->error[0];//�����Ki*������
			
        pid->Dbuf[2] = pid->Dbuf[1];//���֮��
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
        pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
        LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
        LimitMax(pid->out, pid->max_out);//PID����޷�
    }
    else if (pid->mode == PID_DELTA)//���PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
	else if (pid->mode == PID_DISTENCE) //refΪ��ǰ�ߵľ��� setΪĿ�����
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
		pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
		LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
		LimitMax(pid->out, pid->max_out);//PID����޷�
	}
    return pid->out;
}


/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;//������
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;//������֮��
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;//����������
    pid->fdb = pid->set = 0.0f;//���Ŀ��ֵ
}


fp32 PID_Calc_Angle(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		
	  if( pid->error[0] >  4096.0f)  pid->error[0] -= 8191.0f; 
	  if( pid->error[0] < -4096.0f)  pid->error[0] += 8191.0f; 
	
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}



fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set, uint16_t ecd_range)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
		//���㴦��
    pid->error[0] = ecd_zero(set, ref, ecd_range);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


//���㺯��
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd, uint16_t ecd_range)
{
		int32_t relative_ecd = ecd - offset_ecd;
		uint16_t half_ecd_range = ecd_range / 2;
	
		if(relative_ecd > half_ecd_range)
		{
					relative_ecd -= ecd_range;
		}
		if(relative_ecd < - half_ecd_range)
		{
					relative_ecd += ecd_range;
		}

		return relative_ecd;
}


//ǰ��PID
float K_FF_Cal_shoot(pid_type_def *pid, float ref, float set,float fabs_max,float fabs_min)
{
	static TickType_t Real_Time,Last_Time,dt;
	float range,rat;
	// Ŀ��ֵ�ĵ�������Ŀ��ֵ�ı仯�ʣ�dt ��ʱ����
  float aim_derivative = 0;
	Real_Time = xTaskGetTickCount();
	if(Last_Time != 0) //��һ�β����㶯̬
	{
		dt = Real_Time - Last_Time;
	}
	Last_Time = Real_Time;
    if (dt > 0)
		{
        aim_derivative = (set - pid->last_aim) / dt;
    }
		pid->last_aim = set;
		
		range = fabs_max-fabs_min;
		rat = 1.0f/range;
    // ���㾲̬ǰ����
    float K_ff_static = pid->K_ff_static; // ��̬ǰ������
		
		if(set != fabs_min)
		{
			if(set > 0)
				pid->feedforward_static_out = 1.0f*K_ff_static * (fabs_max-my_fabs(set))*rat;
			else if(set<0)
				pid->feedforward_static_out = -1.0f*K_ff_static * (fabs_max-my_fabs(set))*rat;
		}
		else if(set == fabs_min)
			pid->feedforward_static_out = 0;

		
		
    // ���㶯̬ǰ����
    float K_ff_dynamic = pid->K_ff_dynamic; // ��̬ǰ������
   pid->feedforward_dynamic_out = K_ff_dynamic * aim_derivative;
		
		


		
	uint8_t a=0.5;
    if (pid == NULL)
    {
        return 0.0f;
    }
	if(pid->data_mode == DATA_GYRO)
	{
	  if( pid->error[0] >  180.0f)  pid->error[0] -= 360.0f; 
	  if( pid->error[0] < -180.0f)  pid->error[0] += 360.0f; 
	}
		
	if(pid->data_mode == DATA_NORMAL)
	{
       //��������
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//Ŀ��ֵ
    pid->fdb = ref;//����ֵ
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//�������
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//�����Kp*���
			
        pid->Iout += pid->Ki * pid->error[0];//�����Ki*������
			
        pid->Dbuf[2] = pid->Dbuf[1];//���֮��
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
        pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
        LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
        LimitMax(pid->out, pid->max_out);//PID����޷�
    }
    else if (pid->mode == PID_DELTA)//���PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
	else if (pid->mode == PID_DISTENCE) //refΪ��ǰ�ߵľ��� setΪĿ�����
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
		pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
		LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
		LimitMax(pid->out, pid->max_out);//PID����޷�
	}
    return (pid->out+pid->feedforward_dynamic_out+pid->feedforward_static_out); //���ﲻֱ�Ӽ�����Ϊ��Ӱ�챾��pid��outֵ������һ��pid���㣬ǰ����pid�ǻ��������
}

//ǰ��PID
float K_FF_Cal(pid_type_def *pid, float ref, float set)
{
	static TickType_t Real_Time,Last_Time,dt;
	// Ŀ��ֵ�ĵ�������Ŀ��ֵ�ı仯�ʣ�dt ��ʱ����
    float aim_derivative = 0;
	Real_Time = xTaskGetTickCount();
	if(Last_Time != 0) //��һ�β����㶯̬
	{
		dt = Real_Time - Last_Time;
	}
	Last_Time = Real_Time;
    if (dt > 0)
		{
        aim_derivative = (set - pid->last_aim) / dt;
    }
    pid->last_aim = set;

    // ���㾲̬ǰ����
    float K_ff_static = pid->K_ff_static; // ��̬ǰ������
    pid->feedforward_static_out = K_ff_static * set;

    // ���㶯̬ǰ����
    float K_ff_dynamic = pid->K_ff_dynamic; // ��̬ǰ������
    pid->feedforward_dynamic_out= K_ff_dynamic * aim_derivative;
		
		
		
	uint8_t a=0.5;
    if (pid == NULL)
    {
        return 0.0f;
    }
	if(pid->data_mode == DATA_GYRO)
	{
	  if( pid->error[0] >  180.0f)  pid->error[0] -= 360.0f; 
	  if( pid->error[0] < -180.0f)  pid->error[0] += 360.0f; 
	}
		
	if(pid->data_mode == DATA_NORMAL)
	{
       //��������
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//Ŀ��ֵ
    pid->fdb = ref;//����ֵ
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//�������
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//�����Kp*���
			
        pid->Iout += pid->Ki * pid->error[0];//�����Ki*������
			
        pid->Dbuf[2] = pid->Dbuf[1];//���֮��
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
        pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
        LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
        LimitMax(pid->out, pid->max_out);//PID����޷�
    }
    else if (pid->mode == PID_DELTA)//���PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
	else if (pid->mode == PID_DISTENCE) //refΪ��ǰ�ߵľ��� setΪĿ�����
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//�������֮��,������΢��
		pid->Dout = pid->Kd * pid->Dbuf[0];//΢���Kd*���΢��
		LimitMax(pid->Iout, pid->max_iout);//PID�����޷�
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID���
		LimitMax(pid->out, pid->max_out);//PID����޷�
	}
    return (pid->out+pid->feedforward_dynamic_out+pid->feedforward_static_out); //���ﲻֱ�Ӽ�����Ϊ��Ӱ�챾��pid��outֵ������һ��pid���㣬ǰ����pid�ǻ��������
}


//ǰ��PID��ʼ��
void K_FF_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,float K_ff_static,float K_ff_dynamic)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		/*PIDģʽѡ����������*/
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		/*PID����޷��ͻ����޷�*/
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		/*�����PID*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
	pid->K_ff_static=K_ff_static;
	pid->K_ff_dynamic=K_ff_dynamic;
}



///*********ģ��pid����*/

// 
//#define NL   -3
//#define NM	 -2
//#define NS	 -1
//#define ZE	 0
//#define PS	 1
//#define PM	 2
//#define PL	 3
// 
//static const float fuzzyRuleKp[7][7]={
//	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
//	PL,	PL,	PM,	PS,	PS,	ZE,	NS,
//	PM,	PM,	PM,	PS,	ZE,	NS,	NS,
//	PM,	PM,	PS,	ZE,	NS,	NM,	NM,
//	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
//	PS,	ZE,	NS,	NM,	NM,	NM,	NL,
//	ZE,	ZE,	NM,	NM,	NM,	NL,	NL
//};
// 
//static const float fuzzyRuleKi[7][7]={
//	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
//	NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
//	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
//	NM,	NM,	NS,	ZE,	PS,	PM,	PM,
//	NS,	NS,	ZE,	PS,	PS,	PM,	PL,
//	ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
//	ZE,	ZE,	PS,	PM,	PM,	PL,	PL
//};
// 
//static const float fuzzyRuleKd[7][7]={
//	PS,	NS,	NL,	NL,	NL,	NM,	PS,
//	PS,	NS,	NL,	NM,	NM,	NS,	ZE,
//	ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
//	ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
//	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
//	PL,	NS,	PS,	PS,	PS,	PS,	PL,
//	PL,	PM,	PM,	PM,	PS,	PS,	PL
//};
// 

// //�ؼ��㷨
//void fuzzy( FuzzyPID*  fuzzy_PID)
//{
//     float e = fuzzy_PID ->PreError/ fuzzy_PID->stair;
//	   float ec = (fuzzy_PID ->Out - fuzzy_PID ->Out_last) / fuzzy_PID->stair;
//     short etemp,ectemp;
//     float eLefttemp,ecLefttemp;    //������
//     float eRighttemp ,ecRighttemp; 
// 
//     short eLeftIndex,ecLeftIndex;  //��ǩ
//     short eRightIndex,ecRightIndex;

//	  //ģ����
//     if(e>=PL)
//			 etemp=PL;//������Χ
//		 else if(e>=PM)
//			 etemp=PM;
//		 else if(e>=PS)
//			 etemp=PS;
//		 else if(e>=ZE)
//			 etemp=ZE;
//		 else if(e>=NS)
//			 etemp=NS;
//		 else if(e>=NM)
//			 etemp=NM;
//		 else if(e>=NL)
//			 etemp=NL;
//		 else 
//			 etemp=2*NL;
// 
//		 if( etemp == PL)
//		{
//		 //����E������
//				eRighttemp= 0 ;    //�����
//				eLefttemp= 1 ;
//			
//     //�����ǩ
//	   eLeftIndex = 6 ;      
//	   eRightIndex= 6 ;
//			
//		}else if( etemp == 2*NL )
//    {

//			//����E������
//				eRighttemp = 1;    //�����
//				eLefttemp = 0;
//	
//     //�����ǩ
//	   eLeftIndex = 0 ;       
//	   eRightIndex = 0 ;
//			
//		}	else 
//    {

//			//����E������
//				eRighttemp=(e-etemp);  //���Ժ�����Ϊ��������
//				eLefttemp=(1- eRighttemp);
//			
//     //�����ǩ
//	   eLeftIndex =(short) (etemp-NL);       //���� etemp=2.5��NL=-3����ô�õ������к�Ϊ5  ��0 1 2 3 4 5 6��
//	   eRightIndex=(short) (eLeftIndex+1);
//			
//		}		
//	   
//		
//		 if(ec>=PL)
//			 ectemp=PL;
//		 else if(ec>=PM)
//			 ectemp=PM;
//		 else if(ec>=PS)
//			 ectemp=PS;
//		 else if(ec>=ZE)
//			 ectemp=ZE;
//		 else if(ec>=NS)
//			 ectemp=NS;
//		 else if(ec>=NM)
//			 ectemp=NM;
//		 else if(ec>=NL)
//			 ectemp=NL;
//		 else 
//			 ectemp=2*NL;
//		 
//	  
//   if( ectemp == PL )
//	 {
//    //����EC������		 
//		 ecRighttemp= 0 ;      //�����
//		 ecLefttemp= 1 ;
//			
//		 ecLeftIndex = 6 ;  
//	   ecRightIndex = 6 ;	 
//	 
//	 } else if( ectemp == 2*NL)
//	 {
//    //����EC������		 
//		 ecRighttemp= 1 ;
//		 ecLefttemp= 0 ;
//			
//		 ecLeftIndex = 0 ;  
//	   ecRightIndex = 0 ;	 	 
//	 }else
//	 {
//    //����EC������		 
//		 ecRighttemp=(ec-ectemp);
//		 ecLefttemp=(1- ecRighttemp);
//			
//		 ecLeftIndex =(short) (ectemp-NL);  
//	   ecRightIndex= (short)(eLeftIndex+1);
//	 }	

// 
//	 

///*************************************��ģ��*************************************/
// 
//	fuzzy_PID->dKp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
//   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
//   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
//   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
// 
//	fuzzy_PID->dKi = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
//   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
//   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
//   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

// 
//	fuzzy_PID->dKd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
//   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
//   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
//   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
// 
//}

//float FuzzyPID_Calc(FuzzyPID *P)
//{
//	
//	  P->LastError = P->PreError;
//	  
//	  if((fabs(P->PreError)< P->DeadZone ))   //��������
//		{
//			P->PreError = 0.0f;			
//		}
//		else
//		{
//			P->PreError = P->SetPoint - P->ActualValue;
//		}
//		
//		fuzzy(P);      //ģ������  kp,ki,kd   �β�1��ǰ���β�2ǰ�����Ĳ�ֵ
//	
//    float Kp = P->Kp0 + P->dKp , Ki = P->Ki0 + P->dKi , Kd = P->Kd0 + P->dKd ;   //PID��ģ��
////	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //��PD��ģ��
////	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //��P��ģ��

//		
//		
//		      //΢������
//		float DM = Kd*(P->Out - P->Out_last);   //΢������	
//         //���ٻ���
//    if(fabs(P->PreError) < P->I_L )			
//		{
//	       //���λ���
//		P->SumError += (P->PreError+P->LastError)/2;    
//		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
//		}
//		 else if( fabs(P->PreError) < P->I_U )
//		{
//	       //���λ���
//		P->SumError += (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);    
//		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
//		}
//			
//		P->POut = Kp * P->PreError;
//		
//		P->IOut = Ki * P->SumError;
//		    
//		    //����ȫ΢��
//		P->DOut_last = P->DOut; 
//		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
//		
//		P->Out_last  = P->Out;
//		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
//		
//    return P->Out;                             

//}


