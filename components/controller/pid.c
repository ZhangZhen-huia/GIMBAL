/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
	*PID限幅
	*积分限幅，PID总输出限幅
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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		/*PID模式选择和三项输出*/
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		/*PID输出限幅和积分限幅*/
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		/*先清除PID*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
/*Kp*err+Ki*err的积分+Kd*err的微分*/
/*对误差加了低通滤波*/
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
       //不做处理
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//目标值
    pid->fdb = ref;//返回值
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//本次误差
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//比例项：Kp*误差
			
        pid->Iout += pid->Ki * pid->error[0];//积分项：Ki*误差积分
			
        pid->Dbuf[2] = pid->Dbuf[1];//误差之差
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
        pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
        LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
        LimitMax(pid->out, pid->max_out);//PID输出限幅
    }
    else if (pid->mode == PID_DELTA)//差分PID
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
	else if (pid->mode == PID_DISTENCE) //ref为当前走的距离 set为目标距离
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
		pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
		LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
		LimitMax(pid->out, pid->max_out);//PID输出限幅
	}
    return pid->out;
}


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;//清除误差
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;//清除误差之差
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;//清除三项输出
    pid->fdb = pid->set = 0.0f;//清除目标值
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
		//过零处理
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


//过零函数
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


//前馈PID
float K_FF_Cal_shoot(pid_type_def *pid, float ref, float set,float fabs_max,float fabs_min)
{
	static TickType_t Real_Time,Last_Time,dt;
	float range,rat;
	// 目标值的导数，即目标值的变化率，dt 是时间间隔
  float aim_derivative = 0;
	Real_Time = xTaskGetTickCount();
	if(Last_Time != 0) //第一次不计算动态
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
    // 计算静态前馈项
    float K_ff_static = pid->K_ff_static; // 静态前馈增益
		
		if(set != fabs_min)
		{
			if(set > 0)
				pid->feedforward_static_out = 1.0f*K_ff_static * (fabs_max-my_fabs(set))*rat;
			else if(set<0)
				pid->feedforward_static_out = -1.0f*K_ff_static * (fabs_max-my_fabs(set))*rat;
		}
		else if(set == fabs_min)
			pid->feedforward_static_out = 0;

		
		
    // 计算动态前馈项
    float K_ff_dynamic = pid->K_ff_dynamic; // 动态前馈增益
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
       //不做处理
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//目标值
    pid->fdb = ref;//返回值
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//本次误差
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//比例项：Kp*误差
			
        pid->Iout += pid->Ki * pid->error[0];//积分项：Ki*误差积分
			
        pid->Dbuf[2] = pid->Dbuf[1];//误差之差
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
        pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
        LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
        LimitMax(pid->out, pid->max_out);//PID输出限幅
    }
    else if (pid->mode == PID_DELTA)//差分PID
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
	else if (pid->mode == PID_DISTENCE) //ref为当前走的距离 set为目标距离
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
		pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
		LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
		LimitMax(pid->out, pid->max_out);//PID输出限幅
	}
    return (pid->out+pid->feedforward_dynamic_out+pid->feedforward_static_out); //这里不直接加是因为不影响本身pid的out值便于下一次pid计算，前馈与pid是互相独立的
}

//前馈PID
float K_FF_Cal(pid_type_def *pid, float ref, float set)
{
	static TickType_t Real_Time,Last_Time,dt;
	// 目标值的导数，即目标值的变化率，dt 是时间间隔
    float aim_derivative = 0;
	Real_Time = xTaskGetTickCount();
	if(Last_Time != 0) //第一次不计算动态
	{
		dt = Real_Time - Last_Time;
	}
	Last_Time = Real_Time;
    if (dt > 0)
		{
        aim_derivative = (set - pid->last_aim) / dt;
    }
    pid->last_aim = set;

    // 计算静态前馈项
    float K_ff_static = pid->K_ff_static; // 静态前馈增益
    pid->feedforward_static_out = K_ff_static * set;

    // 计算动态前馈项
    float K_ff_dynamic = pid->K_ff_dynamic; // 动态前馈增益
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
       //不做处理
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//目标值
    pid->fdb = ref;//返回值
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//本次误差
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//比例项：Kp*误差
			
        pid->Iout += pid->Ki * pid->error[0];//积分项：Ki*误差积分
			
        pid->Dbuf[2] = pid->Dbuf[1];//误差之差
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
        pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
        LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
        LimitMax(pid->out, pid->max_out);//PID输出限幅
    }
    else if (pid->mode == PID_DELTA)//差分PID
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
	else if (pid->mode == PID_DISTENCE) //ref为当前走的距离 set为目标距离
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
		pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
		LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
		LimitMax(pid->out, pid->max_out);//PID输出限幅
	}
    return (pid->out+pid->feedforward_dynamic_out+pid->feedforward_static_out); //这里不直接加是因为不影响本身pid的out值便于下一次pid计算，前馈与pid是互相独立的
}


//前馈PID初始化
void K_FF_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,float K_ff_static,float K_ff_dynamic)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		/*PID模式选择和三项输出*/
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		/*PID输出限幅和积分限幅*/
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		/*先清除PID*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
	pid->K_ff_static=K_ff_static;
	pid->K_ff_dynamic=K_ff_dynamic;
}



///*********模糊pid部分*/

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

// //关键算法
//void fuzzy( FuzzyPID*  fuzzy_PID)
//{
//     float e = fuzzy_PID ->PreError/ fuzzy_PID->stair;
//	   float ec = (fuzzy_PID ->Out - fuzzy_PID ->Out_last) / fuzzy_PID->stair;
//     short etemp,ectemp;
//     float eLefttemp,ecLefttemp;    //隶属度
//     float eRighttemp ,ecRighttemp; 
// 
//     short eLeftIndex,ecLeftIndex;  //标签
//     short eRightIndex,ecRightIndex;

//	  //模糊化
//     if(e>=PL)
//			 etemp=PL;//超出范围
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
//		 //计算E隶属度
//				eRighttemp= 0 ;    //右溢出
//				eLefttemp= 1 ;
//			
//     //计算标签
//	   eLeftIndex = 6 ;      
//	   eRightIndex= 6 ;
//			
//		}else if( etemp == 2*NL )
//    {

//			//计算E隶属度
//				eRighttemp = 1;    //左溢出
//				eLefttemp = 0;
//	
//     //计算标签
//	   eLeftIndex = 0 ;       
//	   eRightIndex = 0 ;
//			
//		}	else 
//    {

//			//计算E隶属度
//				eRighttemp=(e-etemp);  //线性函数作为隶属函数
//				eLefttemp=(1- eRighttemp);
//			
//     //计算标签
//	   eLeftIndex =(short) (etemp-NL);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
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
//    //计算EC隶属度		 
//		 ecRighttemp= 0 ;      //右溢出
//		 ecLefttemp= 1 ;
//			
//		 ecLeftIndex = 6 ;  
//	   ecRightIndex = 6 ;	 
//	 
//	 } else if( ectemp == 2*NL)
//	 {
//    //计算EC隶属度		 
//		 ecRighttemp= 1 ;
//		 ecLefttemp= 0 ;
//			
//		 ecLeftIndex = 0 ;  
//	   ecRightIndex = 0 ;	 	 
//	 }else
//	 {
//    //计算EC隶属度		 
//		 ecRighttemp=(ec-ectemp);
//		 ecLefttemp=(1- ecRighttemp);
//			
//		 ecLeftIndex =(short) (ectemp-NL);  
//	   ecRightIndex= (short)(eLeftIndex+1);
//	 }	

// 
//	 

///*************************************反模糊*************************************/
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
//	  if((fabs(P->PreError)< P->DeadZone ))   //死区控制
//		{
//			P->PreError = 0.0f;			
//		}
//		else
//		{
//			P->PreError = P->SetPoint - P->ActualValue;
//		}
//		
//		fuzzy(P);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值
//	
//    float Kp = P->Kp0 + P->dKp , Ki = P->Ki0 + P->dKi , Kd = P->Kd0 + P->dKd ;   //PID均模糊
////	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //仅PD均模糊
////	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //仅P均模糊

//		
//		
//		      //微分先行
//		float DM = Kd*(P->Out - P->Out_last);   //微分先行	
//         //变速积分
//    if(fabs(P->PreError) < P->I_L )			
//		{
//	       //梯形积分
//		P->SumError += (P->PreError+P->LastError)/2;    
//		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
//		}
//		 else if( fabs(P->PreError) < P->I_U )
//		{
//	       //梯形积分
//		P->SumError += (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);    
//		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
//		}
//			
//		P->POut = Kp * P->PreError;
//		
//		P->IOut = Ki * P->SumError;
//		    
//		    //不完全微分
//		P->DOut_last = P->DOut; 
//		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
//		
//		P->Out_last  = P->Out;
//		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
//		
//    return P->Out;                             

//}


