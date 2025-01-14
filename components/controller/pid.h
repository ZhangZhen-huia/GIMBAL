/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
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
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA,
	PID_DISTENCE,
};

enum DATA_MODE
{
    DATA_GYRO = 0,
    DATA_NORMAL,
};


typedef enum pid_Function_e
{
    NONE = 0X00,                        															//0000 0000 0000 0000
    Integral_Limit = 0x01,              															//0000 0000 0000 0001
    Derivative_On_Measurement = 0x02,   															//0000 0000 0000 0010
    Trapezoid_Intergral = 0x04,         															//0000 0000 0000 0100
    Proportional_On_Measurement = 0x08, 															//0000 0000 0000 1000
    OutputFilter = 0x10,                															//0000 0000 0001 0000
    ChangingIntegralRate = 0x20,        															//0000 0000 0010 0000
    DerivativeFilter = 0x40,            															//0000 0000 0100 0000
    ErrorHandle = 0x80,                 															//0000 0000 1000 0000
		ProportionalComponent_SecondOrder = 0x100													//0000 0001 0000 0000 
} PID_Function_e;


typedef struct __attribute__((packed))
{
    uint8_t mode;
		uint8_t data_mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;
		float S_Kp;
    float max_out;  //������
    float max_iout; //���������
		float SP_out;
		
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
		fp32 error_all;
		float K_ff_static;
		uint16_t function;
		float K_ff_dynamic;
		float last_aim;
		float feedforward_static_out;
		float feedforward_dynamic_out;
} pid_type_def;


/*********ģ��pid����*/
typedef struct
{
		float SetPoint;			//�趨Ŀ��ֵ
	
		float ActualValue;  //ʵ��ֵ

    float DeadZone;
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float DOut_last;    //��һ��΢�����
		float OutMax;       //�޷�
	  float Out;          //�����
		float Out_last;     //��һ�����
		
		float I_U;          //���ٻ�������
		float I_L;          //���ٻ�������
		
		float RC_DM;        //΢�������˲�ϵ��
		float RC_DF;        //����ȫ΢���˲�ϵ��
	
	  float Kp0;          //PID��ֵ
	  float Ki0;
  	float Kd0;
	
	  float dKp;          //PID�仯��
	  float dKi;
  	float dKd;
	
    float stair ;	      //��̬�����ݶ�   //0.25f
	  float Kp_stair;                      //0.015f
	  float Ki_stair;                      //0.0005f
	  float Kd_stair;                      //0.001f
	  
}FuzzyPID;

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
void PID_init(pid_type_def *pid, uint8_t mode,uint8_t data_mode, const float *PID, float max_out, float max_iout,uint16_t function);

/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);
float K_FF_Cal_shoot(pid_type_def *pid, float ref, float set,float fabs_max,float fabs_min);

fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set,uint16_t ecd_range);
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd, uint16_t ecd_range);
void K_FF_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,float K_ff_static,float K_ff_dynamic);
float K_FF_Cal(pid_type_def *pid, float ref, float set);

#endif
