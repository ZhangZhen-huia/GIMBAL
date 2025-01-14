#include "ins_task.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "bsp_spi.h"
#include "stm32f4xx_hal_spi.h"
#include "tim.h"
#include "bsp_buzzer.h"
uint8_t Bee_flag=1;
uint16_t psc = 1;
uint16_t pwm = 5000;

bmi088_real_data_t bmi088_real_data;

//static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad
fp32 gyro_offset[3];
fp32 bmi088_radians[3];

void AHRS_init(fp32 quat[4]);
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
//static void imu_temp_control(fp32 temp);
void imu_offset_accquier(fp32 cali_offset[3]);
void imu_offset_remove(fp32 gyro[3],fp32 cali_offset[3]);
void imu_turn(void);
fp32 imu_to_pc[1];

void INS_task(void const * argument)
{

	//bmi088��ʼ��
	while(BMI088_init())
	{
		osDelay(100);
	}
	
	//��������ٶȺͼ��ٶȣ��¶�
	BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
	
	PID_init(&imu_temp_pid, PID_POSITION, DATA_NORMAL,imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT,NONE);
	
	//��Ԫ����ʼ��
	AHRS_init(INS_quat);

	//���������������ٶȵ���Ư����ֵ
	imu_offset_accquier(gyro_offset);
	
	while(1)
	{
		
		//��ȡ�����ǡ����ٶȼơ��¶�����
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
		
		//�����������ݽ��в�������������Ư
		imu_offset_remove(bmi088_real_data.gyro,gyro_offset);
		
		//�¶ȿ���
//		imu_temp_control(bmi088_real_data.temp);
		
		//��Ԫ�����㣨���ᣩ
		MahonyAHRSupdateIMU(INS_quat,bmi088_real_data.gyro[0],bmi088_real_data.gyro[1],bmi088_real_data.gyro[2],bmi088_real_data.accel[0],bmi088_real_data.accel[1],bmi088_real_data.accel[2]);
		get_angle(INS_quat, bmi088_real_data.INS_angle + INS_YAW_ADDRESS_OFFSET, bmi088_real_data.INS_angle + INS_PITCH_ADDRESS_OFFSET, bmi088_real_data.INS_angle + INS_ROLL_ADDRESS_OFFSET);
		for(uint8_t i=0;i<3;i++)
		bmi088_radians[i] = bmi088_real_data.INS_angle[i];
		//ת���ɶ�
		for(uint8_t i=0;i<3;i++)
		{
			bmi088_real_data.INS_angle[i]*=57.2957795f;
		}		
		imu_to_pc[0] = bmi088_real_data.INS_angle[1];
		bmi088_real_data.INS_angle[1] *= -1.0f;
		
	//	CAN_cmd_INS(bmi088_real_data.INS_angle[0], -bmi088_real_data.INS_angle[1], bmi088_real_data.gyro[2], bmi088_real_data.gyro[1]);
		//��������Ϊ2ms������Ԫ�������еĽ���Ƶ���йأ�
			if(Bee_flag==1)
			{	
					Bee_flag=0;
					buzzer_on(psc, pwm);
					osDelay(500);
					buzzer_off();

			}
		osDelay(1);
	}
}

//�����¶�
//static void imu_temp_control(fp32 temp)
//{
//    uint16_t tempPWM;
//    static uint8_t temp_constant_time = 0;
//    if (first_temperate)
//    {
//        PID_calc(&imu_temp_pid, temp, 45.0f);
//        if (imu_temp_pid.out < 0.0f)
//        {
//            imu_temp_pid.out = 0.0f;
//        }
//        tempPWM = (uint16_t)imu_temp_pid.out;
//        IMU_temp_PWM(tempPWM);
//    }
//    else
//    {
//        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
//        //in beginning, max power
//        if (temp > 45.0f)
//        {
//            temp_constant_time++;
//            if (temp_constant_time > 200)
//            {
//                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
//                //
//                first_temperate = 1;
//                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
//            }
//        }

//        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
//    }
//}

//��ʼ����Ԫ��
//w����ʵ����xyz�����鲿
void AHRS_init(fp32 quat[4])
{
		quat[0] = 1.0f; //�� w ��������Ϊ 1.0����ʾû����ת������λ��Ԫ����������3D�ռ��еġ�����ת��״̬����
		quat[1] = 0.0f; //�� x ��������Ϊ 0.0����ʾ�� x ��û����ת��
		quat[2] = 0.0f; //�� y ��������Ϊ 0.0����ʾ�� y ��û����ת��
		quat[3] = 0.0f; //�� z ��������Ϊ 0.0����ʾ�� z ��û����ת��

}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

void imu_offset_accquier(fp32 cali_offset[3])
{
	uint16_t sap_num = 20000;//25000
	
	cali_offset[0]=0;
	cali_offset[1]=0;
	cali_offset[2]=0;

	for(uint16_t i = 0;i < sap_num; i++)
	{
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
		cali_offset[0] += bmi088_real_data.gyro[0];
		cali_offset[1] += bmi088_real_data.gyro[1];
		cali_offset[2] += bmi088_real_data.gyro[2];
		
	}
	cali_offset[0] /= sap_num;
	cali_offset[1] /= sap_num;
	cali_offset[2] /= sap_num;
		
}

void imu_offset_remove(fp32 gyro[3],fp32 cali_offset[3])
{
	gyro[0] -= cali_offset[0];
	gyro[1] -= cali_offset[1];
	gyro[2] -= cali_offset[2];
}

//���������ǽǶ���Ϣ
  fp32* get_INS_angle(uint8_t offset)
{
	return &bmi088_real_data.INS_angle[(offset & 0x03)];
}

fp32 * get_INS_pitch_to_minpc(void)
{
	return &imu_to_pc[0];

}
const  bmi088_real_data_t* get_INS_data_point(void)
{
	return &bmi088_real_data;
}


