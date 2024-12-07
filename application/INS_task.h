#ifndef INS_TASK
#define INS_TASK

#include "main.h"
#include "BMI088driver.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)   

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2


#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2


#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.1f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 1.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

void ins_task(void const * argument);
 fp32* get_INS_angle(uint8_t offset);
const bmi088_real_data_t* get_INS_data_point(void);
extern bmi088_real_data_t bmi088_real_data;

extern fp32 bmi088_radians[3];
fp32 * get_INS_pitch_to_minpc(void);


#endif
