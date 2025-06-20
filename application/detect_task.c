#include "detect_task.h"
#include "cmsis_os.h"


/**
	* @brief          init error_list, assign  offline_time, online_time, priority.
	* @param[in]      time: system time
	* @retval         none
	*/
/**
	* @brief          初始化error_list,赋值 offline_time, online_time, priority
	* @param[in]      time:系统时间
	* @retval         none
	*/
static void detect_init(uint32_t time);




error_t error_list[ERROR_LIST_LENGHT + 1];


#if INCLUDE_uxTaskGetStackHighWaterMark      /* 获取任务堆栈历史剩余最小值 */
uint32_t detect_task_stack;
#endif


/**
	* @brief          detect task
	* @param[in]      pvParameters: NULL
	* @retval         none
	*/
/**
	* @brief          检测任务
	* @param[in]      pvParameters: NULL
	* @retval         none
	*/
void detect_task(void const *pvParameters)
{
		static uint32_t system_time;
		system_time = xTaskGetTickCount();//获取自 vTaskStartScheduler 被调用起的 tick 数
		//init,初始化
		detect_init(system_time);
		//wait a time.空闲一段时间
		vTaskDelay(DETECT_TASK_INIT_TIME);

		while (1)
		{
			
				static uint8_t error_num_display = 0;
				system_time = xTaskGetTickCount();

				error_num_display = ERROR_LIST_LENGHT;
				error_list[ERROR_LIST_LENGHT].is_lost = 0;
				error_list[ERROR_LIST_LENGHT].error_exist = 0;

				for (int i = 0; i < ERROR_LIST_LENGHT; i++)
				{
						//disable, continue
						//未使能，跳过
						if (error_list[i].enable == 0)
						{
								continue;
						}

						//judge offline.判断掉线
						if (system_time - error_list[i].new_time > error_list[i].set_offline_time)//
						{
								if (error_list[i].error_exist == 0)
								{
										//record error and time
										//记录错误以及掉线时间
										error_list[i].is_lost = 1;
										error_list[i].error_exist = 1;
										error_list[i].lost_time = system_time;
								}
								//judge the priority,save the highest priority ,
								//判断错误优先级， 保存优先级最高的错误码
								if (error_list[i].priority > error_list[error_num_display].priority)
								{
										error_num_display = i;
								}
								

								error_list[ERROR_LIST_LENGHT].is_lost = 1;
								error_list[ERROR_LIST_LENGHT].error_exist = 1;
								//if solve_lost_fun != NULL, run it
								//如果提供解决函数，运行解决函数
								if (error_list[i].solve_lost_fun != NULL)
								{
										error_list[i].solve_lost_fun();
								}
						}
						else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
						{
								//just online, maybe unstable, only record
								//刚刚上线，可能存在数据不稳定，只记录不丢失，
								error_list[i].is_lost = 0;//不丢失
								error_list[i].error_exist = 1;//记录错误存在
						}
						else
						{
								error_list[i].is_lost = 0;
								//判断是否存在数据错误
								//judge if exist data error
								if (error_list[i].data_is_error != NULL)
								{
										error_list[i].error_exist = 1;
								}
								else
								{
										error_list[i].error_exist = 0;
								}
								//calc frequency
								//计算频率
								if (error_list[i].new_time > error_list[i].last_time)
								{
										error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
								}
						}
				}

				vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
				detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
		}
}


/**
	* @brief          get toe error status
	* @param[in]      toe: table of equipment
	* @retval         true (eror) or false (no error)
	*/
/**
	* @brief          获取设备对应的错误状态
	* @param[in]      toe:设备目录
	* @retval         true(错误) 或者false(没错误)
	*/
bool_t toe_is_error(uint8_t toe)
{
		return (error_list[toe].error_exist == 1);
}

/**
	* @brief          record the time
	* @param[in]      toe: table of equipment
	* @retval         none
	*/
/**
	* @brief          记录时间
	* @param[in]      toe:设备目录
	* @retval         none
	*/
void detect_hook(uint8_t toe)
{
		error_list[toe].last_time = error_list[toe].new_time;
		error_list[toe].new_time = xTaskGetTickCount();
		
		if (error_list[toe].is_lost)
		{
				error_list[toe].is_lost = 0;
				error_list[toe].work_time = error_list[toe].new_time;
		}
		
		if (error_list[toe].data_is_error_fun != NULL)
		{
				if (error_list[toe].data_is_error_fun())
				{
						error_list[toe].error_exist = 1;
						error_list[toe].data_is_error = 1;

						if (error_list[toe].solve_data_error_fun != NULL)
						{
								error_list[toe].solve_data_error_fun();
						}
				}
				else
				{
						error_list[toe].data_is_error = 0;
				}
		}
		else
		{
				error_list[toe].data_is_error = 0;
		}
}

/**
	* @brief          get error list
	* @param[in]      none
	* @retval         the point of error_list
	*/
/**
	* @brief          得到错误列表
	* @param[in]      none
	* @retval         error_list的指针
	*/
const error_t *get_error_list_point(void)
{
		return error_list;
}

//extern void OLED_com_reset(void);
// 
static void detect_init(uint32_t time)
{
		//设置离线时间，上线稳定工作时间，优先级 
		uint16_t set_item[ERROR_LIST_LENGHT][3] =
				{
						{30, 40, 15},   //SBUS
            {10, 10, 11},   //yaw
						{10, 10, 10},   //pitch
            {10, 10, 10},    //fric_l
            {10, 10, 10},     //fric_r
						{5, 10, 11},    //AIMBOT_TOE
//            {10, 10, 10},     //Chassis
//            {10, 10, 12},   //trigger
//            {2, 3, 7},      //board gyro
//            {5, 5, 7},      //board accel
//            {40, 200, 7},   //board mag
            {500, 100, 15},  //referee
//            {10, 10, 7},    //rm imu
//            {100, 100, 1},  //oled
				};

		for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
		{
				error_list[i].set_offline_time = set_item[i][0];
				error_list[i].set_online_time = set_item[i][1];
				error_list[i].priority = set_item[i][2];
				error_list[i].data_is_error_fun = NULL;
				error_list[i].solve_lost_fun = NULL;
				error_list[i].solve_data_error_fun = NULL;

				error_list[i].enable = 1;
				error_list[i].error_exist = 1;
				error_list[i].is_lost = 1;
				error_list[i].data_is_error = 1;
				error_list[i].frequency = 0.0f;
				error_list[i].new_time = time;
				error_list[i].last_time = time;
				error_list[i].lost_time = time;
				error_list[i].work_time = time;
		}

//    error_list[OLED_TOE].data_is_error_fun = NULL;
//    error_list[OLED_TOE].solve_lost_fun = OLED_com_reset;
//    error_list[OLED_TOE].solve_data_error_fun = NULL;

//    error_list[DBUS_TOE].dataIsErrorFun = RC_data_is_error;
//    error_list[DBUS_TOE].solveLostFun = slove_RC_lost;
//    error_list[DBUS_TOE].solveDataErrorFun = slove_data_error;

}
