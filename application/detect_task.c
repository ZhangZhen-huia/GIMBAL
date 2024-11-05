#include "detect_task.h"


#if INCLUDE_uxTaskGetStackHighWaterMark      /* ��ȡ�����ջ��ʷʣ����Сֵ */
uint32_t detect_task_stack;
#endif

error_t error_list[ERROR_LIST_LENGHT + 1];


void detect_task(void const * argument)
{
	 static uint32_t system_time;
   system_time = xTaskGetTickCount();//��ȡ�� vTaskStartScheduler ��������� tick ��
	
    //init,��ʼ��
    detect_init(system_time);
    //wait a time.����һ��ʱ��
    vTaskDelay(DETECT_TASK_INIT_TIME);
	
	while(1)
	{
		static uint8_t error_num_display = 0;
    system_time = xTaskGetTickCount();//��ȡ�� vTaskStartScheduler ��������� tick ��
		error_num_display = ERROR_LIST_LENGHT;//�����б������
	  error_list[ERROR_LIST_LENGHT].is_lost = 0;
    error_list[ERROR_LIST_LENGHT].error_exist = 0;
		for (int i = 0; i < ERROR_LIST_LENGHT; i++)
		{
			//disable, continue
      //δʹ�ܣ�����
      if (error_list[i].enable == 0)
        {
           continue;
        }
				
				//judge offline.�жϵ���
			if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
        {
            if (error_list[i].error_exist == 0)//��ǰû�д������
						{
                 //record error and time
                 //�ͼ�¼�����Լ�����ʱ��
                 error_list[i].is_lost = 1;
                 error_list[i].error_exist = 1;
                 error_list[i].lost_time = system_time;
             }
             //judge the priority,save the highest priority ,
             //�жϴ������ȼ��� �������ȼ���ߵĴ�����
             if (error_list[i].priority > error_list[error_num_display].priority)//��ǰ�Ĵ������ȼ�����֮ǰ�Լ�������������ȼ�
             {
                 error_num_display = i;//�ͱ��浱ǰ�Ĵ������ȼ�
             }
             
							/****������***/
             error_list[ERROR_LIST_LENGHT].is_lost = 1;
             error_list[ERROR_LIST_LENGHT].error_exist = 1;
						 
             //if solve_lost_fun != NULL, run it
             //����ṩ������������н������
             if (error_list[i].solve_lost_fun != NULL)
             {
                 error_list[i].solve_lost_fun();
             }
         }
			  else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //just online, maybe unstable, only record
                //�ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
                error_list[i].is_lost = 0;//����ʧ
                error_list[i].error_exist = 1;//��¼�������
            }
				else
            {
                error_list[i].is_lost = 0;
                //�ж��Ƿ�������ݴ���
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
                //����Ƶ��
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



static void detect_init(uint32_t time)
{
	 //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� 
    uint16_t set_item[ERROR_LIST_LENGHT][3] ={30, 40, 15};
	  for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
			error_list[i].set_offline_time = set_item[i][0];//��������ʱ��
			error_list[i].set_online_time = set_item[i][1];//�����ȶ�����ʱ��
			error_list[i].priority = set_item[i][2];//���ȼ� 
			
			/*��������������ʱ����*/
			error_list[i].data_is_error_fun = NULL;
			error_list[i].solve_lost_fun = NULL;
			error_list[i].solve_data_error_fun = NULL;
			
			error_list[i].enable = 1;//ʹ��
			error_list[i].error_exist = 0;
			error_list[i].is_lost = 0;
			error_list[i].data_is_error = 0;
			error_list[i].frequency = 0.0f;
			error_list[i].new_time = time;
			error_list[i].last_time = time;
			error_list[i].lost_time = time;
			error_list[i].work_time = time;
		}
		error_list[DBUS_TOE].data_is_error_fun = RC_data_is_error;
    error_list[DBUS_TOE].solve_lost_fun = slove_RC_lost;
    error_list[DBUS_TOE].solve_data_error_fun = slove_data_error;
}


/*Ϊ"toe"�Ĵ����Ƿ���ڣ�1��ʾ���ڣ����Ǵ��ڣ���᷵��true�����򷵻�false��*/
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          ��¼ʱ��
  * @param[in]      toe:�豸Ŀ¼
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)//������
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;//����ʱ����ڵ�ǰʱ��
    }
    
    if (error_list[toe].data_is_error_fun != NULL)//�д������ݴ�����
    {
        if (error_list[toe].data_is_error_fun())//�ж������Ƿ���ڴ���-����
        {
            error_list[toe].error_exist = 1;//�������
            error_list[toe].data_is_error = 1;//���ݴ������

            if (error_list[toe].solve_data_error_fun != NULL)//���ڱ������ݴ�����
            {
                error_list[toe].solve_data_error_fun();//����
            }
        }
        else//���ݲ����ڴ���
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else//û���ж����ݴ�����
    {
        error_list[toe].data_is_error = 0;
    }
}







