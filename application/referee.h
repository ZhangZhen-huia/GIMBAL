#ifndef __REFEREE_H
#define __REFEREE_H

#include "main.h"

#include "user_lib.h"

//������շ���BUFFER���ݳ���
#define RS_RX_BUF_NUM  512u
#define RS_TX_BUF_NUM  128u


#define HERO


// �����������λ��λ��
#define CONFIRM_RESURRECTION_BIT          (0)
#define CONFIRM_IMMEDIATE_RESURRECTION_BIT (1)
#define EXCHANGE_AMMO_VALUE_BITS         (2)
#define REMOTE_AMMO_EXCHANGE_REQUEST_COUNT_BITS (13)
#define REMOTE_HEALTH_EXCHANGE_REQUEST_COUNT_BITS (17)


//ÿ���豸ID��ö��
typedef enum
{
	Hero_R         = 1,
	Engineer_R     = 2,
	Standard_1_R   = 3,
	Standard_2_R   = 4,
	Standard_3_R   = 5,
	Aerial_R       = 6,	//����
	Sentry_R       = 7,	//�ڱ�
	Darts_R		   = 8,	//����
	Radar_R		   = 9,	//�״�
	

	Hero_B         = 101,
	Engineer_B     = 102,
	Standard_1_B   = 103,
	Standard_2_B   = 104,
	Standard_3_B   = 105,
	Aerial_B       = 106,
	Sentry_B       = 107,
	Darts_B		   = 108,	
	Radar_B		   = 109,	

	Hero_R_Client         = 0x0101,
	Engineer_R_Client     = 0x0102,
	Standard_1_R_Client   = 0x0103,
	Standard_2_R_Client   = 0x0104,
	Standard_3_R_Client   = 0x0105,
	Aerial_R_Client       = 0x0106,

	Hero_B_Client         = 0x0165,
	Engineer_B_Client     = 0x0166,
	Standard_1_B_Client   = 0x0167,
	Standard_2_B_Client   = 0x0168,
	Standard_3_B_Client   = 0x0169,
	Aerial_B_Client       = 0x016A,
	
}device_ID_n; 



//�жϿͻ���
#ifdef HERO
#define MY_Client_ADD	0
#endif

#ifdef ENGIN
#define MY_Client_ADD	1
#endif

#ifdef STAND_3
#define MY_Client_ADD 	2
#endif

#ifdef STAND_4
#define MY_Client_ADD 	3
#endif

#ifdef STAND_5
#define MY_Client_ADD	4
#endif

#ifdef AERIAL
#define MY_Client_ADD	5
#endif


typedef enum
{
	NO_POWER =0,
	ONE_POWER,
	TWO_POWER,
	THREE_POWER,
	FOUR_POWER,
	FIVE_POWER,
	
}SuperPowerUI_n;



#pragma pack(push,1)



/*----------------------����ϵͳ���ݽṹ������----------------------*/


typedef struct  //����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
{
	/*1��RoboMaster ���״�ʦ����
	  2��RoboMaster ���״�ʦ��������
	  3��ICRA RoboMaster �˹�������ս����
		4��������3V3��
		5��������1V1��*/
	uint8_t  game_type : 4;
	/*0��δ��ʼ������
	  1��׼���׶Σ�
	  2���Լ�׶Σ�
	  3��5s����ʱ��
	  4����ս�У�
	  5�����������У�*/
	uint8_t  game_progress : 4;

	uint16_t stage_remain_time;//��ǰ�׶�ʣ��ʱ�䣬��λs
	uint64_t SyncTimeStamp; 
} ext_game_status_t;

		

typedef struct //����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�
{
	uint8_t winner;//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��
	
} ext_game_result_t;


		
typedef __packed struct //������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
{
	uint16_t red_hero_HP ;        //�췽Ӣ��Ѫ��
	uint16_t red_engineer_HP;     //�췽����Ѫ��
	uint16_t red_standard_1_HP;   //�췽����1Ѫ��
	uint16_t red_standard_2_HP;   //�췽����2Ѫ��
	uint16_t red_standard_3_HP;   //�췽����3Ѫ��
	uint16_t red_sentry_HP;       //�췽�ڱ�Ѫ��
	uint16_t red_outpost_HP;      //�췽ǰ��վѪ��
	uint16_t red_base_HP;         //�췽����Ѫ��
	
	uint16_t blue_hero_HP ;       //����Ӣ��Ѫ��
	uint16_t blue_engineer_HP;    //��������Ѫ��
	uint16_t blue_standard_1_HP;  //��������1Ѫ��
	uint16_t blue_standard_2_HP;  //��������2Ѫ��
	uint16_t blue_standard_3_HP;  //��������3Ѫ��
	uint16_t blue_sentry_HP;      //�����ڱ�Ѫ��
	uint16_t blue_outpost_HP;     //����ǰ��վѪ��
	uint16_t blue_base_HP;        //��������Ѫ��
	
} ext_game_robot_HP_t;
		

typedef struct //���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�
{
	/*������ڵĶ��飺
	1���췽����
	2����������*/
	uint8_t dart_belong;
	
	//����ʱ��ʣ�����ʱ��
	uint16_t stage_remaining_time;
	
} ext_dart_status_t;
	
	
typedef struct //�˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ�����л����ˡ�
{
	//״̬��Ϣ
	uint8_t F1_zone_status:1; 
	uint8_t F1_zone_buff_debuff_status:3; 
	uint8_t F2_zone_status:1; 
	uint8_t F2_zone_buff_debuff_status:3; 
	uint8_t F3_zone_status:1; 
	uint8_t F3_zone_buff_debuff_status:3; 
	uint8_t F4_zone_status:1; 
	uint8_t F4_zone_buff_debuff_status:3; 
	uint8_t F5_zone_status:1; 
	uint8_t F5_zone_buff_debuff_status:3; 
	uint8_t F6_zone_status:1; 
	uint8_t F6_zone_buff_debuff_status:3;
	//ʣ�൯��
	uint16_t red1_bullet_left;
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
}ext_ICRA_buff_debuff_zone_status_t;
	
	
typedef struct //�����¼����ݣ�0x0101������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ�����������ˡ�
{
    /*������Ѫվ״̬;
			1Ϊ�Ѿ�ռ��
    */
	uint32_t blood_station1 : 1;
	uint32_t blood_station2 : 1;
	uint32_t blood_station3 : 1;
	
    /* ������������״̬��
     1Ϊ�Ѿ�����
    */
	uint32_t shocks : 1;
	uint32_t small_buff_state : 1;
	uint32_t big_buff_state   : 1;
	
    /*�����������⻤��״̬
	1Ϊ���������⻤��Ѫ��
	0Ϊ���������⻤��Ѫ��
     */
	uint32_t Base_shield : 1;
		/*�ߵ�ռ��״̬��
			1Ϊ�Ѿ�ռ��
		*/
	uint32_t annular_highland: 1;
	uint32_t trapezoidal_highland1: 1;
	uint32_t trapezoidal_highland2: 1;
	
	uint32_t  :22;
	
} ext_event_data_t;
		


typedef struct //����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�
{

	//����վ�� ID��1��1 �Ų����ڣ�2��2 �Ų�����
	uint8_t supply_projectile_id;

	/*���������� ID��0 Ϊ��ǰ�޻����˲�����1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽����
	�����˲�����3/4/5 Ϊ�췽���������˲�����101 Ϊ����Ӣ�ۻ����˲�����102 Ϊ����
	���̻����˲�����103/104/105 Ϊ�������������˲���*/
	uint8_t supply_robot_id;

	//�����ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ�����
	uint8_t supply_projectile_step;

	//����������50��50 ���ӵ���100��100 ���ӵ���150��150 ���ӵ���200��200 ���ӵ���
	uint8_t supply_projectile_num;
	
} ext_supply_projectile_action_t;
	
	


typedef struct //���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�
{
	//����ȼ�
	uint8_t level; 
	
	/*���������ID��
	1���Լ�5������ʱ��������IDΪ0
	�����ļ�����ʱ��������IDΪ���������ID*/
	uint8_t foul_robot_id;
   uint8_t count; 
}ext_referee_warning_t;
	
	

typedef struct //���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ�����������ˡ�
{
	//15s ����ʱ
	uint8_t dart_remaining_time;
  uint16_t dart_info; 
}ext_dart_remaining_time_t;
	
	

typedef struct //����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
{
	/*������ ID��
	1���췽Ӣ�ۻ����ˣ�
	2���췽���̻����ˣ�
	3/4/5���췽���������ˣ�
	6���췽���л����ˣ�
	7���췽�ڱ������ˣ�
	8���췽���ڻ����ˣ�
	9���췽�״�վ��
	101������Ӣ�ۻ����ˣ�
	102���������̻����ˣ�
	103/104/105���������������ˣ�
	106���������л����ˣ�
	107�������ڱ������ˡ�
	108���������ڻ����ˣ�
	109�������״�վ��*/
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
	
  uint16_t shooter_cooling_rate; //ǹ������ÿ����ȴֵ
  uint16_t shooter_cooling_limit;  	//ǹ����������
  uint16_t chassis_power_limit;  				//���̹�������
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 

	
} ext_game_robot_state_t;
	


typedef struct //ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�
{
	uint16_t chassis_volt;         //���������ѹ   ��λ ����
	uint16_t chassis_current;      //�����������   ��λ ����
	  fp32   chassis_power;        //�����������   ��λ W ��
	uint16_t chassis_power_buffer; //���̹��ʻ���   ��λ J ����
	uint16_t shooter_id1_17mm_cooling_heat;        //17mm ǹ������
	uint16_t shooter_id2_17mm_cooling_heat;					//17mm ǹ������
	uint16_t shooter_id1_42mm_cooling_heat;        //42mm ǹ������
	
} ext_power_heat_data_t;
	
	

typedef struct //������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
{
	float x;  //λ��x���꣬��λm
	float y;  //λ��y���꣬��λm
//	float z;  //λ��z���꣬��λm
	float yaw; //λ��ǹ�ڣ���λ��

}ext_game_robot_pos_t;
	
	

typedef struct  //���������棺0x0204������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ����һ�����ˡ�
{
  uint8_t recovery_buff;  
  uint8_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
}ext_buff_t;
	

typedef struct //���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
{
  uint8_t airforce_status; 
  uint8_t time_remain; 
}ext_aerial_robot_energy_t;



typedef struct //�˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ�����ˡ�
{
	/*bit 0-3����Ѫ���仯����Ϊװ���˺�������װ��ID��
	������ֵΪ0-4�Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ�
	�ñ�����ֵΪ0��*/
	uint8_t armor_id : 4;
	
	/*Ѫ���仯����
	0x0 װ���˺���Ѫ��
	0x1 ģ����߿�Ѫ��
	0x2 �����ٿ�Ѫ��
	0x3 ��ǹ��������Ѫ��
	0x4 �����̹��ʿ�Ѫ��
	0x5 װ��ײ����Ѫ*/
	uint8_t hurt_type : 4;
	
} ext_robot_hurt_t;
	

typedef struct //ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ������
{
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed;  
} ext_shoot_data_t;


typedef struct
{
   int16_t mouse_x;
   int16_t mouse_y;
   int16_t mouse_z;
   uint8_t left_button_down;
   uint8_t right_button_down;
   uint16_t keyboard_value;
   uint16_t reserved;
}remote_control_t;


typedef struct  //�ӵ�ʣ�෢������0x0208������Ƶ�ʣ�1Hz���ڷ��ͣ����л����ˣ��ڱ��������Լ�ICRA���������ط��ͣ����ͷ�Χ����һ�����ˡ�
{ 
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;


typedef struct  //������RFID״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�
{ 
	/*bit 0�����������RFID״̬��
	bit 1���ߵ������RFID״̬��
	bit 2���������ؼ����RFID״̬��
	bit 3�����������RFID״̬��
	bit 4��ǰ�ڸ������RFID״̬��
	bit 5����Դ�������RFID״̬��
	bit 6����Ѫ�������RFID״̬��
	bit 7�����̻����˲�Ѫ��RFID״̬��
	bit 8-31������
	RFID ״̬����ȫ�����Ӧ������򴦷�״̬������з���ռ��ĸߵ�����㣬���ܻ�ȡ��Ӧ������Ч����*/	
	
   uint32_t rfid_status;
	
} ext_rfid_status_t;


typedef struct  //���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
{ 
	/*��ǰ���ڷ���ڵ�״̬
	0���رգ�
	1�����ڿ������߹ر���
	2���Ѿ�����*/
	uint8_t dart_launch_opening_status; 
	/*���ڵĴ��Ŀ�꣬Ĭ��Ϊǰ��վ��
	1��ǰ��վ��
	2�����أ�*/
	uint8_t dart_attack_target; 
	/*�л����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ�룬��δ�л�Ĭ��Ϊ0��*/
	uint16_t target_change_time; 
	/*��⵽�ĵ�һö�����ٶȣ���λ 0.1m/s/LSB, δ�����Ϊ0��*/
	uint8_t first_dart_speed; 
	/*��⵽�ĵڶ�ö�����ٶȣ���λ 0.1m/s/LSB��δ�����Ϊ0��*/
	uint8_t second_dart_speed; 
	/*��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB��δ�����Ϊ0��*/
	uint8_t third_dart_speed;
	/*��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB��δ�����Ϊ0��*/
	uint8_t fourth_dart_speed;
	/*���һ�εķ�����ڵı���ʣ��ʱ�䣬��λ�룬��ʼֵΪ0��*/
	uint16_t last_dart_launch_time; 
	/*���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��, ��ʼֵΪ0��*/
	uint16_t operate_launch_cmd_time;
	
}ext_dart_client_cmd_t;





typedef struct 
{ 
 uint16_t data_cmd_id; 
 uint16_t sender_id; 
 uint16_t receiver_id; 
 uint8_t user_data[2]; 
}robot_interaction_data_t;
/*-----------------С��ͼ�������ݽṹ������-----------------*/

//�ͻ���
typedef struct	//С��ͼ������Ϣ��ʶ��0x0305��������Ƶ�ʣ�10Hz��
{
	//�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ������

	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	float target_robot_receive;	
	
} ext_client_map_command_t;


//�ͻ����·����������������ֿͻ��˷��ͣ�
typedef	struct //С��ͼ������Ϣ��ʶ��0x0303		����Ƶ�ʣ���������
{
	float target_position_x;	//Ŀ�� x λ�����꣬��λ m
	float target_position_y;	//Ŀ�� y λ�����꣬��λ m
	float target_position_z;	//Ŀ�� z λ�����꣬��λ m
	
	uint8_t 	commd_keyboard;	//����ָ��ʱ����̨�ְ��µļ�����Ϣ
	uint16_t	target_robot_ID;//Ҫ���õ�Ŀ������� ID
	
}ext_robot_radar_command_t;


/*----------------------�����˽������ݽṹ������-------------------*/


//��������ͻ��˽����и����ܶ�Ӧ��id
typedef enum
{
	graphic_delete_id 		= 0x0100,
	graphic_single_id		= 0x0101,
	graphic_double_id		= 0x0102,
	graphic_five_id			= 0x0103,
	graphic_seven_id		= 0x0104,
	graphic_character_id	= 0x0110,
	interactive_id 			= 0x0301,
	
}client_communicate_ID_e;


//typedef struct //�������� �����˼�ͨ�ţ�0x0301�� ����ID:0x0200~0x02FF
//{ 
////   uint8_t data[];
//	
//} robot_interactive_data_t;



typedef struct //�ͻ���ɾ��ͼ�� �����˼�ͨ�ţ�0x0301�� ����ID: 0x0100
{ 
	/*0: �ղ�����
	1: ɾ��ͼ�㣻
	2: ɾ�����У�*/
	uint8_t operate_tpye; 
	
	//ͼ������0~9
	uint8_t layer; 
	
} ext_client_custom_graphic_delete_t;


typedef struct //ͼ�����ݽṹ��
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9; 
	uint32_t end_angle:9; 
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
	
} graphic_data_struct_t;

typedef struct //���������ݽṹ��
{ 
	uint8_t 	graphic_name[3]; 
	uint32_t 	operate_tpye:3; 
	uint32_t 	graphic_tpye:3; 
	uint32_t 	layer:4; 
	uint32_t 	color:4; 
	uint32_t 	start_angle:9; 	//�����С
	uint32_t 	end_angle:9; 	//С��λ��Ч����
	uint32_t 	width:10; 
	
	uint32_t 	start_x:11; 
	uint32_t 	start_y:11; 
	int32_t 	graphic_float; 

} graphic_float_struct_t;


typedef struct  //�ͻ��˻���һ��ͼ�� �����˼�ͨ�ţ�0x0301�� ����ID: 0x0101
{ 
   graphic_data_struct_t grapic_data_struct;

} ext_client_custom_graphic_single_t;



typedef struct //�ͻ��˻��ƶ���ͼ�� �����˼�ͨ�ţ�0x0301��  ����ID: 0x0102
{ 
   graphic_data_struct_t grapic_data_struct[2]; 
 
} ext_client_custom_graphic_double_t;


typedef struct //�ͻ��˻������ͼ�� �����˼�ͨ�ţ�0x0301��  ����ID: 0x0103
{ 
   graphic_data_struct_t grapic_data_struct[5]; 
 
} ext_client_custom_graphic_five_t;


typedef struct //�ͻ��˻����߸�ͼ�� �����˼�ͨ�ţ�0x0301��  ����ID: 0x0104
{ 
   graphic_data_struct_t grapic_data_struct[7]; 
	
} ext_client_custom_graphic_seven_t;


typedef struct //�ͻ��˻����ַ� �����˼�ͨ�ţ�0x0301�� ����ID: 0x0110
{ 
   graphic_data_struct_t grapic_data_struct; 
   uint8_t data[30]; 
	
} ext_client_custom_character_t;




/*-----------------��ʽָ������-----------------*/


//����ϵͳ��֡����֡ͷ��ʽָ������

typedef struct 
{
	struct
	{
		uint8_t SOF;
		uint16_t data_length;
		uint8_t seq;
		uint8_t CRC_8;	
		
	}frame_header;
		
	uint16_t cmd_id;
	
	uint8_t data_0;
		
} RS_frame_point_t;


//0x0301���ݶ�֡ͷ��ʽָ��˵��

typedef struct 
{
	uint16_t  data_cmd_id; 
	uint16_t  sender_ID; 
	uint16_t  receiver_ID;
	
	 uint8_t  data_0;
	
} Data_frame_point_t;


/*-----------------����ϵͳ�ṹ������-----------------*/




typedef struct 
{
	uint8_t  RS_rx_buf[2][RS_RX_BUF_NUM]; //ԭʼ��������
	uint8_t  RS_tx_buf[RS_TX_BUF_NUM];    //ԭʼ��������
	
	uint16_t  this_time_rx_len;            //���ν��յ������ݳ���

  RS_frame_point_t    *RS_frame_point;    //����ϵͳ֡ͷ��ʽָ��
  Data_frame_point_t  *Data_frame_point;  //���ݶ�֡ͷ��ʽָ��
	
	remote_control_t                     Image_trans_remote;//ͼ������	

}Referee_System_t;



#pragma pack(pop)

void  Referee_Sys_Init(void);      //����ϵͳ��ʼ��
Referee_System_t *get_referee_data_point(void);  //��ȡ����ϵͳָ��
void Referee_TX_send(uint32_t cmd,uint8_t *data, uint8_t num);	//����ͼ�����ݵ�����ϵͳ
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
uint8_t get_robot_id(void);

extern Referee_System_t Referee_System;
#endif



