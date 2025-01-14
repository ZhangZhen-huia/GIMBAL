#ifndef __REFEREE_H
#define __REFEREE_H

#include "main.h"

#include "user_lib.h"

//定义接收发送BUFFER数据长度
#define RS_RX_BUF_NUM  512u
#define RS_TX_BUF_NUM  128u


#define HERO


// 定义各个比特位的位置
#define CONFIRM_RESURRECTION_BIT          (0)
#define CONFIRM_IMMEDIATE_RESURRECTION_BIT (1)
#define EXCHANGE_AMMO_VALUE_BITS         (2)
#define REMOTE_AMMO_EXCHANGE_REQUEST_COUNT_BITS (13)
#define REMOTE_HEALTH_EXCHANGE_REQUEST_COUNT_BITS (17)


//每个设备ID号枚举
typedef enum
{
	Hero_R         = 1,
	Engineer_R     = 2,
	Standard_1_R   = 3,
	Standard_2_R   = 4,
	Standard_3_R   = 5,
	Aerial_R       = 6,	//空中
	Sentry_R       = 7,	//哨兵
	Darts_R		   = 8,	//飞镖
	Radar_R		   = 9,	//雷达
	

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



//判断客户端
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



/*----------------------裁判系统数据结构体声明----------------------*/


typedef struct  //比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
{
	/*1：RoboMaster 机甲大师赛；
	  2：RoboMaster 机甲大师单项赛；
	  3：ICRA RoboMaster 人工智能挑战赛；
		4：联盟赛3V3；
		5：联盟赛1V1：*/
	uint8_t  game_type : 4;
	/*0：未开始比赛；
	  1：准备阶段；
	  2：自检阶段；
	  3：5s倒计时；
	  4：对战中；
	  5：比赛结算中；*/
	uint8_t  game_progress : 4;

	uint16_t stage_remain_time;//当前阶段剩余时间，单位s
	uint64_t SyncTimeStamp; 
} ext_game_status_t;

		

typedef struct //比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
{
	uint8_t winner;//0 平局 1 红方胜利 2 蓝方胜利
	
} ext_game_result_t;


		
typedef __packed struct //机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
{
	uint16_t red_hero_HP ;        //红方英雄血量
	uint16_t red_engineer_HP;     //红方工程血量
	uint16_t red_standard_1_HP;   //红方步兵1血量
	uint16_t red_standard_2_HP;   //红方步兵2血量
	uint16_t red_standard_3_HP;   //红方步兵3血量
	uint16_t red_sentry_HP;       //红方哨兵血量
	uint16_t red_outpost_HP;      //红方前哨站血量
	uint16_t red_base_HP;         //红方基地血量
	
	uint16_t blue_hero_HP ;       //蓝方英雄血量
	uint16_t blue_engineer_HP;    //蓝方工程血量
	uint16_t blue_standard_1_HP;  //蓝方步兵1血量
	uint16_t blue_standard_2_HP;  //蓝方步兵2血量
	uint16_t blue_standard_3_HP;  //蓝方步兵3血量
	uint16_t blue_sentry_HP;      //蓝方哨兵血量
	uint16_t blue_outpost_HP;     //蓝方前哨站血量
	uint16_t blue_base_HP;        //蓝方基地血量
	
} ext_game_robot_HP_t;
		

typedef struct //飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。
{
	/*发射飞镖的队伍：
	1：红方飞镖
	2：蓝方飞镖*/
	uint8_t dart_belong;
	
	//发射时的剩余比赛时间
	uint16_t stage_remaining_time;
	
} ext_dart_status_t;
	
	
typedef struct //人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz周期发送，发送范围：所有机器人。
{
	//状态信息
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
	//剩余弹量
	uint16_t red1_bullet_left;
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
}ext_ICRA_buff_debuff_zone_status_t;
	
	
typedef struct //场地事件数据：0x0101。发送频率：1Hz周期发送，发送范围：己方机器人。
{
    /*己方补血站状态;
			1为已经占领
    */
	uint32_t blood_station1 : 1;
	uint32_t blood_station2 : 1;
	uint32_t blood_station3 : 1;
	
    /* 己方能量机关状态：
     1为已经激活
    */
	uint32_t shocks : 1;
	uint32_t small_buff_state : 1;
	uint32_t big_buff_state   : 1;
	
    /*己方基地虚拟护盾状态
	1为基地有虚拟护盾血量
	0为基地无虚拟护盾血量
     */
	uint32_t Base_shield : 1;
		/*高地占领状态：
			1为已经占领
		*/
	uint32_t annular_highland: 1;
	uint32_t trapezoidal_highland1: 1;
	uint32_t trapezoidal_highland2: 1;
	
	uint32_t  :22;
	
} ext_event_data_t;
		


typedef struct //补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
{

	//补给站口 ID：1：1 号补给口；2：2 号补给口
	uint8_t supply_projectile_id;

	/*补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程
	机器人补弹，3/4/5 为红方步兵机器人补弹，101 为蓝方英雄机器人补弹，102 为蓝方
	工程机器人补弹，103/104/105 为蓝方步兵机器人补弹*/
	uint8_t supply_robot_id;

	//出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
	uint8_t supply_projectile_step;

	//补弹数量：50：50 颗子弹；100：100 颗子弹；150：150 颗子弹；200：200 颗子弹。
	uint8_t supply_projectile_num;
	
} ext_supply_projectile_action_t;
	
	


typedef struct //裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
{
	//警告等级
	uint8_t level; 
	
	/*犯规机器人ID：
	1级以及5级警告时，机器人ID为0
	二三四级警告时，机器人ID为犯规机器人ID*/
	uint8_t foul_robot_id;
   uint8_t count; 
}ext_referee_warning_t;
	
	

typedef struct //飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz周期发送，发送范围：己方机器人。
{
	//15s 倒计时
	uint8_t dart_remaining_time;
  uint16_t dart_info; 
}ext_dart_remaining_time_t;
	
	

typedef struct //比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。
{
	/*机器人 ID：
	1：红方英雄机器人；
	2：红方工程机器人；
	3/4/5：红方步兵机器人；
	6：红方空中机器人；
	7：红方哨兵机器人；
	8：红方飞镖机器人；
	9：红方雷达站；
	101：蓝方英雄机器人；
	102：蓝方工程机器人；
	103/104/105：蓝方步兵机器人；
	106：蓝方空中机器人；
	107：蓝方哨兵机器人。
	108：蓝方飞镖机器人；
	109：蓝方雷达站。*/
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
	
  uint16_t shooter_cooling_rate; //枪口热量每秒冷却值
  uint16_t shooter_cooling_limit;  	//枪口热量限制
  uint16_t chassis_power_limit;  				//底盘功率限制
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 

	
} ext_game_robot_state_t;
	


typedef struct //实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。
{
	uint16_t chassis_volt;         //底盘输出电压   单位 毫伏
	uint16_t chassis_current;      //底盘输出电流   单位 毫安
	  fp32   chassis_power;        //底盘输出功率   单位 W 瓦
	uint16_t chassis_power_buffer; //底盘功率缓冲   单位 J 焦耳
	uint16_t shooter_id1_17mm_cooling_heat;        //17mm 枪口热量
	uint16_t shooter_id2_17mm_cooling_heat;					//17mm 枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;        //42mm 枪口热量
	
} ext_power_heat_data_t;
	
	

typedef struct //机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。
{
	float x;  //位置x坐标，单位m
	float y;  //位置y坐标，单位m
//	float z;  //位置z坐标，单位m
	float yaw; //位置枪口，单位度

}ext_game_robot_pos_t;
	
	

typedef struct  //机器人增益：0x0204。发送频率：1Hz周期发送，发送范围：单一机器人。
{
  uint8_t recovery_buff;  
  uint8_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
}ext_buff_t;
	

typedef struct //空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人
{
  uint8_t airforce_status; 
  uint8_t time_remain; 
}ext_aerial_robot_energy_t;



typedef struct //伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。
{
	/*bit 0-3：当血量变化类型为装甲伤害，代表装甲ID，
	其中数值为0-4号代表机器人的五个装甲片，其他血量变化类型，
	该变量数值为0。*/
	uint8_t armor_id : 4;
	
	/*血量变化类型
	0x0 装甲伤害扣血；
	0x1 模块掉线扣血；
	0x2 超射速扣血；
	0x3 超枪口热量扣血；
	0x4 超底盘功率扣血；
	0x5 装甲撞击扣血*/
	uint8_t hurt_type : 4;
	
} ext_robot_hurt_t;
	

typedef struct //实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人
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


typedef struct  //子弹剩余发射数：0x0208。发送频率：1Hz周期发送，空中机器人，哨兵机器人以及ICRA机器人主控发送，发送范围：单一机器人。
{ 
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;


typedef struct  //机器人RFID状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
{ 
	/*bit 0：基地增益点RFID状态；
	bit 1：高地增益点RFID状态；
	bit 2：能量机关激活点RFID状态；
	bit 3：飞坡增益点RFID状态；
	bit 4：前哨岗增益点RFID状态；
	bit 5：资源岛增益点RFID状态；
	bit 6：补血点增益点RFID状态；
	bit 7：工程机器人补血卡RFID状态；
	bit 8-31：保留
	RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果。*/	
	
   uint32_t rfid_status;
	
} ext_rfid_status_t;


typedef struct  //飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
{ 
	/*当前飞镖发射口的状态
	0：关闭；
	1：正在开启或者关闭中
	2：已经开启*/
	uint8_t dart_launch_opening_status; 
	/*飞镖的打击目标，默认为前哨站；
	1：前哨站；
	2：基地；*/
	uint8_t dart_attack_target; 
	/*切换打击目标时的比赛剩余时间，单位秒，从未切换默认为0。*/
	uint16_t target_change_time; 
	/*检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为0。*/
	uint8_t first_dart_speed; 
	/*检测到的第二枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
	uint8_t second_dart_speed; 
	/*检测到的第三枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
	uint8_t third_dart_speed;
	/*检测到的第四枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
	uint8_t fourth_dart_speed;
	/*最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为0。*/
	uint16_t last_dart_launch_time; 
	/*最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为0。*/
	uint16_t operate_launch_cmd_time;
	
}ext_dart_client_cmd_t;





typedef struct 
{ 
 uint16_t data_cmd_id; 
 uint16_t sender_id; 
 uint16_t receiver_id; 
 uint8_t user_data[2]; 
}robot_interaction_data_t;
/*-----------------小地图交互数据结构体声明-----------------*/

//客户端
typedef struct	//小地图接收信息标识：0x0305。最大接收频率：10Hz。
{
	//雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。

	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	float target_robot_receive;	
	
} ext_client_map_command_t;


//客户端下发（本方其他操作手客户端发送）
typedef	struct //小地图交互信息标识：0x0303		发送频率：触发发送
{
	float target_position_x;	//目标 x 位置坐标，单位 m
	float target_position_y;	//目标 y 位置坐标，单位 m
	float target_position_z;	//目标 z 位置坐标，单位 m
	
	uint8_t 	commd_keyboard;	//发送指令时，云台手按下的键盘信息
	uint16_t	target_robot_ID;//要作用的目标机器人 ID
	
}ext_robot_radar_command_t;


/*----------------------机器人交互数据结构体声明-------------------*/


//机器人与客户端交互中各功能对应的id
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


//typedef struct //交互数据 机器人间通信：0x0301。 内容ID:0x0200~0x02FF
//{ 
////   uint8_t data[];
//	
//} robot_interactive_data_t;



typedef struct //客户端删除图形 机器人间通信：0x0301。 内容ID: 0x0100
{ 
	/*0: 空操作；
	1: 删除图层；
	2: 删除所有；*/
	uint8_t operate_tpye; 
	
	//图层数：0~9
	uint8_t layer; 
	
} ext_client_custom_graphic_delete_t;


typedef struct //图形数据结构体
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

typedef struct //浮点数数据结构体
{ 
	uint8_t 	graphic_name[3]; 
	uint32_t 	operate_tpye:3; 
	uint32_t 	graphic_tpye:3; 
	uint32_t 	layer:4; 
	uint32_t 	color:4; 
	uint32_t 	start_angle:9; 	//字体大小
	uint32_t 	end_angle:9; 	//小数位有效个数
	uint32_t 	width:10; 
	
	uint32_t 	start_x:11; 
	uint32_t 	start_y:11; 
	int32_t 	graphic_float; 

} graphic_float_struct_t;


typedef struct  //客户端绘制一个图形 机器人间通信：0x0301。 内容ID: 0x0101
{ 
   graphic_data_struct_t grapic_data_struct;

} ext_client_custom_graphic_single_t;



typedef struct //客户端绘制二个图形 机器人间通信：0x0301。  内容ID: 0x0102
{ 
   graphic_data_struct_t grapic_data_struct[2]; 
 
} ext_client_custom_graphic_double_t;


typedef struct //客户端绘制五个图形 机器人间通信：0x0301。  内容ID: 0x0103
{ 
   graphic_data_struct_t grapic_data_struct[5]; 
 
} ext_client_custom_graphic_five_t;


typedef struct //客户端绘制七个图形 机器人间通信：0x0301。  内容ID: 0x0104
{ 
   graphic_data_struct_t grapic_data_struct[7]; 
	
} ext_client_custom_graphic_seven_t;


typedef struct //客户端绘制字符 机器人间通信：0x0301。 内容ID: 0x0110
{ 
   graphic_data_struct_t grapic_data_struct; 
   uint8_t data[30]; 
	
} ext_client_custom_character_t;




/*-----------------格式指针声明-----------------*/


//裁判系统整帧数据帧头格式指针声明

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


//0x0301数据段帧头格式指针说明

typedef struct 
{
	uint16_t  data_cmd_id; 
	uint16_t  sender_ID; 
	uint16_t  receiver_ID;
	
	 uint8_t  data_0;
	
} Data_frame_point_t;


/*-----------------裁判系统结构体声明-----------------*/




typedef struct 
{
	uint8_t  RS_rx_buf[2][RS_RX_BUF_NUM]; //原始接收数据
	uint8_t  RS_tx_buf[RS_TX_BUF_NUM];    //原始发送数据
	
	uint16_t  this_time_rx_len;            //当次接收到的数据长度

  RS_frame_point_t    *RS_frame_point;    //裁判系统帧头格式指针
  Data_frame_point_t  *Data_frame_point;  //数据段帧头格式指针
	
	remote_control_t                     Image_trans_remote;//图传数据	

}Referee_System_t;



#pragma pack(pop)

void  Referee_Sys_Init(void);      //裁判系统初始化
Referee_System_t *get_referee_data_point(void);  //获取裁判系统指针
void Referee_TX_send(uint32_t cmd,uint8_t *data, uint8_t num);	//发送图形数据到裁判系统
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
uint8_t get_robot_id(void);

extern Referee_System_t Referee_System;
#endif



