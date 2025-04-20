/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "main.h"
#include "bsp_rc.h"




//遥控器死区
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                         						 \
            (output) = 0;                                \
        }                                                \
    }													 \

		
		
		
#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_SW_RIGHT   1
#define RC_SW_LEFT  	0
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/*快捷编写按键宏*/
#define ISKEYDOWN_W(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_W)		== KEY_PRESSED_OFFSET_W ? 1 : 0)
#define ISKEYDOWN_S(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_S)		== KEY_PRESSED_OFFSET_S ? 1 : 0)
#define ISKEYDOWN_A(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_A)		== KEY_PRESSED_OFFSET_A ? 1 : 0)
#define ISKEYDOWN_D(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_D)		== KEY_PRESSED_OFFSET_D ? 1 : 0)
#define ISKEYDOWN_SHIFT(RC_Data_Point)  ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_SHIFT)	== KEY_PRESSED_OFFSET_SHIFT ? 1 : 0)
#define ISKEYDOWN_CTRL(RC_Data_Point)   ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_CTRL)	== KEY_PRESSED_OFFSET_CTRL ? 1 : 0)
#define ISKEYDOWN_Q(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_Q)		== KEY_PRESSED_OFFSET_Q ? 1 : 0)
#define ISKEYDOWN_E(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_E)		== KEY_PRESSED_OFFSET_E ? 1 : 0)
#define ISKEYDOWN_R(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_R)		== KEY_PRESSED_OFFSET_R ? 1 : 0)
#define ISKEYDOWN_F(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_F)		== KEY_PRESSED_OFFSET_F ? 1 : 0)
#define ISKEYDOWN_G(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_G)		== KEY_PRESSED_OFFSET_G ? 1 : 0)
#define ISKEYDOWN_Z(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_Z)		== KEY_PRESSED_OFFSET_Z ? 1 : 0)
#define ISKEYDOWN_X(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_X)		== KEY_PRESSED_OFFSET_X ? 1 : 0)
#define ISKEYDOWN_C(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_C)		== KEY_PRESSED_OFFSET_C ? 1 : 0)
#define ISKEYDOWN_V(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_V)		== KEY_PRESSED_OFFSET_V ? 1 : 0)
#define ISKEYDOWN_B(RC_Data_Point)      ((RC_Data_Point->key.v & KEY_PRESSED_OFFSET_B)		== KEY_PRESSED_OFFSET_B ? 1 : 0)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];		//ch[0]右水平，ch[1]右竖直，ch[2]左水平，ch[3]左竖直，左正下负，ch[4]侧边,下660上-660（正常遥控器）
                char s[2];				//s[0]右边，s[1]左边
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
								uint16_t last_v;
        } key;
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);


#define RC_sl_channel 1
#define RC_sr_channel 0

extern RC_ctrl_t rc_ctrl;
uint8_t RC_data_is_error(void);
void slove_RC_lost(void);
void slove_data_error(void);

#endif
