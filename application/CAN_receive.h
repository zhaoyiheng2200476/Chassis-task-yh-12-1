/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define MI_CAN hcan1
#define GIMBAL_CAN hcan2
#define M6001_CAN hcan2



typedef struct 
{
	fp32 fb_pos;
	fp32 set_speed;
	fp32 fb_speed;
	uint8_t error_flag;
	uint32_t error_cnt;
}motorMi_measure_t;//底盘小米电机

// 定义电机位置信息结构体
typedef struct {
    uint16_t position_6001;//位置
    uint16_t speed_6001;//速度
} motor_6001_feedback_t;



extern void CAN_cmd_Mi(uint32_t tar_id,fp32 data);
extern void chassis_get_feedback(uint32_t tar_id);
extern void clean_error(uint32_t error_id);
extern void motor_loopcontrol(uint32_t motor_id);


extern void CAN_cmd_6001(uint32_t motor6001_id,uint32_t m6001data);//6001发送的结构体
extern void chassis_get_feedback_6001(uint32_t tar_m6001id);//获取反馈的信息




/**
  * @brief          return the chassis Mi motor data point
  * @param[in]      i: motor number,range [1,2]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 Mi电机数据指针
  * @param[in]      i: 电机编号,范围[1,2]
  * @retval         电机数据指针
  */
extern const motorMi_measure_t *get_chassis_motorMi_measure_point(uint8_t i);


///**
//  * @brief          return the chassis Mi motor data point
//  * @param[in]      i: motor number,range [1,2]
//  * @retval         motor data point
//  */
///**
//  * @brief          返回转向电机6001电机数据指针
//  * @param[in]      i: 电机编号,范围为多少？？？？？是指ID么还是数量
//  * @retval         电机数据指针
//  */
//extern const motor_6001_feedback_t *get_chassis_motor6001_measure_point(uint8_t i);////是否正确






/**
  * @brief          发送位置反馈指令
	* @param[in]      frequency: 6001电机信息反馈频率, 范围 [0，1000]
  * @retval         none
  */
extern void CAN_send_m6001_back(int16_t frequency);






/**
  * @brief          发送定义零点指令
	* @param[in]      将当前位置设置为零点
  * @retval         2023.
  */
extern void CAN_send_m6001_setzero(void);


/**
  * @brief          发送6001电机位置控制
  * @retval         none
  */
extern void CAN_send_m6001_position(int16_t p6001);



/**
  * @brief          发送6001电机位置中值控制
  * @retval         none
  */
extern void CAN_send_m6001_midleposition(int16_t p6001) ;


/**
  * @brief          发送6001电机全方位位置控制
  * @retval         none
  */
extern void CAN_send_m6001_allposition(int16_t set_angle, int16_t fb_angle);


/**
  * @brief          发送6001电机PWM位置控制
  * @retval         none
  */
extern	void CAN_send_m6001_position_pid(int16_t set_angle, int16_t fb_angle) ;//单环
extern void CAN_send_m6001_position_pid_double(int16_t set_angle, int16_t fb_angle,int16_t fb_speed);//双环


/**
  * @brief          6001电机位置解锁
  * @retval         none
  */
extern void CAN_send_m6001_unlock(void) ;

/**
  * @brief          6001电机位置锁死
  * @retval         none
  */
extern void CAN_send_m6001_lock(void) ;

extern motor_6001_feedback_t feedback_data;
#endif


