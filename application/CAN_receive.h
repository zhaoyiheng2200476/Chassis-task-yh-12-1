/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
}motorMi_measure_t;//����С�׵��

// ������λ����Ϣ�ṹ��
typedef struct {
    uint16_t position_6001;//λ��
    uint16_t speed_6001;//�ٶ�
} motor_6001_feedback_t;



extern void CAN_cmd_Mi(uint32_t tar_id,fp32 data);
extern void chassis_get_feedback(uint32_t tar_id);
extern void clean_error(uint32_t error_id);
extern void motor_loopcontrol(uint32_t motor_id);


extern void CAN_cmd_6001(uint32_t motor6001_id,uint32_t m6001data);//6001���͵Ľṹ��
extern void chassis_get_feedback_6001(uint32_t tar_m6001id);//��ȡ��������Ϣ




/**
  * @brief          return the chassis Mi motor data point
  * @param[in]      i: motor number,range [1,2]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� Mi�������ָ��
  * @param[in]      i: ������,��Χ[1,2]
  * @retval         �������ָ��
  */
extern const motorMi_measure_t *get_chassis_motorMi_measure_point(uint8_t i);


///**
//  * @brief          return the chassis Mi motor data point
//  * @param[in]      i: motor number,range [1,2]
//  * @retval         motor data point
//  */
///**
//  * @brief          ����ת����6001�������ָ��
//  * @param[in]      i: ������,��ΧΪ���٣�����������ָIDô��������
//  * @retval         �������ָ��
//  */
//extern const motor_6001_feedback_t *get_chassis_motor6001_measure_point(uint8_t i);////�Ƿ���ȷ






/**
  * @brief          ����λ�÷���ָ��
	* @param[in]      frequency: 6001�����Ϣ����Ƶ��, ��Χ [0��1000]
  * @retval         none
  */
extern void CAN_send_m6001_back(int16_t frequency);






/**
  * @brief          ���Ͷ������ָ��
	* @param[in]      ����ǰλ������Ϊ���
  * @retval         2023.
  */
extern void CAN_send_m6001_setzero(void);


/**
  * @brief          ����6001���λ�ÿ���
  * @retval         none
  */
extern void CAN_send_m6001_position(int16_t p6001);



/**
  * @brief          ����6001���λ����ֵ����
  * @retval         none
  */
extern void CAN_send_m6001_midleposition(int16_t p6001) ;


/**
  * @brief          ����6001���ȫ��λλ�ÿ���
  * @retval         none
  */
extern void CAN_send_m6001_allposition(int16_t set_angle, int16_t fb_angle);


/**
  * @brief          ����6001���PWMλ�ÿ���
  * @retval         none
  */
extern	void CAN_send_m6001_position_pid(int16_t set_angle, int16_t fb_angle) ;//����
extern void CAN_send_m6001_position_pid_double(int16_t set_angle, int16_t fb_angle,int16_t fb_speed);//˫��


/**
  * @brief          6001���λ�ý���
  * @retval         none
  */
extern void CAN_send_m6001_unlock(void) ;

/**
  * @brief          6001���λ������
  * @retval         none
  */
extern void CAN_send_m6001_lock(void) ;

extern motor_6001_feedback_t feedback_data;
#endif


