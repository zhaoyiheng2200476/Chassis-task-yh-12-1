/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       enc_task.c/h
  * @brief      enc read。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-29-2022     Lu              
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 T-DT****************************
  */
#ifndef ENCODER_READ_TASK_H
#define ENCODER_READ_TASK_H


#include "struct_typedef.h"
#include "Encoder.h"
#include "CAN_receive.h"

/**
  * @brief          enc read task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          enc task 任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
//extern fp32 fbAngle=0;
extern void encoder_read_task(void const * argument);
/**
  * @brief          get the Yaw angle,  unit deg
  * @param[in]      none
  * @retval         the point of fbAngle
  */
/**
  * @brief          获取Yaw角, 单位 deg
  * @param[in]      none
  * @retval         fbAngle的指针
  */
extern const fp32 *get_Yaw_Axis_point(void);

#endif

