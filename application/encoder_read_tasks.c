/**
  ****************************(C) COPYRIGHT 2022 T-DT****************************
  * @file       enc_task.c/h
  * @brief     enc read task��
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
#include "encoder_read_task.h"
#include "cmsis_os.h"
#include "main.h"
/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGB����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
#include "struct_typedef.h"
fp32  fbAngle;
extern void encoder_read_task(void const * argument)
{


    while(1)
    {
		getEncoder(&fbAngle);
		chassis_get_feedback(0x109);
		chassis_get_feedback(0x309);
		osDelay(2);
    }
}
/**
  * @brief          ��ȡYaw��Ƕ� :yaw ��λ deg
  * @param[in]      none
  * @retval         fbAngle��ָ��
  */
const fp32 *get_Yaw_Axis_point(void)
{
    return &fbAngle;
}
