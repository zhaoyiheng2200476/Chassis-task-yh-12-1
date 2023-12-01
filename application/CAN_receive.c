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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"


#include "detect_task.h"
#include "string.h"
uint8_t rx6001_can_data[8];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
motor_6001_feedback_t feedback_data;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//motor data read
#define get_motor3510_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_pos = (ptr)->pos;                                   \
        (ptr)->pos = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->torque = (uint16_t)((data)[2] << 8 | (data)[3]);      \
    }

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���
�������, 0:���̵��1 3510���,  1:���̵��2 3510���*/
static motorMi_measure_t motorMi_chassis[4];
static CAN_TxHeaderTypeDef  Mi_tx_message;
static CAN_TxHeaderTypeDef  M6001_tx_message;
static uint8_t              Mi_can_send_data[8];
static uint8_t              M6001_can_send_data[8];
//static CAN_RxHeaderTypeDef  Mi_rx_message;
//static uint8_t              Mi_can_receive_data[8];
int motor2_errcnt=0;
		



		
		
		
		
		
		/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		uint8_t id;
		fp32 pos,speed;
    switch (rx_header.StdId)
    {	
			case 0x109:
				{
					id=0;
					memcpy(&pos, &rx_data[0], sizeof(fp32)); // ��ȡ�����յ������ݺ����ֽڣ����ٶ�ֵ,���ٶ�ֵ��16����ת��Ϊ������
					motorMi_chassis[id].fb_pos = pos;
					memcpy(&speed, &rx_data[4], sizeof(fp32)); // ��ȡ�����յ������ݺ����ֽڣ����ٶ�ֵ,���ٶ�ֵ��16����ת��Ϊ������
					motorMi_chassis[id].fb_speed = speed;
					detect_hook(CHASSIS_MOTOR1_TOE + id);
				break;
				}
			case 0x309:
				{
					id=1;
					memcpy(&pos, &rx_data[0], sizeof(fp32)); // ��ȡ�����յ������ݺ����ֽڣ����ٶ�ֵ,���ٶ�ֵ��16����ת��Ϊ������
					motorMi_chassis[id].fb_pos = pos;
					memcpy(&speed, &rx_data[4], sizeof(fp32)); // ��ȡ�����յ������ݺ����ֽڣ����ٶ�ֵ,���ٶ�ֵ��16����ת��Ϊ������
					motorMi_chassis[id].fb_speed = speed;
					detect_hook(CHASSIS_MOTOR1_TOE + id);
				break;				
				}
			case(0x101):
				{
					id=0;
					if (rx_data[1] == 0x01 || rx_data[4] == 0x01)//�жϵ���Ƿ�����������������error_flag��1
					{
						motorMi_chassis[id].error_flag = 1;
						//log_err("motor: %d",id);
					}
					motorMi_chassis[id].error_flag =motorMi_chassis[id].error_flag;
				detect_hook(CHASSIS_MOTOR1_TOE + id);
				break;
				}
			case(0x301):
				{
					id=1;
					if (rx_data[1] == 0x01 || rx_data[4] == 0x01)//�жϵ���Ƿ�����������������error_flag��1
					{
						motorMi_chassis[id].error_flag = 1;
						//log_err("motor: %d",id);
					}
					motorMi_chassis[id].error_flag =motorMi_chassis[id].error_flag;
					detect_hook(CHASSIS_MOTOR1_TOE + id);
				break;
				}		

      case(0x0e):
				{
				void parse_6001_feedback(const uint8_t *can_data, motor_6001_feedback_t *feedback_data) ;
        parse_6001_feedback(rx_data, &feedback_data);
				break;
				}									
			default:
        {
            break;
        }
			}


}
/**
  * @brief          ���ܵ��������λ������
  * @param[in]      λ��ֵ��Խ�������ֽڣ���λ�ڵ����ֽڣ���λ�ڵڶ��ֽ�
  * @param[in]      �ٶ�ֵ��Խ�������ֽڣ���λ�ڵ����ֽڣ���λ�ڵڶ��ֽ�
  * @param[in]      6001���ص�λ�����ݷ�ΧΪ [0,4095]
  * @retval         none
  */
void parse_6001_feedback(const uint8_t *can_data, motor_6001_feedback_t *feedback_data) {
    if (can_data[0] == 0x1A) {
        // ʶ���ʶλΪ0x1A��ʾλ����Ϣ
        feedback_data->position_6001 = (uint16_t)((can_data[2] << 8) | can_data[1]);
        feedback_data->speed_6001 = (uint16_t)((can_data[4] << 8) | can_data[3]);
			 //feedback_data->fb_angle = (float)feedback_data->position * (360.0 / 4096.0);
    } else {
        // ��λ����Ϣ�����Ը�����Ҫ������������߼�
    }
}		


//������أ�odrive��ѡ���ٶȡ�λ�á����ؿ���ģʽ������ѡ�����ؿ��ƣ���������أ�

/**
  * @brief          send control current of motor (0x201, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_Mi(uint32_t tar_id,fp32 data)
{
    uint32_t send_mail_box;
    Mi_tx_message.StdId = tar_id;//����id
    Mi_tx_message.IDE = CAN_ID_STD;//��׼
    Mi_tx_message.RTR = CAN_RTR_DATA;//����
    Mi_tx_message.DLC = 0x08;//���ݳ���
		memcpy(&Mi_can_send_data[0],&data,sizeof(fp32));// ������ת��Ϊ16����
    HAL_CAN_AddTxMessage(&MI_CAN, &Mi_tx_message, Mi_can_send_data, &send_mail_box);//ͨ��MI_CAN��CAN���߷�����Ϣ���Ƿ���Ҫ�ؽ�
}


//���뷴��
extern void chassis_get_feedback(uint32_t tar_id)//����0x10
{ 
	uint32_t send_mail_box;
	for(int i=0;i<8;i++)
	{
		Mi_can_send_data[i]=0;
	}
	Mi_tx_message.StdId = tar_id;
	Mi_tx_message.IDE = CAN_ID_STD; // ��׼֡
	Mi_tx_message.RTR = CAN_RTR_REMOTE; // ң��֡
	Mi_tx_message.DLC = 8; // ֡����
	Mi_can_send_data[0]=0x10;
	HAL_CAN_AddTxMessage(&MI_CAN, &Mi_tx_message, Mi_can_send_data, &send_mail_box);
}

//�������
void clean_error(uint32_t error_id)
{
	uint32_t send_mail_box;	
	for(int i=0;i<8;i++)
	{
		Mi_can_send_data[i]=0;
	}
	Mi_tx_message.StdId = error_id;;
	Mi_tx_message.IDE = CAN_ID_STD; // ��׼֡
	Mi_tx_message.RTR = CAN_RTR_REMOTE; // ң��֡
	Mi_tx_message.DLC = 8; // ֡����
	HAL_CAN_AddTxMessage(&MI_CAN, &Mi_tx_message, Mi_can_send_data, &send_mail_box);
}

//�ָ��ջ�����
void motor_loopcontrol(uint32_t motor_id)
{
	uint32_t send_mail_box;
	for(int i=0;i<8;i++)
	{
		Mi_can_send_data[i]=0;
	}
	Mi_tx_message.StdId = motor_id;
	Mi_tx_message.IDE = CAN_ID_STD; // ��׼֡
	Mi_tx_message.RTR = CAN_RTR_REMOTE; // ң��֡
	Mi_tx_message.DLC = 8; // ֡����
	Mi_can_send_data[0]=0x08;
	HAL_CAN_AddTxMessage(&MI_CAN, &Mi_tx_message, Mi_can_send_data, &send_mail_box);
}





/**
  * @brief          ����λ�÷���ָ��
	* @param[in]      frequency: 6001�����Ϣ����Ƶ��, ��Χ [0��1000]
  * @retval         2023
  */
void CAN_send_m6001_back(int16_t frequency)
{
    uint32_t send_mail_box;
    M6001_tx_message.StdId = 0x0e;
    M6001_tx_message.IDE = CAN_ID_STD;
    M6001_tx_message.RTR = CAN_RTR_DATA;
    M6001_tx_message.DLC = 0x08;
    M6001_can_send_data[0] = 0x19;
    M6001_can_send_data[1] = frequency;
    M6001_can_send_data[2] = frequency >> 8;
    M6001_can_send_data[3] = 00;
    M6001_can_send_data[4] = 00;
    M6001_can_send_data[5] = 00;
    M6001_can_send_data[6] = 00;
    M6001_can_send_data[7] = 00;

     HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}



/**
  * @brief          ���Ͷ������ָ��
	* @param[in]      ����ǰλ������Ϊ���
  * @retval         2023
  */
void CAN_send_m6001_setzero(void)
{
    uint32_t send_mail_box;
     M6001_tx_message.StdId = 0x0e;
     M6001_tx_message.IDE = CAN_ID_STD;
     M6001_tx_message.RTR = CAN_RTR_DATA;
     M6001_tx_message.DLC = 0x03;
     M6001_can_send_data[0] = 0x0d;
     M6001_can_send_data[1] = 0x00;
     M6001_can_send_data[2] = 0x00;
     HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}


///**
//  * @brief          ����6001���λ�ÿ���
//  * @param[in]      p6001: 6001����Ƕ�, ��Χ [0��359]
//  * @retval         2023/10
//  */
void CAN_send_m6001_position(int16_t p6001)
{
    uint32_t send_mail_box;
    M6001_tx_message.StdId = 0x0e;
    M6001_tx_message.IDE = 0;
    M6001_tx_message.RTR = 0;
    M6001_tx_message.DLC = 0x08;
    M6001_can_send_data[0] = 0x03;
    M6001_can_send_data[1] = p6001;
    M6001_can_send_data[2] = p6001 >> 8;
    M6001_can_send_data[3] = 0xe8;
    M6001_can_send_data[4] = 0x03;
    M6001_can_send_data[5] = 0x00;
    M6001_can_send_data[6] = 0x00;
    M6001_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}

///**
//  * @brief          ����6001����е�λ�ÿ���
//  * @param[in]      p6001: 6001����Ƕ�, ��Χ [+180��-180]
//  * @retval         2023/10
//  */
void CAN_send_m6001_midleposition(int16_t p6001) {
    uint32_t send_mail_box;
   M6001_tx_message.StdId = 0x0e;
    M6001_tx_message.IDE = 0;
    M6001_tx_message.RTR = 0;
    M6001_tx_message.DLC = 0x08;

    if (p6001 <= 0) {
			 // ӳ�� 0 �� -180 �� 0 �� 2047
        M6001_can_send_data[0] = 0x15;
        M6001_can_send_data[1] = (uint8_t)((-p6001 * 2047 / 180));
        M6001_can_send_data[2] = (uint8_t)((-p6001 * 2047 / 180) >> 8);
        
    } else {
       
			// ӳ�� 0 �� +180 �� 4096 �� 4088
        M6001_can_send_data[0] = 0x15;
        M6001_can_send_data[1] = (uint8_t)((4096 - (p6001 * 4 / 90)) >> 8);
        M6001_can_send_data[2] = (uint8_t)(4096 - (p6001 * 4 / 90));
    }

    M6001_can_send_data[3] = 0xe8;
    M6001_can_send_data[4] = 0x04;
    M6001_can_send_data[5] = 0x00;
    M6001_can_send_data[6] = 0x00;
    M6001_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}


// ��ֵ����
float interpolation_step = 3.0;

// ��ֵ����
float interpolate(float start, float end, float step) {
    if (start < end) {
        return (start + step < end) ? start + step : end;
    } else {
        return (start - step > end) ? start - step : end;//ȷ����ֵ���ֵ�������end
    }
}
///**
//  * @brief          ����6001PWM PID λ�õ�������
//  * @param[in]      p6001: 6001����Ƕ�, ��Χ [+350��-350]
//  * @retval         2023/10
//  */
float kp_position = 3.95;   // λ�û�P����3.35                           4                 
float ki_position = 0.00002;  // λ�û�I����0.00002                0.00003
float kd_position = 0.00002; // λ�û�D����0.000005            0.00001

float kp_speed = 0.1;       // �ٶȻ�P����
float ki_speed = 0;      // �ٶȻ�I����
float kd_speed = 0.00002;     // �ٶȻ�D����

// Variables for PID control
float position_integral = 0;
float last_position_error = 0;
float speed_integral = 0;
float last_speed_error = 0;
float pwm_Z = 550;//550
float pwm_F = 64988;//64733

int16_t last_set_angle = 0;// ��ӱ������ڴ洢��һ�ε� set_angle
uint16_t pwm = 0;

// Calculate PID control output
float previous_set_angle = 0.0;  // ��ʼֵ�趨Ϊ 0.0 ���ʵ�ֵ
float position_error;
float speed_error;
float fb_seesee;
float fb_seespeed;
float use_set_angle;
void CAN_send_m6001_position_pid(int16_t set_angle, int16_t fb_angle) {
    // Calculate position error

	  // LiM6001t the set_angle to the range of -350 to 350 degrees
    if (set_angle > 335) {
        set_angle = 335;
    } else if (set_angle < -335) {
        set_angle = -335;
    }	
		

		if (set_angle - fb_angle < -180) {
    fb_seesee = fb_angle - 360;
} 
else {
    fb_seesee = fb_angle;
}

if (set_angle - fb_seesee > 10) {
        set_angle = interpolate(previous_set_angle, set_angle, interpolation_step);
    } else if (set_angle - fb_seesee < -10) {
        set_angle = interpolate(previous_set_angle, set_angle, interpolation_step);
    }

     

		// LiM6001t the set_angle to the range of -350 to 350 degrees
    if (set_angle > 340 && fb_seesee > 340) {
    set_angle = 340;
    fb_seesee = 340;
    }
    else if (set_angle < -340 && fb_seesee < -340) {
        set_angle = -340;
			  fb_seesee = -340;
    }

		previous_set_angle = set_angle;
		
		
//				if(set_angle - fb_seesee> 10)
//			set_angle =fb_seesee + 10;
//		else if(set_angle - fb_seesee<-10)
//			set_angle =fb_seesee - 10;
//		else 
//			set_angle = set_angle;
//		
//		
  		position_error = set_angle - fb_seesee;
				

    // Ϊ��һ�ε���������ǰ��set_angle
   
    
    // ����λ�ÿ������
    float position_output = kp_position * position_error
                          + ki_position * position_integral
                          + kd_position * (position_error - last_position_error);
    
    //����λ�û��ֺ����Ĵ���
    position_integral += position_error;
    last_position_error = position_error;
		


    // ����PWMֵ(ȷ����ָ����Χ��)
    //uint16_t pwm = 0;

    //������ת
    if (position_output >= 0) {
        pwm = (uint16_t)position_output;
        pwm = (pwm > pwm_Z) ? pwm_Z : pwm;  // LiM6001t to 1000       //499
    } 
    // ������ת
    else {
        pwm = 65535 - (uint16_t)(-position_output);
        pwm = (pwm < pwm_F) ? pwm_F : pwm;  // LiM6001t to 64535    64933
    }

    uint32_t send_mail_box;
    M6001_tx_message.StdId = 0x0e;
    M6001_tx_message.IDE = 0;
    M6001_tx_message.RTR = 0;
    M6001_tx_message.DLC = 0x08;
		 
		M6001_can_send_data[0] = 0x17;
    M6001_can_send_data[1] = pwm;
    M6001_can_send_data[2] = pwm >> 8;
    M6001_can_send_data[3] = 0x00;
    M6001_can_send_data[4] = 0x00;
    M6001_can_send_data[5] = 0x00;
    M6001_can_send_data[6] = 0x00;
    M6001_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}

///**
//  * @brief          ����6001PWM PID λ��s�ٶ�˫������
//  * @param[in]      p6001: 6001����Ƕ�, ��Χ [+350��-350]
//  * @param[in]      p6001: 6001����ٶ�, ��Χ ˳ʱ��[+5759��-4759]
//                                              //��ʱ��[0��1000]
//  * @retval         2023/10
//  */
void CAN_send_m6001_position_pid_double(int16_t set_angle, int16_t fb_angle,int16_t fb_speed) {
    // Calculate position error
/////////////////////////////////////////////////////////////////////////		     

		//���Ƶ���˶�����-300~+300

	  // LiM6001t the set_angle to the range of -350 to 350 degrees
    if (set_angle > 335) {
        set_angle = 335;
    } else if (set_angle < -335) {
        set_angle = -335;
    }	
		

/////////////////////////////��Ȧ����//////////////////////////////////////////////////		
// ����������ĽǶȣ�ȷ������Ŀ��Ƕ�֮����-180��180��֮��

if (set_angle - fb_angle < -180) {
    fb_seesee = fb_angle - 360;
} 
else {
    fb_seesee = fb_angle;
}

if (fb_speed < 1100) {
    fb_seespeed = fb_speed;
} 
else {
    fb_seespeed = fb_speed-5759;
}

if (set_angle - fb_seesee > 10) {
        set_angle = interpolate(previous_set_angle, set_angle, interpolation_step);
    } else if (set_angle - fb_seesee < -10) {
        set_angle = interpolate(previous_set_angle, set_angle, interpolation_step);
    }

/////////////////////////////////////////////////////�����뷨��һ����ͨ����SET��ǰֵ����һ��ֵ�Ĳ�ֵ��һ�����趨ֵ�뷴��ֵ�Ĳ�ֵ
      previous_set_angle = set_angle;
		
//		
//				if(set_angle - fb_seesee> 10)
//			set_angle =fb_seesee + 10;
//		else if(set_angle - fb_seesee<-10)
//			set_angle =fb_seesee - 10;
//		else 
//			set_angle = set_angle;

  		position_error = set_angle - fb_seesee;

    float position_output = kp_position * position_error
                          + ki_position * position_integral
                          + kd_position * (position_error - last_position_error);

    position_integral += position_error;
    last_position_error = position_error;
		
     speed_error = position_error - fb_seespeed;
		
		float speed_output = kp_speed * speed_error
                       + ki_speed * speed_integral
                       + kd_speed * (speed_error - last_speed_error);

    // Update integral and last error for speed
    speed_integral += speed_error;
    last_speed_error = speed_error;

    // Combine position and speed outputs
    float combined_output = position_output + speed_output;
		
		
    if (combined_output >= 0) {
        pwm = (uint16_t)combined_output;
        pwm = (pwm > pwm_Z) ? pwm_Z : pwm;  // LiM6001t to 1000       //499
    } 
    else {
        pwm = 65535 - (uint16_t)(-combined_output);
        pwm = (pwm < pwm_F) ? pwm_F : pwm;  // LiM6001t to 64535    64933
    }

    uint32_t send_mail_box;
    M6001_tx_message.StdId = 0x0e;
    M6001_tx_message.IDE = 0;
    M6001_tx_message.RTR = 0;
    M6001_tx_message.DLC = 0x08;
		 
		M6001_can_send_data[0] = 0x17;
    M6001_can_send_data[1] = pwm;
    M6001_can_send_data[2] = pwm >> 8;
    M6001_can_send_data[3] = 0x00;
    M6001_can_send_data[4] = 0x00;
    M6001_can_send_data[5] = 0x00;
    M6001_can_send_data[6] = 0x00;
    M6001_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}






///**
//  * @brief          6001���λ�ý���
//  * @param[in]      ���������ĵ���ڵ�ǰλ�ý���������˶���ĳһλ�ã�Ĭ���������ģ�
//  * @retval         2023/10
//  */
void CAN_send_m6001_unlock(void)
{
    uint32_t send_mail_box;
     M6001_tx_message.StdId = 0x0e;
     M6001_tx_message.IDE = 0;
     M6001_tx_message.RTR = 0;
     M6001_tx_message.DLC = 0x08;
     M6001_can_send_data[0] = 0x2f;
    HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}




///**
//  * @brief          6001���λ������
//  * @param[in]      ����������ڵ�ǰλ��
//  * @retval         2023/10
//  */
void CAN_send_m6001_lock(void)
{
    uint32_t send_mail_box;
     M6001_tx_message.StdId = 0x0e;
     M6001_tx_message.IDE = 0;
     M6001_tx_message.RTR = 0;
     M6001_tx_message.DLC = 0x08;
     M6001_can_send_data[0] = 0x11;
     HAL_CAN_AddTxMessage(&M6001_CAN, &M6001_tx_message, M6001_can_send_data, &send_mail_box);
}

/**
  * @brief          ���ص��̵�� 3510�������ָ��
  * @param[in]      i: ������,��Χ[1,2]
  * @retval         �������ָ��
  */
const motorMi_measure_t *get_chassis_motorMi_measure_point(uint8_t i)
{
    return &motorMi_chassis[(i & 0x03)];
}
