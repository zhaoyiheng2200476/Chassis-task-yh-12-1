/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "filter.h"
#include "detect_task.h"
#include "INS_task.h"
#include "encoder_read_task.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIFFERENCE 100
//#define  L 190 //lengthe between two Yaw angle,unit: mm
#define  L 230.0 // lengthe between two Yaw axis for XiaoMi,unit: mm
//#define  l 45 //distance between two Yaw angle,unit: mm 
//#define  l 55.8 //distance between two Yaw angle,unit: mm GM3510+MakeBlock
#define  l 135.0 //distance between wheel and Yaw axis,unit: mm  for XiaoMi		135
#define RAD2DEG 57.2958
#define Yaw_Start 161.0
#define Yaw_End 311.39
#define K 1.0	//360/(Yaw_Start-Yaw_End)
		//motor_6001_feedback_t feedback_data;
/* USER CODE END PD */		
  /**************************************************************************
�ο���ƽ��С��֮��
**************************************************************************/
float ZHONGZHI=-2.7;
float Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_ZPwm;
uint16_t Flag_Target;

uint16_t TURN_FLAG=0;
uint32_t Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
uint8_t Way_Angle=2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
uint8_t Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
uint8_t Flag_Stop=0,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
float Encoder_Left=0,Encoder_Right=0;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Angle_Z,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
uint32_t Distance;                               //���������
uint8_t delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //��ʱ�͵��εȱ���
float Acceleration_Z;                       //Z����ٶȼ�  
//float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;//PID����
//float Balance_Kp=950,Balance_Kd=-42,Velocity_Kp=240,Velocity_Ki=1.2;//PID���� Balance_Kp=950,Balance_Kd=-62,Velocity_Kp=300,Velocity_Ki=1.5; //float Balance_Kp=950,Balance_Kd=-42,Velocity_Kp=200,Velocity_Ki=1.0;
//float Balance_Kp=-0.045,Balance_Kd=0.0045,Velocity_Kp=-0.32,Velocity_Ki=-0.0016; //comment 2023.9.27
float Balance_Kp=-0.038,Balance_Kd=0.0048,Velocity_Kp=-0.17,Velocity_Ki=-0.0016; //changed 2023.9.27 Balance_Kp=-0.05,Balance_Kd=0.0080,

uint16_t PID_Parameter[10],Flash_Parameter[10];  //Flash�������	
float X_x,Y_y,phy,beta,Yaw_Angle=0,Yaw_Anglelast=0,Enc_Dir=0,Target_Velocity=0,Target_vx=0,Target_vy=0,Target_Yaw=0,Target_Wz,i_Gz=0;
int Velocity_Count=0;
int sw_L=0,sw_L_last=0,Balance_Flag=1;
int sw_R=0,sw_R_last=0;
float Encoder_Dif=0;
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
float myabs(float a)
{ 		   
	  float temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
uint8_t Turn_Off(float angle, int voltage)
{
	    uint8_t temp;
			//if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//��ص�ѹ����11.1V�رյ�� comment 2023.2.4
			if(angle<-70||angle>70)//ֻ�ж����
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                            //===Flag_Stop��1�رյ��
//			AIN1=0;                                            
//			AIN2=0;
//			BIN1=0;
//			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��INS  2�������� 3�������˲�
����  ֵ����
**************************************************************************/

void Get_Angle(uint8_t way,chassis_move_t *chassis_move_control)
{ 
	    float Accel_X,Accel_Angle,Accel_Z,Gyro_Y,Gyro_Z;
//	   	Temperature=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
			{	
					Angle_Balance=chassis_move_control->chassis_pitch*RAD2DEG;             //===����ƽ�����
					Gyro_Balance=-chassis_move_control->chassis_gyro[1]*RAD2DEG;            //===����ƽ����ٶ�
					Angle_Z=chassis_move_control->chassis_yaw*RAD2DEG;             //===����ƽ�⳵�庽���
					Gyro_Turn=chassis_move_control->chassis_gyro[2]*RAD2DEG;              //===����ת����ٶ�
				  Acceleration_Z=chassis_move_control->chassis_accel[2];         //===����Z����ٶȼ�
			}			
      else
      {
			Gyro_Y=-chassis_move_control->chassis_gyro[1]*RAD2DEG;    //��ȡY��������
			Gyro_Z=chassis_move_control->chassis_gyro[2]*RAD2DEG;    //��ȡZ��������
		  Accel_X=chassis_move_control->chassis_accel[1];//��ȡX����ٶȼ�
	  	Accel_Z=chassis_move_control->chassis_accel[2]; //��ȡZ����ٶȼ�
//		  if(Gyro_X>32768)  Gyro_X-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
//			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
//	  	if(Accel_Y>32768) Accel_Y-=65536;                      //��������ת��
//		  if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
			Gyro_Balance=Gyro_Y;                                  //����ƽ����ٶ�
	   	Accel_Angle=atan2(Accel_X,Accel_Z)*180/PI;                 //�������	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Angle,Gyro_Y);//�������˲�	
			else if(Way_Angle==3)   Yijielvbo(Accel_Angle,Gyro_Y);    //�����˲�
	    Angle_Balance=angle;                                   //����ƽ�����
			Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�
			Acceleration_Z=Accel_Z;                                //===����Z����ٶȼ�	
		}
}

/**************************************************************************
�������ܣ����ֿ����ٶȽ��� ���ߣ�½־��
��ڲ�����Yaw��Ƕ�
����  ֵ���ٶȱ���ϵ�����ٶȻ���������
**************************************************************************/
void Trans(float q_yaw)
{  
float yaw;
yaw=q_yaw/57.3;
	if(myabs(q_yaw)>=110&&myabs(q_yaw)<170)
	{
		Balance_Flag=0;
	}
	else
	{
		Balance_Flag=1;
	}
 
 // q_bike=asin(l/L);
  phy=atan(l*sin(yaw)/(0.5*L+l*cos(yaw))); // angle between chasis long Axis and pitch Axis
  beta=yaw-phy; // beta is the angle between wheel shaft and pitch Axis
 // yaw_d=q_yaw;
 // X_x=-sin(beta);
  Y_y=cos(beta); 
  if(myabs(Y_y)<0.3) 
  {if(Y_y>=0)
    {Y_y=0.3;}
  else
    {Y_y=-0.3;}  
  }  
  if (Y_y<0)
 //   Enc_Dir=-1; // ���Blance Control ������ֻ����һ��Y_y�� Enc_Dir ��Y_yΪ����ʱȡ��
		Enc_Dir=1; // ���Blance Control ������ֻ����һ��Y_y�� Enc_Dir ʼ��Ϊ��
  else
    Enc_Dir=1;
}
		/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
float balance(float Angle,float Gyro)
{  
   float Bias;
	 float balance;
	 Bias=Angle-ZHONGZHI;                       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
float velocity(float enc_right,float enc_left)
{  
     static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;//,Target_Velocity;
	  //=============ң��ǰ�����˲���=======================// 
//	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //����������ģʽ,�Զ��������ģʽ
//    else 	                         Target_Velocity=110;                 
		if(Target_vx>0)    	
		{
			Movement=Target_Velocity/Flag_sudu;  // comment 2023.6.28 �����ٶ���С����/Flag_sudu;	         //===ǰ����־λ��1 
			Flag_Qian=1;
			Flag_Hou=0;
			Flag_Stop=0;
		}
		else if(Target_vx<0)
		{
			Movement=Target_Velocity/Flag_sudu; // comment 2023.6.28 �����ٶ���С����/Flag_sudu;         //===���˱�־λ��1
			Flag_Qian=0;
			Flag_Hou=1;
			Flag_Stop=0;
		}
	  else  
			{
				Movement=0;	
				Flag_Qian=0;
				Flag_Hou=0;
				Flag_Stop=1;
			}
	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
	  Movement=Target_Velocity/Flag_sudu;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(enc_left+enc_right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
		Encoder_Dif += enc_left-enc_right;
		if(myabs(Yaw_Angle)>10||myabs(Target_Yaw)>15)
		{
			Encoder_Dif=0;
		}
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>Target_Velocity*12)  	Encoder_Integral=Target_Velocity*12;             //===�����޷�
		if(Encoder_Integral<Target_Velocity*12)	Encoder_Integral=Target_Velocity*12;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   
			{
				Encoder_Integral=0;      //===����رպ��������
				//Encoder_Dif=0;
			}
	  return Velocity;
}
/**************************************************************************
�������ܣ�ת����� ���ߣ�ƽ��С��֮��
��ڲ�����Z��������
����  ֵ��ת�����PWM
**************************************************************************/
uint8_t turnPre=0;	//!!!��ת׼����ÿ�ο�����תǰ�ȷ���
float lastTarAng;	//!!!
float e_yaw;
float turn(float target_angle,float yaw_angle,float Gz)//ת�����
{
  static float Turn_Target, yaw_last,i_yaw=0;
  //float Turn_Amplitude = 50, Kp = 3.18, Kd = 0.00636,KdGy=0.005,Ki=0.001,e_yaw=0, w_yaw=0;  //PD����
  //float Turn_Amplitude = 5000, Kp = 110, Kd = 8.25,KdGy=0.5,Ki=0.0,e_yaw=0, w_yaw=0; //float Turn_Amplitude = 5000, Kp = 110, Kd = 8.25,KdGy=0.5,Ki=2.0,e_yaw=0, w_yaw=0;
//	float Turn_Amplitude = 0.3, Kp = 0.0009, Kd = 0.00009,KdGz=0.00085,Ki=0.00001,e_yaw=0, w_yaw=0; //changed 2023.6.28 for XiaoMiWheel Turn_Amplitude = 5.0, Kp = 0.110, Kd = 0.00825,KdGy=0.0005,Ki=0.0,e_yaw=0, w_yaw=0;
//  float Kp_h=0.008,Kd_h=0.0005;
//	float Turn_Amplitude = 0.3, Kp = 0.001, Kd = 0.00008,KdGz=0,Ki=0.000001,e_yaw=0, w_yaw=0; //changed 2023.6.28 for XiaoMiWheel Turn_Amplitude = 5.0, Kp = 0.110, Kd = 0.00825,KdGy=0.0005,Ki=0.0,e_yaw=0, w_yaw=0;
 	float Turn_Amplitude = 0.3, Kp = 0.0017, Kd = -0.00045,KdGz=0,Ki=0.00001,e_yaw=0, w_yaw=0; //changed 2023.6.28 for XiaoMiWheel Turn_Amplitude = 5.0, Kp = 0.110, Kd = 0.00825,KdGy=0.0005,Ki=0.0,e_yaw=0, w_yaw=0;
  float Kp_h=0.19,Kd_h=-0.005;
	e_yaw=target_angle-yaw_angle;
	//target_angle*=0.6; // comment 2023.7.13
	
	if(e_yaw<1 && e_yaw>-1)
	{
		e_yaw=0;//e_yaw=0;
	}
  i_yaw+=e_yaw;
	if(target_angle==0 && myabs(e_yaw)<5)
	{
		i_yaw=0;
	}

	Turn_Target = e_yaw  * Kp + w_yaw * Kd + i_yaw * Ki + Gz * KdGz ;         //===���Z�������ǽ���PD����	
	if(myabs(Turn_Target)<0.0025)
	{
		Turn_Target=0;
	}	
	if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude; //===ת���ٶ��޷�
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
// comment 2023.6.28 end		
  return Turn_Target;
}
/**************************************************************************
�������ܣ�����ת����� ���ߣ�½־��
��ڲ�����Yaw�ǣ�Z��������
����  ֵ��ת�����PWM
**************************************************************************/
float turn_chasis(float Targetwz, float Anglez,float encoderDif)//ת�����
{
  static float Turn_Z, encoderDif_last=0, i_wz=0,i_encoderDif=0,Targetwz_last;
  //float Turn_Amplitude = 50, Kp = 3.18, Kd = 0.00636,KdGy=0.005,Ki=0.001,e_yaw=0, w_yaw=0;  //PD����
  //float Turn_Amplitude = 0.05, Kp = 0.00030, Kd = 0.0001,KdGy=0.00005,Ki=0.00001,e_Z=0, w_Z=0, e_encoderDif=0;
	float Turn_Amplitude = 0.05, Kp = 0.00030, Kd = 0,KdGy=0,Ki=0,e_Z=0, w_Z=0, e_encoderDif=0;
	float KpZ=0.0020, KdZ=0, KiZ=0;

		
/*  
  if (1 == Flag_Left)             Turn_Target += Turn_Convert;  //����ң��ָ��ı�ת��ƫ��
  else if (1 == Flag_Right)       Turn_Target -= Turn_Convert;//����ң��ָ��ı�ת��ƫ��
  else Turn_Target = 0;
*/
if(Targetwz==0)
{
	i_wz=0;
	e_encoderDif=encoderDif-encoderDif_last;
	i_encoderDif+=e_encoderDif;
  Turn_Z = encoderDif  * Kp + e_encoderDif * Kd + i_encoderDif * Ki;// + Gz * KdGy ;         //===���Z�������ǽ���PD����
	encoderDif_last=encoderDif;
}
else 
{

	i_encoderDif=0;
	e_Z=Targetwz-Targetwz_last;
	i_wz+=e_Z;
  Turn_Z = Targetwz  * KpZ + e_Z * KdZ + i_wz * KiZ;// + Gz * KdGy ;         //===���Z�������ǽ���PD����
	Targetwz_last=Targetwz;
}
  if (Turn_Z > Turn_Amplitude)  Turn_Z = Turn_Amplitude; //===ת���ٶ��޷�
  if (Turn_Z < -Turn_Amplitude) Turn_Z = -Turn_Amplitude;

  return Turn_Z;
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
//void Set_Pwm(int moto1,int moto2)
//{
//    	if(moto1>0)			AIN2=0,			AIN1=1;
//			else 	          AIN2=1,			AIN1=0;
//			PWMA=myabs(moto1);
//		  if(moto2>0)	BIN1=0,			BIN2=1;
//			else        BIN1=1,			BIN2=0;
//			PWMB=myabs(moto2);	
//}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
		if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE��һ������ƽ��С������ͻ�е��װ�����һ��������ֱ���������������С�����и��õ�һ���ԡ�
	  if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_balance_control(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//�����˶�����
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //���̳�ʼ��
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    CAN_send_m6001_back(800);

    //�жϵ��̵���Ƿ�����
// comment 2022.4.7 only use 2 gm3510, so comment chassis motor number checking
//    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//    {
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }
		  /* ����ǰ�����ó���� */
	  //CAN_send_m6001_setzero();
//	  /* ���޸����ݷ�����Ƶ��0-100 */
    CAN_send_m6001_back(800);//
		/* �Ƕ�-360-+360ָ��� */
  //CAN_send_m6001_allposition(80,340);//////////////80Ϊ�趨Ŀ��Ƕȣ�Ҳ����ң�����Ƕȣ�340Ϊ��ǰcan�����Ƕ�
	// CAN_send_m6001_position(0);
	//CAN_send_m6001_position(0);
    while (1)
    {
        //set chassis control mode
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //�������ݸ���
				// ���뷴������
				//chassis_get_feedback(0x109);
				//chassis_get_feedback(0x309);			
        chassis_feedback_update(&chassis_move);
				chassis_rc_to_control_vector(& Target_vx, & Target_Yaw, & Target_Wz,& chassis_move);
        //set chassis control set-point 
        //���̿���������
        //chassis_set_contorl(&chassis_move); //comment 2023.2.1
					chassis_balance_control(&chassis_move); //added 2023.2.1
        //chassis control pid calculate
        //���̿���PID����
        //chassis_control_loop(&chassis_move);	
//			
//			          CAN_send_m6001_position(0);
//		      HAL_Delay(1000);
//		      CAN_send_m6001_position(180);
//		      HAL_Delay(1000);


        //make sure  one motor is online at least, so that the control CAN message can be received
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE)||chassis_move.chassis_RC->rc.s[0]==2||Turn_Off(Angle_Balance,Voltage)==1)
            {
							//CAN_cmd_Mi(209,0);
							//CAN_cmd_Mi(210,0);
							//	// ���Ť��
							CAN_cmd_Mi(0x30E, 0);
							CAN_cmd_Mi(0x10E, 0);

							
							CAN_cmd_Mi(0x20E, 0);
							CAN_cmd_Mi(0x40E, 0);
							
							// CAN_cmd_Mi(0x30E, 0.01);
							// CAN_cmd_Mi(0x40E, 0.01);
            }
            else
            {
                //send control current
                //���Ϳ��Ƶ���
                							//	// ���Ť��

							CAN_cmd_Mi(0x30E, chassis_move.motorMi_chassis[1].torque);
							CAN_cmd_Mi(0x10E, chassis_move.motorMi_chassis[0].torque);
//							CAN_cmd_Mi(0x30E, 0.02);//left wheel
//							CAN_cmd_Mi(0x10E, 0.02);//right wheel

							
							CAN_cmd_Mi(0x20E, 0.02);//left wheel
							CAN_cmd_Mi(0x40E, 0.02);//left wheel	

            }
						sw_L=chassis_move.chassis_RC->rc.s[1];
						if(sw_L_last==3)
						{
							if(sw_L==1)
							{
								ZHONGZHI=ZHONGZHI+0.1;
							}
							else if(sw_L==2)
							{
								ZHONGZHI=ZHONGZHI-0.1;
							}
						}
						sw_L_last=sw_L;         //
						
        }
//								sw_R=chassis_move.chassis_RC->rc.s[0];
//							if(sw_R==2)
//							{
//								 CAN_send_m6001_unlock();

//							}
//							if(sw_R==3)
//							{	
//								CAN_send_m6001_setzero();
//							}
//							
//														if(sw_R==1)
//							{	
//								CAN_send_m6001_position_pid(Use_Target_Yaw, position6001);
//							}
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {Mi_MOTOR_SPEED_PID_KP, Mi_MOTOR_SPEED_PID_KI, Mi_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //���̽Ƕ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		const static fp32 chassis_yaw_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		const static fp32 chassis_wz_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning�� chassis mode is raw 
    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gyro sensor anglur velocity point
    //��ȡ�����ǽ��ٶ�ָ��
    chassis_move_init->chassis_INS_gyro = get_gyro_data_point();
    //get gyro sensor acceleration point
    //��ȡ�����Ǽ��ٶ�ָ��
    chassis_move_init->chassis_INS_accel = get_accel_data_point();	
    //get encoder Yaw axis angle point
    //��ȡYaw��������Ƕ�ָ��
    chassis_move_init->Yaw_Axis_angle = get_Yaw_Axis_point();
    //get gimbal motor data point
    //��ȡ��̨�������ָ��
//    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 2; i++)
    {
        chassis_move_init->motorMi_chassis[i].chassis_motorMi_measure = get_chassis_motorMi_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, Mi_MOTOR_SPEED_PID_MAX_OUT, Mi_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_yaw, CHASSIS_CONTROL_TIME, chassis_yaw_order_filter);
		first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_wz_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //���������̽Ƕ�ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
extern float fb_seesee;
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 2; i++)
    {		//int32_t speed_temp=0;
        //update motor pos, speed
        //���µ���ٶ�
				//chassis_get_feedback(0X109);
				//chassis_get_feedback(0X309);
				chassis_move_update->motorMi_chassis[i].pos = chassis_move_update->motorMi_chassis[i].chassis_motorMi_measure->fb_pos;
 				chassis_move_update->motorMi_chassis[i].speed = chassis_move_update->motorMi_chassis[i].chassis_motorMi_measure->fb_speed;

    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motorMi_chassis[0].speed + chassis_move_update->motorMi_chassis[1].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motorMi_chassis[0].speed - chassis_move_update->motorMi_chassis[1].speed ) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motorMi_chassis[0].speed - chassis_move_update->motorMi_chassis[1].speed ) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));       // - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));   //- chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
    chassis_move_update->chassis_gyro[0] = *(chassis_move_update->chassis_INS_gyro + INS_YAW_ADDRESS_OFFSET);       // - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_gyro[1] = *(chassis_move_update->chassis_INS_gyro + INS_PITCH_ADDRESS_OFFSET);   //- chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_gyro[2] = *(chassis_move_update->chassis_INS_gyro + INS_ROLL_ADDRESS_OFFSET);
    chassis_move_update->chassis_accel[0] = *(chassis_move_update->chassis_INS_accel + INS_YAW_ADDRESS_OFFSET);       // - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_accel[1] = *(chassis_move_update->chassis_INS_accel + INS_PITCH_ADDRESS_OFFSET);   //- chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_accel[2] = *(chassis_move_update->chassis_INS_accel + INS_ROLL_ADDRESS_OFFSET);		
		chassis_move_update->yaw_axis_angle=*(chassis_move_update->Yaw_Axis_angle);
//		Yaw_Angle=K*(chassis_move_update->yaw_axis_angle-Yaw_Start); // comment 2023.9.27 for Mi2 Test
//		Yaw_Angle=0; //added 2023.9.27 for Mi2 Test
		Yaw_Angle= fb_seesee *0.25; //added 2023.10.25 for Mi2 add UBtech Servo Motor Test
//		Yaw_Angle=chassis_move_update->yaw_axis_angle;
		Get_Angle(1,chassis_move_update); //added 2023.2.1
		Voltage=1200;
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *yaw_set, fp32 *wz_set,chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || yaw_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel, wz_channel,yaw_channel;
    fp32 vx_set_channel, vy_set_channel,wz_set_channel,yaw_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], yaw_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);
		
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    yaw_set_channel = yaw_channel * CHASSIS_ANGLE_YAW_RC_SEN;//comment 2023.2.11 * -CHASSIS_VY_RC_SEN;2023.3.6 add *1.2 for large turn
		wz_set_channel = -wz_channel * CHASSIS_ANGLE_YAW_RC_SEN;//;// * CHASSIS_ANGLE_YAW_RC_SEN;
    //keyboard set speed set-point
    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
		// uncomment 2023.7.13 added first order filter
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_yaw, yaw_set_channel);
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, wz_set_channel);
    //stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

//    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN) //comment 2023.2.11
    if (yaw_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_YAW_RC_SEN && yaw_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_YAW_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_yaw.out = 0.0f;
    }
		
		if (wz_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_YAW_RC_SEN && wz_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_YAW_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
//    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out; //comment 2023.2.11
		//*yaw_set =yaw_set_channel; // no use first_order_filter
		*yaw_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_yaw.out; // added 2023.7.13 use first order filter
		*wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
//        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
//        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
//        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        //set control relative angle  set-point
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //������תPID���ٶ�
//        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //set chassis yaw angle set-point
        //���õ��̿��ƵĽǶ�
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //calculate rotation speed
        //������ת�Ľ��ٶ�
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //speed limit
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
//		chassis_move_control->wz_set =chassis_move_control->yaw_axis_angle; //2023.1.31 added for test
        
}
/**
  * @brief          set chassis balance control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ���ƽ���������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */

extern int resDataOnce;
uint16_t position6001;
uint16_t speed6001;
uint16_t Use_Target_Yaw;
int need_yaw;
static void chassis_balance_control(chassis_move_t *chassis_move_control)
{ static int  Turn_Count,Yaw=0,Yaw_Count=0, Wz_Count=0;
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	fp32 wheel_speed[2] = {0.0f, 0.0f};
	uint8_t i = 0;
	position6001 = feedback_data.position_6001*360/4096;
	speed6001 = feedback_data.speed_6001*360/4096;
  
	
	    if (chassis_move_control == NULL)
    {
        return;
    }



 Balance_Pwm = balance(Angle_Balance,  Gyro_Balance);//ֱ��PD���� ��������5ms  
 
  if (++Velocity_Count >= 8) //�ٶȿ��ƣ���������40ms
  {
		
 		//Encoder_Left=chassis_move_control->motor_chassis[1].speed;
 		//Encoder_Right=-chassis_move_control->motor_chassis[0].speed;
		Encoder_Left=chassis_move_control->motorMi_chassis[1].speed;
 		Encoder_Right=-chassis_move_control->motorMi_chassis[0].speed;
		//Encoder_Left++;
    Velocity_Pwm = velocity(Encoder_Right, Encoder_Left);//�ٶ�PI���ƣ���������40ms
    Velocity_Count = 0;
  }

	//Yaw_Angle=0;
    if (++Turn_Count >= 1)//ת����ƣ���������20ms
  {
//    Turn_Pwm = turn(chassis_move_control->wz_set,Yaw_Angle,chassis_move_control->chassis_gyro[0]);
		if(Target_Wz==0)
		{
			Turn_Pwm = turn(Target_Yaw,Yaw_Angle,Gyro_Turn);//ת�����
			Turn_ZPwm = turn_chasis(Target_Wz, Angle_Z,Encoder_Dif);
			Wz_Count=0;
		}
		else
		{
			Encoder_Dif = 0;//��ֹ������תʱ��ת2023.10.27
			Turn_ZPwm = turn_chasis(Target_Wz, Angle_Z,Encoder_Dif);
		}
//		if(resDataOnce==1)
//		{
//		//CAN_send_m6001_allposition(Target_Yaw*4,0);//���ʵ���յ�һ����Ϣ������һ�Σ�������һֱ��
//		
//		resDataOnce=0;
//		}

	//CAN_send_m6001_position_pid(Target_Yaw*4, position6001);
		//CAN_send_m6001_pid(Target_Yaw*4, position6001, speed6001);
	sw_R=chassis_move.chassis_RC->rc.s[0];
							if(sw_R==2)
							{
								 CAN_send_m6001_unlock();
								Target_Velocity=(Target_vx*myabs(Y_y))/2;

							}
							if(sw_R==3)
							{	
//								CAN_send_m6001_position_pid_double(Target_Wz*2, position6001,speed6001);
								
								CAN_send_m6001_position_pid(Target_Yaw*2, position6001);
								
		            Target_Velocity=(Target_vx*myabs(Y_y));
								
							}
							
														if(sw_R==1)
							{	
//								CAN_send_m6001_position_pid(Target_Yaw*4, position6001);
								CAN_send_m6001_position_pid_double(Target_Yaw*4, position6001,speed6001);
								Target_Velocity=Target_vx*myabs(Y_y)/2;
							}
							

		

//		CAN_send_m6001_allposition(Target_Yaw*4,position6001);
//		CAN_send_m6001_midleposition(Target_Yaw*2);
/* comment 2023.7.13		
		else if(Target_Wz>0)
		{ 
			Encoder_Dif=0;
			if(Yaw_Angle>=0||(Wz_Count>1&&Yaw_Angle>=4))
			{
				if(Yaw_Angle>=4)
				{
					Wz_Count=0;
				}
				Wz_Count++;
				if(Wz_Count<=1)
				{
					Target_Yaw=-5.1;
					Turn_Pwm = turn(Target_Yaw,Yaw_Angle,Gyro_Turn);//ת�����
				}					
			}
			else if(Yaw_Angle<0||(Wz_Count>1&&Yaw_Angle<4))
			{
				Wz_Count++;
				if(Yaw_Angle>-20)
				{
					if(Wz_Count>=1000)
					{
						Wz_Count=1000;
					}
					Target_Yaw=Yaw_Angle;
					Turn_ZPwm = turn_chasis(Target_Wz*(Wz_Count/1000.0), Angle_Z,Encoder_Dif);
				}
				else
				{
					Target_Yaw=Yaw_Angle+4.8;					
					Turn_Pwm = turn(Target_Yaw,Yaw_Angle,Gyro_Turn);//ת�����	
				}	
			}
			
			
		}
		else if(Target_Wz<0)
		{
			
			Encoder_Dif=0;
			if(Yaw_Angle<0||(Wz_Count>1&&Yaw_Angle<=-4))
			{
				if(Yaw_Angle<=-4)
				{
					Wz_Count=0;
				}
				Wz_Count++;
				if(Wz_Count<=1)
				{
					Target_Yaw=5.1;			
					Turn_Pwm = turn(Target_Yaw,Yaw_Angle,Gyro_Turn);//ת�����
				}
			}
			else if(Yaw_Angle>=0||(Wz_Count>1&&Yaw_Angle>- 4))
			{
				if(Yaw_Angle<20)
				{
					Wz_Count++;
					if(Wz_Count>=1000)
					{
						Wz_Count=1000;
					}
					Target_Yaw=Yaw_Angle;
					Turn_ZPwm = turn_chasis(Target_Wz*(Wz_Count/1000.0), Angle_Z,Encoder_Dif);
				}
				else
				{
					Target_Yaw=Yaw_Angle-4.8;			
					Turn_Pwm = turn(Target_Yaw,Yaw_Angle,Gyro_Turn);//ת�����	
				}
			}


		}
*/
	//	Turn_ZPwm = turn_chasis(Angle_Z, Angle_Z,Gyro_Turn); //changed 2023.2.11
		
//		Turn_Pwm=0; //added 2023.2.6 for testing
    //Turn_Count = 0;
		
  } 

  Trans(Yaw_Angle);	

//	Turn_Pwm=0; // comment Turn_Pwm for test
//	wheel_speed[0] = -(Balance_Pwm/(Y_y)*Enc_Dir + Velocity_Pwm)*Balance_Flag - Turn_Pwm-Turn_ZPwm;  //ֱ���ٶ�ת�򻷵ĵ���
//  wheel_speed[1] = (Balance_Pwm/(Y_y)*Enc_Dir + Velocity_Pwm)*Balance_Flag - Turn_Pwm-Turn_ZPwm; //ֱ���ٶ�ת�򻷵ĵ���
//	wheel_speed[0] = -Turn_Pwm;  //ֱ���ٶ�ת�򻷵ĵ���
//  wheel_speed[1] = -Turn_Pwm; //ֱ���ٶ�ת�򻷵ĵ���
	
/*comment 2023.6.27 start	
	wheel_speed[0] = -(Balance_Pwm/(Y_y)*Enc_Dir + Velocity_Pwm*(Y_y)*Enc_Dir)*Balance_Flag - Turn_Pwm-Turn_ZPwm;  //ֱ���ٶ�ת�򻷵ĵ���
  wheel_speed[1] = (Balance_Pwm/(Y_y)*Enc_Dir + Velocity_Pwm*(Y_y)*Enc_Dir)*Balance_Flag - Turn_Pwm-Turn_ZPwm; //ֱ���ٶ�ת�򻷵ĵ���
comment 2023.6.27 end */	
	
// added 2023.6.27 for test	
//	if(Balance_Pwm>10)
//	{
//		Balance_Pwm = 10;
//	}
//	else if(Balance_Pwm<-10)
//	{
//		Balance_Pwm = -10;
//	}	
//Balance_Pwm=0;//2023.10.25
//Velocity_Pwm=0;//2023.10.25     
Turn_Pwm=0;

	wheel_speed[0] =  Balance_Pwm/(Y_y) + Velocity_Pwm*(Y_y)- Turn_Pwm-Turn_ZPwm;
	wheel_speed[1] = -Balance_Pwm/(Y_y) - Velocity_Pwm*(Y_y)- Turn_Pwm-Turn_ZPwm;
	    //calculate the max speed in four wheels, limit the max speed
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 2; i++)
    {
         chassis_move_control->motorMi_chassis[i].speed_set = wheel_speed[i];
         temp = fabs(chassis_move_control->motorMi_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 2; i++)
        {
             chassis_move_control->motorMi_chassis[i].speed_set *= vector_rate;
        }
    }
		
/* comment 2023.6.27 start		
//calculate pid
    //����pid
    for (i = 0; i < 2; i++)
    {
         PID_calc(&chassis_move_control->motor_speed_pid[i], chassis_move_control->motorMi_chassis[i].speed, chassis_move_control->motorMi_chassis[i].speed_set);
    }
    //��ֵ����ֵ
    for (i = 0; i < 2; i++)
    {
         chassis_move_control->motorMi_chassis[i].torque = (fp32)(chassis_move_control->motor_speed_pid[i].out);
    }
		comment 2023.6.27 end */
// no pid comment 2023.2.6
				for (i = 0; i < 2; i++)
	{
			chassis_move_control->motorMi_chassis[i].torque = (fp32)(chassis_move_control->motorMi_chassis[i].speed_set);
	}

//    //calculate pid
//    //����pid
//    for (i = 0; i < 2; i++)
//    {
//        PID_calc(&chassis_move_control->motor_speed_pid[i], chassis_move_control->motor_chassis[i].speed, chassis_move_control->motor_chassis[i].speed_set);
//    }



//    //��ֵ����ֵ
//    for (i = 0; i < 2; i++)
//    {
//        chassis_move_control->motor_chassis[i].give_current = (int16_t)(chassis_move_control->motor_speed_pid[i].out);
//    }
//  Xianfu_Pwm();//�޷�
}
/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    //wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    //wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[0] =  - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set; //changed 2023.3.6 delete vx_set
    wheel_speed[1] =  - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
        //wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    //wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[2] = {0.0f, 0.0f};
    uint8_t i = 0;

    //balance control wheel speed calculation
    //ƽ�����

    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 2; i++)
        {
            chassis_move_control_loop->motorMi_chassis[i].torque = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw����ֱ�ӷ���
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 2; i++)
    {
        chassis_move_control_loop->motorMi_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motorMi_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 2; i++)
        {
            chassis_move_control_loop->motorMi_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //����pid
    for (i = 0; i < 2; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motorMi_chassis[i].speed, chassis_move_control_loop->motorMi_chassis[i].speed_set);
    }



    //��ֵ����ֵ
    for (i = 0; i < 2; i++)
    {
        chassis_move_control_loop->motorMi_chassis[i].torque = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
