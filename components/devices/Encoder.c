#include "Encoder.h"
void CS_Enc(void)
{

		HAL_GPIO_WritePin(CS2_ENC_GPIO_Port,CS2_ENC_Pin,GPIO_PIN_RESET);

}
void NCS_Enc(void)
{

		HAL_GPIO_WritePin(CS2_ENC_GPIO_Port,CS2_ENC_Pin,GPIO_PIN_SET);

}

extern void getEncoder(fp32 *angle)
{
	uint8_t Enc_databyte[2],txData[2]={0x80,0x80};
	uint16_t Enc_data=0;
	CS_Enc();
	HAL_SPI_TransmitReceive(&hspi2,txData,Enc_databyte,1,1000);
	HAL_SPI_TransmitReceive(&hspi2,txData,Enc_databyte+1,1,1000);
//	HAL_SPI_TransmitReceive(&hspi2,txData,Enc_databyte,2,1000);
	NCS_Enc();
	Enc_data=(((Enc_databyte[0]<<8)|Enc_databyte[1]))&0x3FFF;
	*angle=360.0*(8192-Enc_data)/8192.0;
	
}
