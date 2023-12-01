#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "struct_typedef.h"
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

//14Bit

extern int16_t encode; 

extern void getEncoder(fp32 *angle);


#endif //__ENCODER_H__
