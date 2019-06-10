#ifndef _PUBLIC_H_
#define _PUBLIC_H_
#include "MK60_port.h"
#include "MK60_gpio.h"
#include  "MK60_FTM.h"
#include "MK60_flash.h"
#include "FIRE_OV7725_Eagle.h"

//main.c中定义的全局变量外部声明
#define DirectMiddle 1082//舵机中值(加大偏左)
#define DirectLeft	 (DirectMiddle+81)// 1163
#define DirectRight	 (DirectMiddle-77) //1005

#define BEEP_ON  gpio_init(PTB20,GPO,1)
#define BEEP_OFF gpio_init(PTB20,GPO,0)

extern int k1,k2,k3,k4;
extern int BOMA;

#define NVIC_PriorityGroup_0          ((uint32)0x7) /* 0 bits for pre-emption priority
                                                      4 bits for subpriority */
#define NVIC_PriorityGroup_1          ((uint32)0x6) /* 1 bits for pre-emption priority
                                                      3 bits for subpriority */
#define NVIC_PriorityGroup_2          ((uint32)0x5) /* 2 bits for pre-emption priority
                                                      2 bits for subpriority */
#define NVIC_PriorityGroup_3          ((uint32)0x4) /* 3 bits for pre-emption priority
                                                      1 bits for subpriority */
#define NVIC_PriorityGroup_4          ((uint32)0x3) /* 4 bits for pre-emption priority
                                                      0 bits for subpriority */


#endif