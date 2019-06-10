#ifndef __SENSER_H__
#define __SENSER_H__

extern float turn_error,pre_turn_error;
extern float Fuzzy_Kp(float P, float D);   //ģ����������,����Kpֵ
extern float Fuzzy_Kd(float P, float D);   //ģ����������,����Kdֵ


extern int16 turn_out_cal();
extern void Ring_Control();
extern void Run_Control();
extern void speed_control();
extern void turn_control();
extern void AD_Collect();
extern void AD_Init();


#define SECTOR_AD 254//ѡ������
#define SENSOR_NUMBER 5//�������

#define AD_Vol ADC1_SE14//��ص�ѹ�ɼ�

#define AD1 ADC0_SE14//��Ӧ���ұߵĵ�У���������Ϊ1234
#define AD2 ADC1_SE15
#define AD3 ADC0_SE13
#define AD4 ADC0_SE12

#define AD5 ADC1_DM0//��Ӧ����ߵĵ�У���������Ϊ5678
#define AD6 ADC1_DP0
#define AD7 ADC0_DM0
#define AD8 ADC0_DP0


#endif
