/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_ftm.c
 * @brief      FTM��ʱ��������
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-08-22
 */

#ifndef _MK60_FTM_H_
#define _MK60_FTM_H_

//����FTMģ���
typedef enum
{
    FTM0,
    FTM1,
    FTM2,
    FTM3,

    FTM_MAX,
} FTMn_e;

//����FTM ͨ����
typedef enum
{

    FTM_CH0,
    FTM_CH1,
    FTM_CH2,
    FTM_CH3,
    FTM_CH4,
    FTM_CH5,
    FTM_CH6,
    FTM_CH7,

} FTM_CHn_e;

//��Ƶϵ��
typedef enum
{
    FTM_PS_1,
    FTM_PS_2,
    FTM_PS_4,
    FTM_PS_8,
    FTM_PS_16,
    FTM_PS_32,
    FTM_PS_64,
    FTM_PS_128,

    FTM_PS_MAX,
}FTM_PS_e;      //��Ƶֵ =  (1<< FTM_PS_e) ,����  FTM_PS_2  ��Ӧ�� ��Ƶֵ = (1<<FTM_PS_2) = (1<<1) = 2

extern FTM_MemMapPtr FTMN[FTM_MAX];

/*********************** PWM **************************/

#define FTM0_PRECISON 10000u     //����ռ�ձȾ��ȣ�100������Ϊ1%��1000u�򾫶�Ϊ0.1%������ռ�ձ� duty �βδ��룬��ռ�ձ�Ϊ duty/FTM_PRECISON
#define FTM1_PRECISON 10000u     //����ռ�ձȾ��ȣ�100������Ϊ1%��1000u�򾫶�Ϊ0.1%������ռ�ձ� duty �βδ��룬��ռ�ձ�Ϊ duty/FTM_PRECISON
#define FTM2_PRECISON 10000u     //����ռ�ձȾ��ȣ�100������Ϊ1%��1000u�򾫶�Ϊ0.1%������ռ�ձ� duty �βδ��룬��ռ�ձ�Ϊ duty/FTM_PRECISON
#define FTM3_PRECISON 10000u     //����ռ�ձȵľ��ȣ������
extern void  FTM_PWM_init(FTMn_e, FTM_CHn_e, uint32 freq, uint32 duty);  //��ʼ��FTM��PWM���ܲ�����Ƶ�ʡ�ռ�ձȡ�����ͨ�����ռ�ձȡ�ͬһ��FTM����ͨ����PWMƵ����һ���ģ���3��FTM

extern void  FTM_PWM_Duty(FTMn_e, FTM_CHn_e,              uint32 duty);  //����ͨ��ռ�ձ�,ռ�ձ�Ϊ ��duty * ���ȣ� % ����� FTM_PRECISON ����Ϊ 1000 ��duty = 100 ����ռ�ձ� 100*0.1%=10%
extern void  FTM_PWM_freq(FTMn_e,            uint32 freq);               //����FTM��Ƶ�ʣ���Ƶ�ʺ���Ҫ��������ռ�ձȣ�


/*********************** ���벶׽ **************************/
//FTM ���벶׽����
typedef enum
{
    FTM_Rising,               //�����ز�׽
    FTM_Falling,              //�½��ز�׽
    FTM_Rising_or_Falling     //�����ز�׽
} FTM_Input_cfg;


extern void ftm_input_init(FTMn_e ftmn, FTM_CHn_e ch, FTM_Input_cfg cfg,FTM_PS_e ps);   //���벶׽��ʼ������
extern uint16   ftm_input_get   (FTMn_e, FTM_CHn_e);
extern void     ftm_input_clean (FTMn_e );
extern void FTM1_Input_test_handler(void);                      //�ɹ��ο��� FTM1 ���벶׽�жϷ�����

#define FTM_IRQ_EN(FTMn,CHn)        FTM_CnSC_REG(FTMN[FTMn],CHn) |= FTM_CnSC_CHIE_MASK       //���� FTMn_CHn �ж�
#define FTM_IRQ_DIS(FTMn,CHn)       FTM_CnSC_REG(FTMN[FTMn],CHn) &= ~FTM_CnSC_CHIE_MASK      //�ر� FTMn_CHn �ж�

/*********************** �������빦�� **************************/
//������ ��δ����
extern void     FTM_QUAD_Init(FTMn_e ftmn);         //��ʼ��FTM ���������� ����
extern int16    FTM_QUAD_get(FTMn_e ftmn);          //��ȡFTM �������� ��������(������ʾ������)
extern void     FTM_QUAD_clean(FTMn_e ftmn);        //�� FTM �������� ��������

#endif  //_MK60_FTM_H_



