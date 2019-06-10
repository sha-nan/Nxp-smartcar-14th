/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ������������������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_wdog.c
 * @brief      ���Ź���������
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-07-02
 */

#include "common.h"
#include "MK60_wdog.h"

//�ڲ���������
static void wdog_unlock(void);          //���Ź�����

/*!
 *  @brief      ��ʼ�����Ź�������ι��ʱ��
 *  @param      cnt     ι��ʱ�䣨��λΪ ms��
 *  @since      v5.0
 */
void wdog_init_ms(uint32 ms)
{
    ASSERT(ms >= 4);                                //���ԣ�����ʱ����СΪ4��ʱ�����ڣ�WDOG_TOVALL��˵����

    wdog_unlock();                                  //�������Ź��������������ÿ��Ź�

    WDOG_PRESC = WDOG_PRESC_PRESCVAL(0);            //���÷�Ƶϵ�� = PRESCVAL +1(PRESCVALȡֵ��ΧΪ0~7)

    WDOG_TOVALH = ms >> 16;                         //����ι��ʱ��
    WDOG_TOVALL = (uint16)ms;

    WDOG_STCTRLH = ( 0
                    | WDOG_STCTRLH_WDOGEN_MASK     //WDOGEN��λ��ʹ�� ���Ź�
                    //| WDOG_STCTRLH_CLKSRC_MASK   //���Ź�ʱ��ѡ��0Ϊ LDO ��1Ϊbus ʱ�ӣ�
                    | WDOG_STCTRLH_ALLOWUPDATE_MASK
                    | WDOG_STCTRLH_STOPEN_MASK
                    | WDOG_STCTRLH_WAITEN_MASK
                    | WDOG_STCTRLH_STNDBYEN_MASK
                    //|
                    );
}


/*!
 *  @brief      ���ÿ��Ź�
 *  @since      v5.0
 */
void wdog_enable(void)
{
    wdog_unlock();                                  //�������Ź��������������ÿ��Ź�

    WDOG_STCTRLH |= WDOG_STCTRLH_WDOGEN_MASK;       //WDOGEN��λ��ʹ�� ���Ź�
}


/*!
 *  @brief      ���ÿ��Ź�
 *  @since      v5.0
 */
void wdog_disable(void)
{
    wdog_unlock();                                  //�������Ź��������������ÿ��Ź�

    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;      //WDOGEN��0������ ���Ź�
}

/*!
 *  @brief      ι��
 *  @since      v5.0
 */
void wdog_feed(void)
{
    //�˺������ܵ���ִ��

    //WDOG_REFRESH �Ĵ�����������������˼Ĵ���д��0xA602 ��0xB480���ɽ�����
    //�м䲻�ó���20��ʱ�����ڣ������Ҫ�ȹ����ж�
#if  (defined(__ICCARM__) || defined(__CC_ARM))
    uint8 tmp = __get_BASEPRI();        //__get_BASEPRI �� IAR �Դ� intrinsics.h �ﶨ�� �����ڻ�ȡ �ж�״̬(MDKҲ֧��)
    //���ڷ��ؼĴ��� PRIMASK ��ֵ(1bit)
    //1��ʾ���жϣ�0��ʾ���ж�
#else
    uint8 tmp = 0;                      //Ĭ�ϱ�ʾ���ж�
#endif

    //�ر����жϣ������п���û���� 20������������д�� WDOG_UNLOCK
    DisableInterrupts;

    //���� ���Ź���ι����
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;


    if(tmp == 0)
    {
        EnableInterrupts;
    }
}

/*!
 *  @brief      �������Ź�
 *  @since      v5.0
 */
void wdog_unlock(void)
{
    //�˺������ܵ���ִ��

    //WDOG_UNLOCK �Ĵ�����������������˼Ĵ���д��0xC520��0xD928���ɽ�����
    //�м䲻�ó���20��ʱ�����ڣ������Ҫ�ȹ����ж�
#if  (defined(__ICCARM__) || defined(__CC_ARM))
    uint8 tmp = __get_BASEPRI();        //__get_BASEPRI �� IAR �Դ� intrinsics.h �ﶨ�� �����ڻ�ȡ �ж�״̬(MDKҲ֧��)
    //���ڷ��ؼĴ��� PRIMASK ��ֵ(1bit)
    //1��ʾ���жϣ�0��ʾ���ж�
#else
    uint8 tmp = 0;                      //Ĭ�ϱ�ʾ���ж�
#endif


    //�ر����жϣ������п���û���� 20������������д�� WDOG_UNLOCK
    DisableInterrupts;

    //���� ���Ź�
    WDOG_UNLOCK = 0xC520;
    WDOG_UNLOCK = 0xD928;

    if(tmp == 0)
    {
        EnableInterrupts;
    }
}
