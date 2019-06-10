#include "common.h"
/**
* @������: Constrain_float
* @��  ��: float�����޷�
* @��  ��: float input    ����ֵ
* @��  ��: float min      �޷�����
* @��  ��: float max      �޷�����
* @��  ��: input/min/max
* @��  ��: ��
* @ע  �⣺��
*/
float Constrain_float(float input, float min, float max) 
{
    return ((input < min) ? (min) : ((input > max) ? (max) : (input)));
}

/**
* @������: Constrain_int
* @��  ��: int�����޷�
* @��  ��: int input      ����ֵ
* @��  ��: int min        �޷�����
* @��  ��: int max        �޷�����
* @��  ��: input/min/max
* @��  ��: ��
* @ע  �⣺��
*/
int Constrain_int(int input, int min, int max) 
{
    return ((input < min) ? (min) : ((input > max) ? (max) : (input)));
}

/**
* @������: Constrain_int_output
* @��  ��: int�����޷�
* @��  ��: int input      ����ֵ
* @��  ��: int output     ���ֵ
* @��  ��: int min        �޷�����
* @��  ��: int max        �޷�����
* @��  ��: input/output
* @��  ��: ��
* @ע  �⣺input���޷���Χ�����output
*/
int Constrain_int_output(int input, int output, int min, int max) 
{
    return ((input < min) ? (output) : ((input > max) ? (output) : (input)));
}