#include "common.h"
/**
* @函数名: Constrain_float
* @功  能: float类型限幅
* @参  数: float input    输入值
* @参  数: float min      限幅下限
* @参  数: float max      限幅上限
* @返  回: input/min/max
* @简  例: 无
* @注  意：无
*/
float Constrain_float(float input, float min, float max) 
{
    return ((input < min) ? (min) : ((input > max) ? (max) : (input)));
}

/**
* @函数名: Constrain_int
* @功  能: int类型限幅
* @参  数: int input      输入值
* @参  数: int min        限幅下限
* @参  数: int max        限幅上限
* @返  回: input/min/max
* @简  例: 无
* @注  意：无
*/
int Constrain_int(int input, int min, int max) 
{
    return ((input < min) ? (min) : ((input > max) ? (max) : (input)));
}

/**
* @函数名: Constrain_int_output
* @功  能: int类型限幅
* @参  数: int input      输入值
* @参  数: int output     输出值
* @参  数: int min        限幅下限
* @参  数: int max        限幅上限
* @返  回: input/output
* @简  例: 无
* @注  意：input在限幅范围外输出output
*/
int Constrain_int_output(int input, int output, int min, int max) 
{
    return ((input < min) ? (output) : ((input > max) ? (output) : (input)));
}