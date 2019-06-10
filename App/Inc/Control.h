#ifndef _CONTROL_H_
#define _CONTROL_H_

extern int16 Error;
extern int16 Car_Sudu;

extern void Turn_PD(float Input);
extern int16 Speed_PID(int16 Goal,int16 Input);

extern uint8 Run_Flag;


extern void AD_Date_analyse();

typedef struct speed_PID 
  {
        float Goal; // 设定目标Desired value
	float Kp; // 比例常数Proportional Const
	float Ki; // 积分常数Integral Const
	float Kd; // 微分常数Derivative Const
	float LastError; // Error[-1]	
	float PrevError; // Error[-2]
        float ErrorSum; // Sums of Errors
        float I_MAX_sum;      // 积分极限正
        float I_MIN_sum;      // 积分极限负
        float output;		// 输出量
        float output_Min;   // 输出限位最小量
        float output_Max;   // 输出限位最大量
  }PID;

#endif