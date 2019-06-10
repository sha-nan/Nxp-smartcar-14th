#ifndef _PID_H_
#define _PID_H


struct PID_servo
{
	float Goal; 	    // 目标
	float Input; 		// 输入量
	float KT;			// 总比例

	float KP; 			// 比例段1系数
	float I_MAX_P;      // 积分极限正
	float I_MAX_N;      // 积分极限负
	float KD; 			// 微分系数

	float Output;		// 输出量
	float Output_Min;   // 输出限位最小量
	float Output_Max;   // 输出限位最大量

	float ErrorSum; 	// 误差和
	float LastError; 	// 上一次误差
  //float PreError;
} PID_SERVO;



struct PID_fuzzy
{

        float Goal; 	    // 目标
	float P_Output;		// P输出量
	float D_Output;		// D输出量
        
	float P_Output_Min;   // P输出限位最小量
	float P_Output_Max;   // P输出限位最大量
	float D_Output_Min;   // D输出限位最小量
	float D_Output_Max;   // D输出限位最大量

	float LastError; 	// 上一次误差
		
	float Ke;//Error_Max;    //偏差量最大值
	float Kc;//DError_Max;   //偏差微分最大值
        
        float KPu;
        float KDu;
        
        float E;                 //偏差
        float Ec;                //偏差变化量

	float KP;//DP_Max;       //比例变化量
	float KD;//DD_Max;       //微分变化量
	
} PID_FUZZY;


//函数声明
extern void PID_Init(void);
#endif