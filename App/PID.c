#include "common.h"
#include "Public.h"
#include "Control.h"

void vcan_sendware(void *wareaddr, uint32_t waresize);

int16 Out_Speed;
extern float Speed_P,Speed_I,Speed_D;
extern float Turn_KP,Turn_KD;

int16 Speed_PID(int16 Goal,int16 Input)
{ 
   int Speed_error,last_Speederror,last_lastSpeederror;//�ٶȲ�ֵ(����,�ϴΣ����ϴ�)
   int Speed_error_SUM,dSpeed_error;
   
   Speed_error  = Goal - Input;//����
   Speed_error_SUM = Speed_error*Speed_I;//����
   last_Speederror = Speed_error;
   last_lastSpeederror = last_Speederror;
   
   dSpeed_error = Speed_error -2*last_Speederror+last_lastSpeederror;//΢��      
   Out_Speed += (int16)((Speed_error-last_Speederror)*Speed_P+Speed_error_SUM+dSpeed_error*Speed_D);
   
   return Out_Speed;
}
