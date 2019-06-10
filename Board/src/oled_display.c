#include "common.h"
#include "Public.h"
#include  "OLED.h"
#include "Sensor.h"
 extern uint16  sensor1,sensor2,sensor11,sensor21;
void display1(void)
{
  int i,j;
  for(j = 0;j<60;j++)
  {   
    for(i=0;i<80;i++)
    {
      if(ImageData[j][i]==255) LCD_DrawPoint( i,j,1);
      else LCD_DrawPoint( i,j,0);
   
    }
   // MidBlackLine[j]=(RightBlack[j]+LeftBlack[j])/2;
  //  LCD_DrawPoint( MidBlackLine[j],j,0);
    
  }
 LCD_Refresh_Gram();
}
void display(void)
{
  int i,j;
  for( j = 0;j<60;j++)
  {   
       if(j>(CAMERA_H-MidNumbers)||j>=4)
       {
           for( i=0;i<80;i++)
           {
              if (i==rightLine[j])
                 LCD_DrawPoint(i,j,1);
              else if(i==leftLine[j])
                 LCD_DrawPoint( i,j,1);
           /*   else if(j==10)
                 LCD_DrawPoint(i,j,1);*/
              else if(i==midLine[j]&&midLine[j]!=0)
                 LCD_DrawPoint( i,j,1);
            //  else if(j==13)
            //    LCD_DrawPoint(i,j,1);
              else
                 LCD_DrawPoint( i,j,0);  
           }
       }
       else 
         for( i=0;i<80;i++)
           LCD_DrawPoint( i,j,0);
  }
 LCD_Refresh_Gram();
}


void display3(void)
{
  uint8 S1[8],S2[8],S3[8],S4[8];
           sprintf((uint8*)S1,"sensor1:%5d",sensor1);             
           LCD_single_P8x16Str(0,0,S1); 
           sprintf((uint8*)S2,"sensor2:%5d",sensor2);             
            LCD_single_P8x16Str(0,2,S2);
           // sprintf((uint8*)S3,"chazhi:%5d",abs(sensor1-sensor2));
           sprintf((uint8*)S3,"sensor11:%5d",sensor11);
            LCD_single_P8x16Str(0,4,S3);
            //sprintf((uint8*)S4,"turn_error:%1d",(int)(turn_error*100));
            sprintf((uint8*)S4,"sensor21:%5d",sensor21);
            LCD_single_P8x16Str(0,6,S4);
 
 
}


/*void display2(void)
{
  int i=0;
  i++;
         
 
 
     uint8 S1[8],S2[8],S3[8],S4[8],S5[8], S6[8];
      uint8 S7[8],S8[8],S9[8],S10[8];
     sprintf((char*)S1,"%d",BlackEndL); //1/410
     LCD_P8x16Str(0,0,"L:"); 
     LCD_P8x16Str(15,0,S1); 
     
     sprintf((char*)S2,"%d",BlackEndM); //1/2
     LCD_P8x16Str(45,0,"M:");
     LCD_P8x16Str(60,0,S2);
     
     sprintf((char*)S3,"%d",BlackEndR);//3/470
     LCD_P8x16Str(90,0,"R:");
     LCD_P8x16Str(105,0,S3);
 
     
    sprintf((char*)S4,"%d",Licount); //有效中线行数
     LCD_P8x16Str(0,2,"Li:");
     LCD_P8x16Str(15,2,S4);
     
     sprintf((char*)S5,"%d",Ricount);  //最大连续全白
     LCD_P8x16Str(45,2,"Ri:");
     LCD_P8x16Str(60,2,S5);
     
     sprintf((char*)S6,"%d",last_shizi_flag);   //BlackEndL - BlackEndR相邻中线偏差和
     LCD_P8x16Str(80,2,"sz:");
     LCD_P8x16Str(95,2,S6);
     
   
      
    sprintf((char*)S1,"%d",BlackEndL10); //1/410
     LCD_P8x16Str(0,4,"L:"); 
     LCD_P8x16Str(15,4,S1); 
     
   
     
     sprintf((char*)S3,"%d",BlackEndR70);//3/470
     LCD_P8x16Str(90,4,"R:");
     LCD_P8x16Str(105,4,S3);
    // sprintf((char*)S8,"%d",TopE2); //方差
    // LCD_P8x16Str(45,4,"O:");
    // LCD_P8x16Str(60,4,S8);

    //sprintf((char*)S8,"%d",error_40); //方差
    //LCD_P8x16Str(90,4,"P:");
    //LCD_P8x16Str(105,4,S8);
     
   sprintf((char*)S9,"%d", Round_flag );//zhangaiflag); stop_f       //顶部偏差和
     
      LCD_P8x16Str(0,6,"e:");
      LCD_P8x16Str(15,6,S9);
     
     sprintf((char*)S10,"%d",RoadType);       //底部偏差和
     LCD_P8x16Str(64,6,S10);
if(i>15)

    { 
          i=0;
LCD_Refresh_Gram();
}

}*/









