#include "common.h"
#include "fire_img2sd.h"
#include "ff.h"
#include "FIRE_camera.h"       //摄像头总头文件

FATFS firefs;    //文件系统
FIL   firesrc;  //文件
void img_sd_init(void)
{
    int   fireres;
    char  myfilename[20];
    uint16 imgsize[] = {CAMERA_H, CAMERA_W};
    uint32 mybw;
    uint32 Imag_num = 0;
    
    f_mount(0, &firefs);
    
    do
    {
        Imag_num ++;
        sprintf(myfilename, "0:/Fire%d.sd", Imag_num);
        fireres = f_open( &firesrc , myfilename, FA_CREATE_NEW | FA_WRITE);
        if(firefs.fs_type == 0)
        {
            firesrc.fs = 0;
            return;
        }
    }while(fireres == FR_EXIST);        //如果文件存在，则命名继续加1
    
    if ( fireres == FR_OK )
    {
        fireres = f_write(&firesrc, imgsize, sizeof(imgsize), (uint32 *)&mybw);  //先写入高和宽，方便上位机处理
    }
    else
    {
        f_close(&firesrc);
        firesrc.fs = 0;
    }
}
void img_sd_save(uint8 * imgaddr)
{
#define SD_ONEWRITE_MAX 600
    int   fireres;
    uint32 mybw;
    static uint8 time = 0;
    uint32 size = CAMERA_SIZE;
    if(firesrc.fs != 0)
    {
        time ++;
        while(size > SD_ONEWRITE_MAX)
        {
            fireres = f_write(&firesrc, imgaddr, SD_ONEWRITE_MAX , (uint32 *)&mybw);  
            imgaddr +=   SD_ONEWRITE_MAX;  
            size -=  SD_ONEWRITE_MAX; 
        }
        
        fireres = f_write(&firesrc, imgaddr, size , (uint32 *)&mybw);  
        
        if(fireres != FR_OK)
        {
            f_close(&firesrc);
            firesrc.fs = 0;
        }
        
        if(time > 30)
        {
            time = 0 ;
            f_sync(&firesrc);
        }
    }
}