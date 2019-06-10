
#ifndef __KEY_MENU_H__
#define __KEY_MENU_H__


extern float Error;
extern uint16_t Error1;
extern float EPerCount;
/* 按下ADD键*/ 
void onAddKey(void) ;

/* 按下SUB键*/ 
void onSubKey(void) ;

/* 按下OK键*/ 
void onOkKey(void) ;

/* 按下Cancel键*/ 
void onRightKey(void);

/* 菜单框架显示*/ 
void  OLED_SHOW_SMENU(void);
void  OLED_SHOW_MENU1(void);
void  OLED_SHOW_MENU2(void);
void  OLED_SHOW_MENU3(void);
void  OLED_SHOW_MENU4(void);

/*加键*/
void menuAdd(void);


/*减键*/
void menuSub(void);


/* O K */
void menuLEFT(void);


/* CANCEL */
void menuRIGHT(void);

/*下一行*/
void onNEXT(void);

/*上一行*/
void onPREV(void);
void onDDDD(void);
////////////////////////////////////////////
//函数名：Key_Init
//功  能：按键初始化
////////////////////////////////////////////

void Funcation_key(void);
void key(void);
extern int currentFocusMenu;
#endif  




