
#ifndef __KEY_MENU_H__
#define __KEY_MENU_H__


extern float Error;
extern uint16_t Error1;
extern float EPerCount;
/* ����ADD��*/ 
void onAddKey(void) ;

/* ����SUB��*/ 
void onSubKey(void) ;

/* ����OK��*/ 
void onOkKey(void) ;

/* ����Cancel��*/ 
void onRightKey(void);

/* �˵������ʾ*/ 
void  OLED_SHOW_SMENU(void);
void  OLED_SHOW_MENU1(void);
void  OLED_SHOW_MENU2(void);
void  OLED_SHOW_MENU3(void);
void  OLED_SHOW_MENU4(void);

/*�Ӽ�*/
void menuAdd(void);


/*����*/
void menuSub(void);


/* O K */
void menuLEFT(void);


/* CANCEL */
void menuRIGHT(void);

/*��һ��*/
void onNEXT(void);

/*��һ��*/
void onPREV(void);
void onDDDD(void);
////////////////////////////////////////////
//��������Key_Init
//��  �ܣ�������ʼ��
////////////////////////////////////////////

void Funcation_key(void);
void key(void);
extern int currentFocusMenu;
#endif  




