/*********************************************************
------------------------------------------------
ʹ��˵����SD�����İ�ר�ó���
OLED��Դʹ��3.3V��
----------------
G    ��Դ��
3.3V ��3.3V��Դ
CS   �ѽӵأ����ý�
*********************************************************/
#include "common.h"
#include "MK60_gpio.h"
#include "MK60_port.h"
#include  "OLED.h"


#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF

/*
4��SPIʹ��˵����
VBT ���ڲ�DC-DC��ѹ��3.3~4.3V�����ʹ��5V��ѹ��Ϊ���������һ��100~500ŷ�ĵ���
VCC ���ڲ��߼���ѹ 1.8~6V
GND ��

BS0 �͵�ƽ
BS1 �͵�ƽ
BS2 �͵�ƽ

CS  Ƭѡ�ܽ�
DC  ��������ѡ��ܽ�
RES ģ�鸴λ�ܽ�
D0��SCLK�� ��ʱ�ӽţ���MCU����
D1��MOSI�� ����������������ݽţ���MCU����

D2 ����
D3-D7 �� �͵�ƽ �� Ҳ�����գ��������Ϊ�͵�ƽ
RD  �͵�ƽ ��Ҳ�����գ��������Ϊ�͵�ƽ
RW  �͵�ƽ ��Ҳ�����գ��������Ϊ�͵�ƽ
RD  �͵�ƽ ��Ҳ�����գ��������Ϊ�͵�ƽ
*/

#define X_WIDTH 128
#define Y_WIDTH 64
//======================================
const byte F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};

//======================================================
// 128X64IҺ���ײ�����[8X16]�����
// �����: powerint
// ��  ��: [8X16]�����ַ�����ģ���� (����ȡģ,�ֽڵ���)
// !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//======================================================
const byte F8X16[]=
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94

};


const byte LIBLOGO96x40[] = {
 /*--  ������һ��ͼ��C:\Users\Administrator\Desktop\OLED@5110\lanz.bmp  --*/
/*--  ����x�߶�=96x40  --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0x60,0x20,0x50,0x20,0x30,0x2C,0x3C,
0x3C,0x3C,0x3C,0x7C,0x7C,0x7C,0xF8,0xF8,0xF0,0xF0,0xE0,0xC0,0xC0,0x80,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x50,0xD8,0x58,0x98,0xD0,0x10,0xD0,0xD0,0x50,0x58,0x58,
0x58,0x50,0x00,0x00,0xB0,0xB0,0x90,0x90,0x90,0xD8,0xD8,0x90,0x90,0x90,0xB0,0xB0,
0x00,0x00,0xF0,0xF0,0xB0,0xB0,0xB0,0xF8,0xF8,0xF0,0xB0,0xB0,0xF0,0xF0,0x00,0x00,
0x08,0x18,0x18,0x18,0x18,0x18,0x18,0x98,0xD8,0xD8,0x78,0x38,0x18,0x00,0x00,0x00,
0x80,0xE0,0xF0,0xFC,0x7E,0x9F,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0xC3,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,
0xFC,0xF0,0x80,0x00,0x00,0xC1,0xFB,0xFD,0xCC,0xFD,0xFC,0xCD,0xFF,0xFF,0xCE,0xCF,
0xFE,0xFB,0x00,0x00,0xFF,0xFF,0xCC,0xCC,0xCC,0xFF,0xFF,0xCC,0xCC,0xCC,0xFF,0xFF,
0x00,0x00,0x1F,0x1F,0x1B,0x1B,0x1B,0x7F,0x7F,0x5F,0x5B,0x5B,0x5F,0x5F,0x40,0x00,
0x06,0x46,0x46,0x46,0x46,0x46,0x46,0x47,0x7F,0x7F,0x06,0x06,0x06,0x00,0x00,0x00,
0x0F,0xFF,0xFF,0xFF,0xE0,0x15,0xFF,0xFF,0xFE,0xFE,0xFE,0xFC,0xFE,0xFC,0x7C,0x1C,
0x1C,0x1C,0x0E,0x0C,0x0E,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x0F,0x0F,
0x0F,0x1F,0x2F,0x00,0x00,0x0C,0x8C,0x8C,0x8C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,
0x0C,0x0C,0x8C,0x0C,0x0C,0x0C,0x8C,0x8C,0x8C,0x0C,0x8C,0x0C,0x0C,0x0C,0x8C,0x0C,
0x0C,0x8C,0x8C,0x8C,0x8C,0x8C,0x8C,0x0C,0x0C,0x0C,0x0C,0x8C,0x8C,0x8C,0x8C,0x8C,
0x8C,0x8C,0x0C,0x0C,0x0C,0x8C,0x8C,0x8C,0x8C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00,
0x00,0x00,0x01,0x07,0x1F,0x1F,0x7C,0x71,0xE7,0xDF,0x9F,0xBF,0x7F,0x7F,0xF8,0xF0,
0xE0,0xE0,0xE0,0xC0,0xC0,0xC0,0xC0,0xC0,0x80,0x40,0x00,0x80,0x40,0x40,0x20,0x10,
0x00,0x00,0x00,0x00,0x00,0xC0,0xFF,0xFF,0xBF,0x80,0x80,0xE0,0xE0,0xF0,0x78,0x4C,
0x46,0xFF,0xFF,0xFD,0x00,0x00,0xFF,0xFF,0x03,0x1F,0x3F,0xFF,0xFC,0xE1,0xFF,0x01,
0x00,0xFF,0xFF,0xFF,0x01,0x01,0xFF,0xFF,0x7E,0x00,0x80,0xC7,0xF1,0xFD,0xBF,0xDF,
0xC7,0x01,0xF8,0xFC,0xFF,0x87,0x83,0xC3,0xFF,0x7F,0x3E,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x03,0x07,0x06,0x06,
0x06,0x05,0x06,0x05,0x06,0x05,0x02,0x02,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,
0x00,0x01,0x01,0x01,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x00,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x00,0x00,0x01,0x03,0x03,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

};
void LCD_WrDat(byte data) //����
{
    byte i=8;
    //LCD_CS=0;;
//    GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(12));;;;
    OLED_DC=1;
    asm("nop");
//    GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(15));;;;
    OLED_D0=0;
    asm("nop");
  while(i--)
  {
    if(data&0x80){OLED_D1=1;}
    else{OLED_D1=0;}
    OLED_D0=1;
    asm("nop");;;;
		//asm("nop");
    OLED_D0=0;
    data<<=1;
  }
	//LCD_CS=1;
}


void LCD_WrCmd(byte cmd)//����
{
  byte i=8;

  //LCD_CS=0;;
//  GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(12));;;;;
//  GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(15));;;;;
  OLED_DC=0;
  OLED_D0=0;
  //asm("nop");
  while(i--)
  {
    if(cmd&0x80){OLED_D1=1;}
    else{OLED_D1=0;}
    OLED_D0=1;

    asm("nop");;;; //4
		//asm("nop");
    OLED_D0=0;
    cmd<<=1;;;;;
  }
	//LCD_CS=1;
}
void LCD_Set_Pos(byte x, byte y) //�йأϣ̣ţ�Һ����ʾ��ַ�Ķ�λ������
{
  LCD_WrCmd(0xb0+y); //����ҳ��ַ ÿҳ8��
  LCD_WrCmd(((x&0xf0)>>4)|0x10); //�����и���λ��ַ
  LCD_WrCmd((x&0x0f)|0x00);  //�����е���λ��ַ
}


void LCD_Fill(byte bmp_data)
{
	byte y,x;

	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);  //����ÿҳX��ʼ��ַ����ͷ��ʼ
		LCD_WrCmd(0x01); //����λ�е�ַ����
		LCD_WrCmd(0x10); //����λ�е�ַ����
		for(x=0;x<X_WIDTH;x++)    //X_WIDTH=128,д���ݽ�ÿһ���������Ϊ0x00
			LCD_WrDat(bmp_data);
	}
}

void LCD_CLS(void)  //����
{
	byte y,x;
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(0);
	}
}
void LCD_DLY_ms(word ms)
{
  word a;
  while(ms)
  {
    a=13350;
    while(a--);
    ms--;
  }
  return;
}
void OLED_Init(void)
{

          gpio_init (PTA13, GPO,1);
          gpio_init (PTA14, GPO,1);
          gpio_init (PTA15, GPO,1);
          gpio_init (PTA16, GPO,1);


	//LCD_CS=1;	//Ԥ��SLK��SSΪ�ߵ�ƽ

	OLED_RST=0;
	LCD_DLY_ms(50);
	OLED_RST=1;
  LCD_WrCmd(0xae);//--turn off oled panel ����ʾ
  LCD_WrCmd(0x00);//---set low column address ���õ��е�ַ
  LCD_WrCmd(0x10);//---set high column address ���ø��е�ַ
  LCD_WrCmd(0x40);//--set start line address ��ʼ�� Set Mapping RAM Display Start Line (0x00~0x3F)
  LCD_WrCmd(0x81);//--set contrast control register ��������ƼĴ���
  LCD_WrCmd(0xcf); // Set SEG Output Current Brightness ���������������
  LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
  LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
  LCD_WrCmd(0xa6);//--set normal display ����������ʾ
  LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64) ���ö�·������
  LCD_WrCmd(0x3f);//--1/64 duty
  LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  LCD_WrCmd(0x00);//-not offset
  LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency ʱ��Ƶ��
  LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  LCD_WrCmd(0xd9);//--set pre-charge period  �������
  LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  LCD_WrCmd(0xda);//--set com pins hardware configuration
  LCD_WrCmd(0x12);
  LCD_WrCmd(0xdb);//--set vcomh
  LCD_WrCmd(0x40);//Set VCOM Deselect Level
  LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  LCD_WrCmd(0x02);//
  LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
  LCD_WrCmd(0x14);//--set(0x10) disable
  LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4������ʾ/0xa5�������)
  LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6������ʾ/a7��ɫ��ʾ)
  LCD_WrCmd(0xaf);//--turn on oled panel ����ʾ
  LCD_Fill(0x00);  //��ʼ����
  LCD_Set_Pos(0,0);

}

//==============================================================
//�������� void LCD_PutPixel(byte x,byte y)
//��������������һ���㣨x,y��
//��������ʵ����ֵ(x,y),x�ķ�Χ0��127��y�ķ�Χ0��64
//���أ���
//==============================================================
void LCD_PutPixel(byte x,byte y)
{
	byte data1;  //data1��ǰ�������

                               //����ÿҳx��ʼ��ַ����ͷ��ʼ
        data1 = 0x01<<(y%8); //ȷ��ÿҳ�е��λ��
        LCD_WrCmd(0xb0+(y>>3));  //�ڵڼ�ҳ��ʾ
        LCD_WrCmd(((x&0xf0)>>4)|0x10);
        LCD_WrCmd((x&0x0f)|0x00);
        LCD_WrDat(data1); //��y=??
}
//==============================================================
//�������� void LCD_Rectangle(byte x1,byte y1,
//                   byte x2,byte y2,byte color,byte gif)
//��������������һ��ʵ�ľ���
//���������Ͻ����꣨x1,y1��,���½����꣨x2��y2��
//      ����x1��x2�ķ�Χ0��127��y1��y2�ķ�Χ0��63������ʵ����ֵ
//���أ���
//==============================================================
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif)
{
	byte n,m;

        LCD_Set_Pos(x1,y1>>3);
          for(n=x1;n<=x2;n++)
          {
                  LCD_WrDat(0x01<<(y1%8));
                  if(gif == 1) 	LCD_DLY_ms(50);
          }


	LCD_Set_Pos(x1,y2>>3);
        for(n=x1;n<=x2;n++)
        {
                LCD_WrDat(0x01<<(y2%8));

                if(gif == 1) 	LCD_DLY_ms(5);
        }
     /*    LCD_Set_Pos(x1,y1>>3);
          LCD_WrDat(0xff);
         LCD_Set_Pos(x1,y2>>3);
          LCD_WrDat(0x07);
         LCD_Set_Pos(x2,y1>>3);
          LCD_WrDat(0xff);
         LCD_Set_Pos(x2,y2>>3);
          LCD_WrDat(0x07); */

}
//==============================================================
//��������LCD_P6x8Str(byte x,byte y,byte *p)
//����������д��һ���׼ASCII�ַ���
//��������ʾ��λ�ã�x,y����yΪҳ��Χ0��7��Ҫ��ʾ���ַ���
//���أ���
//==============================================================
void LCD_P6x8Str(byte x,byte y,byte ch[]) //ָ��λ����ʾ6*8�ַ�
{
  byte c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}   //0~128��6���������126���������126�����һ���ַ���ʾ������(ӦΪ121)
    LCD_Set_Pos(x,y);
    for(i=0;i<6;i++)
      LCD_WrDat(F6x8[c][i]);  //����λ���ϣ�����λ����    ������ʾ
    x+=6;
    j++;
  }
}
/********************************************************/
//   д��6*8�ַ�������λ�Ͷ�λ��ʾ��
/********************************************************/
void LCD_single_P6x8Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0;
  for(;ch[j]!='\0';j++)
  {
      if(ch[1]=='\0')
     {
        c =ch[j]-32;
        if(x>126){x=0;y++;}   //0~128��6���������126���������126�����һ���ַ���ʾ������(ӦΪ121)
        LCD_Set_Pos(x,y);
        for(i=0;i<6;i++)
          LCD_WrDat(F6x8[c][i]);  //����λ���ϣ�����λ����    ������ʾ
        LCD_Set_Pos(x+6,y);
        for(i=0;i<6;i++)
          LCD_WrDat(F6x8[0][i]);
        LCD_Set_Pos(x+6+6,y);
        for(i=0;i<6;i++)
          LCD_WrDat(F6x8[0][i]);
        x+=6;

     }
     else
     {
       c =ch[j]-32;
       if(x>126){x=0;y++;}
       LCD_Set_Pos(x,y);
       for(i=0;i<6;i++)
        LCD_WrDat(F6x8[c][i]);
       x+=6;
     }
  }
}
//==============================================================
//��������LCD_P8x16Str(byte x,byte y,byte *p)
//����������д��һ���׼ASCII�ַ���
//��������ʾ��λ�ã�x,y����yΪҳ��Χ0��7��Ҫ��ʾ���ַ���
//���أ���
//==============================================================
void LCD_P8x16Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    LCD_Set_Pos(x,y);
  	for(i=0;i<8;i++)
         LCD_WrDat(F8X16[c*16+i]);
  	LCD_Set_Pos(x,y+1);
  	for(i=0;i<8;i++)
  	  LCD_WrDat(F8X16[c*16+i+8]);
  	x+=8;
  	j++;
  }
}
//д��8*16�ַ�������ͬ�ַ�λ����ʾ��
void LCD_single_P8x16Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0,n;
 for(;ch[j]!='\0';j++)
  {
      if(ch[j+1]=='\0')
      {
        c =ch[j]-32;
        if(x>120){x=0;y++;}
        LCD_Set_Pos(x,y);
          for(i=0;i<8;i++)
           LCD_WrDat(F8X16[c*16+i]);
        LCD_Set_Pos(x,y+1);
          for(i=0;i<8;i++)
           LCD_WrDat(F8X16[c*16+i+8]);
        for(n=0;n<7-j;n++)
        {
          x=x+8;
          LCD_Set_Pos(x,y);
          for(i=0;i<8;i++)
           LCD_WrDat(0);
          LCD_Set_Pos(x,y+1);
          for(i=0;i<8;i++)
           LCD_WrDat(0);
        }
       }
        else
        {
           c =ch[j]-32;
           if(x>120){x=0;y++;}
           LCD_Set_Pos(x,y);
              for(i=0;i<8;i++)
               LCD_WrDat(F8X16[c*16+i]);
           LCD_Set_Pos(x,y+1);
              for(i=0;i<8;i++)
               LCD_WrDat(F8X16[c*16+i+8]);
           x+=8;
        }
  }

}

//������ֺ��ַ�����ַ���
void LCD_Print(byte x, byte y, byte ch[])
{
	byte ch2[3];
	byte ii=0;
	while(ch[ii] != '\0')
	{
		if(ch[ii] > 127)
		{
			ch2[0] = ch[ii];
	 		ch2[1] = ch[ii + 1];
			ch2[2] = '\0';			//����Ϊ�����ֽ�
			LCD_P14x16Str(x , y, ch2);	//��ʾ����
			x += 14;
			ii += 2;
		}
		else
		{
			ch2[0] = ch[ii];
			ch2[1] = '\0';			//��ĸռһ���ֽ�
			LCD_P8x16Str(x , y , ch2);	//��ʾ��ĸ
			x += 8;
			ii+= 1;
		}
	}
}

//==============================================================
//�������� void Draw_BMP(byte x,byte y)
//������������ʾBMPͼƬ128��64
//��������ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7
//���أ���
//==============================================================
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[])
{
  word ii=0;
  byte x,y;

  if(y1%8==0) y=y1/8;
  else y=y1/8+1;
	for(y=y0;y<=y1;y++)
	{
		LCD_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {
	    	LCD_WrDat(bmp[ii++]);
	    }
	}
}

void Draw_LibLogo(void)
{
  word ii=0;
  byte x,y;

  for(y=2;y<7;y++)
	{
            LCD_Set_Pos(0,y);
            for(x=16;x<112;x++)
            {
                LCD_WrDat(LIBLOGO96x40[ii++]);
            }
        }
}

uint8 LCD_GRAM[128][8];
//�����Դ�LCD
void LCD_Refresh_Gram(void)
{
	uint8 i,n;
	for(i=0;i<8;i++)
	{
		LCD_WR_Byte (0xb0+i,0);
		LCD_WR_Byte (0x00,0);
		LCD_WR_Byte (0x10,0);
		for(n=0;n<128;n++)LCD_WR_Byte(LCD_GRAM[n][i],1);
	}
}

//дһ���ֽ�
//0���� 1����
void LCD_WR_Byte(uint8 dat,uint8 cmd)
{
	uint8 i;
	OLED_DC=cmd; //D��?����?
//	OLED_CS=0;
	for(i=0;i<8;i++)
	{
		OLED_D0=0;
		if(dat&0x80)OLED_D1=1;
		else OLED_D1=0;
		OLED_D0=1;
		dat<<=1;
	}
//	OLED_CS=1;
	OLED_DC=1;
}


//����
void LCD_DrawPoint(uint8 x,uint8 y,uint8 t)
{
	uint8   pos,bx,temp=0;
	if(x>127||y>63)return;//3?3?��??�쨢?.
	pos=y/8;   //ҳ����  7-
	bx=y%8;
	temp=1<<bx;  //�㵹��  7-
	if(t)LCD_GRAM[x][pos]|=temp;
	else LCD_GRAM[x][pos]&=~temp;
}
