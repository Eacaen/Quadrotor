#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED0 PDout(2)
#define LED1 PCout(12)
#define LED2 PCout(11)
#define LED3 PCout(10)

void LED_Init(void);//��ʼ��
void Only_Led0(void);
void Only_Led1(void);
void Only_Led2(void);
void Only_Led3(void);
		 				    
#endif
