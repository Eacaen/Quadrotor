#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "pwm.h"
#include "pid.h"
#include "24l01.h" 
#include "exti.h"
#include "math.h"
#include "stdlib.h"
#include "hmc.h"
#include "mpu6050.h"
#include "myiic.h"
#include "suanfa.h"
#include "motor.h"

extern EularAngle EA;
extern EularAngle EA_Origin;
extern int Power;										//����
extern float KP;
extern float TD;


float cord[3];
double Speed_Target=90;

u8 NRF_flag = 0;
u8 tmp_buf[30]={0};

void msg_check_2()
{
	
	char a[6][40];
	int i=0,t=0,u=0,num=0;
// 	if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
// 		{
// // 				for(i=0;i<30;i++)
// // 						{
// // 							printf("tmp_buf[%d]==---->%d\r\n",i,tmp_buf[i]-48);		
// // 						}		
// 					NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
// 		}
// 		
// 		i=0;
		
	while(tmp_buf[t] != NULL)
			{		
					if(tmp_buf[t] != 32)
					{
						a[u][num]=tmp_buf[t];
						num++;
						t++;
					}
					while(tmp_buf[t] == 32)
					{
						t++;
						if(tmp_buf[t] != 32)
						{
							a[u][num]='\0';
							u++;
							num=0;
							break;
						}

					}
					
			}
	Power = atof(a[0]);
	KP = atof(a[1]);
	TD= atof(a[2]);			
	for(i=0;i<u+1;i++)
		{
			for(num = 0;num< 20;num++)a[i][num] = NULL;
		}		
	i=0;
	t=0;
	u=0;
	num=0;
}


 int main(void)
 {		
	u16 sta , i=0;
	int t = 200;
	delay_init();	    	 //��ʱ������ʼ��	
	LED_Init();
	LED0 = 0;delay_ms(200); LED0 = 1;
	LED1 = 0;delay_ms(200); LED1 = 1;
	LED2 = 0;delay_ms(200); LED2 = 1;
	LED3 = 0;delay_ms(200); LED3 = 1;
	 
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
	 
	EXTIX_Init( );
 	NRF24L01_Init();    	//��ʼ��NRF24L01 
	
	NRF24L01_RX_Mode();		
 	while(NRF24L01_Check() == 1)//���NRF24L01�Ƿ���λ.	
	{
			LED3=!LED3;				printf("NRF ERROR!!\r\n");

			delay_ms(100 );
	}
	printf("NRF okk!!\r\n");
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
	
	
	TIM2_PWM_Init(999,9);  //PWM OUT
	TIM3_Int_Init(0xffff,71);  //����ʱ����
	TIM4_Int_Init(49,7199);	  		//PID�����ж�
	IIC_Init();	 
	InitMPU6050();
	Init_HMC5883();
	
	suanfa_GetOrigin(); //��ʼŷ���� 

 while(1){
			suanfa();
			printf("%.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	 
			if(NRF_flag == 1)
			{
					if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
		      {
						
						for(i=0;i<4;i++)
						{
							printf("tmp_buf[%d]  %x\r\n",i,tmp_buf[i]);		
						}
					NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		      }
					
			if( tmp_buf[0] != 0xff && tmp_buf[1] != 0xff ) msg_check_2();
			
      if( tmp_buf[0] == 0xff && tmp_buf[1] == 0xff ) 	
			{
				
				if(tmp_buf[2] == 0x01 )Power += 100; Power = (Power>900)?900:Power;
// 			  if(tmp_buf[2] == 0x02 )Power -= 50;  Power = (Power<0)? 0:Power;

        if(tmp_buf[2] == 0x02 )Power = 0; 

				
// 				if(tmp_buf[3] == 0x01 )Power += 100; Power = (Power>900)?900:Power;
// 				 if(tmp_buf[3] == 0x02 )Power -= 100;Power = (Power<10)? 0:Power;
// 				
// 				if(tmp_buf[4] == 0x01 )Power += 100; Power = (Power>900)?900:Power;
// 			 if(tmp_buf[4] == 0x02 )Power -= 100;  Power = (Power<10)? 0:Power;
				
			}
			
					
			
				
			//nrf�жϴ���������� ���� BUG����	
// 			sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
// 			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
// 			NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
			NRF_flag = 0;
			TIM_Cmd(TIM4, ENABLE);
			Nrf_Star;
			LED2 =!LED2;	
		}		
	}
}
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)		//PR 1 �����ж�����
	{		
			EXTI_ClearITPendingBit(EXTI_Line5);  //���c5�ϵ��жϱ�־λ
			
			TIM_Cmd(TIM4, DISABLE);
			NRF_flag= 1;
			Nrf_Stop;
	}
	LED2 =!LED2;
}


void TIM4_IRQHandler(void)   //TIM4�ж�
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			
			PID_Deal(cord);
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 

}
