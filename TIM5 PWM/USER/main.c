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
extern int Power;										//油门
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
// 	if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
// 		{
// // 				for(i=0;i<30;i++)
// // 						{
// // 							printf("tmp_buf[%d]==---->%d\r\n",i,tmp_buf[i]-48);		
// // 						}		
// 					NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
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
	delay_init();	    	 //延时函数初始化	
	LED_Init();
	LED0 = 0;delay_ms(200); LED0 = 1;
	LED1 = 0;delay_ms(200); LED1 = 1;
	LED2 = 0;delay_ms(200); LED2 = 1;
	LED3 = 0;delay_ms(200); LED3 = 1;
	 
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 	//串口初始化为9600
	 
	EXTIX_Init( );
 	NRF24L01_Init();    	//初始化NRF24L01 
	
	NRF24L01_RX_Mode();		
 	while(NRF24L01_Check() == 1)//检查NRF24L01是否在位.	
	{
			LED3=!LED3;				printf("NRF ERROR!!\r\n");

			delay_ms(100 );
	}
	printf("NRF okk!!\r\n");
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
	
	
	TIM2_PWM_Init(999,9);  //PWM OUT
	TIM3_Int_Init(0xffff,71);  //做计时器用
	TIM4_Int_Init(49,7199);	  		//PID调速中断
	IIC_Init();	 
	InitMPU6050();
	Init_HMC5883();
	
	suanfa_GetOrigin(); //初始欧拉角 

 while(1){
			suanfa();
			printf("%.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	 
			if(NRF_flag == 1)
			{
					if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
		      {
						
						for(i=0;i<4;i++)
						{
							printf("tmp_buf[%d]  %x\r\n",i,tmp_buf[i]);		
						}
					NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
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
			
					
			
				
			//nrf中断处理结束程序 必须 BUG！！	
// 			sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
// 			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
// 			NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
			NRF_flag = 0;
			TIM_Cmd(TIM4, ENABLE);
			Nrf_Star;
			LED2 =!LED2;	
		}		
	}
}
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)		//PR 1 发生中断请求
	{		
			EXTI_ClearITPendingBit(EXTI_Line5);  //清除c5上的中断标志位
			
			TIM_Cmd(TIM4, DISABLE);
			NRF_flag= 1;
			Nrf_Stop;
	}
	LED2 =!LED2;
}


void TIM4_IRQHandler(void)   //TIM4中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			
			PID_Deal(cord);
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 

}
