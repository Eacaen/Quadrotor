#include "allinclude.h"


float cord[3];

extern EularAngle EA;
extern EularAngle EA_Origin;
extern int Power;	//����
extern float KP;
extern float TD;
extern u8 NRF_flag;
extern u8 tmp_buf[30];
extern u8 LOCK,UN_LOCK;
extern float Target_x,Target_y,Target_yaw;		//Ŀ��ֵ 


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

	IIC_Init();	 
	InitMPU6050();
	Init_HMC5883();
	
	suanfa_GetOrigin(); //��ʼŷ���� 
	
	LOCK = 1;
	UN_LOCK = 0;
	
	TIM4_Int_Init(49,7199);	  		//PID�����ж� ��������ʼ������ֹ��ϽǶ�У׼
	while(1){
		if(LOCK)
		{
			lock();
			LED0 = 0;delay_ms(200); LED0 = 1;
			Power = 0;
			Target_x = 0;
			Target_y = 0;
		}
		
		if(UN_LOCK)
		{
			suanfa();
			Data_Receive_Anl();
		}
		
// 		printf("--p\r\n");
// 			printf("%.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	 
			
					}
}
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)		//PR 1 �����ж�����
	{		
			EXTI_ClearITPendingBit(EXTI_Line5);  //���c5�ϵ��жϱ�־λ
			
			TIM_Cmd(TIM4, DISABLE);
			NRF_flag= 1;
// 			printf("%d\r\n",NRF_flag);
			Nrf_Stop;
	}
	LED2 =!LED2;
}


void TIM4_IRQHandler(void)   //TIM4�ж�
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
// 			Data_Receive_Anl();
			PID_Deal(cord);
// 			printf("p\r\n");
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 

}
