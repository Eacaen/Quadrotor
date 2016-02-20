#include "data_transfer.h"
#include "allinclude.h"
u8 tmp_buf[30]={0};
T_RC_Data Rc_D;
u8 NRF_flag = 0;
extern int Power;
void Data_Receive_Anl()
{
	static u8 i =0 ;
		if(NRF_flag == 1)
					{
							if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
							{
								
								while(tmp_buf[i]!= '\0')
								{
									printf("tmp_buf[%d]  %x\r\n",i,tmp_buf[i]);
									i++;
								}
								i=0;
							NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
							}
							
		// 			if( tmp_buf[0] != 0xff && tmp_buf[1] != 0xff );
						
					if( tmp_buf[0] == 0xff && tmp_buf[1] == 0xff ) 	
					{
						
						Rc_D.THROTTLE	= (vs16)(tmp_buf[2]<<8)|(tmp_buf[3]);
		      	Rc_D.PITCH 		= (vs16)(tmp_buf[4]<<8)|(tmp_buf[5]);
						Rc_D.ROLL			= (vs16)(tmp_buf[6]<<8)|(tmp_buf[7]);
						Rc_D.YAW			= (vs16)(tmp_buf[8]<<8)|(tmp_buf[9]);
						
					}
					NRF_flag = 0;
					TIM_Cmd(TIM4, ENABLE);
					Nrf_Star;
					LED2 =!LED2;	
				}		
}
