#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	
#include "spi.h"
#include "exti.h"
#include "timer.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "STM32_I2C.h"
#include "math.h"
#include "24L01.h"	
#include "define.h"
#include "imu.h"
#include "control.h"

extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;  
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ HMC;
extern float rool,pitch;
extern int Motor1,Motor2,Motor3,Motor4; 
extern struct DATA_XYZ ACC;
extern float Pitch,Rool,Yaw;

u8 RX_buf[32]={0}; 
u8 TX_buf[32]={0};
u8 delays=10;	 
u8 wireless=0;
u32 loss_wireless=0;
extern u8 FLY_EN;  
int main(void)			
{	u8 start=0;	
    u8 i=0;	  
	Stm32_Clock_Init(9);	//ϵͳʱ������	 
	delay_init(72);	   		//��ʱ��ʼ��
	delay_ms(1000);			 //Ӳ������׼��
	RCC->APB2ENR|=1<<0;	  	  // ������ʱ��	
	AFIO->MAPR = 0x02000000;  // ����JTAG	 
	LED_Init();				   //LED��ʼ��
	Show_str_leds();		   //��˸LED��ص��������źŶԴ�
 	NRF24L01_Init();			//  ���߳�ʼ��
  	while( NRF24L01_Check())	LED_GRE_ON;	//ALL_LEDS_OFF();���ܻ��뷢�������ƥ��
 	NRF24L01_RX_Mode();	//�ó����ݰ�
    while(start==0)	
   {
   ALL_LEDS_ON();
   NRF24L01_RxPacket(RX_buf,TX_buf);
   if(RX_buf[0]==1)
     {  
	 start=1;
	 ALL_LEDS_OFF();
	 for(i=0;i<32;i++)RX_buf[i]=0;
	 }
    else
	  start=0;
    }
	delays=20;	     		
    MPU6050_INIT();delay_ms(50);
	MPU6050_INIT();delay_ms(50);
	Get_OFFSET(500); 	  //ȡ500�ε�ƽ��ֵ���ԽǶȵı궨	
    delays=10;			  	
	if(ACC_OFFSET.X==0&&ACC_OFFSET.Y==0){while(1)	LED_GRE_ON;}	 //LED_RED_ON; ������ʧ��,�������� 
	PID_INIT();	  
	TIM2_PWM_Init(999,2);  	 //42us��PWM  �ٷ�֮�ٶ�Ӧֵ 1000
	TIM3_Int_Init(20,7199); // ��ʱ2MS 	 
	delay_ms(800); 		//��ʱ��Ӳ������׼��
	ALL_LEDS_OFF();	  //�ص�2��LED
	while(1) 
	{	  
		if(wireless==1) LED_twinkle();	   //�����ź�������LED��˸
	} 		  		 
}
//////��������������������2ms���жϡ�����������������������������������////////
float 	angle=0;
long acc_hold[3]={0};
u16 tms=0;
float p_long=0.0,r_long=0.0;
float adj_r=0.0,adj_p=0.0;

void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{					       				   				     	    	 
    	MPU6050_READ();	   //��IIC��ȡ������MPU6050���ź�
		MPU6050_CONVENT(); //����ȡ���ź�ת���ɽǶ���Ǽ��ٶ�
		ACC_SMOOTH(10);  // �Ƕȵ�ƽ���˲�
		RX_buf[31]= 0;   
		MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);	//������ת�ɻ���		
		NRF24L01_RxPacket(RX_buf,TX_buf);	// ��ȡ�����ź�			   		
		if(RX_buf[31]==0)		  //  δ����������
		{
			loss_wireless++;
		if(	FLY_EN==1&&loss_wireless>50)  //�źŶ�ʧ 100ms 
			{
				wireless=0;
			
			}
		}
		else if(RX_buf[31]==0XAA)  // ��������  
		{
			READ_CONTROL_COMMAND(RX_buf); //ת����ȡ�ķ����ź�	
			LED_GRE_OFF;	
			loss_wireless=0;
			wireless=1;	
		} 	
		STABLE_WITH_PID();	  //�������ź��봫�����ź��ںϣ���PID�����������Ʒ���������      			   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    			    		  			   
}
 
