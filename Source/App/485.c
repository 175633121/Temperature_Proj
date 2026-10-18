/**********************************************************************
   *日期:2011/10	 2010.11.20安装
   *项目描述：聊城电网 485方案程序
   *硬件方案：stm32f103c6t6(双串口)	+8路模拟开关CD4051+	光耦6N137+隔离电源B0505+485收发器MAX1483
   *通讯每帖由0xff 0xaa +测试站地址级成
   *USART1用来接收模块数据 19200kpbs ;  USART3用于发数到上位机 9600kpbs
**********************************************************************/

#include "stm32f10x.h"

#define res1		GPIOA->ODR&=0xef;   		//PA4=0
#define res2		GPIOA->ODR&=0xdf;   		//PA5=0
#define res3		GPIOA->ODR&=0xbf;   		//PA6=0
#define res4		GPIOA->ODR&=0x7f;   		//PA7=0
#define res5		GPIOB->ODR&=0xFFFE;      	//PB0=0
#define res6		GPIOB->ODR&=0xFFFD;   		//PB1=0
#define send485		GPIOB->ODR|=0x1000;
#define rec485		GPIOB->ODR&=~0x1000;


//关接收
#define receiveoff	USART1->CR1 &=0XFFfb
//开接收
#define receiveon	USART1->CR1 |= ~0XFFfb

u8 temp;


//10M晶振、1ms延时	这里取200ms延时
void Delay(int count)
{
	 int n;
  	 for(;count!=0;count--)
	   for(n=2000; n != 0; n--)
	        ;
	 
}
void delay(unsigned int nCount)
{
  for(; nCount != 0; nCount--);
}

void delayus(unsigned char us)
{
	while(us--) ;
}



void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
    // Configure USART3 Rx (Pb.11) as input floating 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);  

  	// Configure USART3 Tx (Pb.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	  	
	// Configure USART1 Rx (PA.10) as alternate function push-pull 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
/*      
	//pa4-pa7控制复位引脚   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	
*/
	//pb13\14\15-4051地址选择  pb12-MAX485收发控制   pb0\pb1--模块复位
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

}

//系统中断管理
void NVIC_Configuration(void)
{ 
  
     NVIC_InitTypeDef NVIC_InitStructure;
  	/* Configure the NVIC Preemption Priority Bits */  
  	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif
	/*	// Enable the USART1 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	//HSE=10Mhz
  	RCC_HSEConfig(RCC_HSE_ON);
  	delay(1000);
  	HSEStartUpStatus = RCC_WaitForHSEStartUp(); 

	  if(HSEStartUpStatus == SUCCESS)
	  {
	 
	    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
	  
	    RCC_PCLK2Config(RCC_HCLK_Div1); 
	
	    RCC_PCLK1Config(RCC_HCLK_Div1);
	
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_2);
	
	    RCC_PLLCmd(ENABLE);
	
	    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)		;
	
	    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	    while(RCC_GetSYSCLKSource() != 0x08)   		;
	
	  } 

 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
	 						| RCC_APB2Periph_GPIOB	| RCC_APB2Periph_AFIO , ENABLE );

 	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3   , ENABLE );

}

void USART1_Configuration(void)
{
	USART_InitTypeDef 		USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitStructure.USART_BaudRate            = 19200  ;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx  ;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	  
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	USART_Init(USART1, &USART_InitStructure);    
	USART_Cmd(USART1, ENABLE);   
//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //
}

void USART3_Configuration(void)
{
	USART_InitTypeDef 		USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitStructure.USART_BaudRate            = 9600  ;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx|USART_Mode_Tx;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_ClockInit(USART3, &USART_ClockInitStructure);
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);     
}

/*
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
	//Read one byte from the receive data register 
	address = (USART_ReceiveData(USART1) & 0x7F);         
	
	// Clear the USART1 Receive interrupt 
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	flag=1;
	}
}
*/
//配置所有外设
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
	NVIC_Configuration();
	USART1_Configuration();
	USART3_Configuration();

}

int main(void)
{  
	unsigned char i,j,k;
	unsigned char buffer[8]={0};
	unsigned char address[3];

	Init_All_Periph();

	GPIOB->ODR=0;
//	send485;
	Delay(500);
 	while(1)
  	{

		i=0;
		//485置接收状态
		rec485;
		Delay(10);
		while(i<3)
		{
			while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != SET);
			address[i]=	USART3->DR;	
			if(address[i]==0xff)
			{
			  i=0;
			  address[i]=0xff;
			}
			  i++;
		} 
		//判断从机地址 
		if(address[2]==0x6&&address[0]==0xff&&address[1]==0xaa)
		{
 /*
			GPIOA->ODR&=~0X1000;			
			GPIOA->ODR|=0X0800;			
			for(j=0;j<6;)				
			{
				GPIOB->ODR =(j<<13)|0x1000;	  //CD4051地址选择、并置485发送
				k=0;
				while(k<8)
				{
					while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != SET);
					buffer[k]=	USART2->DR;	
					k++;	
				}		
				if(buffer[6]==0x32)	   //接收无错误
				{						
//					sum=buffer[3]*256+buffer[2]-2732;		
					for(k=0;k<8;k++)				
					{
						USART_SendData(USART1, buffer[k]);
						while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );  						
					}					
					j++;
					GPIOA->ODR&=~0X0800;	
//					Delay(50);		
							
				}
				//接收错乱情况
				else
				{
					//关接收
					receiveoff;
					temp =	USART2->DR;	
					
					for(i=0;i<8;i++)
					{	
						buffer[i]=0;
					}
					Delay(10);
	
					//开接收
					receiveon	 ;
				}
			}
		
		*/	   
		send485;
		Delay(5);
 		USART_SendData(USART3, 22);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET );  
		USART_SendData(USART3, 0xaa);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET );  
		Delay(300);	
		}	
		

	
	}	 								
		
 }


