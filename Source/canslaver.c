/**********************************************************************
   *date:2011/10
   *brief:CAN×ÜÏß°æ²âÎÂ
   *USART1ÓÃÀ´½ÓÊÕÄ£¿éÊý¾Ý 19200kpbs ;  can½Ó¿Ú·¢ËÍÊý¾Ý
**********************************************************************/

#include "stm32f10x.h"

#define res1		GPIOA->ODR&=0xef;   		//PA4=0
#define res2		GPIOA->ODR&=0xdf;   		//PA5=0
#define res3		GPIOA->ODR&=0xbf;   		//PA6=0
#define res4		GPIOA->ODR&=0x7f;   		//PA7=0
#define res5		GPIOB->ODR&=0xFFFE;      	//PB0=0
#define res6		GPIOB->ODR&=0xFFFD;   		//PB1=0

#define D4on		GPIOB->ODR|=0X0400			 //pb10
#define D4off		GPIOB->ODR&=~0X0400
#define D3on		GPIOB->ODR|=0X0800			 //pb11
#define D3off		GPIOB->ODR&=~0X0800

//¹Ø½ÓÊÕ
#define receiveoff	USART1->CR1 &=0XFFfb
//¿ª½ÓÊÕ
#define receiveon	USART1->CR1 |= ~0XFFfb

char count=0,setcount=0;	   // ½ÓÊÕ¡¢¶¨Ê±¡¢ÉèÖÃ
u8 temp;
unsigned char address=100;
  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
volatile TestStatus TestRx;			
	  
unsigned char canrec[4][8]=	{{0},{0},{0},{0}};


//10M¾§Õñ¡¢1msÑÓÊ±	ÕâÀïÈ¡200msÑÓÊ±
void Delay(int count)
{
	 int n;
  	 for(;count!=0;count--)
//	   for(n=2000; n != 0; n--)
		for(n=3500; n != 0; n--)
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

   	// Configure USART1 Rx (PA.10) as alternate function push-pull 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10		;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	// Configure USART1 Tx (PA.09) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//pa4-pa7¿ØÖÆ¸´Î»Òý½Å   pa8 rs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	//pb13\14\15-4051µØÖ·Ñ¡Ôñ  pb0\pb1--Ä£¿é¸´Î»
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_0|GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

}
/*	GPIO_ReadInputDataBit
void DMA_Configuration()
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40004804;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
//	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);  //ÔÊÐíDMA´«ÊäÍê³ÉÖÐ¶Ï		
//	DMA_Cmd(DMA1_Channel3,  ENABLE);	
}
*/
//ÏµÍ³ÖÐ¶Ï¹ÜÀí
void NVIC_Configuration(void)
{ 
  
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//* Enable the USART3 Interrupt 
}

//ÅäÖÃÏµÍ³Ê±ÖÓ,Ê¹ÄÜ¸÷ÍâÉèÊ±ÖÓ
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	//HSE=10Mhz
  	RCC_HSEConfig(RCC_HSE_ON);
  	delay(1000);
  	HSEStartUpStatus = RCC_WaitForHSEStartUp(); 

	  if(HSEStartUpStatus == SUCCESS)
	  {
	 
	    RCC_HCLKConfig(RCC_SYSCLK_Div2); 
	  
	    RCC_PCLK2Config(RCC_HCLK_Div2); 
	
	    RCC_PCLK1Config(RCC_HCLK_Div1);
	
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);//12Mhz 2±¶Æµ pllclk= 24Mhz
	
	    RCC_PLLCmd(ENABLE);
	
	    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)		;
	
	    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	    while(RCC_GetSYSCLKSource() != 0x08)   		;
	
	  } 

 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
	 						| RCC_APB2Periph_GPIOB	| RCC_APB2Periph_AFIO , ENABLE );

  	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
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
	USART_InitStructure.USART_Mode                = USART_Mode_Rx ;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	  
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	USART_Init(USART1, &USART_InitStructure);    
	USART_Cmd(USART1, ENABLE);   
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); 
}


void TIM_Configuration()
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period=1000;		 //ARRµÄÖµ
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //²ÉÑù·ÖÆµ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //ÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM2,0x2b33,TIM_PSCReloadMode_Immediate);//Ê±ÖÓ·ÖÆµÏµÊý1000£¬ËùÒÔ¶¨Ê±Æ÷Ê±ÖÓÎª110.592Khz
	TIM_ARRPreloadConfig(TIM2, ENABLE);//½ûÖ¹ARRÔ¤×°ÔØ»º³åÆ÷
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
//	TIM_Cmd(TIM2, ENABLE);	//¿ªÆôÊ±Ó	
}
void CAN_Config(void)
{
   CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  		GPIO_InitTypeDef GPIO_InitStructure;
	// Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* CAN register init */
  CAN_DeInit(CAN1);

  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  
  /* Baudrate = 500kbps*/
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler=20;
  CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;

  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;


  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* transmit */
  TxMessage.StdId=address;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.DLC=8;
  TxMessage.Data[0]=0xCA;
  TxMessage.Data[1]=0xFE;
  TxMessage.Data[2]=0x34;
  TxMessage.Data[3]=0x0b;
  TxMessage.Data[4]=0xaa;
  TxMessage.Data[5]=0x55;
  TxMessage.Data[6]=0x32;
  TxMessage.Data[7]=0x34;


	  /* receive */
	RxMessage.StdId=0x00;
  	RxMessage.RTR=CAN_RTR_DATA;
	RxMessage.IDE=CAN_ID_STD;
	RxMessage.DLC=8;
	RxMessage.Data[0]=0x00;
	RxMessage.Data[1]=0x00;
	RxMessage.Data[2]=0x00;
	RxMessage.Data[3]=0x00;
	RxMessage.Data[4]=0x00;
	RxMessage.Data[5]=0x00;
	RxMessage.Data[6]=0x00;
	RxMessage.Data[7]=0x00;


}

//ÅäÖÃËùÓÐÍâÉè
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
//	NVIC_Configuration();
	USART1_Configuration();
	CAN_Config();
//	TIM_Configuration();
//	DMA_Configuration();
}
//void CAN_send(unsigned char add,unsigned char data1,unsigned char data2)


/*
void CAN1_RX0_IRQHandler(void)
{
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  if ((RxMessage.StdId == 0x64)&&(RxMessage.IDE == CAN_RTR_REMOTE) && (RxMessage.DLC == 8))
  {
			led2on	 ;
		 	Delay(550);
			led2off	  ;
  }
}
*/
int main(void)
{  
	unsigned char i,j,k;
//	unsigned char buffer[8]={0},rec[2];
//	int sum=0;
	Init_All_Periph();
	//Áù¸öÄ£¿é¸´Î»
	GPIOA->ODR=0XF0;
	GPIOB->ODR=0X03;



 	while(1)
  	{
	
		   while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1) && (i != 0xFFff))
		  {
		    i++;
		  }
		  i=0;

		  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		  for(i=0;i<8;i++)
		  {
		  		canrec[0][i]=RxMessage.Data[i];
		  }	
	  	  
		  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		  for(i=0;i<8;i++)
		  {
		  		canrec[1][i]=RxMessage.Data[i];
		  }			 
		  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		  for(i=0;i<8;i++)
		  {
		  		canrec[2][i]=RxMessage.Data[i];
		  }	
			D4on	 ;
		 	Delay(550);
			D4off	  ;	
	 /*		  for(i=0;i<8;i++)
		  {
		  		canrec[2][i]=RxMessage.Data[i];
		  }			 
		  if(canrec[0][1]>18) 	
		  {	
		      for(i=1;i<(canrec[0][1]+6)/8;i++)
			  {
			  		canrec[i][i]=RxMessage.Data[i];
			  }


		  }
 */
		if ((RxMessage.IDE==CAN_ID_STD)&&(RxMessage.DLC==8))
		{
			D4on	 ;
		 	Delay(550);
			D4off	  ;
		}

/*

		k=0;
		
			for(j=0;j<6;)
			{
				
				if(j<2)
				{
					res1;
					Delay(4000);
				}
				GPIOB->ODR =j<<13;	  //CD4051µØÖ·Ñ¡Ôñ¡¢²¢ÖÃ485·¢ËÍ
				k=0;
				while(k<8)
				{
					while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != SET);
					buffer[k]=	USART1->DR;	
					k++;
			
				}
			
				if(buffer[6]==0x32)	   //½ÓÊÕÎÞ´íÎó
				{	



					j++;		
	
				}
				//½ÓÊÕ´íÂÒÇé¿ö
				else
				{
					//¹Ø½ÓÊÕ
					receiveoff;
					temp =	USART1->DR;	
					
					for(i=0;i<8;i++)
					{	
						buffer[i]=0;
					}
					Delay(10);
	
					//¿ª½ÓÊÕ
					receiveon	 ;
				}
			}
		 */
	 								
	}	
 }


