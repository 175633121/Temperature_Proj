/**********************************************************************
   *date:2011/10
   *brief:CAN◊‹œﬂ∞Ê≤‚Œ¬
   *USART1”√¿¥Ω” ’ƒ£øÈ ˝æ› 19200kpbs ;  canΩ”ø⁄∑¢ÀÕ ˝æ›
**********************************************************************/

#include "stm32f10x.h"

#define D4on		GPIOB->ODR|=0X0400			 //pb10
#define D4off		GPIOB->ODR&=~0X0400
#define D3on		GPIOB->ODR|=0X0800			 //pb11
#define D3off		GPIOB->ODR&=~0X0800

//πÿΩ” ’
#define receiveoff	USART1->CR1 &=0XFFfb
//ø™Ω” ’
#define receiveon	USART1->CR1 |= ~0XFFfb

void canerror(char );

char count=0,finish=0;	   //º« ˝∫ÕΩ” ’≥…π¶±Í÷æ 

unsigned char Id=65;

unsigned int add[6]={0x4001,0x4002,0x4003,0x4004,0x4005,0x4006}		;

unsigned char length=0;

unsigned char VSQ=0x80;

//can◊‹œﬂ√¸¡Ó
#define typcall		0x64
#define typtime		103
#define typerror	31
#define typread		102
#define typset		48
#define typsets		136

CanTxMsg TxMessage;
CanRxMsg RxMessage;

unsigned char TransmitMailbox;
unsigned char buffer[8]={0};

//typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
//volatile TestStatus TestRx;

unsigned char replyhead[]={0x68,0x0e,0x0e,0x68,0x84,0x01,0x00,0x09};
unsigned char replybody1[8]={0x01 ,0x14,0x00,0x01, 0x00 ,0x01, 0x40 ,0};
unsigned char replybody2[8]={0,0,0xa9,0x16,0,0,0,0};
unsigned char cs=0;

unsigned int overtime=0;   //Ω” ’ƒ£øÈ≥¨ ±≈–∂œ

unsigned char canrec[5][8]=	{{0},{0},{0},{0},{0}};

unsigned char reset[]={0xef,0xdf,0xbf,0x7f};
//10Mæß’Ò°¢1ms—” ±	’‚¿Ô»°200ms—” ±
void Delay(int count)
{
	 int n;
  	 for(;count!=0;count--)
//	   for(n=2000; n != 0; n--)
		for(n=4800; n != 0; n--)
	        ;
	 
}
void delay(int nCount)
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
    
	//pa4-pa7øÿ÷∆∏¥Œª“˝Ω≈   pa8 rs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	//pb13\14\15-4051µÿ÷∑—°‘Ò  pb0\pb1--ƒ£øÈ∏¥Œª
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_0|GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

}
/*
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
//	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);  //‘ –ÌDMA¥´ ‰ÕÍ≥…÷–∂œ		
//	DMA_Cmd(DMA1_Channel3,  ENABLE);	
}
*/
//œµÕ≥÷–∂œπ‹¿Ì
void NVIC_Configuration(void)
{ 
  
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// Enable the USART1 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//≈‰÷√œµÕ≥ ±÷”, πƒ‹∏˜Õ‚…Ë ±÷”
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
	
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);//11.0592Mhz 2±∂∆µ pllclk= 22.118400Mhz
	
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
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


void TIM_Configuration()
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period=1000;		 //ARRµƒ÷µ
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //≤…—˘∑÷∆µ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //œÚ…œº∆ ˝ƒ£ Ω
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM2,0x2b33,TIM_PSCReloadMode_Immediate);// ±÷”∑÷∆µœµ ˝1000£¨À˘“‘∂® ±∆˜ ±÷”Œ™110.592Khz
	TIM_ARRPreloadConfig(TIM2, ENABLE);//Ω˚÷πARR‘§◊∞‘ÿª∫≥Â∆˜
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
//	TIM_Cmd(TIM2, ENABLE);	//ø™∆Ù ±”	
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
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	
	/* Baudrate = 500kbps*/				 //f=12M/(5+6+1)/2=500K
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;
	CAN_InitStructure.CAN_Prescaler=2;
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
}
	
void sample()
{
		unsigned char i,j,k;
		int sum;
		for(j=0;j<6;)				 
		{		       
//			pin=GPIOA->IDR&&0x0f;
//					reset= ~(j<<4);
//					reset|=pin;
			if(j<4)
			{
				GPIOA->ODR=reset[j];		  //ƒ£øÈ∏¥Œª
				GPIOB->ODR =j<<13|0x0403;	 		  //CD4051µÿ÷∑—°‘Ò,pbø⁄µÕ¡ΩŒª «∏¥Œªøÿ÷∆
			}
	   		if(j==4||j==5)
				GPIOA->ODR|=0XF0;

			if(j==4)
				GPIOB->ODR =j<<13|0x0402;
			if(j==5)
				GPIOB->ODR =j<<13|0x0401;			
			Delay(1500);			
				  
		    overtime=0;
			finish=0;
			while(!finish) 
			{	
				k=0;
				overtime=0;
				while(k<8)
				{
					while(((USART1->SR & 0x0020)==0)&&overtime<10000)	
					{
					 	overtime++;
						delayus(100);				
					}
					if(overtime<10000)
					{
						buffer[k]=	USART1->DR;
						k++;	
						overtime=0;
					}
					else
					{
						k=8;
						D3on;
						Delay(250);
						D3off;
					//	canerror(j);
						finish=1;
						j++;
					}
				}
				if(buffer[6]==0x32)
				{
					finish=1;
					cs=0;	
					replyhead[2]=0x0e;
					replyhead[1]=0x0e;
//						replybody1[0]=1;
					replybody1[5]=add[j];
					replybody1[6]=add[j]>>8;

					sum=buffer[3]*256+ buffer[2]-2732;
					if(sum<0)
					{
						sum=~sum+1;
					}

					replybody1[7]=sum;
					replybody2[0]=sum>>8;	   

					for(i=4;i<8;i++)
						cs+=replyhead[i];
					
					for(i=0;i<8;i++)
						cs+=replybody1[i];
//						for(i=0;i<8;i++)
					cs+=replybody2[0];
					cs+=replybody2[1];

					replybody2[2]=cs;
					D3on;
					for(i=0;i<8;i++)
						TxMessage.Data[i] =replyhead[i];
					TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
					while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;	
					overtime=0;
					delayus(100);

					for(i=0;i<8;i++)
						TxMessage.Data[i] =replybody1[i];
					TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
					while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;	
					overtime=0;
					delayus(100);

					TxMessage.DLC=4;
					for(i=0;i<4;i++)
						TxMessage.Data[i] =replybody2[i];
					TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
					while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;					
					overtime=0;
					TxMessage.DLC=8;
					j++;
					D3off;
			   }
			   else			   //ƒ£øÈΩ” ’¥ÌŒÛ
			   {
				   	receiveoff;	
					for(k=0;k<8;k++)
						buffer[k]=0;
					Delay(10);
					receiveon  ;	
			   }
			}

		}
	
}

void set()
{

}
//ƒ£øÈ¥ÌŒÛ –≈œ¢
void canerror(char j)
{
	CanTxMsg TxMessage1;
	TxMessage1.RTR=CAN_RTR_DATA;
	TxMessage1.IDE=CAN_ID_STD;
	TxMessage1.DLC=8;
	TxMessage1.Data[0]=0xaa;
	TxMessage1.Data[1]=0x55;
	TxMessage1.Data[2]=j;
//	CAN_Transmit(CAN1, &TxMessage);	
	TransmitMailbox=CAN_Transmit(CAN1, &TxMessage1);	
	while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xfff) overtime++;		
	overtime=0;
}

//≈‰÷√À˘”–Õ‚…Ë
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
	NVIC_Configuration();
	USART1_Configuration();
	CAN_Config();
//	TIM_Configuration();
//	DMA_Configuration();
}
/*
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		buffer[count]=USART1->DR;     		
		count++;
	}
	if(count==8)
	{
		count=0;		
		if(buffer[6]==0x32)
		finish=1;
	}
}
*/
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
	unsigned char j,pin;
	unsigned int i;
	
	Init_All_Periph();
	//¡˘∏ˆƒ£øÈ∏¥Œª
	GPIOA->ODR=0XF0;
	GPIOB->ODR=0X03;		  
	
	// ‘À––÷∏ æ
	D4on;

	// receive 
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
	 
	TxMessage.StdId=Id;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.DLC=8;  

 	while(1)
  	{
		pin=GPIOA->IDR&0x0f;
		Id=pin+65;
		TxMessage.StdId=Id;	

		while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1));// && (i != 0xFFff))
//		{
//			i++;
//		}
//		i=0;
		RxMessage.DLC=8;
		for(j=0;j<2;j++)
		{
			CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
			for(i=0;i<8;i++)
			{
				canrec[j][i]=RxMessage.Data[i];
			}	
		}
		if(canrec[0][1]>10)			 //√¸¡Ó¥Û”⁄¡Ω◊÷Ω⁄
		{
			for(j=2;j<(canrec[0][1]+6)/8;j++)
			{
				CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
				for(i=0;i<8;i++)
				{
					canrec[j][i]=RxMessage.Data[i];
				}				
			}

			RxMessage.DLC=(canrec[0][1]+6)%8;
			for(i=0;i<(canrec[0][1]+6)%8;i++)
				 canrec[j][i]=RxMessage.Data[i];
		}			 
		switch(canrec[0][7])
		{
			case 0x64: 		 //œÏ”¶◊‹’–ªΩ	
				sample();
				break;
			case 48:		 //…Ë∂®≤Œ ˝
				set();	break;
			default:
				break;	
		}
		canrec[0][7]=0;
				
	 sample();
	}

		
}


