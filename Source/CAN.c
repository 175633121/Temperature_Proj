/**********************************************************************
   *date:2012/3
   *brief:Ó¦ÓÃÓÚÁÄ³ÇCAN×ÜÏß²âÎÂµçÂ·
   *USART1ÓÃÀ´½ÓÊÕÄ£¿éÊý¾Ý 19200kpbs ;  can½Ó¿Ú·¢ËÍÊý¾Ý
   *Ã¿¸öµçÂ·°åÄ£¿éÊýÁ¿²»¶¨£¬Ñ­»·²âÍêÒ»´Îºó´ò°üÉÏ´«Ä£¿éµØÖ·+ÎÂ¶È
**********************************************************************/

#include "stm32f10x.h"

#define D4on		GPIOB->ODR|=0X0400			 //pb10
#define D4off		GPIOB->ODR&=~0X0400
#define D3on		GPIOB->ODR|=0X0800			 //pb11
#define D3off		GPIOB->ODR&=~0X0800

#define res1		GPIOB->ODR&=0Xfffe
#define res2		GPIOB->ODR&=0Xfffc
#define res3		GPIOB->ODR&=0Xff7c
#define res4		GPIOB->ODR&=0Xff3c
#define	res5		GPIOB->ODR&=0Xff1c
#define res6		GPIOB->ODR&=0Xfe1c

//¹Ø½ÓÊÕ
#define receiveoff	USART1->CR1 &=0XFFfb
//¿ª½ÓÊÕ
#define receiveon	USART1->CR1 |= ~0XFFfb

void canerror(char );

char count=0,finish=0;	   //¼ÇÊýºÍ½ÓÊÕ³É¹¦±êÖ¾ 
char canfinish=0;
char cancount=0;
unsigned char Id=10;

CanTxMsg TxMessage;
CanRxMsg RxMessage;

unsigned char TransmitMailbox;
unsigned char buffer[8]={0};

//unsigned char replybody2[]={1,0,0,0, 2,0,0,0, 3,0,0,0, 4,0,0,0, 5,0,0,0, 6,0,0,0};
unsigned char replyhead[]={0x68,0xc,0xc,0x68,0x84,0x01,0x00,0x09};   //Í·	
unsigned char cs;

unsigned int overtime=0;   //½ÓÊÕÄ£¿é³¬Ê±ÅÐ¶Ï

unsigned char  canrec[2][8]={{0},{0}};
unsigned char tbuf[12]={0};

unsigned char reset[]={0xef,0xdf,0xbf,0x7f};
//10M¾§Õñ¡¢1msÑÓÊ±	ÕâÀïÈ¡200msÑÓÊ±
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
	
	//IDÑ¡Ôñ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//pa4-pa7¿ØÖÆ¸´Î»Òý½Å   pa8 rs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	//pb13\14\15-4051µØÖ·Ñ¡Ôñ  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//485¿ØÖÆ	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

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
	
	// Enable the USART1 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
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
	
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);//11.0592Mhz 2±¶Æµ pllclk= 22.118400Mhz
	
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
	CAN_InitStructure.CAN_ABOM=ENABLE;	 //Ê¹ÄÜµÃÐÂ»Ö¸´
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;	 //Ê¹ÄÜ×Ô¶¯ÖØ´«
	CAN_InitStructure.CAN_RFLM=DISABLE;	 //½ûÖ¹±¨ÎÄËø¶¨£¬²»¶ªÊ§
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	
	/* Baudrate = 500kbps*/				 //f=12M/(5+6+1)/100=10K
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;
	CAN_InitStructure.CAN_Prescaler=100;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);

	RxMessage.StdId=0;
	RxMessage.RTR=CAN_RTR_DATA;
	RxMessage.IDE=CAN_ID_STD;
	RxMessage.DLC=8;
	 
	TxMessage.StdId=0;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.DLC=8;  

}
	
void sample()
{
		unsigned char i,j,k;
		int sum;
		char length;

		unsigned char replybody[]={0x80,0x14,0x01,0x00,0x01,0x00,0x00,0};
		
		length=0;		
//		CAN_ITConfig(CAN1,CAN_IT_FMP0,DISABLE);
		for(j=0;j<6;)				 
		{		       
			GPIOB->ODR =j<<13|0X0E1C;			
	//		Delay(10);				  
		    overtime=0;
			finish=0;

			while(!finish) 
			{	
				k=0;
				overtime=0;
				while(k<8)
				{
					while(((USART1->SR & 0x0020)==0)&&overtime<13000)	
					{
					 	overtime++;
						delayus(100);				
					}
					if(overtime<13000)
					{
						buffer[k]=	USART1->DR;
						k++;	
						overtime=0;
					}
					else
					{
						k=8;
						finish=1;
						j++;
						length++;
					}
				}
				if(buffer[6]==0x32)
				{
					finish=1;
					tbuf[2*j]=buffer[2];
					tbuf[2*j+1]=buffer[3];
//				 	length++;
					j++;	   
			   }
			   else			   //Ä£¿é½ÓÊÕ´íÎó
			   {
				   	receiveoff;	
					for(k=0;k<8;k++)
						buffer[k]=0;
					Delay(10);
					receiveon  ;	
			   }
			}
		}
		length=6-length;
	
		//Ìî³ä±¨ÎÄÍ·	 length±íÊ¾Êµ¼ÊÄ£¿é¸öÊý	
		replybody[0]=length;

		replyhead[5]=Id;
		//·¢ËÍ±¨ÎÄ
		for(j=0;j<length;j++)
		{
			cs=0xa2+length+Id;			
			TxMessage.DLC=8;
			//±¨ÎÄÍ·
			for(i=0;i<8;i++)
				TxMessage.Data[i] =replyhead[i];
			TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);
			while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;	
			overtime=0;								 
			delayus(100);
		
//			cs=0xa3+length;

			for(i=0;i<4;i++)	
				TxMessage.Data[i] =replybody[i];
			TxMessage.Data[4] =j+1;
			TxMessage.Data[5] =0;
			TxMessage.Data[6] =tbuf[2*j];
			TxMessage.Data[7] =tbuf[2*j+1];
			TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
			while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;	
			overtime=0;
			delayus(100);
		
			cs+=j;
			cs+=1;
			cs+=tbuf[2*j];
			cs+=tbuf[2*j+1];
			TxMessage.DLC=2;
			TxMessage.Data[0] =cs;
			TxMessage.Data[1] =0x16;
			TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
			while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;					
			overtime=0;	
			Delay(150);	
		}
//		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}

//Ä£¿é´íÎó ÐÅÏ¢
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

//ÅäÖÃËùÓÐÍâÉè
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
	NVIC_Configuration();
	USART1_Configuration();
	CAN_Config();

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
	
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	char i ;	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	for(i=0;i<8;i++)
	{
		canrec[cancount][i]=RxMessage.Data[i];
	}
	cancount++;
	if(cancount==2)
	{ 
	 	cancount=0;
		canfinish=1;
	}
}

int main(void)
{  
	unsigned char j,pin;
	unsigned int i;
	unsigned char check=0;
	Init_All_Periph();
	//Áù¸öÄ£¿é¸´Î»
	GPIOB->ODR=0;	
	cancount=0;

	// receive 

	Delay(1500);
 	while(1)
  	{
		check=0;
		Id=10;
		pin=GPIOA->IDR&0xff;
		Id+=pin;
		TxMessage.StdId=Id;			
/*
		while((CAN_MessagePending(CAN1, CAN_FIFO0) == 0));// && (i != 0xFFff))

		RxMessage.DLC=8;
		for(j=0;j<2;j++)
		{
			CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
			for(i=0;i<8;i++)
			{
				canrec[j][i]=RxMessage.Data[i];
			}	
		}		
			
		//Ð£Ñé¼ÓºÍ
		for(i=4;i<8;i++)
		  check+= canrec[0][i]	;
		for(i=0;i<4;i++)
		  check+= canrec[1][i]	;
				 
		if(canrec[0][5]==Id&&check==canrec[1][6])
		{
			sample();
		}
		canrec[0][7]=0;


		while(!canfinish)  ;
		canfinish=0;

		//Ð£Ñé¼ÓºÍ
		for(i=4;i<8;i++)
		  check+= canrec[0][i]	;
		for(i=0;i<4;i++)
		  check+= canrec[1][i]	;
		if(canrec[0][5]==Id&&check==canrec[1][6])
		{
			sample();	
		}
*/
//	 sample();	
	}
			
}
 	

