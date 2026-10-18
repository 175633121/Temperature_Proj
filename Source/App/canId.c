/**********************************************************************
   *Date:2012/3
   *Auther:ORZ
   *PCB编号：ORZ-TX-V1.0
   *USART1用来接收模块数据 19200kpbs ;  can接口发送数据，波特率10K
   *聊城市电网测温，最长距离3公里。
   *CAN通讯采用问答形式，11位标准Id。主机发送8字节数据，从机发送6字节数据
   *主机Id：0x05  ;  测温站Id分配：0x0a～0x25。 共28个测温站 79个测温点
   *主机发送报文格式：
	 05 00 00 00 08  68 aa 55 68 73 站地址 0 64  00 00 00 00 00 00 
   *测温站发送报文Id分配如下： 高8位为测温站地址，低三位为测温点(模块)地址
    6字节数据：测温站地址 测温点地址 TL TH LED PD

**********************************************************************/

#include "stm32f10x.h"

//关接收
#define receiveoff	USART1->CR1 &=0XFFfb
//开接收
#define receiveon	USART1->CR1 |= ~0XFFfb

char count=0,finish=0;	   //记数和接收成功标志 

u32 Id=10;

unsigned char length=0;

CanTxMsg TxMessage;
CanRxMsg RxMessage;

unsigned char TransmitMailbox;
unsigned char buffer[8]={0};

unsigned char replyhead[]={0xff,0xaa,0x0,0,0,0,0x00,0x0};

unsigned int overtime=0;   //接收模块超时判断
unsigned char tbuf[12]={0};
unsigned char canrec[8]={0};
unsigned char canfinish=0;

//10M晶振、1ms延时	这里取200ms延时
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

void delayus(int us)
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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	//pb13\14\15-4051地址选择  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
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
//	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);  //允许DMA传输完成中断		
//	DMA_Cmd(DMA1_Channel3,  ENABLE);	
}
*/
//系统中断管理
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
	
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_2);//12
	
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

/*
void TIM_Configuration()
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period=1000;		 //ARR的值
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //采样分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM2,0x2b33,TIM_PSCReloadMode_Immediate);//时钟分频系数1000，所以定时器时钟为110.592Khz
	TIM_ARRPreloadConfig(TIM2, ENABLE);//禁止ARR预装载缓冲器
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
//	TIM_Cmd(TIM2, ENABLE);	//开启时?
}
*/
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
	CAN_InitStructure.CAN_NART=DISABLE;//ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
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
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x05<<5;		//主机ID是5
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0	;
	
	
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
//	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}
	
void sample()
{
		unsigned char i,j,k;
		for(j=0;j<6;)				 
		{		       
			GPIOB->ODR =j<<13;			  
		    overtime=0;
			finish=0;
			while(!finish) 
			{		
				k=0;
				overtime=0;
				while(k<8)
				{
					while(((USART1->SR & 0x0020)==0)&&overtime<10000)	//1秒等待时间，没有数据跳过
					{
					 	overtime++;
						delayus(300);				
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
						finish=1;
						j++;
					}
				}
				if(buffer[6]==0x32&&finish==0)
				{
					finish=1;
					TxMessage.Data[0]=Id	  ;
					TxMessage.Data[1]=j+1;
					TxMessage.Data[2]=buffer[2];
					TxMessage.Data[3]=buffer[3];
					TxMessage.Data[4]=buffer[4];
					TxMessage.Data[5]=buffer[5];

					TxMessage.StdId=Id<<3;
					TxMessage.StdId|=j+1;
					TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);	
					while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&overtime!=0xffff) overtime++;	
					overtime=0;
					delayus(100);
					j++;
			   }
			   else			   //模块接收错误
			   {
				   	receiveoff;	
					for(i=0;i<8;i++)
						buffer[i]=0;
					Delay(10);
					receiveon  ;	
			   }
			}
			Delay(25);
		}
}


//配置所有外设
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
	NVIC_Configuration();
	USART1_Configuration();
	CAN_Config();
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
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	RxMessage.DLC=8;
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	if ((RxMessage.StdId == 5)&&RxMessage.Data[5]==0x0a)
	{
		 canfinish=1;
	}
}
*/
int main(void)
{  

	Init_All_Periph();	  

	Id=10;

	RxMessage.StdId=0x00;
	RxMessage.RTR=CAN_RTR_DATA;
	RxMessage.IDE=CAN_ID_STD;
	RxMessage.DLC=8;	 
	TxMessage.StdId=Id;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.DLC=6;  
//	TxMessage.Data[0]=0xff;
//	TxMessage.Data[1]=0x55;


 	while(1)
  	{
		Id=GPIOA->IDR&0xf3  ;				//通过拔码开关选择Id，每个循环读取1次
		Id|=(GPIOB->IDR&0xc00)>>8;
		Id+=10;
		TxMessage.StdId=Id;

//		Id=	GPIOA->IDR&0xff ;
//		Id+=10;

		while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1));// && (i != 0xFFff))

		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		if(RxMessage.StdId==0x05&&RxMessage.Data[5]==Id) 	 //判断主机Id及寻址测温站
//		if(RxMessage.Data[5]==Id) 
//		if(RxMessage.StdId==0x05)
			sample();

	}

		
}


