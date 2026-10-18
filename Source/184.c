/**********************************************************************
   *日期:2012/3	 
   *项目描述：烟台项目50个测温站共300个测温点，采用RS485总线通讯 波特率9600kpbs
   *硬件方案：stm32f103c6t6(双串口)	+8路模拟开关CD4051+	光耦6N137+隔离电源B0505+485收发器MAX1483
   *PCB编号：ORZ_TX_V1.0
   *主机命令格式 0xaa 0x55 站地址
   *USART1用来接收模块数据 19200kpbs ;  USART2用于发数到上位机 9600kpbs
   *将6路温度及LED、PD参数打包上传，共34个字节
	格式：
	0xaa 0x55 站地址 点地址1 TL TH LED PD …… 点地址6 TL TH LED PD CRC_L CRC_H
**********************************************************************/

#include "stm32f10x.h"

#define send485		GPIOB->ODR|=0x1000;
#define rec485		GPIOB->ODR&=~0x1000;


//关接收
#define receiveoff	USART1->CR1 &=0XFFfb
//开接收
#define receiveon	USART1->CR1 |= ~0XFFfb

static const unsigned char crc_tab8[256]={               /* CRC余式表 */ 
0x00,0xfe,0xff,0x01,0xfd,0x03,0x02,0xfc,0xf9,0x07,0x06,0xf8,0x04,0xfa,0xfb,0x05,
0xf1,0x0f,0x0e,0xf0,0x0c,0xf2,0xf3,0x0d,0x08,0xf6,0xf7,0x09,0xf5,0x0b,0x0a,0xf4,
0xe1,0x1f,0x1e,0xe0,0x1c,0xe2,0xe3,0x1d,0x18,0xe6,0xe7,0x19,0xe5,0x1b,0x1a,0xe4,
0x10,0xee,0xef,0x11,0xed,0x13,0x12,0xec,0xe9,0x17,0x16,0xe8,0x14,0xea,0xeb,0x15,
0xc1,0x3f,0x3e,0xc0,0x3c,0xc2,0xc3,0x3d,0x38,0xc6,0xc7,0x39,0xc5,0x3b,0x3a,0xc4,
0x30,0xce,0xcf,0x31,0xcd,0x33,0x32,0xcc,0xc9,0x37,0x36,0xc8,0x34,0xca,0xcb,0x35,
0x20,0xde,0xdf,0x21,0xdd,0x23,0x22,0xdc,0xd9,0x27,0x26,0xd8,0x24,0xda,0xdb,0x25,
0xd1,0x2f,0x2e,0xd0,0x2c,0xd2,0xd3,0x2d,0x28,0xd6,0xd7,0x29,0xd5,0x2b,0x2a,0xd4,
0x81,0x7f,0x7e,0x80,0x7c,0x82,0x83,0x7d,0x78,0x86,0x87,0x79,0x85,0x7b,0x7a,0x84,
0x70,0x8e,0x8f,0x71,0x8d,0x73,0x72,0x8c,0x89,0x77,0x76,0x88,0x74,0x8a,0x8b,0x75,
0x60,0x9e,0x9f,0x61,0x9d,0x63,0x62,0x9c,0x99,0x67,0x66,0x98,0x64,0x9a,0x9b,0x65,
0x91,0x6f,0x6e,0x90,0x6c,0x92,0x93,0x6d,0x68,0x96,0x97,0x69,0x95,0x6b,0x6a,0x94,
0x40,0xbe,0xbf,0x41,0xbd,0x43,0x42,0xbc,0xb9,0x47,0x46,0xb8,0x44,0xba,0xbb,0x45,
0xb1,0x4f,0x4e,0xb0,0x4c,0xb2,0xb3,0x4d,0x48,0xb6,0xb7,0x49,0xb5,0x4b,0x4a,0xb4,
0xa1,0x5f,0x5e,0xa0,0x5c,0xa2,0xa3,0x5d,0x58,0xa6,0xa7,0x59,0xa5,0x5b,0x5a,0xa4,
0x50,0xae,0xaf,0x51,0xad,0x53,0x52,0xac,0xa9,0x57,0x56,0xa8,0x54,0xaa,0xab,0x55
}; 





static const unsigned int crc_tab[256]={               /* CRC余式表 */ 
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0

}; 

unsigned char buffer[8]={0};
unsigned char address[6]={0};
char reccount=0;
u32 Id=0;
unsigned char sentdata[6][5]={0};
char flag=0,flag_equ=0;

void send_485(void);

/////////////////////
unsigned char crc_8(unsigned char *p,unsigned char n)  //unsigned char len)  //计算6个8位数的CRC
{
	unsigned char i,c;
	c=0xff;
 
 /*
	p1=crc_tab8;
	for(i=0;i<n;i++)
	{
		c^=*p++;
		c=*(p1+c);
	}
*/
	for(i=0;i<n;i++)
	{
		c^=*p++;//*p++;
//		j=c;

		c=crc_tab8[c];
	}

	return (c);
}

//10M晶振、1ms延时	这里取200ms延时
void Delay(int count)
{
	 int n;
  	 for(;count!=0;count--)
	   for(n=2000; n != 0; n--)
	        ;
	 
}
void delay1(unsigned int nCount)
{
  for(; nCount != 0; nCount--);
}

void delayus(int us)
{
	while(us--) ;
}

/*///////////////////
unsigned char crc_8(unsigned char *p,unsigned char n)   //计算n个字节的8位CRC
{
	unsigned char *p1,i,c;
	c=0xff;
	p1=crc_tab;
	for(i=0;i<n;i++)
	{
		c^=*p++;
		c=*(p1+c);
	}
	return (c);
}
*/
//16位CRC校验
unsigned int crc16(unsigned char *ptr,int len)
{
	unsigned int crc;
	unsigned char da;
	crc=0;
	while(len--!=0) 
	{
		da=crc>>8;    /* 以8位二进制数的形式暂存CRC的高8位 */
		crc<<=8;              /* 左移8位，相当于CRC的低8位乘以  */
		crc^=crc_tab[da^*ptr];   /* 高8位和当前字节相加后再查表求CRC ，再加上以前的CRC */
		ptr++;
	}
	return(crc);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

   	// Configure USART1 Rx (PA.10) as alternate function push-pull 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	// Configure USART1 Tx (PA.09) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

   	// Configure USART2 Rx (PA.10) as alternate function push-pull 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	// Configure USART2 Tx (PA.09) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	//pb13\14\15-4051地址选择  pb12-MAX485收发控制  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

}

//系统中断管理
void NVIC_Configuration(void)
{ 
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
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
  	delay1(1000);
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
	 						| RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO , ENABLE );

 	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2   , ENABLE );

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

//usart2 --485通讯接口定义   速率：9600kpbs
void USART2_Configuration(void)
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

	USART_ClockInit(USART2, &USART_ClockInitStructure);
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);    
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //开中断	 
}

//中断接收命令
void USART2_IRQHandler()
{
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
//		if(reccount<5)
		address[reccount] = USART2->DR;
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

		if((reccount>0)&&(address[reccount-1]==0xaa)&&(address[reccount]==0x55))
		{
			reccount=1;
		  	address[0]=0xaa;
		  	address[1]=0x55;
		}
		reccount++;
		if(reccount>=4)
		{
			if(address[3]==0xaa)
			{
				reccount=1;
				address[0]=0xaa;
			}
			else {reccount=0;}
//			flag=0;
			if((address[0]==0xaa)&&(address[1]==0x55)&&(reccount==0))
			{
				flag=1;
//				USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	
			}			
			if((address[0]==0xaa)&&(address[1]==0x55)&&(address[2]==Id)&&(address[3]==crc_8(address,3)))
			{
				delay1(1000);
				send_485();
				address[2]=0;
				address[1]=0;
				reccount=0;
				flag_equ=1;
		
			}//	flag=1;
		}
		
	}

}

/************************************************************
将6路温度及LED、PD参数打包上传，共34个字节
格式：
	0xaa 0x55 站地址 点地址1 TL TH LED PD …… 点地址6 TL TH LED PD CRC_L CRC_H
其中：
    TL TH LED PD全部为0xff为模块无数据输出情况
*************************************************************/
void send_485(void)
{
	unsigned char i,m,n;
	unsigned char crc[31]={0};
	unsigned int crcreg;

	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	 
	
	send485;
	delay1(1000);
	USART2->DR=0xbb;
	while((USART2->SR&0x40)== 0 );  		
	USART2->DR=0x66;
	while((USART2->SR&0x40)== 0 );  
	USART2->DR=Id;
	while((USART2->SR&0x40)== 0 );
	crc[0]=Id;
	n=1;
	for(i=0;i<6;i++)
	{
		for(m=0;m<5;m++)
		{
			USART2->DR=sentdata[i][m];
			while((USART2->SR&0x40)== 0 );
			crc[n++]=  sentdata[i][m];
		}  													
	}
	crcreg= crc16(crc,31);		//最后一位发送16位CRC校验码。低位在前，高位在后
	USART2->DR=crcreg;
	while((USART2->SR&0x40)== 0 );  
	USART2->DR=crcreg>>8;
	while((USART2->SR&0x40)== 0 );  
	
	rec485;
	
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //开中断	 
	
}

void caiyang()
{
	unsigned char j, k;
	unsigned char finish=0;
	unsigned int overtime=0,j1,i;

	
	for(j=0;j<6;j++)
	{
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   // 

		j1=j;
		j1<<=13;
		GPIOB->ODR =j1;	  //CD4051地址选择、并置485接收
		
		delayus(100);
//		USART1->SR&=~0x0020;
		receiveon;
//		k=0;
//		overtime=0;
		finish=0;
		while(finish==0) 
	    {
			k=0;
			while(k<8)
			{
				overtime=0;

				while(((USART1->SR & 0x0020)==0)&&(overtime<20000))  //模块0.6秒钟超时判断
				{
				 	overtime++;
					if((k>0)&&(overtime>=30))break;
					delayus(100);
				}
				USART1->SR&=~0x0020;

				if(((k==0)&&(overtime<20000))||((k>0)&&(overtime<30)))
				{
					buffer[k]=	USART1->DR;
					k++;	
					overtime=0;
				}
				else if((k==0)&&(overtime>=20000))		 //模块坏，读不到数据情况
				{
					k=8;
					finish=1;
//					flag=0;
//	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	 

					sentdata[j][0]=j+1;
					sentdata[j][1]=0xff;
					sentdata[j][2]=0xff;
					sentdata[j][3]=0xff;
					sentdata[j][4]=0xff;
					break;
					
				}
				else if((k>0)&&(overtime>=30))
				{
				 	
					for(k=0;k<8;k++)
					{	
						buffer[k]=0;
					}
					finish=1;
					k=8;
					break;
				}
			}	
			if((buffer[3]>8)&&(buffer[3]<0x13)&&(buffer[6]==0x32)&&(finish==0))	   //接收无错误
			{								
				finish=1;
				sentdata[j][0]=j+1;
//	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	 
				
				sentdata[j][1]=buffer[2];
				sentdata[j][2]=buffer[3];
				sentdata[j][3]=buffer[4];
				sentdata[j][4]=buffer[5];
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   // 

				for(k=0;k<8;k++)
				{	
					buffer[k]=0;
				}
				break;
//				j++;				
			}
			else				   //读错数据，重读
			{
				//关接收
				//if(overtime>=30000)finish=1;
				receiveoff;
				
				for(k=0;k<8;k++)
				{	
					buffer[k]=0;
				}
			//	Delay(10);

				//开接收
				receiveon	 ;
				finish=1;
				break;
			}
		}	  
		receiveoff;
		
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
		for(i=0;i<30000;i++)
		{	
			//delayus(100);
			for(k=0;k<80;k++);//_nop_();
			if(j==5)
			{
				j1=j;
				if((i%2)==0)j1+=2;
				j1<<=13;
				GPIOB->ODR =j1;	  //测试用,4051 B脚有高低电平变化
				
			}
			
			if(flag_equ==1)		//接收到自身的上位机命令并有回发，则退出并关中断
			{
				i=40000;
				flag_equ=0;
				flag=0;
			}
			else if(flag==1)//如果接收到了不是自身地址的上位机命令
			{
				if(Id>address[2])
				{
					if((Id-address[2])>17)i=40000;
				}
				else{
					j1=Id+50;
					if((j1-address[2])>17)i=40000;
				}//如果上位机命令的地址和自身地址差值小于18则继续等待上位机命令，否则转去读下一个模块数据
				flag=0;
//				if(i<20000)USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
			}
			
		}
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断
		reccount=0;
			 
/*		
		k=1;
	  	for(i=0;i<6;i++)
		{
			if((sentdata[i][1]!=0xff)||(sentdata[i][2]!=0xff))k=0;
		}
		if(k==1)
		{
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	 
			break;
		}
*/
	}	
}

//配置所有外设
void Init_All_Periph(void)
{
	RCC_Configuration();	
	GPIO_Configuration();
	NVIC_Configuration();
	USART1_Configuration();
	USART2_Configuration();
}

void main(void)
{  
//	unsigned char i,j;
reset0:
	Init_All_Periph();

	Delay(100);

	Id=GPIOA->IDR&0xf3  ;				//通过拔码开关选择Id，每个循环读取1次
	Id|=(GPIOB->IDR&0xc00)>>8;
	Id+=1;

	rec485;
//	for(i=0;i<6;i++)for(j=1;j<5;j++)sentdata[i][j]=0;
	
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //开中断	 

	while(1)
  	{

		//485置接收状态

//		Delay(1);

		caiyang();
/*
		j=1;
	  	for(i=0;i<6;i++)
		{
			if((sentdata[i][1]!=0xff)||(sentdata[i][2]!=0xff))j=0;
		}
		if(j==1)
		{
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   //关中断	 
			goto reset0;
		}
*/

	}	 										
}


