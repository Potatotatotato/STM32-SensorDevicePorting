#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IIC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
   	   		   

//IO操作函数	 
#define IIC_SCL(x)    {PBout(8)=x;delay_us(10);} //********************移植请看这里**********************//
#define IIC_SDA(x)    {PBout(9)=x;delay_us(10);} //********************移植请看这里**********************//	 
#define IIC_READ_SDA   PBin(9)

void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(u8 byte);
u8 IIC_ReceiveByte(void);
void IIC_SendAck(u8 ack);
u8 IIC_ReceiveAck(void);
void IIC_Init(void);

//void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
//u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















