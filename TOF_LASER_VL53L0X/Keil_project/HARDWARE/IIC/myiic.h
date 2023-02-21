#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
   	   		   

//IO��������	 
#define IIC_SCL(x)    {PBout(8)=x;delay_us(10);} //********************��ֲ�뿴����**********************//
#define IIC_SDA(x)    {PBout(9)=x;delay_us(10);} //********************��ֲ�뿴����**********************//	 
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
















