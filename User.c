#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <RTL.h>
#include <Net_Config.h>
#include <LPC17xx.h>                    /* LPC17xx definitions               */

#include <core_cm3.h>
//#define  FOSC    12000000
//#define  FCCLK   (FOSC*8)
//#define  FPCLK   (FCCLK/4)
#include "User.h"
#include "HTTP.h"
#include "Client.h"
#include "Server.h"
#include "Announce.h"
#include "I2C_EEPROM.h"
#include "USER_EEPROM.h"
#include "HTTP_CGI.h"
#include "Iiot.h"
#include "RTC.h"
#include "Modbus_Server.h"
#include "UART.h"
#include "ADC.h"
#include "GPIO.h"
#include "SPI.h"
#include "TIMER.h" 
#include "SSP.h"
#include "TDSXGL.h"	
#include "TDSXGA.h"
uint8_t GC_RecieveBuff[GK_RECEIVE_LENGTH]={0},GC_RecieveBuff1[GK_RECEIVE_LENGTH]={0},GC_RX_Flag=0,GC_RX_Flag1=0;
static uint8_t  GC_ArrayPutPtr=0;//GC_ArrayPutPtr1=0;
extern unsigned char GC_IECEnableFlag_GA,GC_IECEnableFlag_GL,GC_Ticker30Sec_Flag;
extern uint64_t GL_Ticker30Sec;
uint8_t G_TxBuff[40]={0}; 
uint8_t G_TxBuff1[40]={0}; 
uint16_t AXE410_ModRtuCRC(uint8_t *,uint16_t);
static uint32_t GLI_Pclk,GLI_Fdiv;
 uint8_t Byte=0;
char a[]="\n\rApplied Embedded";
uint16_t sec,min,hou,day,mon,year;
unsigned char str3[150];
char on1=1,off1=1, on2=1,off2=1;


void AXE410_ReadHoldingRegistersResponse(void)
{
	uint8_t END_Point=0,LC_i=0,LC_j=3,LC_k=4,Starting_Address=0,Number_of_Reg=0,Function_code=0;	
	uint16_t LSI_CRC=0;
	Function_code = GC_RecieveBuff[1];
	memset(G_TxBuff,0,sizeof(G_TxBuff));
	if(Function_code != READ_HOLDING_REGISTER)
	{ GC_RX_Flag=0;
		return;
	}
	
	else
	{
		Starting_Address = GC_RecieveBuff[3];
		Number_of_Reg =	GC_RecieveBuff[5];	
		END_Point = Starting_Address + Number_of_Reg;
		G_TxBuff[0]=SLAVEID_3;       
		G_TxBuff[1]=READ_HOLDING_REGISTER;
		G_TxBuff[2]=Number_of_Reg*2;
		for(LC_i=Starting_Address; LC_i<END_Point; LC_i++)
		{
			
				G_TxBuff[LC_j] = G_TxBuff1[LC_i]>>8;
				G_TxBuff[LC_k] = G_TxBuff1[LC_i];
			  LC_j=LC_j+2;
			  LC_k=LC_k+2;
		}
		LSI_CRC= AXE410_ModRtuCRC(G_TxBuff, LC_j);
		G_TxBuff[LC_j]=LSI_CRC;
		G_TxBuff[LC_k]=LSI_CRC>>8;	
		LPC_GPIO0->FIOSET= GK_RTS1_DIR;	
	//	AXE410_DelayUs(2910);	// 3.5 char delay to start
	if(	G_TxBuff[0]==0X01 && G_TxBuff[1]==0X03 )
	{
		for(LC_i=0; LC_i<=LC_k; LC_i++)
		{	
			AXE410_Uart1Tx(G_TxBuff[LC_i]);
		//	AXE410_Uart3Tx(G_TxBuff[LC_i]);
		//	AXE410_DelayUs(1250);	// 1.5 char delay for each byte
		}	
		//AXE410_DelayUs(2910);	// 3.5 char delay to stop
		GC_RX_Flag=0;
		uint32_t LLI_i;	
	    LPC_GPIO2->FIOSET = GK_USER_LED4_PIN;
	   for(LLI_i=0;LLI_i<50000;LLI_i++);
	   LPC_GPIO2->FIOCLR = GK_USER_LED4_PIN;
	   for(LLI_i=0;LLI_i<50000;LLI_i++);
		LPC_GPIO0->FIOCLR= GK_RTS1_DIR;	
	}
	}
}

uint16_t AXE410_ModRtuCRC(uint8_t *Buf, uint16_t Len)
{
  uint16_t LSI_Crc = 0xFFFF;
  uint16_t LSI_Pos,LSI_i;
 
 for ( LSI_Pos = 0; LSI_Pos < Len; LSI_Pos++) 
 {
    LSI_Crc ^= (uint16_t)Buf[LSI_Pos];          // XOR byte into least sig. byte of crc
 
    for ( LSI_i = 8; LSI_i != 0; LSI_i--) 
	{    // Loop over each bit
      if ((LSI_Crc & 0x0001) != 0) 
	  { // If the LSB is set
        LSI_Crc >>= 1;                    // Shift right and XOR 0xA001
        LSI_Crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        LSI_Crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return (LSI_Crc);  
}

void ch(void)
{
	  for(int i=0;i<40;i++)
	  {
			G_TxBuff1[i]=1;
		}
}

// This UART configured for RS485 Communication
void AXE410_UART1Init(void)
{
	LPC_SC->PCONP 		 	|= BIT(4);	// Set UART1 Power Control Bit
	LPC_SC->PCLKSEL0 		|= (0x01<<8);	// Set Uart clock is Pclk
	LPC_PINCON->PINSEL0 = (0x40000000);	// Set P0.15 as a TXD1
	LPC_PINCON->PINSEL1 = (0x00001001);	// Set P0.16 pin set as a Rx1, P0.22 set as RTS1
	LPC_PINCON->PINMODE0 = (0x00<<30);	// P0.15 pin pull up is enabled
	LPC_PINCON->PINMODE1 = (0x00<<0);	// P0.16 pin pull up is enabled
	LPC_GPIO0->FIODIR 	|= (1<<15);	// Set P0.15 pin as output
	LPC_GPIO0->FIODIR 	|= (0<<16);	// Set P0.16 pin as input
	LPC_GPIO0->FIODIR 	|= GK_RTS1_DIR;	// RTS1 set as output (P0.22)
	LPC_GPIO0->FIOCLR	= (0xFFFFFFFF);	// Clear output states(Tx,Rx and RTS)
	
	// UART control registers
	LPC_UART1->LCR = 0x83;	// 8data bits,1stop bit,DLA enabled
	GLI_Pclk = SystemCoreClock;	// SystemCoreClock division value depands on PCLKSEL register value
	GLI_Fdiv = (GLI_Pclk/16)/BAUD_RATE1;
//	GLI_Fdiv = (GLI_Pclk/(16*BAUD_RATE1));
	LPC_UART1->DLM = GLI_Fdiv/256;
	LPC_UART1->DLL = GLI_Fdiv%256;
	
	LPC_UART1->LCR = 0x03;	// Disable DLA
	LPC_UART1->FCR = 0x07;	// Uart access enabled. Tx,Rx FIFO got reset.
	LPC_UART1->RS485CTRL = 0x30;	// Auto direction control enabled, Reverse polarity direction control enabled
	NVIC_EnableIRQ(UART1_IRQn);
	NVIC_SetPriority(UART1_IRQn,4);
	LPC_UART1->IER = 1;	// Set UART RX interrupt
}



void AXE410_Uart1Tx(uint8_t Value)
{
	while(!(LPC_UART1->LSR&0x20));	// wait until TSR get empty
	LPC_UART1->THR = Value;	// Fill Transmit Holding Register with out data
}




//////////////////////////////////////////////////////////
void UART1_IRQHandler(void)
{
  while(LPC_UART1->LSR&0x01)	// Wait until data gets receive
	{
		
		
	GC_RecieveBuff[GC_ArrayPutPtr]=LPC_UART1->RBR;	// receive data byte
		
		GC_ArrayPutPtr++;	// Increase array pointer until full packet gets receive
	
		if(GC_RecieveBuff[0] == SLAVEID_3)	// Compare Slave ID to maintain packet format
		{
			if(GC_ArrayPutPtr >= GK_RECEIVE_LENGTH)	// Compare Packet size
			{
				GC_RX_Flag=1;	// Receive flag to get Full packet received confirmation
		   uint32_t LLI_i;	
	    LPC_GPIO2->FIOSET = GK_USER_LED3_PIN;
	   for(LLI_i=0;LLI_i<500000;LLI_i++);
	   LPC_GPIO2->FIOCLR = GK_USER_LED3_PIN;
	   for(LLI_i=0;LLI_i<500000;LLI_i++);for(LLI_i=0;LLI_i<500000;LLI_i++);for(LLI_i=0;LLI_i<500000;LLI_i++);
				GC_ArrayPutPtr=0;
			}
		}
		else GC_ArrayPutPtr=0;	// Reset array pointer when mismatched format packet receive
	}
}

int main(void)
{
	AXE410_UserIoInit();
	
	AXE410_UART1Init();

	
	ch();
  while (1) 
	{
		
	if(GC_RX_Flag==1)
	{
   uint32_t LLI_i;	
	LPC_GPIO2->FIOSET = GK_USER_LED2_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
	LPC_GPIO2->FIOCLR = GK_USER_LED2_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
   AXE410_ReadHoldingRegistersResponse();
//AXE410_Uart0Tx(0X41);
	}
	
	
}
}
