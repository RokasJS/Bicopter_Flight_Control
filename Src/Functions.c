#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"
#include "Functions.h"
#include "main.h"

//RX

void NSS_RX(uint8_t Number)  //NSS rx pin valdymas (reikalingas spi komunikacijoms)
{
	switch(Number)
	{
		case 0:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
		break;
		
		case 1:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);		
		break;
	}	
}

void reciever_settings(uint8_t adress, uint8_t data, uint8_t size)
{
	uint8_t  d[2];
	d[0]=adress|0x80;
	d[1]=data;
	NSS_RX(0);
	HAL_SPI_Transmit(&hspi2, d, size, 1);
	NSS_RX(1);
}


void RFM96WRX_Config(void)
{
		
	//enter sleep mode (for setings configuration)
	reciever_settings(0x01, 0x08, 2);
	HAL_Delay(50);
	
		
	
	//Enter Lora
	reciever_settings(0x01, 0x88, 2);
	
	//setting frequency 434MHz
	reciever_settings(0x06, 0x6c, 2);      //write Msb
	reciever_settings(0x07, 0x80, 2);			//write Mid
	reciever_settings(0x08, 0x00, 2);			//write Lsb

	
	//setting base parameters
	reciever_settings(0x09, 0xF9, 2); //power
	reciever_settings(0x0B, 0x0B, 2); //over current
	
	//LNA settings
	reciever_settings(0x0C, 0x23, 2);
	
	//Bw and CRC coding rate settings
	reciever_settings(0x1D, 0x72, 2); ////
	
	//SFactor and LNA SETINGS and SFactor
	reciever_settings(0x1E, 0x87, 2);     //
	
	//LowDataRateOptimize on, AgcAutoOn 
	reciever_settings(0x26, 0x0C, 2);
	
	//RX symbol time out
	reciever_settings(0x1F, 0xFF, 2);
	
	//Preamble
	reciever_settings(0x20, 0x00, 2);
	reciever_settings(0x21, 0x12, 2);
	
	//Payload settings
	reciever_settings(0x22, 0x5, 2);
	reciever_settings(0x23, 0x5, 2);
	
	//Enter STNDBY
	reciever_settings(0x01, 0x09, 2);
	
	// Normal and Rx
	reciever_settings(0x4D, 0x84, 2);
	
	//RegHopPeriod NO FHSS 
	reciever_settings(0x24, 0x00, 2);
	//Open RxTimeout, RxDone, PayloadCrcError  interrupt,           
	reciever_settings(0x11, 0x00, 2);   // buvo 0x1F
	
	// clear IRQ flags
	reciever_settings(0x12, 0xFF, 2);
	
	//PAYLOUD lenght
	
	reciever_settings(0x22, 5, 2);
	
	// fifo buferio adreso gavimas
	uint8_t adress, res;
	NSS_RX(0);
	adress=0x0F;   
	HAL_SPI_Transmit(&hspi2, &adress, 1, 100);
  HAL_SPI_Receive(&hspi2, &res, 1, 100);
	NSS_RX(1);
	
	reciever_settings(0x0D, res, 2);
	HAL_Delay(10);
	
	// RX continiuos mode 
	reciever_settings(0x01, 0x05+0x08, 2);
	
	
}

void RFM96WRX_recieve(uint8_t *data)
{
	
	uint8_t adress, rez, sizerecieved;
	uint8_t res11[5];
	
	NSS_RX(0);
	adress=0x12;   
	HAL_SPI_Transmit(&hspi2, &adress, 1, 1); //reading flags
  HAL_SPI_Receive(&hspi2, &rez, 1, 1);
	NSS_RX(1);
	// HAL_Delay (1); 
		if(rez==0x50)   //rx done flag valid
		{
		
		rez=0;	
		
	NSS_RX(0);  // last packet adress
	adress=0x10;   
	HAL_SPI_Transmit(&hspi2, &adress, 1, 1);
  HAL_SPI_Receive(&hspi2, &rez, 1, 1);
	NSS_RX(1);
	reciever_settings(0x0D, rez, 2);
					
	NSS_RX(0);   // number of byte recieved
	adress=0x13;   
	HAL_SPI_Transmit(&hspi2, &adress, 1, 1);
  HAL_SPI_Receive(&hspi2, &sizerecieved, 1, 1);
	NSS_RX(1);
			
			
	adress=0;
	NSS_RX(0);			
	HAL_SPI_Transmit(&hspi2, &adress, 1, 1);
	HAL_SPI_Receive(&hspi2, res11, sizerecieved, 1);
	NSS_RX(1);	
	*data=res11[0];
	*(data+1)=res11[1];
	*(data+2)=res11[2];
	*(data+3)=res11[3];
		
	}
	// clear IRQ flags
	reciever_settings(0x12, 0xFF, 2);
}

