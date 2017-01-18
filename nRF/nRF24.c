

#include "nRF24.h"
#include "stm32f10x_spi.h"


/* Configure device into either Receiver or Transmitter */
void nRF_RX_TX_MODE(char mode){

	
}



/* Read values from a specific SPI slave register

	char command - 8-bit combination of register command and register address

*/
char nRF_READ(char command){
	
	CSN_HIGH();
	CSN_LOW();
	
	
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI2, command);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);
 
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI2, NOP);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	
	CSN_HIGH();
	
	return SPI_I2S_ReceiveData(SPI2);
}


/* Write values to a specific SPI slave register

	char reg_val - value to be set in the register
	char command - 8-bit combination of register command and register address

*/
void nRF_WRITE(char command, char reg_val){


	CSN_HIGH();
	CSN_LOW();
	
	
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI2, command);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);
 
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI2, reg_val);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);
	
	CSN_HIGH();
	
}


/* Sets SPI CSN pin to low */
void CSN_LOW(void){
GPIO_ResetBits(SPI_GPIO_Port, SPI_CSN);
}


/* Sets SPI CSN pin to high */
void CSN_HIGH(void){
GPIO_SetBits(SPI_GPIO_Port, SPI_CSN);
}
