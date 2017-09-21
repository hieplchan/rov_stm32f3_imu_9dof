#include "stm32f3xx_hal.h"

#define spi_enable HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define spi_disable HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define SPI_Timeout (uint32_t)0x1000

extern void Error_Handler(void);
extern SPI_HandleTypeDef hspi1;


//Gui nhan byte qua spi1
uint8_t l3gd20_sendbyte(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SPI_Timeout); 
  return receivedbyte;
}

//Gui byte cho l3gd20
void l3gd20_write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)0x40;
  }
  spi_enable;
  l3gd20_sendbyte(WriteAddr);
  
  while(NumByteToWrite >= 0x01)
  {
    l3gd20_sendbyte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }  
  spi_disable;
}

//Doc byte tu l3gd20
void l3gd20_read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)((uint8_t)0x80| (uint8_t)0x40);
  }
  else
  {
    ReadAddr |= (uint8_t)0x80;
  }
	
  spi_enable;
  l3gd20_sendbyte(ReadAddr);
  
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to GYROSCOPE (Slave device) */
    *pBuffer = l3gd20_sendbyte((uint8_t)0x00);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  spi_disable;
}

//Cau hinh co ban cho l3gd20
void l3gd20_init(void)
{
	uint8_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5;
	
	//Mode select
	ctrl1 = 0X3F; //95Hz, Cut-off 25, normal mode, enable xyz, 0b00111111
	
	//High pass config
	ctrl2 = 0X00; // Normal mode res, Cut-off 0Hz
	
	//Disable interrupt
	ctrl3 = 0X00;
	
	//Data mode
	ctrl4 = 0X10; //Continuos update, LSB, 500dps, 4-wire spi
	
	//Enable
	ctrl5 = 0X10;	
	l3gd20_write(&ctrl1, 0x20, 1); //Control register 1
	l3gd20_write(&ctrl2, 0x21, 1); //Control register 2
	l3gd20_write(&ctrl3, 0x22, 1); //Control register 3
	l3gd20_write(&ctrl4, 0x23, 1); //Control register 4	
	l3gd20_write(&ctrl5, 0x24, 1); //Control register 5
}

//Khoi dong laij l3gd20
void l3gd20_reboot(void)
{
  uint8_t tmpreg;  
  l3gd20_read(&tmpreg, 0x24, 1);
  tmpreg |= 0x80;
  l3gd20_write(&tmpreg, 0x24, 1);
}

//Doc trang thai l3gd20
uint8_t l3gd20_status(void)
{
  uint8_t tmpreg;  
  l3gd20_read(&tmpreg, 0x27, 1);  //Status register
  return tmpreg;
}

//Doc du lieu XYZ
void l3gd20_readxyz(float *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  l3gd20_read(&tmpreg, 0x23, 1);  
  l3gd20_read(tmpbuffer, 0x28, 6);
  
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity =(float)114.285f ; //250dps
    break;
    
  case 0x10:
    sensitivity = (float)57.1429f ; //500dps
    break;
    
  case 0x20:
    sensitivity = (float)14.285f; //2000dps
    break;
  }
	
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] / sensitivity);
  }
}
