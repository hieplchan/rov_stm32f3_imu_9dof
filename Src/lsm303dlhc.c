#include "stm32f3xx_hal.h"

#define 	MAG_ADDR 0x3C
#define		ACC_ADDR 0x32

/* Acceleration Registers */
#define LSM303DLHC_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303DLHC_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303DLHC_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303DLHC_CTRL_REG5_A               0x24  /* Control register 2 acceleration */
#define LSM303DLHC_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303DLHC_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303DLHC_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303DLHC_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303DLHC_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */ 

/* Magnetic field Registers */
#define LSM303DLHC_CRA_REG_M                 0x00  /* Control register A magnetic field */
#define LSM303DLHC_CRB_REG_M                 0x01  /* Control register B magnetic field */
#define LSM303DLHC_MR_REG_M                  0x02  /* Control register MR magnetic field */
#define LSM303DLHC_OUT_X_H_M                 0x03  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_X_L_M                 0x04  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_Z_H_M                 0x05  /* Output Register Z magnetic field */
#define LSM303DLHC_OUT_Z_L_M                 0x06  /* Output Register Z magnetic field */ 
#define LSM303DLHC_OUT_Y_H_M                 0x07  /* Output Register Y magnetic field */
#define LSM303DLHC_OUT_Y_L_M                 0x08  /* Output Register Y magnetic field */


extern I2C_HandleTypeDef hi2c1;
extern void Error_Handler(void);

//Doc 1 byte tu thanh ghi lsm303dlhc
uint8_t lsm_read(uint16_t Addr, uint8_t Reg)
{
	uint8_t value = 0;	
	HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 0x00000001U, &value, 1, 0x10000);
	return value;
}

//Ghi 1 byte tu thanh ghi lsm303dlhc
void lsm_write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, 0x00000001U, &Value, 1, 0x10000);
}

//Cau hinh co ban cho lsm303dlhc
void lsm_init(void)
{
	uint8_t ctrl1a, ctrl2a, ctrl4a;
	uint8_t cra_m, crb_m, mr_m;
	
	ctrl1a = 0x47; //01000111
	ctrl2a = 0x90; //10010000
	ctrl4a = 0x08; //00001000
	
	cra_m = 0x1C; //00011100
	crb_m = 0xE0; //11100000
	mr_m = 0x00;
	
	lsm_write(ACC_ADDR, LSM303DLHC_CTRL_REG1_A, ctrl1a);
	lsm_write(ACC_ADDR, LSM303DLHC_CTRL_REG2_A, ctrl2a);
	lsm_write(ACC_ADDR, LSM303DLHC_CTRL_REG4_A, ctrl4a);
	lsm_write(MAG_ADDR, LSM303DLHC_CRA_REG_M, cra_m);
	lsm_write(MAG_ADDR, LSM303DLHC_CRB_REG_M, crb_m);
	lsm_write(MAG_ADDR, LSM303DLHC_MR_REG_M, mr_m);
	
}

//Khoi dong laj bo nho
void lsm_reboot(void)
{
	uint8_t tmpreg;  
  tmpreg = lsm_read(ACC_ADDR, LSM303DLHC_CTRL_REG5_A);
  tmpreg |= 0x80;
  lsm_write(ACC_ADDR, LSM303DLHC_CTRL_REG5_A, tmpreg);
}

//Doc trang thai lsm303dlhc
uint8_t lsm_status(void)
{
	uint8_t tmpreg;  
  tmpreg = lsm_read(ACC_ADDR, LSM303DLHC_STATUS_REG_A);
  return tmpreg;
}

//Doc XYZ Acc
void lsm_accxyz(float *pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2]={0,0};
  int8_t buffer[6];
  uint8_t i = 0;
  uint8_t sensitivity = 1.0f;
  
  ctrlx[0] = lsm_read(ACC_ADDR, LSM303DLHC_CTRL_REG4_A);
  ctrlx[1] = lsm_read(ACC_ADDR, LSM303DLHC_CTRL_REG5_A);
  
  buffer[0] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_X_L_A); 
  buffer[1] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_X_H_A);
  buffer[2] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_Y_L_A);
  buffer[3] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_Y_H_A);
  buffer[4] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_Z_L_A);
  buffer[5] = lsm_read(ACC_ADDR, LSM303DLHC_OUT_Z_H_A);
	
  if(!(ctrlx[0] & 0x40)) 
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
    }
  }
	
  switch(ctrlx[0] & 0x30)
  {
  case 0x00:
    sensitivity = (float) 1.0f;
    break;
  case 0x10:
    sensitivity = (float) 2.0f;
    break;
  case 0x20:
    sensitivity = (float) 4.0f;
    break;
  case 0x30:
    sensitivity = (float) 12.0f;
    break;
  }
  
  for(i=0; i<3; i++)
  {
    pfData[i]=(float) pnRawData[i]*sensitivity/1740;
  }
}
//Doc xyz Mag
void lsm_magxyz(float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  CTRLB = lsm_read(MAG_ADDR, LSM303DLHC_CRB_REG_M);
  
  buffer[0] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_X_H_M);
  buffer[1] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_X_L_M);
  buffer[2] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_Y_H_M);
  buffer[3] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_Y_L_M);
  buffer[4] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_Z_H_M);
  buffer[5] = lsm_read(MAG_ADDR, LSM303DLHC_OUT_Z_L_M);
	

  switch(CTRLB & 0xE0)
  {
  case 0x20:
    Magn_Sensitivity_XY = 1100;
    Magn_Sensitivity_Z = 980;
    break;
  case 0x40:
    Magn_Sensitivity_XY = 855;
    Magn_Sensitivity_Z = 760;
    break;
  case 0x60:
    Magn_Sensitivity_XY = 670;
    Magn_Sensitivity_Z = 600;
    break;
  case 0x80:
    Magn_Sensitivity_XY = 450;
    Magn_Sensitivity_Z = 400;
    break;
  case 0xA0:
    Magn_Sensitivity_XY = 400;
    Magn_Sensitivity_Z = 355;
    break;
  case 0xC0:
    Magn_Sensitivity_XY = 330;
    Magn_Sensitivity_Z = 295;
    break;
  case 0xE0:
    Magn_Sensitivity_XY = 230;
    Magn_Sensitivity_Z = 205;
    break;
  }
  
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}






