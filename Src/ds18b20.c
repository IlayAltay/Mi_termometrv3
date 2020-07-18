#include "ds18b20.h"
//--------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
micros *= (SystemCoreClock / 1000000) / 9;
/* Wait till done */
while (micros--) ;
}
//--------------------------------------------------
void port_init(void)
{
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
  GPIOB->CRH |= GPIO_CRH_MODE11;
  GPIOB->CRH |= GPIO_CRH_CNF11_0;
  GPIOB->CRH &= ~GPIO_CRH_CNF11_1;
}
//----------------------------------------------------
void port_init2(void)
{
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
  GPIOA->CRL |= GPIO_CRL_MODE3;
  GPIOA->CRL |= GPIO_CRL_CNF3_0;
  GPIOA->CRL &= ~GPIO_CRL_CNF3_1;
}
//--------------------------------------------------
uint8_t ds18b20_Reset2(void)
{
  uint16_t status;
	GPIOA->ODR &= ~GPIO_ODR_ODR3;//низкий уровень
  DelayMicro(485);//задержка как минимум на 480 микросекунд
  GPIOA->ODR |= GPIO_ODR_ODR3;//высокий уровень
  DelayMicro(65);//задержка как минимум на 60 микросекунд
  status = GPIOA->IDR & GPIO_IDR_IDR3;//провер€ем уровень
  DelayMicro(500);//задержка как минимум на 480 микросекунд
  //(на вс€кий случай подождЄм побольше, так как могут быть неточности в задержке)
  return (status ? 1 : 0);//вернЄм результат
}
//------------------------------------------------------

uint8_t ds18b20_Reset(void)
{
  uint16_t status;
	GPIOB->ODR &= ~GPIO_ODR_ODR11;//низкий уровень
  DelayMicro(485);//задержка как минимум на 480 микросекунд
  GPIOB->ODR |= GPIO_ODR_ODR11;//высокий уровень
  DelayMicro(65);//задержка как минимум на 60 микросекунд
  status = GPIOB->IDR & GPIO_IDR_IDR11;//провер€ем уровень
  DelayMicro(500);//задержка как минимум на 480 микросекунд
  //(на вс€кий случай подождЄм побольше, так как могут быть неточности в задержке)
  return (status ? 1 : 0);//вернЄм результат
}
//----------------------------------------------------------
uint8_t ds18b20_ReadBit2(void)
{
  uint8_t bit = 0;
  GPIOA->ODR &= ~GPIO_ODR_ODR3;//низкий уровень
  DelayMicro(2);
	GPIOA->ODR |= GPIO_ODR_ODR3;//высокий уровень
	DelayMicro(13);
	bit = (GPIOA->IDR & GPIO_IDR_IDR3 ? 1 : 0);//провер€ем уровень
	DelayMicro(45);
  return bit;
}
//------------------------------------------------------------
uint8_t ds18b20_ReadBit(void)
{
  uint8_t bit = 0;
  GPIOB->ODR &= ~GPIO_ODR_ODR11;//низкий уровень
  DelayMicro(2);
	GPIOB->ODR |= GPIO_ODR_ODR11;//высокий уровень
	DelayMicro(13);
	bit = (GPIOB->IDR & GPIO_IDR_IDR11 ? 1 : 0);//провер€ем уровень
	DelayMicro(45);
  return bit;
}
//--------------------------------------------------------------
uint8_t ds18b20_ReadByte2(void)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
  data += ds18b20_ReadBit2() << i;
  return data;
}
//-----------------------------------------------
uint8_t ds18b20_ReadByte(void)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
  data += ds18b20_ReadBit() << i;
  return data;
}
//-----------------------------------------------
void ds18b20_WriteBit2(uint8_t bit)
{
  GPIOA->ODR &= ~GPIO_ODR_ODR3;
  DelayMicro(bit ? 3 : 65);
  GPIOA->ODR |= GPIO_ODR_ODR3;
  DelayMicro(bit ? 65 : 3);
}
//----------------------------------------------------------

void ds18b20_WriteBit(uint8_t bit)
{
  GPIOB->ODR &= ~GPIO_ODR_ODR11;
  DelayMicro(bit ? 3 : 65);
  GPIOB->ODR |= GPIO_ODR_ODR11;
  DelayMicro(bit ? 65 : 3);
}
//-----------------------------------------------
void ds18b20_WriteByte2(uint8_t dt)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    ds18b20_WriteBit2(dt >> i & 1);
    //Delay Protection
    DelayMicro(5);
  }
}
//------------------------------------------------------------

void ds18b20_WriteByte(uint8_t dt)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    ds18b20_WriteBit(dt >> i & 1);
    //Delay Protection
    DelayMicro(5);
  }
}
//-----------------------------------------------
uint8_t ds18b20_init2(uint8_t mode)
{
	if(ds18b20_Reset2()) return 1;
  if(mode==SKIP_ROM)
  {
		//SKIP ROM
		ds18b20_WriteByte2(0xCC);
		//WRITE SCRATCHPAD
		ds18b20_WriteByte2(0x4E);
		//TH REGISTER 100 градусов
		ds18b20_WriteByte2(0x64);
		//TL REGISTER - 30 градусов
		ds18b20_WriteByte2(0x9E);
		//Resolution 12 bit
		ds18b20_WriteByte2(RESOLUTION_12BIT);
  }
  return 0;
}
//-------------------------------------------------------------
uint8_t ds18b20_init(uint8_t mode)
{
	if(ds18b20_Reset()) return 1;
  if(mode==SKIP_ROM)
  {
		//SKIP ROM
		ds18b20_WriteByte(0xCC);
		//WRITE SCRATCHPAD
		ds18b20_WriteByte(0x4E);
		//TH REGISTER 100 градусов
		ds18b20_WriteByte(0x64);
		//TL REGISTER - 30 градусов
		ds18b20_WriteByte(0x9E);
		//Resolution 12 bit
		ds18b20_WriteByte(RESOLUTION_12BIT);
  }
  return 0;
}
//------------------------------------------------------------
void ds18b20_MeasureTemperCmd2(uint8_t mode, uint8_t DevNum)
{
  ds18b20_Reset2();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte2(0xCC);
  }
  //CONVERT T
  ds18b20_WriteByte2(0x44);
}
//----------------------------------------------------------
void ds18b20_MeasureTemperCmd(uint8_t mode, uint8_t DevNum)
{
  ds18b20_Reset();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte(0xCC);
  }
  //CONVERT T
  ds18b20_WriteByte(0x44);
}
//----------------------------------------------------------
void ds18b20_ReadStratcpad2(uint8_t mode, uint8_t *Data, uint8_t DevNum)
{
  uint8_t i;
  ds18b20_Reset2();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte2(0xCC);
  }
  //READ SCRATCHPAD
  ds18b20_WriteByte2(0xBE);
  for(i=0;i<8;i++)
  {
    Data[i] = ds18b20_ReadByte2();
  }
}
//-----------------------------------------------------------
void ds18b20_ReadStratcpad(uint8_t mode, uint8_t *Data, uint8_t DevNum)
{
  uint8_t i;
  ds18b20_Reset();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte(0xCC);
  }
  //READ SCRATCHPAD
  ds18b20_WriteByte(0xBE);
  for(i=0;i<8;i++)
  {
    Data[i] = ds18b20_ReadByte();
  }
}
//----------------------------------------------------------
uint8_t ds18b20_GetSign(uint16_t dt)
{
  //ѕроверим 11-й бит
  if (dt&(1<<11)) return 1;
  else return 0;
}
//----------------------------------------------------------
float ds18b20_Convert(uint16_t dt)
{
  float t;
  t = (float) ((dt&0x07FF)>>4); //отборосим знаковые и дробные биты
  //ѕрибавим дробную часть
  t += (float)(dt&0x000F) / 16.0f;
  return t;
}
//----------------------------------------------------------
