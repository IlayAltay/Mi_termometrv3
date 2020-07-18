/* USER CODE BEGIN Header */


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//Использовался Энкодер 80 тиков на оборот
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "dwt_stm32_delay.h"
#include "FlashPROM.h"
#include "ds18b20.h"
//#include <stdio>
//#include "iostream.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//using namespace std;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//переменные для работы с датчиком температуры и влажности
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;
int temp_low, temp_high, rh_low, rh_high;
char temp_char1[2], temp_char2, rh_char1[2], rh_char2;
uint8_t check = 0;
uint32_t res_addr = 0;  //адрес для записи во флэш
uint32_t timme=0;  //счетчик времни для записи в память
uint16_t count=0;    //счетчик для сохранения в память
char str1[60];  //ds18b20


//фукции переназначения входа и выхода пин а1 для работы с датчиком
GPIO_InitTypeDef GPIO_InitStruct;
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
//////////////////////////////////////////////////////////////////////
//фунуции работы с датчиком ьемпературы
void DHT22_start (void)
{
	set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);   // pull the pin low
	DWT_Delay_us (500);   // wait for 500us
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 1);   // pull the pin high
	DWT_Delay_us (30);   // wait for 30us
	set_gpio_input ();   // set as input
}

void check_response (void)
{
	DWT_Delay_us (40);
	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))
	{
		DWT_Delay_us (80);
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))) check = 1;
	}
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));   // wait for the pin to go low
}

uint8_t read_data (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));   // wait for the pin to go high
		DWT_Delay_us (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));  // wait for the pin to go low
	}
	return i;
}
////////////////////////////////////////////
 struct Menucomplect
{
	char* Line1;      ///первые четыре строчки для обновления дисплея буфер
	char* Line2;
	char* Line3;
	char* Line4;
	char* Line5;       //вторые четыре строчки  курсорные указатели главного меню
	char* Line6;
	char* Line7;
	char* Line8;
	char* Line9;       //третьи безкурсорные указатели строчки главного меню
	char* Line10;
	char* Line11;
	char* Line12;
}Main_menu;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///функция для первого заполнеия меню
void start_menu(struct Menucomplect *mst)
{
	mst->Line1=">  TIME    ";
	mst->Line2="   Vlagnost";
	mst->Line3="   Temperat";
	mst->Line4="   Nagruzka";
	////


}
void Menu_it_Update()
{
//	ssd1306_Fill(Black);
	ssd1306_SetCursor(0,0);

	ssd1306_WriteString("   TIME     ",Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,15);
	ssd1306_WriteString("   Vlagnost",Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,30);
	ssd1306_WriteString("   Temperat",Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,45);
	ssd1306_WriteString("   Nagruzka",Font_11x18,White);
	ssd1306_UpdateScreen();
}
void Menu_it_Updatestruc(struct Menucomplect *menu )
{
//	ssd1306_Fill(Black);
	ssd1306_SetCursor(0,0);

	ssd1306_WriteString(menu->Line1,Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,15);
	ssd1306_WriteString(menu->Line2,Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,30);
	ssd1306_WriteString(menu->Line3,Font_11x18,White);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,45);
	ssd1306_WriteString(menu->Line4,Font_11x18,White);
	ssd1306_UpdateScreen();

}
void LCD_update(char* chislo)
{
	   ssd1306_Fill(Black);
	    //ssd1306_UpdateScreen();
	    //HAL_Delay(50);
	    ssd1306_SetCursor(0,0);
	    ssd1306_WriteString(chislo,Font_11x18,White);
	    ssd1306_UpdateScreen();
}

//функция сборки меню по результатам перемещений курсора
int set_Menu(uint8_t up,uint8_t pozicia ,struct Menucomplect *lastmenu )
{
	 uint8_t new_poz;
	 char *tuda;
	 if(up==1){
		switch (pozicia)
		{
		case 1:
		        {
		        lastmenu->Line1=lastmenu->Line9;  //пишем безкурсорную
			    lastmenu->Line4=lastmenu->Line8;   // сюда ставим курсор
			    new_poz=4;
			    break;
		        }
		case 2:
				{
				lastmenu->Line2=lastmenu->Line10;  //пишем безкурсорную
				lastmenu->Line1=lastmenu->Line5;   // сюда ставим курсор
				new_poz=1;
				break;
				}
		case 3:
				{
			    lastmenu->Line3=lastmenu->Line11;  //пишем безкурсорную
				lastmenu->Line2=lastmenu->Line6;   // сюда ставим курсор
				new_poz=2;
				break;
				}
		case 4:
				{
				lastmenu->Line4=lastmenu->Line12;  //пишем безкурсорную
			    lastmenu->Line3=lastmenu->Line7;   // сюда ставим курсор
			    new_poz=3;
			    break;
				}
		default:
			break;
		}

	 }else{//иначе идем вниз
	switch(pozicia){
       case 1:
		        {
		        lastmenu->Line1=lastmenu->Line9;  //пишем безкурсорную
			    lastmenu->Line2=lastmenu->Line6;   // сюда ставим курсор
			    new_poz=2;
			    break;
		        }
		case 2:
				{
				lastmenu->Line2=lastmenu->Line10;  //пишем безкурсорную
				lastmenu->Line3=lastmenu->Line7;   // сюда ставим курсор
				new_poz=3;
				break;
				}
		case 3:
				{
			    lastmenu->Line3=lastmenu->Line11;  //пишем безкурсорную
				lastmenu->Line4=lastmenu->Line8;   // сюда ставим курсор
				new_poz=4;
				break;
				}
		case 4:
				{
				lastmenu->Line4=lastmenu->Line12;  //пишем безкурсорную
			    lastmenu->Line1=lastmenu->Line5;   // сюда ставим курсор
			    new_poz=1;
			    break;
				}
		default:
			break;
		}

	 }

	 Menu_it_Updatestruc(lastmenu);
	 return new_poz;
}
//конец функции сборки меню по рез крусора
//////////////////////////////////////////////////////////////////////////////////


//функ
//функция заполнения струтуры
/*
void beginstructMenu(struct Menucomplect *beginmenu, char *mm,int n)
{


	    beginmenu->Line5=mm[0];
	    beginmenu->Line6=mm[1];
	    beginmenu->Line7=mm[2];
	    beginmenu->Line8=mm[3];
}                                              */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//char *Name[100] = {"Ruslan","Artur","Timur","Ivan","Sveta","Kola"};

	char *strMainmenu[4]={"   TIME     ","   Vlagnost","   Temperat","   Nagruzka"};
    char *strcursMainmenu[4]={">  TIME     ",">  Vlagnost",">  Temperat",">  Nagruzka"};
    char buf[25];
	int32_t capture=0, capture_prev=0, encoder=0,encoder_old=0;
	char encoder_ch[10],TEMPch[10],RHch[10];
	char *p,*p2,*p3,*tekmenu,*m1,*m2c;
    uint8_t cursor_up=2,pozicia_cursora=1;  // направление движения курсора 1-вверх 0-вниз 2-никуда
	   //позиция крусора те в какой строке сейчас находится
    uint16_t k_butt=0;
   uint8_t MSByte=0; //старший байт
   uint8_t LSByte=0;  //младший байт
   uint16_t MSBLSB=0; // склееныный
   uint8_t falgset=0;  //флаг что надо сохранить в память
   char str[64] = {0,};
   uint8_t status;             //ds18b20
   uint8_t dt[8];          //ds18b20
   uint16_t raw_temper;    //ds18b20
   float temper;          //ds18b20
   char c;
   uint8_t temrez;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();     //инициализация дисплея
  DWT_Delay_Init ();
  HAL_Delay(1000);
   /*
    ssd1306_Fill(White);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);
    ssd1306_SetCursor(23,23);
    ssd1306_WriteString("Ziiiic",Font_11x18,Black);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);
    ssd1306_SetCursor(0,0);
    ssd1306_WriteString("12345678901",Font_11x18,White);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(0,15);
    ssd1306_WriteString("22222222222",Font_11x18,White);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(0,30);
    ssd1306_WriteString("33333333333",Font_11x18,White);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(0,45);
    ssd1306_WriteString("44444444444",Font_11x18,White);
    ssd1306_UpdateScreen();   */


    ///Encoder///
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
////// Первичное создание главного меню

//инитим структуру меню
    Main_menu.Line5=strcursMainmenu[0];
    Main_menu.Line6=strcursMainmenu[1];
    Main_menu.Line7=strcursMainmenu[2];
    Main_menu.Line8=strcursMainmenu[3];


    Main_menu.Line9=strMainmenu[0];
    Main_menu.Line10=strMainmenu[1];
    Main_menu.Line11=strMainmenu[2];
    Main_menu.Line12=strMainmenu[3];
    //создаем указатель на структуру меню

    tekmenu=&Main_menu;
    start_menu(tekmenu);         //первично заполняем строки
    Menu_it_Updatestruc(tekmenu);  //выводим на дисплей
   HAL_Delay(500);
     m1=&strMainmenu;
    m2c=&strcursMainmenu;

   //erase_flash();
    res_addr = flash_search_adress(STARTADDR, BUFFSIZE * DATAWIDTH);   //
    myBuf_t wdata[BUFFSIZE] = {0x1111, 0x2222, 0x3333, 0x4444, 0x0000};
   // myBuf_t wdata[BUFFSIZE] ;
   // beginstructMenu(tekmenu,strcursMainmenu,4);
    				//смотрим данные при запсуке
    				ssd1306_Fill(Black);
    	 	    	ssd1306_SetCursor(0,0);
    	 	    	snprintf(str, 64, ": MSB_%d \n",MSByte );
    	 	    	ssd1306_WriteString(str,Font_11x18,White);
    	 	    	snprintf(str, 64, ": LSB_%d \n",LSByte );
    	 	    	ssd1306_SetCursor(0,15);
    	 	    	ssd1306_WriteString(str,Font_11x18,White);
    	 	    	snprintf(str, 64, ": MSBLSB%d \n", MSBLSB);
    	 	    	ssd1306_SetCursor(0,30);
    	 	    	ssd1306_WriteString(str,Font_11x18,White);
    	 	    	snprintf(str, 64, ": x% \n",'move' );
    	 	    	ssd1306_SetCursor(0,45);
    	 	    	ssd1306_WriteString(str,Font_11x18,White);
    	 	    	ssd1306_UpdateScreen();
    	 	    	HAL_Delay(500);
    //восстанавливаем занчения из памяти
    	 	    	myBuf_t rdata[BUFFSIZE] = {0,}; // буфер для чтения (не обязательно его создавать, можно использовать буфер для записи)
                    read_last_data_in_flash(rdata); // чтение данных из флеша
                    MSByte=(uint8_t)(rdata[0] >> 8);  //старший байт
                    LSByte=(uint8_t)rdata[0];           //младший байт
                    //смотрим данные при чтении из памяти
                        				ssd1306_Fill(Black);
                        	 	    	ssd1306_SetCursor(0,0);
                        	 	    	snprintf(str, 64, ": MSB_%d \n",MSByte );
                        	 	    	ssd1306_WriteString(str,Font_11x18,White);
                        	 	    	snprintf(str, 64, ": LSB_%d \n",LSByte );
                        	 	    	ssd1306_SetCursor(0,15);
                        	 	    	ssd1306_WriteString(str,Font_11x18,White);
                        	 	    	snprintf(str, 64, ": MSBLSB%d \n", MSBLSB);
                        	 	    	ssd1306_SetCursor(0,30);
                        	 	    	ssd1306_WriteString(str,Font_11x18,White);
                        	 	    	snprintf(str, 64, ": x% \n",'move' );
                        	 	    	ssd1306_SetCursor(0,45);
                        	 	    	ssd1306_WriteString(str,Font_11x18,White);
                        	 	    	ssd1306_UpdateScreen();
                        	 	    	HAL_Delay(500);
 /*  ssd1306_Fill(Black);
       ssd1306_UpdateScreen();
   ssd1306_SetCursor(0,0);
   	ssd1306_WriteString(strMainmenu[3],Font_11x18,White);
   	ssd1306_UpdateScreen();           */
port_init2();  //ds18b20 lib
status=ds18b20_init2(SKIP_ROM);
snprintf(str1,60,"_%d",status);
ssd1306_Fill(Black);
ssd1306_SetCursor(0,0);
ssd1306_WriteString(str1,Font_11x18,White);
ssd1306_SetCursor(0,15);
ssd1306_WriteString("Proshel",Font_11x18,White);
ssd1306_UpdateScreen();
HAL_Delay(2000);
/*
 * port_init();  //ds18b20 lib
status=ds18b20_init(SKIP_ROM);
snprintf(str1,60,"_%d",status);
ssd1306_Fill(Black);
ssd1306_SetCursor(0,0);
ssd1306_WriteString(str1,Font_11x18,White);
ssd1306_SetCursor(0,15);
ssd1306_WriteString("Proshel",Font_11x18,White);
ssd1306_UpdateScreen();
HAL_Delay(2000);
 *было для PB11
 *
 **/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //работа с датчиком  для PA3
	  	  ds18b20_MeasureTemperCmd2(SKIP_ROM, 0);
	  	  HAL_Delay(800);
	  	  ds18b20_ReadStratcpad2(SKIP_ROM, dt, 0);
	  	  snprintf(str1,60,"%02X %02X %02X %02X ",
	  			  dt[0], dt[1], dt[2], dt[3]);
	  	  ssd1306_Fill(Black);
	  	  ssd1306_SetCursor(0,0);
	  	  ssd1306_WriteString(str1,Font_11x18,White);
	  	  snprintf(str1,60,"%02X %02X %02X %02X; ",
	  	  	   dt[4], dt[5], dt[6], dt[7]);
	  	  ssd1306_SetCursor(0,15);
	  	  ssd1306_WriteString(str1,Font_11x18,White);
	  	  raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
	  	  if(ds18b20_GetSign(raw_temper)) c='-';
	  	  else c='+';
	  	  temper = ds18b20_Convert(raw_temper);
	  	  snprintf(str1,60,"%c",c);
	   	  ssd1306_SetCursor(0,30);
	  	  ssd1306_WriteString(str1,Font_11x18,White);
	  	  temrez=(uint8_t)(floor(temper*10));
	  	  snprintf(str1,60,"%c%d.%d'C",c, ((uint8_t)(floor(temper))),temrez%10);
	  	  ssd1306_SetCursor(0,45);
	  	  ssd1306_WriteString(str1,Font_11x18,White);
	  	  ssd1306_UpdateScreen();
	  	  HAL_Delay(150);

	  //работа с датчиком  для PB11
	  /*    раскомментить
	  ds18b20_MeasureTemperCmd(SKIP_ROM, 0);
	  HAL_Delay(800);
	  ds18b20_ReadStratcpad(SKIP_ROM, dt, 0);
	  snprintf(str1,60,"%02X %02X %02X %02X ",
	  dt[0], dt[1], dt[2], dt[3]);
	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(0,0);
	  ssd1306_WriteString(str1,Font_11x18,White);
	 // HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	 // sprintf(str1,"rn");
	  snprintf(str1,60,"%02X %02X %02X %02X; ",
	  	   dt[4], dt[5], dt[6], dt[7]);
	  ssd1306_SetCursor(0,15);
	  ssd1306_WriteString(str1,Font_11x18,White);
	  //ssd1306_UpdateScreen();
	  //HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	  raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
	  if(ds18b20_GetSign(raw_temper)) c='-';
	  else c='+';
	  temper = ds18b20_Convert(raw_temper);
	  //snprintf(str1,"Raw t: 0x%04X; t: %c%.2frn", raw_temper, c, temper);
	  //snprintf(str1,60,"+%s","dfty");
	  snprintf(str1,60,"%c",c);
	//  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	  ssd1306_SetCursor(0,30);
	  ssd1306_WriteString(str1,Font_11x18,White);


	  temrez=(uint8_t)(floor(temper*10));

	  //snprintf(str1,60,"%d.%d", ((uint8_t)(floor(temper))),temrez%10);
	  snprintf(str1,60,"%c%d.%d'C",c, ((uint8_t)(floor(temper))),temrez%10);
	  ssd1306_SetCursor(0,45);
	  ssd1306_WriteString(str1,Font_11x18,White);
	  ssd1306_UpdateScreen();
	  HAL_Delay(150);

       //раскоментить
	  */


	  /*

/////////Нажатие кнопки
	  if(HAL_GPIO_ReadPin(GPIOA, Button_enc_Pin)==GPIO_PIN_RESET)

	                  {
		             while(HAL_GPIO_ReadPin(GPIOA, Button_enc_Pin)==GPIO_PIN_RESET)
		             {
		            	k_butt=k_butt+1;
		            	HAL_Delay(5);

	                  }
		              if(k_butt>20){
		            	  Menu_it_Updatestruc(tekmenu);
		            	 // Menu_it_Update();
		             //HAL_GPIO_TogglePin(GPIOC, led_Pin);

		             		             //HAL_Delay(10);
	                  }
	                  }
	 /////конец нажатия кнопки
//////////Читаем энкодер
	      capture = TIM3->CNT;

	      encoder += capture - capture_prev;


	      if (abs(capture-capture_prev)>3) {     ///32767
	        encoder += (capture<capture_prev ? 1 : -1);  //65535
  	        HAL_GPIO_TogglePin(GPIOC, led_Pin);//светодиод для наглядности при перещелкивании энкодера

           if(encoder>encoder_old){
        	   cursor_up=0;    //так как енкодер перещелкнул вверх
        	   encoder_old=encoder;

           }else if(encoder<encoder_old){
        	   cursor_up=1;
        	   encoder_old=encoder;
           }
       //проверяем границы счетчика
           if(encoder>9){
                   		   encoder=0;
                   		   encoder_old=0;
            }else if(encoder<0){
                   	     encoder=9;
                   	     encoder_old=9;
            }
       //конец границы счетчика
	  //      p = itoa(encoder,encoder_ch,10);
	   //     LCD_update(p);
	      }
	      capture_prev = capture;
/////////Конец чтения энкодера
          if(cursor_up!=2){
        	pozicia_cursora=set_Menu(cursor_up,pozicia_cursora,tekmenu);
             cursor_up=2;
          }
   */

	  /*
	     //читаем поворот энкодера
	  if(HAL_GPIO_ReadPin(GPIOA, Button_enc_Pin)==GPIO_PIN_RESET)     {
	  MSByte=255;
	  LSByte=32;
	  MSBLSB=MSByte<<8|LSByte;
	  falgset=1;  //надо записать в память
	  HAL_Delay(500);
	  }
	  	  	  capture = TIM3->CNT;
	 	      encoder += capture - capture_prev;
	 	     if (abs(capture-capture_prev)>3) {
	 	    	 if(MSByte<255){
	 	    		MSByte++;
	 	    	 }else if(LSByte<255){
	 	    		LSByte++;
	 	    	 }else{
	 	    		MSByte=0;
	 	    		LSByte=0;
	 	    	 }
	 	    	char str[64] = {0,};
	 	       //если было изменение то клеим байт
	 	    	 MSBLSB=MSByte<<8|LSByte;
	 	    	 falgset=1;  //надо записать в память
	 	    	//выводим на дисплей

	 	    	ssd1306_Fill(Black);
	 	    	ssd1306_SetCursor(0,0);
	 	    	snprintf(str, 64, ": MSB_%d \n",MSByte );
	 	    	ssd1306_WriteString(str,Font_11x18,White);
	 	    	snprintf(str, 64, ": LSB_%d \n",LSByte );
	 	    	ssd1306_SetCursor(0,15);
	 	    	ssd1306_WriteString(str,Font_11x18,White);
	 	    	snprintf(str, 64, ": %x \n", 'MSBLSB');
	 	    	ssd1306_SetCursor(0,30);
	 	    	ssd1306_WriteString(str,Font_11x18,White);
	 	    	snprintf(str, 64, ": d% \n",MSBLSB );
	 	    	ssd1306_SetCursor(0,45);
	 	    	ssd1306_WriteString(str,Font_11x18,White);
	 	    	ssd1306_UpdateScreen();

	 	     }


          //тест записи в память

          //if((HAL_GetTick() - timme) > 10000) // интервал  10сек
        	  if(falgset!=0) // интервал  10сек
                  {
        		  	  	  wdata[0]=MSBLSB;
                         // wdata[0] = wdata[0] + 1; // просто для разнообразия
                          wdata[1] = wdata[1] + 1;
                          wdata[2] = wdata[2] + 1;
                          wdata[3] = wdata[3] + 1;

                          write_to_flash(wdata); // запись данных во флеш

                          timme = HAL_GetTick();
                          count++;
                          HAL_GPIO_TogglePin(GPIOC, led_Pin);
                          falgset=0;
                  }
                  */
    //чтение из памяти
        	  /*
                  if(count > 2)
                  {
                          count = 0;

                          myBuf_t rdata[BUFFSIZE] = {0,}; // буфер для чтения (не обязательно его создавать, можно использовать буфер для записи)

                          read_last_data_in_flash(rdata); // чтение данных из флеша

                          char str[64] = {0,};
                          //snprintf(str, 64, "Read data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", rdata[0], rdata[1], rdata[2], rdata[3], rdata[4]);
                          snprintf(str, 64, ": 0x%x \n", rdata[0]);
                        //  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
                            ssd1306_Fill(Black);
                          	ssd1306_SetCursor(0,0);
                          	ssd1306_WriteString(str,Font_11x18,White);
                          	//ssd1306_UpdateScreen();
                          	snprintf(str, 64, ": 0x%x \n", rdata[1]);
                          	ssd1306_SetCursor(0,15);
                          	ssd1306_WriteString(str,Font_11x18,White);
                          //	ssd1306_UpdateScreen();
                          	snprintf(str, 64, ": 0x%x \n", rdata[2]);
                          	ssd1306_SetCursor(0,30);
                          	ssd1306_WriteString(str,Font_11x18,White);

                          //  ssd1306_UpdateScreen();
                          	snprintf(str, 64, ": 0x%x \n", rdata[3]);
                          	ssd1306_SetCursor(0,45);
                          	ssd1306_WriteString(str,Font_11x18,White);
                          	ssd1306_UpdateScreen();

                  }
                  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
          ///работа с датчиком температуры и влажности

       /*   DHT22_start ();
          		check_response ();
          		Rh_byte1 = read_data ();
          		Rh_byte2 = read_data ();
          		Temp_byte1 = read_data ();
          		Temp_byte2 = read_data ();
          		sum = read_data();
          		//if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
          		{
          			TEMP = ((Temp_byte1<<8)|Temp_byte2);
          			RH = ((Rh_byte1<<8)|Rh_byte2);
          		}

          		temp_low = TEMP/10;
          		temp_high = TEMP%10;

          		rh_low = RH/10;
          		rh_high = RH%10;
          		//выводим на дисплей
          		     p3 = itoa(TEMP,TEMPch,10);
          		     p2=itoa(RH,RHch,10);
          			   //     LCD_update(p);
          		    ssd1306_SetCursor(0,15);
          			ssd1306_WriteString(p2,Font_11x18,White);
          			ssd1306_UpdateScreen();
          			ssd1306_SetCursor(0,30);
          			ssd1306_WriteString(p3,Font_11x18,White);
          			ssd1306_UpdateScreen();
          			HAL_Delay(300);
          		///конец работы с датчиком температуры*/
 ////////////////////////////////////////////////
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(temp2_GPIO_Port, temp2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : tempSensor_Pin */
  GPIO_InitStruct.Pin = tempSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(tempSensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_enc_Pin */
  GPIO_InitStruct.Pin = Button_enc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_enc_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : temp2_Pin */
  GPIO_InitStruct.Pin = temp2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(temp2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : temp_Pin */
  GPIO_InitStruct.Pin = temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(temp_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
