/*
 * FlashPROM.c
 *
 *  Created on: 30 РґРµРє. 2019 Рі.
 *      Author: dima
 */

#include "FlashPROM.h"

extern CRC_HandleTypeDef hcrc;
// extern UART_HandleTypeDef huart1;
extern uint32_t res_addr;


//////////////////////// РћР§Р�РЎРўРљРђ РџРђРњРЇРўР� /////////////////////////////
void erase_flash(void)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;     // СЃС‚СЂСѓРєС‚СѓСЂР° РґР»СЏ РѕС‡РёСЃС‚РєРё С„Р»РµС€Р°

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // РїРѕСЃС‚СЂР°РЅРёС‡РЅР°СЏ РѕС‡РёСЃС‚РєР°, FLASH_TYPEERASE_MASSERASE - РѕС‡РёСЃС‚РєР° РІСЃРµРіРѕ С„Р»РµС€Р°
	EraseInitStruct.PageAddress = STARTADDR;
	EraseInitStruct.NbPages = PAGES;
	//EraseInitStruct.Banks = FLASH_BANK_1; // FLASH_BANK_2 - Р±Р°РЅРє в„–2, FLASH_BANK_BOTH - РѕР±Р° Р±Р°РЅРєР°
	uint32_t page_error = 0; // РїРµСЂРµРјРµРЅРЅР°СЏ, РІ РєРѕС‚РѕСЂСѓСЋ Р·Р°РїРёС€РµС‚СЃСЏ Р°РґСЂРµСЃ СЃС‚СЂР°РЅРёС†С‹ РїСЂРё РЅРµСѓРґР°С‡РЅРѕРј СЃС‚РёСЂР°РЅРёРё

	HAL_FLASH_Unlock(); // СЂР°Р·Р±Р»РѕРєРёСЂРѕРІР°С‚СЊ С„Р»РµС€

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK)
	{
		uint32_t er = HAL_FLASH_GetError();
		char str[64] = {0,};
		snprintf(str, 64, "ER %lu\n", er);
		//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
	}
	else
	{
		#if DEBUG
	//	HAL_UART_Transmit(&huart1, (uint8_t*)"Erase OK\n", 9, 100);
		#endif
	}

	HAL_FLASH_Lock();
}

//////////////////////// РџРћР�РЎРљ РЎР’РћР‘РћР”РќР«РҐ РЇР§Р•Р•Рљ /////////////////////////////
uint32_t flash_search_adress(uint32_t address, uint16_t cnt)
{
	uint16_t count_byte = cnt;

	while(count_byte)
	{
		if(0xFF == *(uint8_t*)address++) count_byte--;
		else count_byte = cnt;

		if(address == ENDMEMORY - 1) // РµСЃР»Рё РґРѕСЃС‚РёРіРЅСѓС‚ РєРѕРЅРµС† С„Р»РµС€Р°
		{
			erase_flash();        // С‚РѕРіРґР° РѕС‡РёС‰Р°РµРј РїР°РјСЏС‚СЊ
			#if DEBUG
			//HAL_UART_Transmit(&huart1, (uint8_t*)"New cicle\n", 10, 100);
			#endif
			return STARTADDR;     // СѓСЃС‚Р°РЅР°РІР»РёРІР°РµРј Р°РґСЂРµСЃ РґР»СЏ Р·Р°РїРёСЃРё СЃ СЃР°РјРѕРіРѕ РЅР°С‡Р°Р»Р°
		}
	}

	return address -= cnt;
}

//////////////////////// Р—РђРџР�РЎР¬ Р”РђРќРќР«РҐ /////////////////////////////
void write_to_flash(myBuf_t *buff)
{
	res_addr = flash_search_adress(res_addr, BUFFSIZE * DATAWIDTH); // РёС‰РµРј СЃРІРѕР±РѕРґРЅС‹Рµ СЏС‡РµР№РєРё РЅР°С‡РёРЅР°СЏ СЃ РїРѕСЃР»РµРґРЅРµРіРѕ РёР·РІРµСЃС‚РЅРѕРіРѕ Р°РґСЂРµСЃР°

	//////////////////////// Р—РђРџР�РЎР¬ ////////////////////////////
	HAL_FLASH_Unlock(); // СЂР°Р·Р±Р»РѕРєРёСЂРѕРІР°С‚СЊ С„Р»РµС€

	for(uint16_t i = 0; i < BUFFSIZE; i++)
	{
		if(HAL_FLASH_Program(WIDTHWRITE, res_addr, buff[i]) != HAL_OK)
		{
			uint32_t er = HAL_FLASH_GetError();
			char str[64] = {0,};
			snprintf(str, 64, "ER %lu\n", er);
			//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
		}

		res_addr = res_addr + DATAWIDTH;
	}

	HAL_FLASH_Lock(); // Р·Р°Р±Р»РѕРєРёСЂРѕРІР°С‚СЊ С„Р»РµС€

	//////////////////////// РїСЂРѕРІРµСЂРєР° Р·Р°РїРёСЃР°РЅРЅРѕРіРѕ (СЌС‚Рѕ РјРѕР¶РЅРѕ СѓРґРїР»РёС‚СЊ РµСЃР»Рё РЅРµРѕС…РѕС‚Р° РїСЂРѕРІРµСЂСЏС‚СЊ) ////////////////////////
	uint32_t crcbuff[BUFFSIZE] = {0,};

	for(uint16_t i = 0; i < BUFFSIZE; i++) crcbuff[i] = (uint32_t)buff[i]; // РІ С„СѓРЅРєС†РёСЋ CRC32 РЅСѓР¶РЅРѕ РїРѕРґР°РІР°С‚СЊ 32-С… Р±РёС‚РЅС‹Рµ Р·РЅР°С‡РµРЅРёСЏ, РїРѕСЌС‚РѕРјСѓ РїРµСЂРµРіРѕРЅСЏРµРј 16-С‚Рё Р±РёС‚РЅС‹Р№ Р±СѓС„РµСЂ РІ 32-С… Р±РёС‚РЅС‹Р№

	uint32_t sum1 = HAL_CRC_Calculate(&hcrc, (uint32_t*)crcbuff, BUFFSIZE); // crc Р±СѓС„РµСЂР° РєРѕС‚РѕСЂС‹Р№ С‚РѕР»СЊРєРѕ С‡С‚Рѕ Р·Р°РїРёСЃР°Р»Рё

	buff[0] = 0;
	read_last_data_in_flash(buff); // С‡РёС‚Р°РµРј С‡С‚Рѕ Р·Р°РїРёСЃР°Р»Рё

	for(uint16_t i = 0; i < BUFFSIZE; i++) crcbuff[i] = (uint32_t)buff[i];

	uint32_t sum2 = HAL_CRC_Calculate(&hcrc, (uint32_t*)crcbuff, BUFFSIZE); // crc РїСЂРѕС‡РёС‚Р°РЅРЅРѕРіРѕ

	#if DEBUG
	char str[64] = {0,};
	snprintf(str, 64, "SUM %lu %lu\n", sum1, sum2);
	//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
	#endif

	if(sum1 != sum2) // РµСЃР»Рё СЃСѓРјРјС‹ Р·Р°РїРёСЃР°РЅРЅРѕРіРѕ Рё РїСЂРѕС‡РёС‚Р°РЅРЅРѕРіРѕ РЅРµ СЂР°РІРЅС‹, С‚РѕРіРґР° С‡С‚Рѕ-С‚Рѕ РїРѕС€Р»Рѕ РЅРµ С‚Р°Рє
	{
		#if DEBUG
		//HAL_UART_Transmit(&huart1, (uint8_t*)"Write buff ERROR\n", 17, 100);
		#endif
		return;
	}
	//////////////////////// РєРѕРЅРµС† РїСЂРѕРІРµСЂРєРё Р·Р°РїРёСЃР°РЅРЅРѕРіРѕ ////////////////////////

	#if DEBUG
	//HAL_UART_Transmit(&huart1, (uint8_t*)"Write buff OK\n", 14, 100);
	#endif
}

//////////////////////// Р§РўР•РќР�Р• РџРћРЎР›Р•Р”РќР�РҐ Р”РђРќРќР«РҐ /////////////////////////////
void read_last_data_in_flash(myBuf_t *buff)
{
	if(res_addr == STARTADDR)
	{
		#if DEBUG
		//HAL_UART_Transmit(&huart1, (uint8_t*)"Flash empty\n", 12, 100);
		#endif
		return;
	}

	uint32_t adr = res_addr - BUFFSIZE * DATAWIDTH; // СЃРґРІРёРіР°РµРјСЃСЏ РЅР° РЅР°С‡Р°Р»Рѕ РїРѕСЃР»РµРґРЅРёС… РґР°РЅРЅС‹С…

	for(uint16_t i = 0; i < BUFFSIZE; i++)
	{
		buff[i] = *(myBuf_t*)adr; // С‡РёС‚Р°РµРј
		adr = adr + DATAWIDTH;
	}
}
