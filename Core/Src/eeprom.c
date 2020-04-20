/*
 * eeprom.c
 *
 *  Created on: Nov 14, 2019
 *      Author: Maciej
 */
#include "eeprom.h"

void EEPROM_write (uint32_t address, uint32_t value)
{
  while (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK);
  while (HAL_FLASHEx_DATAEEPROM_Erase(address) != HAL_OK);
  while (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value) != HAL_OK);
  while (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK);
}

uint32_t EEPROM_read (uint32_t address)
{
  return (*(uint32_t *)address);
}
