/*
 * eeprom.h
 *
 *  Created on: Nov 14, 2019
 *      Author: Maciej
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32l0xx_hal.h"

#define EEPROM_BASE_ADDRESS 0x08080000
#define EEPROM_SIZE 0x1800
#define __EEPROM(ADDR) (EEPROM_BASE_ADDRESS+ADDR)

void EEPROM_write (uint32_t address, uint32_t value);
uint32_t EEPROM_read (uint32_t address);

#endif /* INC_EEPROM_H_ */
