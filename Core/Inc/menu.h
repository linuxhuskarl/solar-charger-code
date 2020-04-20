/*
 * menu.h
 *
 *  Created on: Nov 12, 2019
 *      Author: Maciej
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "stm32l0xx_hal.h"
#include "button.h"
#include "main.h"
#include "lcd.h"

#define LCD_ROWS 2
#define LCD_COLS 16

typedef struct menu_struct menu_s;

struct menu_struct{
  char * value;
  menu_s * next;
  menu_s * prev;
  menu_s * parent;
  menu_s * child;
  void (*callback)(void);
};

void menu_init(Lcd_HandleTypeDef * lcd);
void menu_enter(void);
void menu_back(void);
void menu_next(void);
void menu_prev(void);
void menu_refresh(void);
uint8_t menu_level(void);
uint8_t menu_level_item_count(void);

void lcd_buf_clear(void);
void lcd_buf_cursor(uint8_t row, uint8_t col);
void lcd_buf_string(uint8_t * str);
void lcd_buf_send(Lcd_HandleTypeDef * lcd);

void menu_save_config_callback(void);

#endif /* INC_MENU_H_ */
