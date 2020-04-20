/*
 * menu.c
 *
 *  Created on: Nov 14, 2019
 *      Author: Maciej
 */

#include "menu.h"

/* MENU ITEMS CALLBACKS */
void menu_leadacid_cb(void);
void menu_leadgel_cb(void);
void menu_lipo_cb(void);
void menu_display_capacity(void);
void menu_capacity_cb(void);
void menu_autooff_cb(void);
void menu_alwayson_cb(void);
void menu_start_cb(void);
/* BUTTON CALLBACKS */
void capacity_btn0(void);
void capacity_btn1(void);
void capacity_btn2(void);
void capacity_btn3(void);
void start_btn0(void);
void start_btn1(void);
void start_btn2(void);
void start_btn3(void);

Lcd_HandleTypeDef * menu_lcd;
menu_s * current_item;
uint8_t lcd_row_pos;
uint8_t lcd_buffer[LCD_ROWS][LCD_COLS+1];
uint8_t buf_row, buf_col;
uint8_t charger_mode;
uint8_t is_running;
uint8_t numer;

extern batt_type battery_type;
extern uint16_t battery_capacity;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern uint16_t adc_buffer[5];
uint16_t battery_capacity_tmp;
uint8_t PWM_CMP;
uint16_t V_PV, I_PV, V_BATT, I_BATT;
uint32_t P_PV, P_BATT;

menu_s menu1, menu1_1, menu1_2, menu1_1_1, menu1_1_2, menu1_1_3, menu2, menu2_1, menu2_2, menu3, menu4;


/* menu items callbacks */

void menu_leadacid_cb(void)
{
  battery_type = LeadAcid;
  menu_back();
}
void menu_leadgel_cb(void)
{
  battery_type = LeadGel;
  menu_back();
}
void menu_lipo_cb(void)
{
  battery_type = LiPoly;
  menu_back();
}
void menu_display_capacity(void)
{
  static char buf[17];
  lcd_buf_clear();
  lcd_buf_cursor(0, 0);
  lcd_buf_string((uint8_t*)menu1_2.value);
  sprintf(buf,"%3d.%01d", battery_capacity_tmp/10, battery_capacity_tmp%10);
  lcd_buf_cursor(1, 0);
  lcd_buf_string((uint8_t*)buf);
  lcd_buf_send(menu_lcd);
}
void menu_capacity_cb(void)
{
  battery_capacity_tmp = battery_capacity;
  button_assign_callback(0, capacity_btn0);
  button_assign_callback(1, capacity_btn1);
  button_assign_callback(2, capacity_btn2);
  button_assign_callback(3, capacity_btn3);
  menu_display_capacity();
}
void menu_autooff_cb(void)
{
  // TODO
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  menu_back();
}
void menu_alwayson_cb(void)
{
  // TODO
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  menu_back();
}
void menu_start_cb(void)
{
  // zmiana trybu wyswietlania menu
  charger_mode = 1;
  is_running = 1;
  numer=0;
  // przetwornica wlaczona
  HAL_GPIO_WritePin(SHDN_12V_GPIO_Port, SHDN_12V_Pin, GPIO_PIN_SET);
  // pomiar pradu wlaczony
  HAL_GPIO_WritePin(SHDN_HSCA_GPIO_Port, SHDN_HSCA_Pin, GPIO_PIN_RESET);
  // poczatkowy wsp wypelnienia: 80%
  PWM_CMP = 160;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_CMP);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // przypisz odwolania
  button_assign_callback(0, start_btn0);
  button_assign_callback(1, start_btn1);
  button_assign_callback(2, start_btn2);
  button_assign_callback(3, start_btn3);

  menu_refresh();
}
void capacity_btn0(void)
{
  while(button_get_actual_state(0) == GPIO_PIN_RESET)
    {
      battery_capacity_tmp += (battery_capacity_tmp < 999) ? 1 : 0;
      menu_display_capacity();
      HAL_Delay(100);
    }
}
void capacity_btn1(void)
{
  while(button_get_actual_state(1) == GPIO_PIN_RESET)
    {
      battery_capacity_tmp -= (battery_capacity_tmp > 10) ? 1 : 0;
      menu_display_capacity();
      HAL_Delay(100);
    }
}
void capacity_btn2(void)
{
  battery_capacity = battery_capacity_tmp;
  capacity_btn3();
}
void capacity_btn3(void)
{
  button_assign_callback(0, menu_prev);
  button_assign_callback(1, menu_next);
  button_assign_callback(2, menu_enter);
  button_assign_callback(3, menu_back);
  menu_back();
}
void start_btn0(void)
{ // nastepny widok
  /* TODO poki co sluzy do zmieniania wsp wypelnienia
  while(button_get_actual_state(0) == GPIO_PIN_RESET)
      {
        PWM_CMP += (PWM_CMP < 199) ? 2 : 0;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_CMP);
        menu_refresh();
        HAL_Delay(100);
      }
  /* Poki co resetuje przetwornice */
  PWM_CMP = 160;
  if(is_running)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_CMP);
  menu_refresh();
  /**/
}
void start_btn1(void)
{ // poprzedni widok
  /* TODO poki co sluzy do zmieniania wsp wypelnienia
  while(button_get_actual_state(1) == GPIO_PIN_RESET)
      {
        PWM_CMP -= (PWM_CMP > 0) ? 2 : 0;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_CMP);
        menu_refresh();
        HAL_Delay(100);
      }
  /* Poki co zmienia wyswietlane parametry */
  numer=(numer+1)%4;
  menu_refresh();
  /**/
}
void start_btn2(void)
{ // przelacz obciazenie (toggle)
  // TODO ma przelaczac obciazenie :
  if(is_running)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      is_running = 0;
    }
  else
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_CMP);
      is_running = 1;
    }
}
void start_btn3(void)
{ // wylacz przetwornice i pomiar pradu
  charger_mode = 0;
  is_running = 0;
  // wylacz przetwornice
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  // wylacz zasilanie przetwornicy
  HAL_GPIO_WritePin(SHDN_12V_GPIO_Port, SHDN_12V_Pin, GPIO_PIN_RESET);
  // wylacz zasilanie HSCA
  HAL_GPIO_WritePin(SHDN_HSCA_GPIO_Port, SHDN_HSCA_Pin, GPIO_PIN_SET);
  // defaultowe kontrolki
  button_assign_callback(0, menu_prev);
  button_assign_callback(1, menu_next);
  button_assign_callback(2, menu_enter);
  button_assign_callback(3, menu_back);
  // odswiez menio
  menu_refresh();
}

void menu_init(Lcd_HandleTypeDef * lcd)
{
  menu_lcd = lcd;
/*
 * Hierarchy:
 * Battery
 *    Type
 *      Lead Acid 12V
 *      Lead Gel  12V
 *      Li Poly 12.6V
 *    Capacity
 * Behavior
 *    Auto Off
 *    Always on
 * Save config
 * Start
 */
  menu1.value = "Battery"; menu1.next = &menu2; menu1.child = &menu1_1;
  menu1_1.value = "Type"; menu1_1.next = &menu1_2;
  menu1_1.parent = &menu1; menu1_1.child = &menu1_1_1;
  menu1_1_1.value = "Lead Acid 12V"; menu1_1_1.next = &menu1_1_2;
  menu1_1_1.parent = &menu1_1; menu1_1_1.callback = menu_leadacid_cb;
  menu1_1_2.value = "Lead Gel  12V"; menu1_1_2.next = &menu1_1_3;
  menu1_1_2.prev = &menu1_1_1; menu1_1_2.parent = &menu1_1;
  menu1_1_2.callback = menu_leadgel_cb;
  menu1_1_3.value = "Li Poly 12.6V"; menu1_1_3.prev = &menu1_1_2;
  menu1_1_3.parent = &menu1_1; menu1_1_3.callback = menu_lipo_cb;
  menu1_2.value = "Capacity"; menu1_2.prev = &menu1_1; menu1_2.parent = &menu1;
  menu1_2.callback = menu_capacity_cb;
  menu2.value = "Behavior"; menu2.next = &menu3; menu2.prev = &menu1;
  menu2.child = &menu2_1;
  menu2_1.value = "Auto off"; menu2_1.next = &menu2_2;
  menu2_1.parent = &menu2; menu2_1.callback = menu_autooff_cb;
  menu2_2.value = "Always on"; menu2_2.prev = &menu2_1;
  menu2_2.parent = &menu2; menu2_2.callback = menu_alwayson_cb;
  menu3.value = "Save config"; menu3.next = &menu4; menu3.prev = &menu2;
  menu3.callback = save_config_eeprom;
  menu4.value = "Start"; menu4.prev = &menu3;
  menu4.callback = menu_start_cb;

  current_item = &menu1;
  menu_refresh();
}
void menu_enter(void)
{
  if (current_item->callback)
    current_item->callback();
  else if (current_item->child)
    {
      current_item = current_item->child;
      lcd_row_pos = 0;
      menu_refresh();
    }
}

void menu_back(void)
{
  if (current_item->parent)
    {
      current_item = current_item->parent;
      lcd_row_pos = 0;
      menu_refresh();
    }
}

void menu_next(void)
{
  if (current_item->next)
      {
        current_item = current_item->next;
        if(lcd_row_pos < LCD_ROWS - 1)
          lcd_row_pos++;
        menu_refresh();
      }
}

void menu_prev(void)
{
  if (current_item->prev)
      {
        current_item = current_item->prev;
        if(lcd_row_pos > 0)
          lcd_row_pos--;
        menu_refresh();
      }
}
void menu_refresh(void)
{
  menu_s *tmp;
  lcd_buf_clear();
  // TODO
  if(charger_mode == 1)
    {
      static char buf[64];
      lcd_buf_clear();
      lcd_buf_cursor(0, 0);
      //TODO add switching displays
      if(0)
        {
          I_PV = adc_buffer[0];
          V_PV = adc_buffer[3];
          P_PV = V_PV*I_PV;
          I_BATT = adc_buffer[1];
          V_BATT = adc_buffer[4];
          P_BATT = V_PV*I_BATT;
        }
      else
        {
          I_PV = ADC_current(adc_buffer[0]);
          V_PV = ADC_voltage(adc_buffer[3]);
          P_PV = V_PV*I_PV/100;
          I_BATT = ADC_current(adc_buffer[1]);
          V_BATT = ADC_voltage(adc_buffer[4]);
          P_BATT = V_BATT*I_BATT/100;
        }
      switch(numer)
      {
        case 0:
        case 1:
          /*PANEL PV PO KONWERSJI*/
          sprintf(buf,"PV:       %2lu.%01lu W", P_PV/10, P_PV%10);
          lcd_buf_string((uint8_t*)buf);
          lcd_buf_cursor(1, 0);
          sprintf(buf,"%2u.%01u V, %3u.%02u A", V_PV/10, V_PV%10, I_PV/100, I_PV%100);
          lcd_buf_string((uint8_t*)buf);
          break;
        /*case 1:
          /*PANEL PV RAW
          sprintf(buf,"%2u.%01u%%, %8lXW", PWM_CMP/2, (PWM_CMP%2)?5:0, P_PV);
          lcd_buf_string((uint8_t*)buf);
          lcd_buf_cursor(1, 0);
          sprintf(buf,"%4XV, %4XA", V_PV, I_PV);
          lcd_buf_string((uint8_t*)buf);
          break;*/
        case 2:
        case 3:
          /*AKUMULATOR PO KONWERSJI*/
          sprintf(buf,"Aku.:     %2lu.%01lu W", P_BATT/10, P_BATT%10);
          lcd_buf_string((uint8_t*)buf);
          lcd_buf_cursor(1, 0);
          sprintf(buf,"%2u.%01u V, %3u.%02u A", V_BATT/10, V_BATT%10, I_BATT/100, I_BATT%100);
          lcd_buf_string((uint8_t*)buf);
          break;
        /*case 3:
          /*AKUMULATOR RAW
          sprintf(buf,"%2u.%01u%%, %8lXW", PWM_CMP/2, (PWM_CMP%2)?5:0, P_BATT);
          lcd_buf_string((uint8_t*)buf);
          lcd_buf_cursor(1, 0);
          sprintf(buf,"%4XV, %4XA", V_BATT, I_BATT);
          lcd_buf_string((uint8_t*)buf);
          break;*/
        default:
          break;
      }
      lcd_buf_send(menu_lcd);
      //int len = sprintf(buf, "%10lu,%5u,%5u,%5u,%5u\r\n", HAL_GetTick(), V_PV, I_PV, V_BATT, I_BATT);
      //HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
    }
  else
    {
      tmp = current_item;
      for(int i = lcd_row_pos-1; i >= 0; i--)
        {
          // wyswietl poprzednie
          if(tmp->prev)
            {
              tmp = tmp->prev;
              lcd_buf_cursor(i, 2);
              lcd_buf_string((uint8_t*)tmp->value);
            }
          else
            break;
        }
      tmp = current_item;
      for(int i = lcd_row_pos+1; i < LCD_ROWS; i++)
        {
          // wyswietl nastepne
          if(tmp->next)
            {
              tmp = tmp->next;
              lcd_buf_cursor(i, 2);
              lcd_buf_string((uint8_t*)tmp->value);
            }
          else
            break;
        }
      // wyswietl current_item
      lcd_buf_cursor(lcd_row_pos, 0);
      lcd_buf_string((uint8_t*)"> ");
      lcd_buf_string((uint8_t*)current_item->value);
      lcd_buf_send(menu_lcd);
    }
}

uint8_t menu_level(void)
{
  uint8_t ret = 0;
  menu_s * tmp = current_item;
  while(tmp->parent)
    {
      tmp = tmp->parent;
      ret++;
    }
  return ret;
}
uint8_t menu_level_item_count(void)
{
  uint8_t ret = 1;
  menu_s * tmp = current_item;
  while(tmp->next)
    {
      tmp = tmp->next;
      ret++;
    }
  tmp = current_item;
  while(tmp->prev)
    {
      tmp = tmp->prev;
      ret++;
    }
  return ret;
}

void lcd_buf_clear()
{
  for(int i = 0; i < LCD_ROWS; i++)
    {
    for(int j = 0; j < LCD_COLS; j++)
      lcd_buffer[i][j] = ' ';
    lcd_buffer[i][LCD_COLS] = '\0';
    }
}
void lcd_buf_cursor(uint8_t row, uint8_t col)
{
  buf_row = row; buf_col = col;
}
void lcd_buf_string(uint8_t * str)
{
  while(*str != '\0' && buf_col < LCD_COLS)
    {
      lcd_buffer[buf_row][buf_col] = (*str);
      str++;
      buf_col++;
    }
}
void lcd_buf_send(Lcd_HandleTypeDef * lcd)
{
  for(int i = 0; i < LCD_ROWS; i++)
    {
      Lcd_cursor(lcd, i, 0);
      Lcd_string(lcd, (char*)lcd_buffer[i]);
    }
}

void menu_save_config_callback(void)
{
  save_config_eeprom();
}
