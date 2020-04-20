/*
 * button.c
 *
 *  Created on: Nov 14, 2019
 *      Author: Maciej
 */

#include "button.h"
#include "main.h"

btn_s * buttons;
uint8_t button_count;

/* na potrzeby debuggowania*/
extern UART_HandleTypeDef huart2;

void button_init(btn_s * btns, uint8_t count)
{
  buttons = btns;
  button_count = count;
}
GPIO_PinState button_get_state(uint8_t number)
{
  return buttons[number].state;
}
GPIO_PinState button_get_actual_state(uint8_t number)
{
  return HAL_GPIO_ReadPin(buttons[number].port, buttons[number].pin);
}
void button_assign_callback(uint8_t number, void (*callback)(void))
{
  buttons[number].callback = callback;
}

void button_update(uint32_t current_tick)
{
  for(int i = 0; i < button_count; i++)
    {
      if(buttons[i].callback && debounce(buttons+i, current_tick) == FallingEdge)
        {
          buttons[i].callback();
        }
    }
}

Button_Edge debounce(btn_s * btn, uint32_t current_tick)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(btn->port, btn->pin);
  Button_Edge ret = NoEdge;
  if(btn->state != pin_state)
    {
      if(current_tick - btn->last_stable >= BTN_DBC_DELAY)
        {
          btn->state = pin_state;
          ret = (pin_state == GPIO_PIN_SET)?RisingEdge:FallingEdge;
        }
    }
  else
    btn->last_stable = current_tick;
  return ret;
}

void button_default_callback()
{
  HAL_UART_Transmit(&huart2, (uint8_t*)"wcisk", 6, 1000);
}
