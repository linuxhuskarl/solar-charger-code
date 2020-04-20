/*
 * button.h
 *
 *  Created on: Nov 14, 2019
 *      Author: Maciej
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stm32l0xx_hal.h"
#include "main.h"

#define BTN_DBC_DELAY 35

typedef enum
{
  NoEdge,
  RisingEdge,
  FallingEdge
} Button_Edge;

typedef struct
{
  GPIO_TypeDef* port;
  uint16_t pin;
  uint32_t last_stable;
  GPIO_PinState state;
  void (*callback)();
} btn_s;

void button_init(btn_s * buttons, uint8_t count);
GPIO_PinState button_get_state(uint8_t number);
GPIO_PinState button_get_actual_state(uint8_t number);
void button_assign_callback(uint8_t number, void (*callback)(void));
void button_update(uint32_t current_tick);
Button_Edge debounce(btn_s * btn, uint32_t current_tick);

void button_default_callback();

#endif /* INC_BUTTON_H_ */
