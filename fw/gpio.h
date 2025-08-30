#pragma once

#include <stdint.h>

#define GPIOA_BASE 0x50000000
#define GPIOB_BASE 0x50000400

#define GPIOx_MODER(x) (x + 0x00)
#define GPIOx_IDR(x) (x + 0x10)
#define GPIOx_BSRR(x) (x + 0x18)
#define GPIOx_AFRL(x) (x + 0x20)

typedef enum {
  GPIO_A0 = 0x00,
  GPIO_A1 = 0x01,
  GPIO_A2 = 0x02,
  GPIO_A3 = 0x03,
  GPIO_A4 = 0x04,
  GPIO_A5 = 0x05,
  GPIO_A6 = 0x06,
  GPIO_A7 = 0x07,
  GPIO_B0 = 0x10,
  GPIO_B1 = 0x11,
  GPIO_B2 = 0x12,
  GPIO_B3 = 0x13,
  GPIO_B4 = 0x14,
  GPIO_B5 = 0x15,
  GPIO_B6 = 0x16,
  GPIO_B7 = 0x17,
} gpio_pin_t;

typedef enum {
  GPIO_MODE_INPUT = 0,
  GPIO_MODE_OUTPUT = 1,
  GPIO_MODE_AF = 2,
  GPIO_MODE_ANALOG = 3,
} gpio_mode_t;

#define GPIO_BASE(pin) (((uint32_t[]){GPIOA_BASE, GPIOB_BASE})[pin >> 4])
#define GPIO_PIN(pin) (pin & 0xf)

static void gpio_set_mode(gpio_pin_t pin, gpio_mode_t mode) {
  /* GPIOx.MODER.MODER[pin*2+1+1:pin*2]: set Alternate function mode */
  set_bits(GPIOx_MODER(GPIO_BASE(pin)), GPIO_PIN(pin) * 2, 2, mode);
}

static void gpio_set(gpio_pin_t pin, uint8_t state) {
  if (state)
    wr_reg(GPIOx_BSRR(GPIO_BASE(pin)), BIT(GPIO_PIN(pin)));
  else
    wr_reg(GPIOx_BSRR(GPIO_BASE(pin)), BIT(16 + GPIO_PIN(pin)));
}

static uint8_t gpio_get(gpio_pin_t pin) {
  return (rd_reg(GPIOx_IDR(GPIO_BASE(pin))) >> GPIO_PIN(pin)) & 1;
}
