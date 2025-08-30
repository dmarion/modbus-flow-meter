#pragma once

#include <stdint.h>

#define BIT(x) (1U << (x))

static inline uint32_t rd_reg(uintptr_t addr) {
  return *((volatile uint32_t *)addr);
}

static inline void wr_reg(uintptr_t addr, uint32_t value) {
  *((volatile uint32_t *)addr) = value;
}

static inline void set_bits(uintptr_t addr, uint8_t offset, uint8_t n_bits,
                            uint32_t value) {
  uint32_t mask = ((1 << n_bits) - 1) << offset;
  uint32_t reg = rd_reg(addr);
  reg = (reg & ~mask) | ((value << offset) & mask);
  wr_reg(addr, reg);
}

static inline uint32_t get_bits(uintptr_t addr, uint8_t offset,
                                uint8_t n_bits) {
  uint32_t mask = ((1 << n_bits) - 1);
  return (rd_reg(addr) >> n_bits) & mask;
}

static inline void wait_for_reg_bit_set(uintptr_t addr, uint8_t bit_offset) {
  while (!(rd_reg(addr) & (1 << bit_offset)))
    ;
}

static inline void wait_for_reg_bits_set(uintptr_t addr, uint8_t offset,
                                         uint8_t n_bits, uint32_t expected) {
  while (get_bits(addr, offset, n_bits) != expected)
    ;
}
