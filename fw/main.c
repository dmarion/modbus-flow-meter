#include <stdint.h>
#include "main.h"

#define TIM2_BASE 0x40000000
#define TIMx_CR1(x) (x + 0x00)
#define TIMx_SMCR(x) (x + 0x08)
#define TIMx_CNT(x) (x + 0x24)

#define USART2_BASE 0x40004400
#define USARTx_CR1(x) (x + 0x00)
#define USARTx_BRR(x) (x + 0x0C)
#define USARTx_ISR(x) (x + 0x1C)
#define USARTx_ICR(x) (x + 0x20)
#define USARTx_TDR(x) (x + 0x28)

#define LPUART1_BASE 0x40004800
#define LPUARTx_CR1(x) ((x) + 0x00)
#define LPUARTx_CR2(x) ((x) + 0x04)
#define LPUARTx_CR3(x) ((x) + 0x08)
#define LPUARTx_BRR(x) ((x) + 0x0C)
#define LPUARTx_ISR(x) ((x) + 0x1C)
#define LPUARTx_ICR(x) ((x) + 0x20)
#define LPUARTx_RDR(x) ((x) + 0x24)
#define LPUARTx_TDR(x) ((x) + 0x28)

#define PWR_BASE 0x40007000
#define PWR_CR (PWR_BASE + 0x00)

#define DMA1_BASE 0x40020000
#define DMA1_ISR (DMA1_BASE + 0x00)
#define DMA1_IFCR (DMA1_BASE + 0x04)
#define DMA1_CSELR (DMA1_BASE + 0xA8)
#define DMA1_CCR(ch) (DMA1_BASE + 0x08 + 0x14 * ((ch) - 1))
#define DMA1_CNDTR(ch) (DMA1_BASE + 0x0C + 0x14 * ((ch) - 1))
#define DMA1_CPAR(ch) (DMA1_BASE + 0x10 + 0x14 * ((ch) - 1))
#define DMA1_CMAR(ch) (DMA1_BASE + 0x14 + 0x14 * ((ch) - 1))

#define RCC_BASE 0x40021000
#define RCC_CR (RCC_BASE + 0x00)
#define RCC_CFGR (RCC_BASE + 0x0c)
#define RCC_IOPENR (RCC_BASE + 0x2c)
#define RCC_AHBENR (RCC_BASE + 0x30)
#define RCC_APB2ENR (RCC_BASE + 0x34)
#define RCC_APB1ENR (RCC_BASE + 0x38)
#define RCC_CCIPR (RCC_BASE + 0x4c)
#define BIT(x) (1U << (x))

#define GPIOA_BASE 0x50000000
#define GPIOB_BASE 0x50000400
#define GPIOx_MODER(x) (x + 0x00)
#define GPIOx_BSRR(x) (x + 0x18)
#define GPIOx_AFRL(x) (x + 0x20)

#define SYSTICK_BASE 0xE000E010
#define SYST_CSR (SYSTICK_BASE + 0x00)
#define SYST_RVR (SYSTICK_BASE + 0x04)
#define SYST_CVR (SYSTICK_BASE + 0x08)

#define NVIC_BASE 0xE000E100
#define NVIC_ISER0 (NVIC_BASE + 0x00)

#define LPUART1_RX_DMA_CH 3
#define LPUART1_TX_DMA_CH 2

#define MODBUS_SLAVE_ADDR 2

static volatile uint32_t system_ticks = 0;
static volatile uint8_t lpuart1_idle_flag = 0;
static volatile uint8_t lpuart1_tc_flag = 0;
static volatile uint8_t lpuart1_rx_error_flag = 0;
static uint32_t next_second = 1000;
static uint32_t deciliters = 0;
static uint16_t pulses_converted_to_deciliters = 0;
static uint16_t flow_pps = 0;
static uint16_t rx_errors = 0;

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

static uint32_t pps_to_dl_per_min(uint32_t pulses) {
  uint32_t dl = 0;

  for (; pulses >= 45; pulses -= 45, dl += 60)
    ;

  for (; pulses >= 9; pulses -= 9, dl += 12)
    ;

  for (; pulses >= 3; pulses -= 3, dl += 4)
    ;

  dl += pulses == 2 ? 3 : pulses;

  return dl;
}

void usart2_write(char c) {
  /* USART2_ISR.TXE[7]: Transmit data register empty */
  wait_for_reg_bit_set(USARTx_ISR(USART2_BASE), 7);
  wr_reg(USARTx_TDR(USART2_BASE), c);
}

static uint32_t get_deciliters() {
  uint16_t cnt = rd_reg(TIMx_CNT(TIM2_BASE));
  while ((uint16_t)(cnt - pulses_converted_to_deciliters) >= 45) {
    pulses_converted_to_deciliters += 45;
    deciliters += 1;
  }
  return deciliters;
}

static uint16_t modbus_crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];

    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static void rs485_tx(uint8_t *buffer, uint8_t len) {

  if (!len)
    return;

  /* DMA1_CCRx.EN[0]: Enable DMA channel) */
  set_bits(DMA1_CCR(LPUART1_TX_DMA_CH), 0, 1, 0);

  /* DMA1_CMARx = rx_buffer (Set memory address) */
  wr_reg(DMA1_CMAR(LPUART1_TX_DMA_CH), (uint32_t)buffer);

  /* DMA1_CNDTRx = BUFFER_SIZE (Set number of bytes to receive) */
  wr_reg(DMA1_CNDTR(LPUART1_TX_DMA_CH), len);

  /* Set PA1 = High (DE = 1, transmit mode) */
  wr_reg(GPIOx_BSRR(GPIOA_BASE), BIT(1));

  /* DMA1_CCRx.EN[0]: Enable DMA channel) */
  set_bits(DMA1_CCR(LPUART1_TX_DMA_CH), 0, 1, 1);
}

typedef struct {
  uint8_t addr;
  uint8_t func;
  uint16_t start_be;
  uint16_t count_be;
  uint16_t crc16;
} modbus_req_t;

static void modbus_handle_request(const uint8_t *req, uint8_t len) {
  modbus_req_t *mr = (modbus_req_t *)req;
  uint16_t start = __builtin_bswap16(mr->start_be);
  uint16_t count = __builtin_bswap16(mr->count_be);
  uint32_t dl;
  uint16_t v;

  static uint8_t modbus_tx_buf[16] = {
    [0] = MODBUS_SLAVE_ADDR,
    [1] = 0x03,
  };
  uint8_t *p = modbus_tx_buf + 2;

  if (len < 8) {
    printf("modbus nessage too short\r\n");
    return;
  }

  if (mr->addr != MODBUS_SLAVE_ADDR) {
    return;
  }

  if (mr->func != 0x03) {
    printf(" - unsupported modbus func %u\n", mr->func);
    return;
  }

  if (modbus_crc16(req, 6) != mr->crc16) {
    printf("bad crc\r\n");
    return;
  }

  if (start + count > 4) {
    printf("invalid registers requested, start %u count %u\r\n", start, count);
    return;
  }

  *p++ = count * 2;

  dl = get_deciliters();

  for (uint16_t i = 0; i < count; ++i) {
    switch (start + i) {
    case 0:
      v = dl >> 16;
      break;
    case 1:
      v = dl;
      break;
    case 2:
      v = pps_to_dl_per_min(flow_pps);
      break;
    case 3:
      v = rx_errors;
      break;
    default:
      v = 0;
      break;
    };
    *p++ = v >> 8;
    *p++ = v;
  }

  v = modbus_crc16(modbus_tx_buf, p - modbus_tx_buf);
  *p++ = v;
  *p++ = v >> 8;

  rs485_tx(modbus_tx_buf, p - modbus_tx_buf);
}

static void gpio_set_af(uintptr_t base, uint8_t pin, uint8_t af) {
  /* GPIOx.MODER.MODE6[pin*2+1+1:pin*2]: set Alternate function mode */
  set_bits(GPIOx_MODER(base), pin * 2, 2, 2);

  /* GPIOx.AFRL.AFSEL6[pin*4+3:pin*4]: set Alternate function */
  set_bits(GPIOx_AFRL(base), pin * 4, 4, af);
}

int main(void) {
  const int rx_buffer_sz = 256;
  uint8_t rx_buffer[rx_buffer_sz];
  uint16_t last_pulse_cnt = 0;

  /*
   * RCC
   */

  /* RCC_APB2ENR.SYSCFGEN[0]: System configuration controller clock enable */
  set_bits(RCC_APB2ENR, 0, 1, 1);

  /* RCC_APB1ENR.PWREN[28]: Power interface clock enable */
  /* RCC_APB1ENR.TIM2EN[0]: TIM2 clock enable */
  /* RCC_APB1ENR.USART2[17]: USART2 clock enable */
  /* RCC_APB1ENR.LPUART1EN[18] = 1 (Enable LPUART1 clock) */
  wr_reg(RCC_APB1ENR, BIT(0) | BIT(17) | BIT(18) | BIT(28));

  /* PWR_CR.VOS[12:11]: Voltage scaling range selection -> 1.8V) */
  set_bits(PWR_CR, 11, 2, 1);

  /* RCC_CR.HSION[0]: Enable the Internal High Speed oscillator */
  set_bits(RCC_CR, 0, 1, 1);

  /* RCC_CR.HSI16RDYF[2]: Wait till HSI is ready */
  wait_for_reg_bit_set(RCC_CR, 2);

  /* RCC_CFGR.SW[1:0]: = 1 - use HSI16 oscillator as system clock */
  set_bits(RCC_CFGR, 0, 2, 1);

  /* RCC_CFGR.SWS[3:2]: = 1 - wait for HSI16 oscillator to become system clk */
  wait_for_reg_bits_set(RCC_CFGR, 2, 2, 1);

  /* RCC_CCIPR.USART2SEL[3:2]: USART2 clock source selection, 2 = HSI16 */
  /* RCC_CCIPR.LPUART1SEL[11:10] = 2 (HSI16 as clock source) */
  set_bits(RCC_CCIPR, 2, 2, 2);
  set_bits(RCC_CCIPR, 10, 2, 2);

  /* RCC_IOPENR.IOPAEN[0]: 1 = port A clock enabled */
  /* RCC_IOPENR.IOPBEN[1]: 1 = port B clock enabled */
  wr_reg(RCC_IOPENR, BIT(0) | BIT(1));

  /* RCC_AHBENR.DMA1EN[0] = 1 (Enable DMA1 clock) */
  set_bits(RCC_AHBENR, 0, 1, 1);

  /*
   * SysTick - 1KHz tick counter
   */

  /* SYST_RVR.RELOAD[23:0]: SysTick Reload Value Register */
  wr_reg(SYST_RVR, 16000 - 1);

  /* SYST_CVR.CURRENT[23:0]: SysTick Current Value Register */
  wr_reg(SYST_CVR, 0);

  /* SYST_CSR.CLSRC[2]:   1 = cpu clock as SysTick clock src  */
  /* SYST_CSR.TICKINT[1]: 1 = cnt down to 0 asserts SysTick */
  /* SYST_CSR.ENABLE[0]:  1 = Counter enabled */
  wr_reg(SYST_CSR, 0b00000111);

  /*
   *  GPIO
   */
  gpio_set_af(GPIOA_BASE, 0, 2); /* PA0 -> AF2 - TIM2_CH1 */
  gpio_set_af(GPIOA_BASE, 2, 6); /* PA2 -> AF6 - LPUART1_TX */
  gpio_set_af(GPIOA_BASE, 3, 6); /* PA3 -> AF6 - LPUART1_RX */
  gpio_set_af(GPIOB_BASE, 6, 0); /* PB6 -> AF0 - USART2_TX */
  gpio_set_af(GPIOB_BASE, 7, 0); /* PB7 -> AF0 - USART2_RX */

  /* GPIOA_MODER.PA1 = 0b01 → General Purpose Output */
  set_bits(GPIOx_MODER(GPIOA_BASE), 2, 2, 1);

  /*
   *  USART2 - 115200 8N1
   */
  set_bits(USARTx_BRR(USART2_BASE), 0, 16, 139);

  /* USARTx_CR1.TE[3]: USARTx Transmit enable */
  /* USARTx_CR1.RE[2]: USARTx Receive enable */
  /* USARTx_CR1.UE[0]: USARTx enable */
  wr_reg(USARTx_CR1(USART2_BASE), BIT(0) | BIT(2) | BIT(3));

  /*
   *  LPUART2 - 9600 8N1 RS485
   */
  /* Set baud rate: 16000000 / 9600 = 1667 */
  wr_reg(LPUARTx_BRR(LPUART1_BASE), 426666);

  /* LPUART1_CR3.EIE[0] = 1 (line error IRQs: ORE/FE/NE) */
  /* LPUART1_CR3.DMAR[6] = 1 (Enable DMA for RX) */
  /* LPUART1_CR3.DMAT[7] = 1 (Enable DMA for TX) */
  wr_reg(LPUARTx_CR3(LPUART1_BASE), BIT(0) | BIT(6) | BIT(7));

  /* LPUARTx_CR1.UE[0]: LPUARTx enable */
  /* LPUARTx_CR1.RE[2]: LPUARTx Receive enable */
  /* LPUARTx_CR1.TE[3]: LPUARTx Transmit enable */
  /* LPUART1_CR1.IDLEIE[4] = 1 (Enable IDLE line interrupt) */
  /* LPUART1_CR1.TCIE[6] = 1 (Enable TC interrupt) */
  wr_reg(LPUARTx_CR1(LPUART1_BASE), BIT(0) | BIT(2) | BIT(3) | BIT(4) | BIT(6));

  /*
   *  DMA1
   */

  /* DMA1_CSELR.C3S[11:8]: 5 - LPUART1_RX */
  /* DMA1_CSELR.C3S[31:28]: 5 - LPUART1_TX */
  set_bits(DMA1_CSELR, 8, 4, 5);
  set_bits(DMA1_CSELR, 4, 4, 5);

  /* DMA1_CPAR2 = &LPUART1_TDR (Set peripheral address) */
  /* DMA1_CPAR3 = &LPUART1_RDR (Set peripheral address) */
  wr_reg(DMA1_CPAR(LPUART1_TX_DMA_CH), LPUARTx_TDR(LPUART1_BASE));
  wr_reg(DMA1_CPAR(LPUART1_RX_DMA_CH), LPUARTx_RDR(LPUART1_BASE));

  /* DMA1_CMARx = rx_buffer (Set memory address) */
  wr_reg(DMA1_CMAR(LPUART1_RX_DMA_CH), (uint32_t)rx_buffer);

  /* DMA1_CNDTRx = BUFFER_SIZE (Set number of bytes to receive) */
  wr_reg(DMA1_CNDTR(LPUART1_RX_DMA_CH), rx_buffer_sz);

  /* DMA1_CCRx.MINC[7]: Enable memory increment mode) */
  /* DMA1_CCRx.DIR[4]:  0 = Read from peripheral, 1 = write to peripheral */
  /* DMA1_CCRx.TCIE[1]: Enable transfer complete interrupt */
  /* DMA1_CCRx.EN[0]: Enable DMA channel) */
  wr_reg(DMA1_CCR(LPUART1_TX_DMA_CH), BIT(4) | BIT(7));
  wr_reg(DMA1_CCR(LPUART1_RX_DMA_CH), BIT(0) | BIT(7));

  /*
   *  TIM2
   */
  /* TIM2_SMCR.TS[6:3] = 0b101 - Filtered Timer Input 1 (TI1FP1) */
  set_bits(TIMx_SMCR(TIM2_BASE), 4, 3, 0b101);

  /* TIM2_SMCR.SMS[2:0] = 0b111 - External clock mode 1 */
  set_bits(TIMx_SMCR(TIM2_BASE), 0, 3, 0b111);

  /* TIM2_CR1.CEN[0] = 1 - Counter Enable */
  set_bits(TIMx_CR1(TIM2_BASE), 0, 1, 1);

  /*
   *  NVIC
   */
  /* NVIC.ISER0[29] = 1 enable interrupt 29 (LPUART1) */
  set_bits(NVIC_ISER0, 29, 1, 1);

  printf("firmware built on %s %s\r\n", __DATE__, __TIME__);

  while (1) {
    __asm volatile("wfi");

    if (lpuart1_tc_flag) {
      /* Set PA1 = Low (DE = 0, receive mode) */
      wr_reg(GPIOx_BSRR(GPIOA_BASE), BIT(1 + 16));
      lpuart1_tc_flag = 0;
    }

    if (lpuart1_idle_flag || lpuart1_rx_error_flag) {
      set_bits(DMA1_CCR(LPUART1_RX_DMA_CH), 0, 1, 0); /* Disable DMA */
      __asm volatile("dsb");

      if (lpuart1_rx_error_flag == 0) {
        uint32_t len = rx_buffer_sz - rd_reg(DMA1_CNDTR(LPUART1_RX_DMA_CH));

        if (len)
          modbus_handle_request(rx_buffer, len);
      } else {
        rx_errors++;
      }

      /* Re-arm DMA */
      wr_reg(DMA1_CNDTR(LPUART1_RX_DMA_CH), rx_buffer_sz); /* Reset length */
      set_bits(DMA1_CCR(LPUART1_RX_DMA_CH), 0, 1, 1);      /* Enable DMA */
      lpuart1_idle_flag = lpuart1_rx_error_flag = 0;
    }

    if (system_ticks >= next_second) {
      uint16_t cnt = rd_reg(TIMx_CNT(TIM2_BASE));
      while ((uint16_t)(cnt - pulses_converted_to_deciliters) >= 45) {
        pulses_converted_to_deciliters += 45;
        deciliters += 1;
      }
      flow_pps = cnt - last_pulse_cnt;
      last_pulse_cnt = cnt;
      printf("ticks=%u ctr=%u pulses, cvt=%u, vol=%u dL, flow=%u dl/min\r\n",
             system_ticks,
             cnt,
             pulses_converted_to_deciliters,
             deciliters,
             pps_to_dl_per_min(flow_pps));
      next_second += 1000;
    }
  }
}

void reset_handler(void) {
  /* Copy .data section from flash to RAM */
  extern uint32_t _sidata;
  extern uint32_t _sbss, _ebss;
  extern uint32_t _sdata, _edata;
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  /* Zero out .bss section */
  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  main();

  while (1)
    ;
}

static void systick_handler(void) { system_ticks++; }

static void default_isr_handler(void) {
  while (1)
    ;
}

static void lpuart1_isr_handler(void) {
  uint32_t isr = rd_reg(LPUARTx_ISR(LPUART1_BASE));

  /* LPUARTx_ISR.PE[0] = 1 - Parity error */
  /* LPUARTx_ISR.FE[1] = 1 - Framing error or break character is detected */
  /* LPUARTx_ISR.NF[2] = 1 - START bit Noise detected */
  if (isr & (BIT(1) | BIT(2) | BIT(3))) {
    /* LPUARTx_ICR.PECF[0] = 1 - Clear Parity Error flag (PE) */
    /* LPUARTx_ICR.FECF[1] = 1 - Clear Framing Error flag (FE) */
    /* LPUARTx_ICR.NCF[2]  = 1 - Clear Noise detected flag (NF) */
    wr_reg(LPUARTx_ICR(LPUART1_BASE), BIT(1) | BIT(2) | BIT(3));
    (void)rd_reg(LPUARTx_RDR(LPUART1_BASE));
    lpuart1_rx_error_flag = 1;
  }

  /* LPUARTx_ISR.IDLE[4] = 1 - IDLE line detected */
  if (isr & BIT(4)) {
    (void)rd_reg(LPUARTx_RDR(LPUART1_BASE));
    /* LPUARTx_ICR.IDLECF[4]= 1 - Clear IDLE line detected flag (IDLE) */
    wr_reg(LPUARTx_ICR(LPUART1_BASE), BIT(4));
    lpuart1_idle_flag = 1;
  }

  /* LPUARTx_ISR.TC[6] = 1 - Transmission complete */
  if (isr & BIT(6)) {
    /* LPUARTx_ICR.TCCF[6]  = 1 - Clear Transmission Complete flag (TC) */
    wr_reg(LPUARTx_ICR(LPUART1_BASE), BIT(6));
    lpuart1_tc_flag = 1;
  }
}

extern uint32_t _estack;
__attribute__((section(".isr_vector"))) const void *vector_table[48] = {
  [0] = (void *)&_estack,    /* Initial stack pointer */
  [1] = reset_handler,       /* Reset */
  [2] = default_isr_handler, /* NMI */
  [3] = default_isr_handler, /* HardFault */
  /* [4] to [10] are reserved */
  [11] = default_isr_handler, /* SVC */
  /* [12] and [13] are reserved */
  [14] = default_isr_handler, /* PendSV */
  [15] = systick_handler,     /* SysTick */
  /* External interrupts (IRQ 0–31) */
  [16] = default_isr_handler, /* WWDG */
  [17] = default_isr_handler, /* PVD */
  [18] = default_isr_handler, /* RTC */
  [19] = default_isr_handler, /* FLASH */
  [20] = default_isr_handler, /* RCC */
  [21] = default_isr_handler, /* EXTI0_1 */
  [22] = default_isr_handler, /* EXTI2_3 */
  [23] = default_isr_handler, /* EXTI4_15 */
  /* [24] reserved (TSC not present) */
  [25] = default_isr_handler, /* DMA1_Channel1 */
  [26] = default_isr_handler, /* DMA1_Channel2_3 */
  [27] = default_isr_handler, /* DMA1_Channel4_5_6_7 */
  [28] = default_isr_handler, /* ADC1_COMP */
  [29] = default_isr_handler, /* LPTIM1 */
  /* [30] reserved */
  [31] = default_isr_handler, /* TIM2 */
  /* [32] to [35] reserved */
  [36] = default_isr_handler, /* TIM21 */
  /* [37] reserved */
  [38] = default_isr_handler, /* TIM22 */
  [39] = default_isr_handler, /* I2C1 */
  /* [40] reserved */
  [41] = default_isr_handler, /* SPI1 */
  /* [42] and [43] reserved */
  [44] = default_isr_handler, /* USART2 */
  [45] = lpuart1_isr_handler, /* LPUART1 */

  /* [46] and [47] reserved */
};
