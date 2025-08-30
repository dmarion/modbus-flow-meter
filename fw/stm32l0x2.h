#pragma once

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

#define SYSTICK_BASE 0xE000E010
#define SYST_CSR (SYSTICK_BASE + 0x00)
#define SYST_RVR (SYSTICK_BASE + 0x04)
#define SYST_CVR (SYSTICK_BASE + 0x08)

#define NVIC_BASE 0xE000E100
#define NVIC_ISER0 (NVIC_BASE + 0x00)
