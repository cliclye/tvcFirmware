#pragma once

#include <stdint.h>
#include <stdbool.h>

/* STM32F405RG register base addresses */
#define STM32_PERIPH_BASE           0x40000000UL
#define STM32_APB1PERIPH_BASE       (STM32_PERIPH_BASE + 0x00000UL)
#define STM32_APB2PERIPH_BASE       (STM32_PERIPH_BASE + 0x10000UL)
#define STM32_AHB1PERIPH_BASE       (STM32_PERIPH_BASE + 0x20000UL)
#define STM32_AHB2PERIPH_BASE       (STM32_PERIPH_BASE + 0x50000000UL)

#define STM32_RCC_BASE              (STM32_AHB1PERIPH_BASE + 0x3800UL)
#define STM32_GPIOA_BASE            (STM32_AHB1PERIPH_BASE + 0x0000UL)
#define STM32_GPIOB_BASE            (STM32_AHB1PERIPH_BASE + 0x0400UL)
#define STM32_GPIOC_BASE            (STM32_AHB1PERIPH_BASE + 0x0800UL)
#define STM32_SYSCFG_BASE           (STM32_APB2PERIPH_BASE + 0x0000UL)
#define STM32_TIM3_BASE             (STM32_APB1PERIPH_BASE + 0x0400UL)
#define STM32_TIM4_BASE             (STM32_APB1PERIPH_BASE + 0x0800UL)
#define STM32_SPI1_BASE             (STM32_APB2PERIPH_BASE + 0x3000UL)
#define STM32_SPI2_BASE             (STM32_APB1PERIPH_BASE + 0x3800UL)
#define STM32_SPI3_BASE             (STM32_APB1PERIPH_BASE + 0x0C00UL)
#define STM32_I2C1_BASE             (STM32_APB1PERIPH_BASE + 0x5400UL)
#define STM32_I2C2_BASE             (STM32_APB1PERIPH_BASE + 0x5800UL)
#define STM32_USART2_BASE           (STM32_APB1PERIPH_BASE + 0x4400UL)
#define STM32_UART4_BASE            (STM32_APB1PERIPH_BASE + 0x4C00UL)
#define STM32_ADC1_BASE             (STM32_APB2PERIPH_BASE + 0x4100UL)
#define STM32_OTG_FS_BASE           (STM32_AHB2PERIPH_BASE + 0x00000UL)

/* RCC registers */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} STM32_RCC_TypeDef;

/* GPIO registers */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} STM32_GPIO_TypeDef;

/* SPI registers */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} STM32_SPI_TypeDef;

/* I2C registers */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} STM32_I2C_TypeDef;

/* USART registers */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} STM32_USART_TypeDef;

/* Timer registers */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR[4];
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t DMAR2;
} STM32_TIM_TypeDef;

/* ADC registers */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} STM32_ADC_TypeDef;

/* OTG FS registers */
typedef struct {
    volatile uint32_t GOTGCTL;
    volatile uint32_t GOTGINT;
    volatile uint32_t GAHBCFG;
    volatile uint32_t GUSBCFG;
    volatile uint32_t GRSTCTL;
    volatile uint32_t GINTSTS;
    volatile uint32_t GINTMSK;
    volatile uint32_t GRXSTSR;
    volatile uint32_t GRXSTSP;
    volatile uint32_t GRXFSIZ;
    volatile uint32_t DIEPTXF0;
    volatile uint32_t DIEPTXF[15];
    uint32_t RESERVED0[2];
    volatile uint32_t GCCFG;
    volatile uint32_t CID;
    uint32_t RESERVED1[48];
    volatile uint32_t HPTXFSIZ;
    volatile uint32_t DIEPTXF1_HNPTXFSIZ;
    volatile uint32_t GNPTXSTS;
    uint32_t RESERVED2[2];
    volatile uint32_t DEVID;
    volatile uint32_t SNID;
    volatile uint32_t GSNPCFG;
    uint32_t RESERVED3[50];
    volatile uint32_t HCFG;
    volatile uint32_t HFIR;
    volatile uint32_t HFNUM;
    volatile uint32_t HPTXSTS;
    volatile uint32_t HAINT;
    volatile uint32_t HAINTMSK;
    uint32_t RESERVED4[9];
    volatile uint32_t HPRT;
    uint32_t RESERVED5[47];
    volatile uint32_t HCCHAR[16];
    volatile uint32_t HCSPLT[16];
    volatile uint32_t HCINT[16];
    volatile uint32_t HCINTMSK[16];
    volatile uint32_t HCTSIZ[16];
    volatile uint32_t HCDMA[16];
    uint32_t RESERVED6[64];
    volatile uint32_t DCFG;
    volatile uint32_t DCTL;
    volatile uint32_t DSTS;
    uint32_t RESERVED7;
    volatile uint32_t DIEPMSK;
    volatile uint32_t DOEPMSK;
    volatile uint32_t DAINT;
    volatile uint32_t DAINTMSK;
    volatile uint32_t DVBUSDIS;
    volatile uint32_t DVBUSPULSE;
    volatile uint32_t DIEPEMPMSK;
    uint32_t RESERVED8[12];
    volatile uint32_t DIEPCTL[16];
    uint32_t RESERVED9[16];
    volatile uint32_t DIEPINT[16];
    uint32_t RESERVED10[16];
    volatile uint32_t DIEPTSIZ[16];
    volatile uint32_t DIEPDMA[16];
    volatile uint32_t DTXFSTS[16];
    uint32_t RESERVED11[64];
    volatile uint32_t DOEPCTL[16];
    uint32_t RESERVED12[16];
    volatile uint32_t DOEPINT[16];
    uint32_t RESERVED13[16];
    volatile uint32_t DOEPTSIZ[16];
    volatile uint32_t DOEPDMA[16];
    uint32_t RESERVED14[64];
    volatile uint32_t PCGCTL;
} STM32_OTG_FS_TypeDef;

/* Register access macros */
#define STM32_RCC                ((STM32_RCC_TypeDef *) STM32_RCC_BASE)
#define STM32_GPIOA              ((STM32_GPIO_TypeDef *) STM32_GPIOA_BASE)
#define STM32_GPIOB              ((STM32_GPIO_TypeDef *) STM32_GPIOB_BASE)
#define STM32_GPIOC              ((STM32_GPIO_TypeDef *) STM32_GPIOC_BASE)
#define STM32_TIM3               ((STM32_TIM_TypeDef *) STM32_TIM3_BASE)
#define STM32_TIM4               ((STM32_TIM_TypeDef *) STM32_TIM4_BASE)
#define STM32_SPI1               ((STM32_SPI_TypeDef *) STM32_SPI1_BASE)
#define STM32_SPI2               ((STM32_SPI_TypeDef *) STM32_SPI2_BASE)
#define STM32_SPI3               ((STM32_SPI_TypeDef *) STM32_SPI3_BASE)
#define STM32_I2C1               ((STM32_I2C_TypeDef *) STM32_I2C1_BASE)
#define STM32_I2C2               ((STM32_I2C_TypeDef *) STM32_I2C2_BASE)
#define STM32_USART2             ((STM32_USART_TypeDef *) STM32_USART2_BASE)
#define STM32_UART4              ((STM32_USART_TypeDef *) STM32_UART4_BASE)
#define STM32_ADC1               ((STM32_ADC_TypeDef *) STM32_ADC1_BASE)
#define STM32_OTG_FS             ((STM32_OTG_FS_TypeDef *) STM32_OTG_FS_BASE)

/* RCC bit definitions */
#define STM32_RCC_CR_HSEON       (1U << 16)
#define STM32_RCC_CR_HSERDY      (1U << 17)
#define STM32_RCC_CR_HSION       (1U << 0)
#define STM32_RCC_CR_HSIRDY      (1U << 1)
#define STM32_RCC_CR_PLLON       (1U << 24)
#define STM32_RCC_CR_PLLRDY      (1U << 25)

#define STM32_RCC_PLLCFGR_PLLSRC_HSE   (1U << 22)
#define STM32_RCC_PLLCFGR_PLLSRC_HSI   (0U << 22)
#define STM32_RCC_PLLCFGR_PLLM_SHIFT   0
#define STM32_RCC_PLLCFGR_PLLN_SHIFT   6
#define STM32_RCC_PLLCFGR_PLLP_SHIFT   16
#define STM32_RCC_PLLCFGR_PLLQ_SHIFT   24

#define STM32_RCC_CFGR_SW_MASK    (3U << 0)
#define STM32_RCC_CFGR_SW_PLL      (2U << 0)
#define STM32_RCC_CFGR_SW_HSI      (1U << 0)
#define STM32_RCC_CFGR_SWS_MASK    (3U << 2)
#define STM32_RCC_CFGR_SWS_PLL     (2U << 2)
#define STM32_RCC_CFGR_SWS_HSI     (1U << 2)

#define STM32_RCC_AHB1ENR_GPIOAEN   (1U << 0)
#define STM32_RCC_AHB1ENR_GPIOBEN   (1U << 1)
#define STM32_RCC_AHB1ENR_GPIOCEN   (1U << 2)
#define STM32_RCC_AHB1ENR_DMA1EN    (1U << 21)
#define STM32_RCC_AHB1ENR_DMA2EN    (1U << 22)
#define STM32_RCC_AHB2ENR_OTGFSEN   (1U << 7)

#define STM32_RCC_APB1ENR_TIM3EN    (1U << 1)
#define STM32_RCC_APB1ENR_TIM4EN    (1U << 2)
#define STM32_RCC_APB1ENR_SPI2EN    (1U << 14)
#define STM32_RCC_APB1ENR_SPI3EN    (1U << 15)
#define STM32_RCC_APB1ENR_USART2EN  (1U << 17)
#define STM32_RCC_APB1ENR_UART4EN   (1U << 19)
#define STM32_RCC_APB1ENR_I2C1EN    (1U << 21)
#define STM32_RCC_APB1ENR_I2C2EN    (1U << 22)

#define STM32_RCC_APB2ENR_SPI1EN    (1U << 12)
#define STM32_RCC_APB2ENR_USART1EN  (1U << 4)
#define STM32_RCC_APB2ENR_SYSCFGEN  (1U << 14)
#define STM32_RCC_APB2ENR_ADC1EN    (1U << 8)

/* GPIO bit definitions */
#define STM32_GPIO_MODER_INPUT     0U
#define STM32_GPIO_MODER_OUTPUT    1U
#define STM32_GPIO_MODER_ALT_FUNC  2U
#define STM32_GPIO_MODER_ANALOG    3U

#define STM32_GPIO_OSPEEDR_LOW     0U
#define STM32_GPIO_OSPEEDR_MEDIUM  1U
#define STM32_GPIO_OSPEEDR_HIGH    2U
#define STM32_GPIO_OSPEEDR_V_HIGH  3U

#define STM32_GPIO_PUPDR_NONE      0U
#define STM32_GPIO_PUPDR_PULLUP    1U
#define STM32_GPIO_PUPDR_PULLDOWN  2U

/* SPI bit definitions */
#define STM32_SPI_CR1_MSTR         (1U << 2)
#define STM32_SPI_CR1_BR_DIV2      (0U << 3)
#define STM32_SPI_CR1_BR_DIV4      (1U << 3)
#define STM32_SPI_CR1_BR_DIV8      (2U << 3)
#define STM32_SPI_CR1_BR_DIV16     (3U << 3)
#define STM32_SPI_CR1_BR_DIV32     (4U << 3)
#define STM32_SPI_CR1_BR_DIV64     (5U << 3)
#define STM32_SPI_CR1_BR_DIV128    (6U << 3)
#define STM32_SPI_CR1_BR_DIV256    (7U << 3)
#define STM32_SPI_CR1_CPOL_HIGH    (1U << 1)
#define STM32_SPI_CR1_CPHA_2EDGE   (1U << 0)
#define STM32_SPI_CR1_SPE          (1U << 6)
#define STM32_SPI_CR1_SSM          (1U << 9)
#define STM32_SPI_CR1_SSI          (1U << 8)

#define STM32_SPI_SR_TXE           (1U << 1)
#define STM32_SPI_SR_RXNE          (1U << 0)
#define STM32_SPI_SR_BSY           (1U << 7)

/* I2C bit definitions */
#define STM32_I2C_CR1_PE           (1U << 0)
#define STM32_I2C_CR1_START        (1U << 8)
#define STM32_I2C_CR1_STOP         (1U << 9)
#define STM32_I2C_CR1_ACK         (1U << 10)

#define STM32_I2C_SR1_SB           (1U << 0)
#define STM32_I2C_SR1_ADDR        (1U << 1)
#define STM32_I2C_SR1_BTF          (1U << 2)
#define STM32_I2C_SR1_TXE          (1U << 7)
#define STM32_I2C_SR1_RXNE         (1U << 6)

#define STM32_I2C_SR2_MSL          (1U << 0)
#define STM32_I2C_SR2_BUSY         (1U << 1)
#define STM32_I2C_SR2_TRA          (1U << 2)

/* USART bit definitions */
#define STM32_USART_CR1_UE         (1U << 13)
#define STM32_USART_CR1_TE         (1U << 3)
#define STM32_USART_CR1_RE         (1U << 2)
#define STM32_USART_SR_TXE         (1U << 7)
#define STM32_USART_SR_RXNE        (1U << 5)

/* Timer bit definitions */
#define STM32_TIM_CR1_CEN          (1U << 0)
#define STM32_TIM_CR1_ARPE         (1U << 7)
#define STM32_TIM_CCMR1_OC1M_PWM1  (6U << 4)
#define STM32_TIM_CCMR1_OC2M_PWM1  (6U << 12)
#define STM32_TIM_CCMR2_OC3M_PWM1  (6U << 4)
#define STM32_TIM_CCMR2_OC4M_PWM1  (6U << 12)
#define STM32_TIM_CCER_CC1E        (1U << 0)
#define STM32_TIM_CCER_CC2E        (1U << 4)
#define STM32_TIM_CCER_CC3E        (1U << 8)
#define STM32_TIM_CCER_CC4E        (1U << 12)

/* ADC bit definitions */
#define STM32_ADC_CR2_ADON         (1U << 0)
#define STM32_ADC_CR2_SWSTART      (1U << 30)
#define STM32_ADC_SR_EOC           (1U << 1)

/* Function prototypes */
void STM32_SystemInit(void);
void STM32_DelayMicroseconds(uint32_t us);
void STM32_DelayMilliseconds(uint32_t ms);
