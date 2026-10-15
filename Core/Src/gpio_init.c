#include "gpio_init.h"
#include "stm32f4xx.h"
#include "board_pins.h"
#include "vehicle_config.h"

/* LED "off" level: high for an active-low (common-anode) RGB LED. */
#define LED_OFF_SET(pin) ((EASYTVC_LED_ACTIVE_LOW) ? (1U << (pin)) : ((1U << (pin)) << 16))

/* TIM1 sits on APB2 (84 MHz, /2 prescaler) so its kernel clock is 168 MHz. */
#define BUZZER_TIM          ((STM32_TIM_TypeDef *)(STM32_APB2PERIPH_BASE + 0x0000UL))
#define BUZZER_TIM_RCC_EN   (1U << 0)
#define BUZZER_TIM_HZ       (2UL * STM32_APB2_HZ)
#define BUZZER_PERIOD_US    370U   /* ~2.7 kHz, typical transducer resonance */

void EasyTVC_GPIO_Init(void)
{
    STM32_GPIO_TypeDef *gpio;
    uint32_t moder;
    uint32_t ospeedr;
    uint32_t pupdr;

    /* Configure USB pins (PA11, PA12) as alternate function */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (11 * 2));  /* Clear PA11 mode */
    moder &= ~(0x3U << (12 * 2));  /* Clear PA12 mode */
    moder |= (0x2U << (11 * 2));   /* PA11 AF */
    moder |= (0x2U << (12 * 2));   /* PA12 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (11 * 2));
    ospeedr &= ~(0x3U << (12 * 2));
    ospeedr |= (0x3U << (11 * 2));  /* High speed */
    ospeedr |= (0x3U << (12 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (11 * 2));
    pupdr &= ~(0x3U << (12 * 2));
    gpio->PUPDR = pupdr;           /* USB FS: no GPIO pull resistors */

    /* Set alternate function AF10 for USB */
    gpio->AFR[1] &= ~(0xFU << ((11 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((12 - 8) * 4));
    gpio->AFR[1] |= (0xAU << ((11 - 8) * 4));  /* AF10 */
    gpio->AFR[1] |= (0xAU << ((12 - 8) * 4));  /* AF10 */

    /* Configure SPI1 pins (PA5, PA6, PA7) as alternate function */
    moder = gpio->MODER;
    moder &= ~(0x3U << (5 * 2));
    moder &= ~(0x3U << (6 * 2));
    moder &= ~(0x3U << (7 * 2));
    moder |= (0x2U << (5 * 2));   /* PA5 AF */
    moder |= (0x2U << (6 * 2));   /* PA6 AF */
    moder |= (0x2U << (7 * 2));   /* PA7 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (5 * 2));
    ospeedr &= ~(0x3U << (6 * 2));
    ospeedr &= ~(0x3U << (7 * 2));
    ospeedr |= (0x3U << (5 * 2));  /* High speed */
    ospeedr |= (0x3U << (6 * 2));  /* High speed */
    ospeedr |= (0x3U << (7 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (5 * 2));
    pupdr &= ~(0x3U << (6 * 2));
    pupdr &= ~(0x3U << (7 * 2));
    pupdr |= (0x1U << (6 * 2));   /* Pullup on MISO */
    gpio->PUPDR = pupdr;

    /* Set alternate function AF5 for SPI1 */
    gpio->AFR[0] &= ~(0xFU << (5 * 4));
    gpio->AFR[0] &= ~(0xFU << (6 * 4));
    gpio->AFR[0] &= ~(0xFU << (7 * 4));
    gpio->AFR[0] |= (0x5U << (5 * 4));  /* AF5 */
    gpio->AFR[0] |= (0x5U << (6 * 4));  /* AF5 */
    gpio->AFR[0] |= (0x5U << (7 * 4));  /* AF5 */

    /* Configure SPI1 CS pins (PC7, PC8) as GPIO output */
    gpio = STM32_GPIOC;
    moder = gpio->MODER;
    moder &= ~(0x3U << (7 * 2));
    moder &= ~(0x3U << (8 * 2));
    moder |= (0x1U << (7 * 2));   /* PC7 output */
    moder |= (0x1U << (8 * 2));   /* PC8 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (7 * 2));
    ospeedr &= ~(0x3U << (8 * 2));
    ospeedr |= (0x3U << (7 * 2));  /* High speed */
    ospeedr |= (0x3U << (8 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (7 * 2));
    pupdr &= ~(0x3U << (8 * 2));
    pupdr |= (0x1U << (7 * 2));   /* Pullup */
    pupdr |= (0x1U << (8 * 2));   /* Pullup */
    gpio->PUPDR = pupdr;

    /* Set CS pins high (inactive) */
    gpio->BSRR = (1U << 7) | (1U << 8);

    /* Configure SPI2 pins (PB13, PB14, PB15) as alternate function */
    gpio = STM32_GPIOB;
    moder = gpio->MODER;
    moder &= ~(0x3U << (13 * 2));
    moder &= ~(0x3U << (14 * 2));
    moder &= ~(0x3U << (15 * 2));
    moder |= (0x2U << (13 * 2));  /* PB13 AF */
    moder |= (0x2U << (14 * 2));  /* PB14 AF */
    moder |= (0x2U << (15 * 2));  /* PB15 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (13 * 2));
    ospeedr &= ~(0x3U << (14 * 2));
    ospeedr &= ~(0x3U << (15 * 2));
    ospeedr |= (0x3U << (13 * 2));  /* High speed */
    ospeedr |= (0x3U << (14 * 2));  /* High speed */
    ospeedr |= (0x3U << (15 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (13 * 2));
    pupdr &= ~(0x3U << (14 * 2));
    pupdr &= ~(0x3U << (15 * 2));
    pupdr |= (0x1U << (14 * 2));   /* Pullup on MISO */
    gpio->PUPDR = pupdr;

    /* Set alternate function AF5 for SPI2 */
    gpio->AFR[1] &= ~(0xFU << ((13 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((14 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    gpio->AFR[1] |= (0x5U << ((13 - 8) * 4));  /* AF5 */
    gpio->AFR[1] |= (0x5U << ((14 - 8) * 4));  /* AF5 */
    gpio->AFR[1] |= (0x5U << ((15 - 8) * 4));  /* AF5 */

    /* Configure SPI2 CS pin (PC5) as GPIO output */
    gpio = STM32_GPIOC;
    moder = gpio->MODER;
    moder &= ~(0x3U << (5 * 2));
    moder |= (0x1U << (5 * 2));   /* PC5 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (5 * 2));
    ospeedr |= (0x3U << (5 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (5 * 2));
    pupdr |= (0x1U << (5 * 2));   /* Pullup */
    gpio->PUPDR = pupdr;

    /* Set CS pin high (inactive) */
    gpio->BSRR = (1U << 5);

    /* Configure SPI3 pins (PC10, PC11, PC12) as alternate function */
    gpio = STM32_GPIOC;
    moder = gpio->MODER;
    moder &= ~(0x3U << (10 * 2));
    moder &= ~(0x3U << (11 * 2));
    moder &= ~(0x3U << (12 * 2));
    moder |= (0x2U << (10 * 2));  /* PC10 AF */
    moder |= (0x2U << (11 * 2));  /* PC11 AF */
    moder |= (0x2U << (12 * 2));  /* PC12 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (10 * 2));
    ospeedr &= ~(0x3U << (11 * 2));
    ospeedr &= ~(0x3U << (12 * 2));
    ospeedr |= (0x3U << (10 * 2));  /* High speed */
    ospeedr |= (0x3U << (11 * 2));  /* High speed */
    ospeedr |= (0x3U << (12 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (10 * 2));
    pupdr &= ~(0x3U << (11 * 2));
    pupdr &= ~(0x3U << (12 * 2));
    pupdr |= (0x1U << (11 * 2));   /* Pullup on MISO */
    gpio->PUPDR = pupdr;

    /* Set alternate function AF6 for SPI3 */
    gpio->AFR[1] &= ~(0xFU << ((10 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((11 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((12 - 8) * 4));
    gpio->AFR[1] |= (0x6U << ((10 - 8) * 4));  /* AF6 */
    gpio->AFR[1] |= (0x6U << ((11 - 8) * 4));  /* AF6 */
    gpio->AFR[1] |= (0x6U << ((12 - 8) * 4));  /* AF6 */

    /* Configure SPI3 CS pin (PC6) as GPIO output */
    gpio = STM32_GPIOC;
    moder = gpio->MODER;
    moder &= ~(0x3U << (6 * 2));
    moder |= (0x1U << (6 * 2));   /* PC6 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (6 * 2));
    ospeedr |= (0x3U << (6 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (6 * 2));
    pupdr |= (0x1U << (6 * 2));   /* Pullup */
    gpio->PUPDR = pupdr;

    /* Set CS pin high (inactive) */
    gpio->BSRR = (1U << 6);

    /* Configure I2C1 pins (PB6, PB7) as alternate function with pullups */
    gpio = STM32_GPIOB;
    moder = gpio->MODER;
    moder &= ~(0x3U << (6 * 2));
    moder &= ~(0x3U << (7 * 2));
    moder |= (0x2U << (6 * 2));   /* PB6 AF */
    moder |= (0x2U << (7 * 2));   /* PB7 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (6 * 2));
    ospeedr &= ~(0x3U << (7 * 2));
    ospeedr |= (0x3U << (6 * 2));  /* High speed */
    ospeedr |= (0x3U << (7 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (6 * 2));
    pupdr &= ~(0x3U << (7 * 2));
    pupdr |= (0x1U << (6 * 2));   /* Pullup */
    pupdr |= (0x1U << (7 * 2));   /* Pullup */
    gpio->PUPDR = pupdr;

    /* I2C lines are open-drain. Push-pull fights the bus and the other devices. */
    gpio->OTYPER |= (1U << 6) | (1U << 7);

    /* Set alternate function AF4 for I2C1 */
    gpio->AFR[0] &= ~(0xFU << (6 * 4));
    gpio->AFR[0] &= ~(0xFU << (7 * 4));
    gpio->AFR[0] |= (0x4U << (6 * 4));  /* AF4 */
    gpio->AFR[0] |= (0x4U << (7 * 4));  /* AF4 */

    /* Configure I2C2 pins (PB10, PB11) as alternate function */
    gpio = STM32_GPIOB;
    moder = gpio->MODER;
    moder &= ~(0x3U << (10 * 2));
    moder &= ~(0x3U << (11 * 2));
    moder |= (0x2U << (10 * 2));  /* PB10 AF */
    moder |= (0x2U << (11 * 2));  /* PB11 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (10 * 2));
    ospeedr &= ~(0x3U << (11 * 2));
    ospeedr |= (0x3U << (10 * 2));  /* High speed */
    ospeedr |= (0x3U << (11 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (10 * 2));
    pupdr &= ~(0x3U << (11 * 2));
    gpio->PUPDR = pupdr;

    gpio->OTYPER |= (1U << 10) | (1U << 11);

    /* Set alternate function AF4 for I2C2 */
    gpio->AFR[1] &= ~(0xFU << ((10 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((11 - 8) * 4));
    gpio->AFR[1] |= (0x4U << ((10 - 8) * 4));  /* AF4 */
    gpio->AFR[1] |= (0x4U << ((11 - 8) * 4));  /* AF4 */

    /* Configure ADC pin (PA4) as analog input */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (4 * 2));
    moder |= (0x3U << (4 * 2));   /* PA4 analog */
    gpio->MODER = moder;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (4 * 2));
    gpio->PUPDR = pupdr;

    /* Configure LED pins (PB3, PB4, PB5) as GPIO output, start LOW */
    gpio = STM32_GPIOB;
    moder = gpio->MODER;
    moder &= ~(0x3U << (3 * 2));
    moder &= ~(0x3U << (4 * 2));
    moder &= ~(0x3U << (5 * 2));
    moder |= (0x1U << (3 * 2));   /* PB3 output */
    moder |= (0x1U << (4 * 2));   /* PB4 output */
    moder |= (0x1U << (5 * 2));   /* PB5 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (3 * 2));
    ospeedr &= ~(0x3U << (4 * 2));
    ospeedr &= ~(0x3U << (5 * 2));
    ospeedr |= (0x2U << (3 * 2));  /* Medium speed */
    ospeedr |= (0x2U << (4 * 2));  /* Medium speed */
    ospeedr |= (0x2U << (5 * 2));  /* Medium speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (3 * 2));
    pupdr &= ~(0x3U << (4 * 2));
    pupdr &= ~(0x3U << (5 * 2));
    pupdr |= (0x2U << (3 * 2));   /* Pulldown */
    pupdr |= (0x2U << (4 * 2));   /* Pulldown */
    pupdr |= (0x2U << (5 * 2));   /* Pulldown */
    gpio->PUPDR = pupdr;

    /* LEDs off */
    gpio->BSRR = LED_OFF_SET(3) | LED_OFF_SET(4) | LED_OFF_SET(5);

    /* Configure USART2 pins (PA2, PA3) as alternate function */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (2 * 2));
    moder &= ~(0x3U << (3 * 2));
    moder |= (0x2U << (2 * 2));   /* PA2 AF */
    moder |= (0x2U << (3 * 2));   /* PA3 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (2 * 2));
    ospeedr &= ~(0x3U << (3 * 2));
    ospeedr |= (0x3U << (2 * 2));  /* High speed */
    ospeedr |= (0x3U << (3 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (2 * 2));
    pupdr &= ~(0x3U << (3 * 2));
    pupdr |= (0x1U << (3 * 2));   /* Pullup on RX */
    gpio->PUPDR = pupdr;

    /* Set alternate function AF7 for USART2 */
    gpio->AFR[0] &= ~(0xFU << (2 * 4));
    gpio->AFR[0] &= ~(0xFU << (3 * 4));
    gpio->AFR[0] |= (0x7U << (2 * 4));  /* AF7 */
    gpio->AFR[0] |= (0x7U << (3 * 4));  /* AF7 */

    /* Configure UART4 pins (PA0, PA1) as alternate function */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (0 * 2));
    moder &= ~(0x3U << (1 * 2));
    moder |= (0x2U << (0 * 2));   /* PA0 AF */
    moder |= (0x2U << (1 * 2));   /* PA1 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (0 * 2));
    ospeedr &= ~(0x3U << (1 * 2));
    ospeedr |= (0x3U << (0 * 2));  /* High speed */
    ospeedr |= (0x3U << (1 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (0 * 2));
    pupdr &= ~(0x3U << (1 * 2));
    pupdr |= (0x1U << (1 * 2));   /* Pullup on RX */
    gpio->PUPDR = pupdr;

    /* Set alternate function AF8 for UART4 */
    gpio->AFR[0] &= ~(0xFU << (0 * 4));
    gpio->AFR[0] &= ~(0xFU << (1 * 4));
    gpio->AFR[0] |= (0x8U << (0 * 4));  /* AF8 */
    gpio->AFR[0] |= (0x8U << (1 * 4));  /* AF8 */

    /* Configure PWM pins (PB0, PB1, PB8, PB9) as alternate function */
    gpio = STM32_GPIOB;
    moder = gpio->MODER;
    moder &= ~(0x3U << (0 * 2));
    moder &= ~(0x3U << (1 * 2));
    moder &= ~(0x3U << (8 * 2));
    moder &= ~(0x3U << (9 * 2));
    moder |= (0x2U << (0 * 2));   /* PB0 AF */
    moder |= (0x2U << (1 * 2));   /* PB1 AF */
    moder |= (0x2U << (8 * 2));   /* PB8 AF */
    moder |= (0x2U << (9 * 2));   /* PB9 AF */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (0 * 2));
    ospeedr &= ~(0x3U << (1 * 2));
    ospeedr &= ~(0x3U << (8 * 2));
    ospeedr &= ~(0x3U << (9 * 2));
    ospeedr |= (0x3U << (0 * 2));  /* High speed */
    ospeedr |= (0x3U << (1 * 2));  /* High speed */
    ospeedr |= (0x3U << (8 * 2));  /* High speed */
    ospeedr |= (0x3U << (9 * 2));  /* High speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (0 * 2));
    pupdr &= ~(0x3U << (1 * 2));
    pupdr &= ~(0x3U << (8 * 2));
    pupdr &= ~(0x3U << (9 * 2));
    gpio->PUPDR = pupdr;

    /* Set alternate function AF2 for TIM3/TIM4 */
    gpio->AFR[0] &= ~(0xFU << (0 * 4));
    gpio->AFR[0] &= ~(0xFU << (1 * 4));
    gpio->AFR[0] |= (0x2U << (0 * 4));  /* AF2 */
    gpio->AFR[0] |= (0x2U << (1 * 4));  /* AF2 */
    gpio->AFR[1] &= ~(0xFU << ((8 - 8) * 4));
    gpio->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
    gpio->AFR[1] |= (0x2U << ((8 - 8) * 4));  /* AF2 */
    gpio->AFR[1] |= (0x2U << ((9 - 8) * 4));  /* AF2 */

    /* Buzzer (PA8). The board's transducer may be passive (no internal oscillator), so drive it
     * with a ~2.7 kHz tone from TIM1_CH1 (AF1) instead of a DC level; an active buzzer sounds
     * the same way. Output is held low (CCR1 = 0) until EasyTVC_GPIO_BuzzerSet(true). */
    gpio = STM32_GPIOA;
    gpio->BSRR = ((1U << 8) << 16);
    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (8 * 2));
    pupdr |= (0x2U << (8 * 2));   /* Pulldown while the timer is not driving */
    gpio->PUPDR = pupdr;
    gpio->AFR[1] &= ~(0xFU << ((8 - 8) * 4));
    gpio->AFR[1] |= (0x1U << ((8 - 8) * 4));  /* AF1 = TIM1_CH1 */
    STM32_RCC->APB2ENR |= BUZZER_TIM_RCC_EN;
    BUZZER_TIM->CR1 = 0;
    BUZZER_TIM->PSC = (BUZZER_TIM_HZ / 1000000UL) - 1UL;   /* 1 MHz */
    BUZZER_TIM->ARR = BUZZER_PERIOD_US - 1U;
    BUZZER_TIM->CCR[0] = 0;
    BUZZER_TIM->CCMR1 = (6U << 4) | (1U << 3);             /* OC1 PWM1, preload */
    BUZZER_TIM->CCER = 1U;                                  /* CC1E */
    BUZZER_TIM->BDTR = 1UL << 15;                           /* MOE: advanced timer output on */
    BUZZER_TIM->EGR = 1U;
    BUZZER_TIM->CR1 = (1U << 7) | 1U;                       /* ARPE | CEN */
    moder = gpio->MODER;
    moder &= ~(0x3U << (8 * 2));
    moder |= (0x2U << (8 * 2));   /* PA8 AF */
    gpio->MODER = moder;

    /* PA13 (SWDIO) and PA14 (SWCLK) are deliberately left alone: the hwdef lists PA14 as a
     * "dfu pin" and PA13 as pyro2, but reconfiguring either removes SWD access. */

    /* Pyro outputs PB2 (pyro1) and PC3 (pyro3): drive LOW so a floating gate cannot fire.
     * Nothing in this image ever drives them high (EASYTVC_PYRO_OUTPUTS_COMPILED_IN is 0).
     * PA13 (pyro2) is SWDIO, which resets to a pulled-up input: that net cannot be
     * guaranteed low from firmware, so do not connect an e-match to channel 2. */
    gpio = STM32_GPIOB;
    gpio->BSRR = (1U << 2) << 16;
    moder = gpio->MODER;
    moder &= ~(0x3U << (2 * 2));
    moder |= (0x1U << (2 * 2));
    gpio->MODER = moder;
    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (2 * 2));
    pupdr |= (0x2U << (2 * 2));
    gpio->PUPDR = pupdr;

    gpio = STM32_GPIOC;
    gpio->BSRR = (1U << 3) << 16;
    moder = gpio->MODER;
    moder &= ~(0x3U << (3 * 2));
    moder |= (0x1U << (3 * 2));
    gpio->MODER = moder;
    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (3 * 2));
    pupdr |= (0x2U << (3 * 2));
    gpio->PUPDR = pupdr;

    /* Continuity sense PB12, PC4, PC2 stay inputs (reset default). */
}

#if EASYTVC_LED_BRIGHTNESS_PCT < 10 || EASYTVC_LED_BRIGHTNESS_PCT > 100 || (EASYTVC_LED_BRIGHTNESS_PCT % 10) != 0
#error "EASYTVC_LED_BRIGHTNESS_PCT must be 10, 20, ... 100"
#endif
#define LED_PWM_PERIOD_MS 10U                                   /* 100 Hz: no visible flicker */
#define LED_ON_MS         (EASYTVC_LED_BRIGHTNESS_PCT / 10U)    /* on-time per period */

static volatile uint8_t led_requested;   /* bit n = LED n should be lit */
static volatile bool led_pwm_running;    /* set once the 1 kHz tick has taken over */

static void led_write(uint32_t pin, bool lit)
{
    const bool high = EASYTVC_LED_ACTIVE_LOW ? !lit : lit;
    STM32_GPIOB->BSRR = high ? (1U << pin) : ((1U << pin) << 16);
}

static const uint8_t led_pins[3] = {5, 4, 3};   /* blue, green, red */

void EasyTVC_GPIO_LedSet(uint8_t led, bool state)
{
    if (led > 2U) {
        return;
    }
    if (state) {
        led_requested |= (uint8_t)(1U << led);
    } else {
        led_requested &= (uint8_t)~(1U << led);
    }
    /* Once the 1 kHz tick runs it owns the pins (writing here too would add full-brightness
     * slivers every loop). Before that, or if the clock failed, drive the LED directly. */
    if (!led_pwm_running) {
        led_write(led_pins[led], state);
    }
}

void EasyTVC_GPIO_LedTick(uint32_t now_ms)
{
#if EASYTVC_LED_BRIGHTNESS_PCT < 100
    led_pwm_running = true;
    const bool phase_on = (now_ms % LED_PWM_PERIOD_MS) < LED_ON_MS;
    for (uint8_t led = 0; led < 3U; ++led) {
        led_write(led_pins[led], phase_on && (led_requested & (1U << led)));
    }
#else
    (void)now_ms;
#endif
}

void EasyTVC_GPIO_BuzzerSet(bool state)
{
    BUZZER_TIM->CCR[0] = state ? BUZZER_PERIOD_US / 2U : 0U;
}

void EasyTVC_GPIO_HaltClockFault(void)
{
    EasyTVC_GPIO_Init();
    for (;;) {
        EasyTVC_GPIO_LedSet(2, true);
        for (volatile uint32_t i = 0; i < 400000UL; ++i) {
        }
        EasyTVC_GPIO_LedSet(2, false);
        for (volatile uint32_t i = 0; i < 400000UL; ++i) {
        }
    }
}
