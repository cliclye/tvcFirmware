#include "gpio_init.h"
#include "stm32f4xx.h"
#include "board_pins.h"

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
    pupdr |= (0x1U << (11 * 2));   /* Pullup */
    pupdr |= (0x1U << (12 * 2));   /* Pullup */
    gpio->PUPDR = pupdr;

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

    /* Set LEDs LOW (off) */
    gpio->BSRR = ((1U << 3) << 16) | ((1U << 4) << 16) | ((1U << 5) << 16);

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

    /* Configure buzzer pin (PA8) as GPIO output, start LOW */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (8 * 2));
    moder |= (0x1U << (8 * 2));   /* PA8 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (8 * 2));
    ospeedr |= (0x2U << (8 * 2));  /* Medium speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (8 * 2));
    pupdr |= (0x2U << (8 * 2));   /* Pulldown */
    gpio->PUPDR = pupdr;

    /* Set buzzer LOW (off) */
    gpio->BSRR = ((1U << 8) << 16);

    /* Configure DFU pin (PA14) as GPIO output, start LOW */
    gpio = STM32_GPIOA;
    moder = gpio->MODER;
    moder &= ~(0x3U << (14 * 2));
    moder |= (0x1U << (14 * 2));  /* PA14 output */
    gpio->MODER = moder;

    ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(0x3U << (14 * 2));
    ospeedr |= (0x2U << (14 * 2));  /* Medium speed */
    gpio->OSPEEDR = ospeedr;

    pupdr = gpio->PUPDR;
    pupdr &= ~(0x3U << (14 * 2));
    pupdr |= (0x2U << (14 * 2));   /* Pulldown */
    gpio->PUPDR = pupdr;

    /* Set DFU pin LOW (inactive) */
    gpio->BSRR = ((1U << 14) << 16);

    /* Pyro channels - leave as input (default) per safety requirements */
    /* PB2, PA13, PC3 outputs - not configured */
    /* PB12, PC4, PC2 inputs - not configured */
}

void EasyTVC_GPIO_LedSet(uint8_t led, bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOB;
    uint32_t pin;

    switch (led) {
        case 0: pin = 5; break;  /* Blue */
        case 1: pin = 4; break;  /* Green */
        case 2: pin = 3; break;  /* Red */
        default: return;
    }

    if (state) {
        gpio->BSRR = (1U << pin);
    } else {
        gpio->BSRR = (1U << pin) << 16;
    }
}

void EasyTVC_GPIO_BuzzerSet(bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOA;
    if (state) {
        gpio->BSRR = (1U << 8);
    } else {
        gpio->BSRR = (1U << 8) << 16;
    }
}
