#include "board_pins.h"
#include "stm32f4xx.h"
#include "gpio_init.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "pwm_driver.h"
#include "uart_driver.h"
#include "hardware_bus.h"
#include "bmi088.h"
#include "bme280.h"
#include <string.h>

/* Sensor instances */
static EasyTVCBmi088 bmi088;
static EasyTVCBme280 bme280;

/* Telemetry buffer */
static char telemetry_buffer[256];

/* Simple string functions */
static size_t my_strlen(const char *str) {
    size_t len = 0;
    while (*str++) len++;
    return len;
}

static void my_strcpy(char *dest, const char *src) {
    while (*src) *dest++ = *src++;
    *dest = '\0';
}

static void append_str(char *dest, const char *src) {
    while (*dest) dest++;
    while (*src) *dest++ = *src++;
    *dest = '\0';
}

static void uint32_to_str(uint32_t value, char *str) {
    char temp[16];
    int i = 0;
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    while (value > 0) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    for (int j = 0; j < i; j++) {
        str[j] = temp[i - 1 - j];
    }
    str[i] = '\0';
}

static void int32_to_str(int32_t value, char *str) {
    if (value < 0) {
        str[0] = '-';
        uint32_to_str((uint32_t)(-value), str + 1);
    } else {
        uint32_to_str((uint32_t)value, str);
    }
}

static void int32_to_float_str(int32_t value, char *str) {
    int32_t whole = value / 100;
    int32_t frac = value % 100;
    if (frac < 0) frac = -frac;
    int32_to_str(whole, str);
    int len = my_strlen(str);
    str[len++] = '.';
    str[len++] = '0' + (frac / 10);
    str[len++] = '0' + (frac % 10);
    str[len] = '\0';
}

int main(void)
{
    /* MINIMAL system init */
    STM32_SystemInit();
    
    /* Initialize GPIO */
    EasyTVC_GPIO_Init();
    
    /* Initialize peripherals */
    EasyTVC_SPI1_Init();
    EasyTVC_SPI2_Init();
    EasyTVC_SPI3_Init();
    EasyTVC_I2C1_Init();
    EasyTVC_I2C2_Init();
    EasyTVC_PWM_Init();
    EasyTVC_USART2_Init();
    EasyTVC_UART4_Init();
    
    /* Initialize hardware bus adapters */
    EasyTVC_HardwareBus_Init();
    
    /* Initialize sensors */
    EasyTVC_Bmi088Init(&bmi088, EasyTVC_GetBMI088_AccelBus(), EasyTVC_GetBMI088_GyroBus());
    EasyTVC_Bme280Init(&bme280, EasyTVC_GetBME280Bus());
    
    /* Probe sensors */
    bool bmi088_ok = EasyTVC_Bmi088Probe(&bmi088);
    bool bme280_ok = EasyTVC_Bme280Probe(&bme280);
    
    if (bmi088_ok) {
        bmi088_ok = EasyTVC_Bmi088Configure(&bmi088);
    }
    if (bme280_ok) {
        bme280_ok = EasyTVC_Bme280Configure(&bme280);
    }
    
    /* Set LED status */
    EasyTVC_GPIO_LedSet(0, bmi088_ok);
    EasyTVC_GPIO_LedSet(1, bme280_ok);
    EasyTVC_GPIO_LedSet(2, false);
    
    /* Main loop */
    uint32_t loop_count = 0;
    while (1) {
        loop_count++;
        
        /* Read sensors */
        if (bmi088_ok) {
            if (!EasyTVC_Bmi088Read(&bmi088)) {
                bmi088_ok = false;
                EasyTVC_GPIO_LedSet(0, false);
            }
        }
        
        if (bme280_ok) {
            if (!EasyTVC_Bme280Read(&bme280)) {
                bme280_ok = false;
                EasyTVC_GPIO_LedSet(1, false);
            }
        }
        
        /* Send telemetry every 100ms */
        if (loop_count % 10 == 0) {
            my_strcpy(telemetry_buffer, "EasyTVC: ");
            
            if (bmi088_ok) {
                append_str(telemetry_buffer, "Accel=");
                char accel_str[16];
                int32_to_float_str(bmi088.accel_x, accel_str);
                append_str(telemetry_buffer, accel_str);
                append_str(telemetry_buffer, ",");
                int32_to_float_str(bmi088.accel_y, accel_str);
                append_str(telemetry_buffer, accel_str);
                append_str(telemetry_buffer, ",");
                int32_to_float_str(bmi088.accel_z, accel_str);
                append_str(telemetry_buffer, accel_str);
                append_str(telemetry_buffer, " Gyro=");
                int32_to_float_str(bmi088.gyro_x, accel_str);
                append_str(telemetry_buffer, accel_str);
                append_str(telemetry_buffer, ",");
                int32_to_float_str(bmi088.gyro_y, accel_str);
                append_str(telemetry_buffer, accel_str);
                append_str(telemetry_buffer, ",");
                int32_to_float_str(bmi088.gyro_z, accel_str);
                append_str(telemetry_buffer, accel_str);
            } else {
                append_str(telemetry_buffer, "IMU=FAIL");
            }
            
            if (bme280_ok) {
                append_str(telemetry_buffer, " Temp=");
                char temp_str[16];
                int32_to_float_str(bme280.temperature, temp_str);
                append_str(telemetry_buffer, temp_str);
                append_str(telemetry_buffer, "C Press=");
                uint32_to_str(bme280.pressure / 100, temp_str);
                append_str(telemetry_buffer, temp_str);
                append_str(telemetry_buffer, "hPa");
            } else {
                append_str(telemetry_buffer, " Baro=FAIL");
            }
            
            append_str(telemetry_buffer, "\r\n");
            
            size_t len = my_strlen(telemetry_buffer);
            EasyTVC_USART2_Send((const uint8_t*)telemetry_buffer, len);
        }
        
        /* Toggle blue LED every second (heartbeat) */
        if (loop_count % 100 == 0) {
            static bool led_state = false;
            led_state = !led_state;
            EasyTVC_GPIO_LedSet(0, led_state);
        }
        
        /* Set servos to center */
        EasyTVC_PWM_SetChannel(1, 1500);
        EasyTVC_PWM_SetChannel(2, 1500);
        EasyTVC_PWM_SetChannel(3, 1500);
        EasyTVC_PWM_SetChannel(4, 1500);
        
        /* 10ms delay */
        volatile uint32_t counter;
        for (counter = 0; counter < 40000; counter++) {
            __asm volatile ("nop");
        }
    }
    
    return 0;
}
