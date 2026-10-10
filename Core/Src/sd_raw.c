#include "sd_raw.h"
#include "spi_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

#define SD_CMD0   0
#define SD_CMD8   8
#define SD_CMD16  16
#define SD_CMD17  17
#define SD_CMD24  24
#define SD_CMD55  55
#define SD_CMD58  58
#define SD_ACMD41 41

#define SD_R1_IDLE 0x01U
#define SD_TOKEN_DATA 0xFEU

#define SD_INIT_TIMEOUT_MS   1000U
#define SD_READY_TIMEOUT_MS  250U
#define SD_TOKEN_TIMEOUT_MS  250U
#define SD_WRITE_TIMEOUT_MS  500U

static uint32_t write_started_ms;
static bool write_pending;

static uint8_t xfer(uint8_t byte)
{
    return EasyTVC_SPI3_TransferByte(byte);
}

static void sd_select(void)
{
    EasyTVC_SPI3_CS_SD_Set(false);
}

/* Raise CS and give the card the extra 8 clocks it needs to release DO. */
static void sd_deselect(void)
{
    EasyTVC_SPI3_CS_SD_Set(true);
    (void)xfer(0xFFU);
}

static bool sd_wait_ready(uint32_t timeout_ms)
{
    const uint32_t start = STM32_Millis();
    while ((uint32_t)(STM32_Millis() - start) <= timeout_ms) {
        if (xfer(0xFFU) == 0xFFU) {
            return true;
        }
    }
    return false;
}

/* Sends a command with CS held low and returns R1 (0xFF on timeout). Caller must sd_deselect(). */
static uint8_t sd_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sd_select();
    if (cmd != SD_CMD0 && !sd_wait_ready(SD_READY_TIMEOUT_MS)) {
        return 0xFFU;
    }
    (void)xfer((uint8_t)(0x40U | cmd));
    (void)xfer((uint8_t)(arg >> 24));
    (void)xfer((uint8_t)(arg >> 16));
    (void)xfer((uint8_t)(arg >> 8));
    (void)xfer((uint8_t)arg);
    (void)xfer(crc);

    uint8_t response = 0xFFU;
    for (uint8_t i = 0; i < 10U; i++) {
        response = xfer(0xFFU);
        if ((response & 0x80U) == 0U) {
            break;
        }
    }
    return response;
}

static uint8_t sd_simple_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    const uint8_t r1 = sd_command(cmd, arg, crc);
    sd_deselect();
    return r1;
}

SD_Result SD_Raw_Init(SD_Card *sd)
{
    sd->initialized = 0U;
    sd->block_addressed = 0U;
    sd->capacity = 0U;
    sd->sector_size = 512U;
    write_pending = false;

    EasyTVC_SPI3_SetFast(false);   /* 100-400 kHz is mandatory until the card is initialised */

    /* >= 74 clocks with CS HIGH and MOSI high (the old code toggled CS here). */
    EasyTVC_SPI3_CS_SD_Set(true);
    STM32_DelayMilliseconds(10U);
    for (uint8_t i = 0; i < 10U; i++) {
        (void)xfer(0xFFU);
    }

    uint8_t r1 = 0xFFU;
    for (uint8_t attempt = 0; attempt < 10U && r1 != SD_R1_IDLE; attempt++) {
        r1 = sd_simple_command(SD_CMD0, 0U, 0x95U);
    }
    if (r1 != SD_R1_IDLE) {
        return SD_ERROR;   /* no card, or it is not answering in SPI mode */
    }

    /* CMD8: v2.0+ cards answer with an R7 echoing the voltage/check pattern. v1 cards reject it. */
    bool v2 = false;
    r1 = sd_command(SD_CMD8, 0x000001AAU, 0x87U);
    if (r1 == SD_R1_IDLE) {
        uint8_t r7[4];
        for (uint8_t i = 0; i < 4U; i++) {
            r7[i] = xfer(0xFFU);
        }
        if (r7[2] != 0x01U || r7[3] != 0xAAU) {
            sd_deselect();
            return SD_ERROR;
        }
        v2 = true;
    }
    sd_deselect();

    /* ACMD41 until the card leaves idle; HCS bit only for v2 cards. */
    const uint32_t start = STM32_Millis();
    do {
        r1 = sd_simple_command(SD_CMD55, 0U, 0x65U);
        if (r1 > SD_R1_IDLE) {
            return SD_ERROR;
        }
        r1 = sd_simple_command(SD_ACMD41, v2 ? 0x40000000UL : 0U, 0x77U);
        if (r1 == 0x00U) {
            break;
        }
        STM32_DelayMilliseconds(2U);
    } while ((uint32_t)(STM32_Millis() - start) < SD_INIT_TIMEOUT_MS);
    if (r1 != 0x00U) {
        return SD_TIMEOUT;
    }

    /* OCR: CCS (bit 30) says whether the CMD17/24 argument is a sector or a byte address. */
    if (v2) {
        r1 = sd_command(SD_CMD58, 0U, 0xFDU);
        if (r1 != 0x00U) {
            sd_deselect();
            return SD_ERROR;
        }
        uint8_t ocr[4];
        for (uint8_t i = 0; i < 4U; i++) {
            ocr[i] = xfer(0xFFU);
        }
        sd_deselect();
        sd->block_addressed = (ocr[0] & 0x40U) ? 1U : 0U;
    }
    if (!sd->block_addressed) {
        if (sd_simple_command(SD_CMD16, 512U, 0x15U) != 0x00U) {
            return SD_ERROR;
        }
    }

    EasyTVC_SPI3_SetFast(true);
    sd->initialized = 1U;
    return SD_OK;
}

static uint32_t sd_address(const SD_Card *sd, uint32_t sector)
{
    return sd->block_addressed ? sector : sector * 512U;
}

SD_Result SD_Raw_ReadSector(SD_Card *sd, uint32_t sector, uint8_t *buffer)
{
    if (!sd->initialized) {
        return SD_NOT_INITIALIZED;
    }
    if (write_pending) {
        return SD_BUSY;
    }

    if (sd_command(SD_CMD17, sd_address(sd, sector), 0xFFU) != 0x00U) {
        sd_deselect();
        return SD_ERROR;
    }

    const uint32_t start = STM32_Millis();
    uint8_t token = 0xFFU;
    while (token == 0xFFU) {
        token = xfer(0xFFU);
        if ((uint32_t)(STM32_Millis() - start) > SD_TOKEN_TIMEOUT_MS) {
            sd_deselect();
            return SD_TIMEOUT;
        }
    }
    if (token != SD_TOKEN_DATA) {
        sd_deselect();
        return SD_ERROR;
    }

    for (uint16_t i = 0; i < 512U; i++) {
        buffer[i] = xfer(0xFFU);
    }
    (void)xfer(0xFFU);   /* CRC */
    (void)xfer(0xFFU);
    sd_deselect();
    return SD_OK;
}

SD_Result SD_Raw_WriteStart(SD_Card *sd, uint32_t sector, const uint8_t *buffer)
{
    if (!sd->initialized) {
        return SD_NOT_INITIALIZED;
    }
    if (write_pending) {
        return SD_BUSY;
    }

    if (sd_command(SD_CMD24, sd_address(sd, sector), 0xFFU) != 0x00U) {
        sd_deselect();
        return SD_ERROR;
    }

    (void)xfer(SD_TOKEN_DATA);
    for (uint16_t i = 0; i < 512U; i++) {
        (void)xfer(buffer[i]);
    }
    (void)xfer(0xFFU);   /* dummy CRC */
    (void)xfer(0xFFU);

    const uint8_t response = xfer(0xFFU);
    sd_deselect();
    if ((response & 0x1FU) != 0x05U) {
        return SD_ERROR;   /* data rejected (CRC / write error) */
    }

    write_pending = true;
    write_started_ms = STM32_Millis();
    return SD_OK;
}

SD_Result SD_Raw_WritePoll(SD_Card *sd)
{
    if (!sd->initialized) {
        return SD_NOT_INITIALIZED;
    }
    if (!write_pending) {
        return SD_OK;
    }

    /* While programming, the card holds DO low whenever it is selected. */
    sd_select();
    const uint8_t level = xfer(0xFFU);
    sd_deselect();
    if (level == 0xFFU) {
        write_pending = false;
        return SD_OK;
    }
    if ((uint32_t)(STM32_Millis() - write_started_ms) > SD_WRITE_TIMEOUT_MS) {
        write_pending = false;   /* give up; the caller treats this as a failed sector */
        return SD_TIMEOUT;
    }
    return SD_BUSY;
}

SD_Result SD_Raw_WriteSector(SD_Card *sd, uint32_t sector, const uint8_t *buffer)
{
    SD_Result result = SD_Raw_WriteStart(sd, sector, buffer);
    while (result == SD_OK || result == SD_BUSY) {
        result = SD_Raw_WritePoll(sd);
        if (result == SD_OK) {
            return SD_OK;
        }
    }
    return result;
}
