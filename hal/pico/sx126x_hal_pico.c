#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "sx126x_hal.h"
#include "sx126x_hal_pico.h"
#include <stdbool.h>
#include <stdint.h>

#define SX1276_HAL_BUSY_TIMEOUT_US 20000  // 20ms
#define SX1276_HAL_RESET_HELD_LOW_US 1000 // 1ms
#define SX1276_HAL_RESET_SLEEP_MS 100     // 100ms

bool sx126x_hal_pico_init(sx126x_hal_pico_context_t *ctx) {
    if (!ctx || ctx->initialized)
        return false;

    uint32_t actualRate = spi_init((spi_inst_t *)ctx->spiInst, ctx->speedKHz * 1000);
    if (actualRate != ctx->speedKHz)
        return false;

    gpio_set_function(ctx->sckPin, GPIO_FUNC_SPI);
    gpio_set_function(ctx->txPin, GPIO_FUNC_SPI);
    gpio_set_function(ctx->rxPin, GPIO_FUNC_SPI);

    gpio_init(ctx->csnPin);
    gpio_set_dir(ctx->csnPin, GPIO_OUT);
    gpio_put(ctx->csnPin, 1);

    gpio_init(ctx->rstPin);
    gpio_set_dir(ctx->rstPin, GPIO_OUT);
    gpio_put(ctx->rstPin, 1);

    gpio_init(ctx->busyPin);
    gpio_set_dir(ctx->busyPin, GPIO_IN);

    ctx->initialized = true;

    return true;
}

bool sx126x_hal_pico_deinit(sx126x_hal_pico_context_t *ctx) {
    if (!ctx || !ctx->initialized)
        return false;

    spi_deinit((spi_inst_t *)ctx->spiInst);

    gpio_deinit(ctx->sckPin);
    gpio_deinit(ctx->txPin);
    gpio_deinit(ctx->rxPin);
    gpio_deinit(ctx->csnPin);
    gpio_deinit(ctx->rstPin);
    gpio_deinit(ctx->busyPin);

    ctx->initialized = false;
    return true;
}

static inline void cs_select(const sx126x_hal_pico_context_t *ctx) {
    asm volatile("nop \n nop \n nop");
    gpio_put(ctx->csnPin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(const sx126x_hal_pico_context_t *ctx) {
    asm volatile("nop \n nop \n nop");
    gpio_put(ctx->csnPin, 1);
    asm volatile("nop \n nop \n nop");
}

static bool wait_busy(const sx126x_hal_pico_context_t *ctx) {
    absolute_time_t deadline = make_timeout_time_us(SX1276_HAL_BUSY_TIMEOUT_US);

    unsigned int busyPin = ctx->busyPin;
    while (gpio_get(busyPin) == 1) {
        if (time_reached(deadline)) {
            return false;
        }
    }

    return true;
}

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length) {
    if (!context)
        return SX126X_HAL_STATUS_ERROR;
    const sx126x_hal_pico_context_t *ctx = context;

    if (!ctx->initialized)
        return SX126X_HAL_STATUS_ERROR;

    if (!wait_busy(ctx))
        return SX126X_HAL_STATUS_ERROR;

    cs_select(ctx);

    if (command_length > 0) {
        int lenWritten = spi_write_blocking((spi_inst_t *)ctx->spiInst, command, command_length);
        if (lenWritten != command_length) {
            cs_deselect(ctx);
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    if (data_length > 0) {
        int lenWritten = spi_write_blocking((spi_inst_t *)ctx->spiInst, data, data_length);
        if (lenWritten != data_length) {
            cs_deselect(ctx);
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    cs_deselect(ctx);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length) {
    if (!context)
        return SX126X_HAL_STATUS_ERROR;
    const sx126x_hal_pico_context_t *ctx = context;

    if (!ctx->initialized)
        return SX126X_HAL_STATUS_ERROR;

    if (!wait_busy(ctx))
        return SX126X_HAL_STATUS_ERROR;

    cs_select(ctx);

    if (command_length > 0) {
        int lenWritten = spi_write_blocking((spi_inst_t *)ctx->spiInst, command, command_length);
        if (lenWritten != command_length) {
            cs_deselect(ctx);
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    if (data_length > 0) {
        int lenRead = spi_read_blocking((spi_inst_t *)ctx->spiInst, SX126X_NOP, data, data_length);
        if (lenRead != data_length) {
            cs_deselect(ctx);
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    cs_deselect(ctx);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context) {
    if (!context)
        return SX126X_HAL_STATUS_ERROR;
    const sx126x_hal_pico_context_t *ctx = context;

    if (!ctx->initialized)
        return SX126X_HAL_STATUS_ERROR;

    gpio_put(ctx->rstPin, 0);
    sleep_us(SX1276_HAL_RESET_HELD_LOW_US);
    gpio_put(ctx->rstPin, 1);
    sleep_ms(SX1276_HAL_RESET_SLEEP_MS);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
    // To wakeup, we can just perform a GetStatus command or simply toggle CS.
    // Here we send a GetStatus command (0xC0) with 0 length data.
    uint8_t cmd = 0xC0;
    return sx126x_hal_write(context, &cmd, 1, NULL, 0);
}
