#ifndef SX126X_HAL_PICO_H
#define SX126X_HAL_PICO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool initialized;
    void *spiInst; // spi_inst_t* ("hardware/spi.h")
    uint32_t speedKHz;
    uint32_t sckPin;
    uint32_t txPin;
    uint32_t rxPin;
    uint32_t csnPin;
    uint32_t rstPin;
    uint32_t busyPin;
} sx126x_hal_pico_context_t;

bool sx126x_hal_pico_init(sx126x_hal_pico_context_t *ctx);

bool sx126x_hal_pico_deinit(sx126x_hal_pico_context_t *ctx);

#endif
