#include <string.h>
#include "sx126x_hal.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* cmd, const uint16_t cmd_len,
                                     const uint8_t* data, const uint16_t data_len)
{
    const sx126x_t* radio = (const sx126x_t*) context;

    gpio_set_level(radio->chip_select, 0);

    uint8_t buffer[256];
    if (cmd_len + data_len > sizeof(buffer)) return SX126X_HAL_STATUS_ERROR;
    memcpy(buffer, cmd, cmd_len);
    if (data && data_len) memcpy(buffer + cmd_len, data, data_len);

    spi_transaction_t t = {
        .length = (cmd_len + data_len) * 8,
        .tx_buffer = buffer,
        .rx_buffer = NULL
    };

    esp_err_t res = spi_device_transmit(radio->spi, &t);
    gpio_set_level(radio->chip_select, 1);
    return (res == ESP_OK) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* cmd, const uint16_t cmd_len,
                                    uint8_t* data, const uint16_t data_len)
{
    const sx126x_t* radio = (const sx126x_t*) context;

    gpio_set_level(radio->chip_select, 0);

    uint8_t tx_buffer[256] = {0};
    uint8_t rx_buffer[256] = {0};
    if (cmd_len + data_len > sizeof(tx_buffer)) return SX126X_HAL_STATUS_ERROR;

    memcpy(tx_buffer, cmd, cmd_len);

    spi_transaction_t t = {
        .length = (cmd_len + data_len) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    esp_err_t res = spi_device_transmit(radio->spi, &t);
    gpio_set_level(radio->chip_select, 1);

    if (res == ESP_OK) {
        memcpy(data, rx_buffer + cmd_len, data_len);
        return SX126X_HAL_STATUS_OK;
    }
    return SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
    const sx126x_t* radio = (const sx126x_t*) context;

    gpio_set_level(radio->reset, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(radio->reset, 1);
    vTaskDelay(pdMS_TO_TICKS(6));
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    const sx126x_t* radio = (const sx126x_t*) context;

    gpio_set_level(radio->chip_select, 0);
    esp_rom_delay_us(500);
    gpio_set_level(radio->chip_select, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_init(sx126x_t* radio)
{
    // Настройка NSS (CS)
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << radio->chip_select,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(radio->chip_select, 1);

    // Настройка RESET
    io_conf.pin_bit_mask = 1ULL << radio->reset;
    gpio_config(&io_conf);
    gpio_set_level(radio->reset, 1);

    // Настройка BUSY (вход)
    io_conf.pin_bit_mask = 1ULL << radio->busy;
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // Настройка DIO1 (вход, если используется)
    if (radio->irq != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = 1ULL << radio->irq;
        gpio_config(&io_conf);
    }

    return SX126X_HAL_STATUS_OK;
}

static void sx126x_wait_on_busy(const sx126x_t* radio)
{
    while (gpio_get_level(radio->busy) == 1) {
        esp_rom_delay_us(1);
    }
}
