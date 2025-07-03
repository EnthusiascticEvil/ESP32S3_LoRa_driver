#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx126x_defs.h"

#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 10
#define PIN_NUM_CLK  9
#define PIN_NUM_CS   8
#define PIN_NUM_RST  12
#define PIN_NUM_BUSY 13
#define PIN_NUM_DIO1 14

static const char* TAG = "RECEIVER";

sx126x_t radio = {
    .spi = NULL,
    .chip_select = PIN_NUM_CS,
    .reset = PIN_NUM_RST,
    .busy = PIN_NUM_BUSY,
    .irq = PIN_NUM_DIO1
};

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SPI bus");
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &radio.spi));

    ESP_LOGI(TAG, "Initializing HAL");
    sx126x_hal_init(&radio);

    ESP_LOGI(TAG, "Resetting radio");
    sx126x_hal_reset(&radio);

    ESP_LOGI(TAG, "Enabling TCXO via DIO3");
    sx126x_set_dio3_as_tcxo_ctrl(&radio, SX126X_TCXO_CTRL_3_3V, 5000);  // 5ms

    ESP_LOGI(TAG, "Waking up radio");
    sx126x_hal_wakeup(&radio);

    ESP_LOGI(TAG, "Setting standby mode");
    sx126x_set_standby(&radio, SX126X_STANDBY_CFG_RC);

    ESP_LOGI(TAG, "Setting packet type");
    sx126x_set_pkt_type(&radio, SX126X_PKT_TYPE_LORA);

    sx126x_mod_params_lora_t mod_params = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5
    };
    ESP_LOGI(TAG, "Setting modulation parameters");
    sx126x_set_lora_mod_params(&radio, &mod_params);

    sx126x_pkt_params_lora_t pkt_params = {
        .preamble_len_in_symb = 12,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 21,
        .crc_is_on = false,
        .invert_iq_is_on = true
    };
    ESP_LOGI(TAG, "Setting packet parameters");
    sx126x_set_lora_pkt_params(&radio, &pkt_params);

    ESP_LOGI(TAG, "Setting RF frequency");
    sx126x_set_rf_freq(&radio, 868000000);

    ESP_LOGI(TAG, "Setting IRQ parameters...");
    sx126x_set_dio_irq_params(&radio,
        SX126X_IRQ_ALL,
        SX126X_IRQ_ALL, SX126X_IRQ_NONE, SX126X_IRQ_NONE);

    uint8_t buffer[128];

    while (1) {
        ESP_LOGI(TAG, "Set RX");
        sx126x_set_rx(&radio, 60000);  // Continuous RX

        sx126x_hal_wait_on_busy(&radio);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1s polling interval

        uint16_t irq = 0;
        sx126x_get_irq_status(&radio, &irq);
        ESP_LOGI(TAG, "IRQ status: 0x%04X", irq);

        if (irq & SX126X_IRQ_RX_DONE) {
            ESP_LOGI(TAG, "RX_DONE triggered, reading buffer");
            sx126x_clear_irq_status(&radio, SX126X_IRQ_ALL);

            sx126x_rx_buffer_status_t buf_status = {0};
            sx126x_get_rx_buffer_status(&radio, &buf_status);
            uint8_t len = buf_status.pld_len_in_bytes;

            sx126x_read_buffer(&radio, 0, buffer, len);
            buffer[len] = '\0';

            ESP_LOGI(TAG, "Received: %s", buffer);
        } else if (irq & SX126X_IRQ_TIMEOUT) {
            ESP_LOGW(TAG, "RX Timeout");
            sx126x_clear_irq_status(&radio, SX126X_IRQ_ALL);
        } else {
            ESP_LOGW(TAG, "No packet or unhandled IRQ");
        }
    }
}
