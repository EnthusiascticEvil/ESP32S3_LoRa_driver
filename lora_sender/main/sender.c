#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
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

static const char* TAG = "SENDER";

sx126x_t radio = {
    .spi = NULL,
    .chip_select = PIN_NUM_CS,
    .reset = PIN_NUM_RST,
    .busy = PIN_NUM_BUSY,
    .irq = PIN_NUM_DIO1
};

void generate_random_message(char* buffer, size_t len) {
    static const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (size_t i = 0; i < len - 1; i++) {
        buffer[i] = charset[rand() % (sizeof(charset) - 1)];
    }
    buffer[len - 1] = '\0';
}

void app_main(void)
{
    srand(time(NULL));

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

    ESP_LOGI(TAG, "Resetting and waking up radio");
    sx126x_hal_reset(&radio);

    ESP_LOGI(TAG, "Enabling TCXO via DIO3...");
    sx126x_set_dio3_as_tcxo_ctrl(&radio, SX126X_TCXO_CTRL_3_3V, 5000);

    ESP_LOGI(TAG, "Waking up radio...");
    sx126x_hal_wakeup(&radio);

    sx126x_set_standby(&radio, SX126X_STANDBY_CFG_RC);
    sx126x_set_pkt_type(&radio, SX126X_PKT_TYPE_LORA);

    sx126x_mod_params_lora_t mod_params = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5
    };
    sx126x_set_lora_mod_params(&radio, &mod_params);

    sx126x_pkt_params_lora_t pkt_params = {
        .preamble_len_in_symb = 12,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 0,
        .crc_is_on = false,
        .invert_iq_is_on = true
    };

    sx126x_set_rf_freq(&radio, 868000000);
    sx126x_set_tx_params(&radio, 14, SX126X_RAMP_10_US);

    while (1) {
        char msg[21];  // 20 символов + '\0'
        generate_random_message(msg, sizeof(msg));

        size_t len = strlen(msg);
        pkt_params.pld_len_in_bytes = len;
        sx126x_set_lora_pkt_params(&radio, &pkt_params);

        ESP_LOGI(TAG, "Preparing random packet: %s", msg);
        sx126x_write_buffer(&radio, 0x00, (const uint8_t*)msg, len);
        sx126x_set_tx(&radio, 0);

        ESP_LOGI(TAG, "Packet sent");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
