
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "example_dsi_init.h"
#include "example_dsi_init_config.h"
#include "example_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "P4_AUTO_SYNC";

#define GPIO_HANDSHAKE      20
#define GPIO_SCLK           21
#define GPIO_MOSI           22
#define GPIO_MISO           24
#define GPIO_CS             25


#define SENDER_HOST         SPI2_HOST

#define IMG_WIDTH           160
#define IMG_HEIGHT          120
#define CHUNK_HEIGHT        20
#define BYTES_PER_PIXEL     2 
#define CHUNK_SIZE          (IMG_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
#define TOTAL_CHUNKS        (IMG_HEIGHT / CHUNK_HEIGHT) 

static QueueHandle_t rdySem;

static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    if (currtime_us - lasthandshaketime_us < 1000) return; 
    lasthandshaketime_us = currtime_us;
    BaseType_t mustYield = false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

void app_main(void)
{
    // --- DISPLAY INIT ---
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
    esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
    void *frame_buffer = NULL;
    size_t frame_buffer_size = 0;

    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
        .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer);

    int screen_width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES;
    int screen_height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES;
    frame_buffer_size = screen_width * screen_height * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;

    example_dpi_panel_reset(mipi_dpi_panel);
    example_dpi_panel_init(mipi_dpi_panel);
    memset(frame_buffer, 0, frame_buffer_size);
    esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // --- SPI INIT ---
    esp_err_t ret;
    spi_device_handle_t spi_handle;

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 8192, 
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
        .clock_speed_hz = 1000000, // 1 MHz
        
        // VRACÍME SE K MODE 1 (Ten byl stabilnější)
        .mode = 1,                 
        
        .spics_io_num = GPIO_CS,
        .cs_ena_pretrans = 16, 
        .cs_ena_posttrans = 4,
        .queue_size = 3,
        
        // ZPOŽDĚNÍ NA VSTUPU (Čištění signálu)
        .input_delay_ns = 100, 
    };

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
    };

    rdySem = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);
    spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
    xSemaphoreGive(rdySem);

    uint8_t *chunk_buf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    uint8_t *sendbuf_dummy = heap_caps_malloc(32, MALLOC_CAP_DMA);
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    int chunk_index = 0;
    int sync_state = 0; 
    
    // Proměnná pro detekci nutnosti posunu bitů
    bool need_bit_shift = false;

    int start_x = (screen_width - IMG_WIDTH) / 2;
    int start_y = (screen_height - IMG_HEIGHT) / 2;
    uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

    ESP_LOGI(TAG, "Waiting for SYNC (Auto-Detect)...");

    while (1) {
        memset(chunk_buf, 0, CHUNK_SIZE);
        snprintf((char*)sendbuf_dummy, 32, "Gimme");
        
        t.length = CHUNK_SIZE * 8;
        t.tx_buffer = sendbuf_dummy;
        t.rx_buffer = chunk_buf;

        // Pokud to tady spadne na Timeout, resetuj Kameru!
        if (xSemaphoreTake(rdySem, pdMS_TO_TICKS(1000)) != pdTRUE) {
             ESP_LOGW(TAG, "Timeout waiting for camera. Try resetting Slave.");
             continue;
        }

        ret = spi_device_transmit(spi_handle, &t);
        
        if (ret == ESP_OK) {
            
            // --- HLEDÁNÍ HLAVIČKY ---
            if (sync_state == 0) {
                // VARIANTA A: Perfektní signál (AA CC)
                if (chunk_buf[0] == 0xAA && chunk_buf[1] == 0xCC) {
                    ESP_LOGI(TAG, "SYNC OK (Direct)!");
                    need_bit_shift = false;
                    sync_state = 1;
                    chunk_index = 0;
                }
                // VARIANTA B: Posunutý signál (55 98 nebo podobně)
                // 0x55 je 0xAA >> 1. Pokud vidíme 55, zapneme opravu.
                else if (chunk_buf[0] == 0x55) {
                    ESP_LOGI(TAG, "SYNC OK (Shifted)! Enabling fix.");
                    need_bit_shift = true;
                    sync_state = 1;
                    chunk_index = 0;
                }
                else {
                     // Debug výpis, ať vidíme, co chodí
                     printf("No Sync. Recv: %02X %02X ...\n", chunk_buf[0], chunk_buf[1]);
                     continue; 
                }
            }

            // --- APLIKACE OPRAVY (pokud je potřeba) ---
            if (need_bit_shift) {
                for(int i=0; i<CHUNK_SIZE; i++) {
                    chunk_buf[i] = (chunk_buf[i] >> 1);
                }
            }

            // --- VYKRESLOVÁNÍ ---
            uint16_t *chunk_pixels = (uint16_t *)chunk_buf;
            for (int y = 0; y < CHUNK_HEIGHT; y++) {
                int display_y = start_y + (chunk_index * CHUNK_HEIGHT) + y;
                if (display_y >= screen_height) break;
                int display_idx = (display_y * screen_width) + start_x;
                int chunk_row_start = y * IMG_WIDTH;
                
                for (int x = 0; x < IMG_WIDTH; x++) {
                    uint16_t pixel = chunk_pixels[chunk_row_start + x];
                    // Byte Swap (vždy nutný pro barvy)
                    lcd_pixels[display_idx + x] = (pixel >> 8) | (pixel << 8);
                }
            }

            chunk_index++;
            if (chunk_index >= TOTAL_CHUNKS) {
                esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                
                // Zůstaneme synchronizovaní i pro další snímek, 
                // ale resetujeme index. Pokud se to rozbije, stav 0 se sám nenastaví,
                // což může být riskantní, ale obraz bude plynulejší.
                // Pokud chceš maximální jistotu, odkomentuj: sync_state = 0;
                chunk_index = 0;
            }
        }
    }
}
