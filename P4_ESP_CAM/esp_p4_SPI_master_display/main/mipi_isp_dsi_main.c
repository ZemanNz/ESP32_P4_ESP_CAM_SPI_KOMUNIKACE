// // // // /*
// // // //  * P4 Master SPI Receiver + MIPI DSI Display
// // // //  * Spojení: ZemanNz SPI Master code + MIPI DSI Example
// // // //  */
// // // // #include <stdio.h>
// // // // #include <stdlib.h>
// // // // #include <string.h>
// // // // #include "sdkconfig.h"
// // // // #include "esp_attr.h"
// // // // #include "esp_log.h"
// // // // #include "freertos/FreeRTOS.h"
// // // // #include "freertos/task.h"
// // // // #include "freertos/semphr.h"
// // // // #include "esp_lcd_mipi_dsi.h"
// // // // #include "esp_lcd_panel_ops.h"
// // // // #include "esp_ldo_regulator.h"
// // // // #include "esp_cache.h"
// // // // #include "example_dsi_init.h"
// // // // #include "example_dsi_init_config.h"
// // // // #include "example_config.h"
// // // // #include "driver/spi_master.h"
// // // // #include "driver/gpio.h"
// // // // #include "esp_timer.h"

// // // // static const char *TAG = "P4_CAM_DISPLAY";

// // // // // ==========================================
// // // // // 1. KONFIGURACE SPI (Z tvého kódu)
// // // // // ==========================================
// // // // #define GPIO_HANDSHAKE      20
// // // // #define GPIO_SCLK           21
// // // // #define GPIO_MOSI           22
// // // // #define GPIO_MISO           24
// // // // #define GPIO_CS             25
// // // // #define SENDER_HOST         SPI2_HOST

// // // // // Nastavení obrazu (QQVGA)
// // // // #define IMG_WIDTH           160
// // // // #define IMG_HEIGHT          120
// // // // #define CHUNK_HEIGHT        20
// // // // #define BYTES_PER_PIXEL     2 // RGB565
// // // // // 160 * 20 * 2 = 6400 Bajtů
// // // // #define CHUNK_SIZE          (IMG_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
// // // // #define TOTAL_CHUNKS        (IMG_HEIGHT / CHUNK_HEIGHT) // 6 chunků

// // // // static QueueHandle_t rdySem;

// // // // // Přerušení od Handshake pinu
// // // // static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
// // // // {
// // // //     static uint32_t lasthandshaketime_us;
// // // //     uint32_t currtime_us = esp_timer_get_time();
// // // //     if (currtime_us - lasthandshaketime_us < 1000) return; 
// // // //     lasthandshaketime_us = currtime_us;
// // // //     BaseType_t mustYield = false;
// // // //     xSemaphoreGiveFromISR(rdySem, &mustYield);
// // // //     if (mustYield) portYIELD_FROM_ISR();
// // // // }

// // // // void app_main(void)
// // // // {
// // // //     // --- A. INICIALIZACE DISPLEJE (Původní kód) ---
// // // //     esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
// // // //     esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
// // // //     esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
// // // //     void *frame_buffer = NULL; // Ukazatel na paměť obrazovky
// // // //     size_t frame_buffer_size = 0;

// // // //     // MIPI LDO Power
// // // //     esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
// // // //     esp_ldo_channel_config_t ldo_mipi_phy_config = {
// // // //         .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
// // // //         .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
// // // //     };
// // // //     ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

// // // //     // DSI Init (Získáme frame_buffer!)
// // // //     example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer);

// // // //     int screen_width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES;
// // // //     int screen_height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES;
// // // //     frame_buffer_size = screen_width * screen_height * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;

// // // //     ESP_LOGI(TAG, "Display Resolution: %dx%d", screen_width, screen_height);

// // // //     // Reset a Init panelu
// // // //     example_dpi_panel_reset(mipi_dpi_panel);
// // // //     example_dpi_panel_init(mipi_dpi_panel);

// // // //     // Vyčistíme obrazovku na ČERNOU (místo červené)
// // // //     memset(frame_buffer, 0, frame_buffer_size);
// // // //     esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

// // // //     // --- B. INICIALIZACE SPI (Tvůj kód) ---
// // // //     esp_err_t ret;
// // // //     spi_device_handle_t spi_handle;

// // // //     spi_bus_config_t buscfg = {
// // // //         .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
// // // //         .quadwp_io_num = -1, .quadhd_io_num = -1,
// // // //         .max_transfer_sz = 8192, 
// // // //     };

// // // //     spi_device_interface_config_t devcfg = {
// // // //         .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
// // // //         .clock_speed_hz = 1000000, 
// // // //         .mode = 0,                  
// // // //         .spics_io_num = GPIO_CS,
// // // //         .cs_ena_pretrans = 16, .cs_ena_posttrans = 4,
// // // //         .queue_size = 3
// // // //     };

// // // //     gpio_config_t io_conf = {
// // // //         .intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT,
// // // //         .pull_up_en = GPIO_PULLUP_ENABLE, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
// // // //     };

// // // //     rdySem = xSemaphoreCreateBinary();
// // // //     gpio_config(&io_conf);
// // // //     gpio_install_isr_service(0);
// // // //     gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

// // // //     spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
// // // //     gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);
// // // //     spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
// // // //     xSemaphoreGive(rdySem);

// // // //     // Buffery pro SPI
// // // //     uint8_t *chunk_buf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
// // // //     uint8_t *sendbuf_dummy = heap_caps_malloc(32, MALLOC_CAP_DMA);
    
// // // //     spi_transaction_t t;
// // // //     memset(&t, 0, sizeof(t));
// // // //     int chunk_index = 0;

// // // //     // Výpočet pro vycentrování obrazu (160x120) na displeji
// // // //     int start_x = (screen_width - IMG_WIDTH) / 2;
// // // //     int start_y = (screen_height - IMG_HEIGHT) / 2;
    
// // // //     // Ukazatel na paměť displeje jako uint16_t (pro pixely)
// // // //     uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

// // // //     ESP_LOGI(TAG, "Starting SPI Loop. Image will be at X:%d Y:%d", start_x, start_y);

// // // //     while (1) {
// // // //         memset(chunk_buf, 0, CHUNK_SIZE);
// // // //         snprintf((char*)sendbuf_dummy, 32, "Get Chunk %d", chunk_index);
        
// // // //         t.length = CHUNK_SIZE * 8;
// // // //         t.tx_buffer = sendbuf_dummy;
// // // //         t.rx_buffer = chunk_buf;

// // // //         // 1. Čekáme na data od kamery
// // // //         xSemaphoreTake(rdySem, portMAX_DELAY);
        
// // // //         // 2. SPI Přenos
// // // //         ret = spi_device_transmit(spi_handle, &t);
        
// // // //         if (ret == ESP_OK) {
            
// // // //             // 3. Bit-Shift Oprava (>> 1) na RAW datech
// // // //             for(int i=0; i<CHUNK_SIZE; i++) {
// // // //                 chunk_buf[i] = (chunk_buf[i] >> 1);
// // // //             }

// // // //             // 4. Kopírování Chunku do Frame Bufferu displeje
// // // //             // Chunk má výšku 20 řádků (CHUNK_HEIGHT)
// // // //             // Musíme kopírovat řádek po řádku, protože displej je širší než obrázek.
            
// // // //             // Ukazatel na začátek dat v chunku (přetypováno na uint16_t pixely)
// // // //             uint16_t *chunk_pixels = (uint16_t *)chunk_buf;

// // // //             for (int y = 0; y < CHUNK_HEIGHT; y++) {
// // // //                 // Kde jsme v rámci celého obrázku?
// // // //                 int current_img_y = (chunk_index * CHUNK_HEIGHT) + y;
                
// // // //                 // Cílová Y souřadnice na displeji (s centrováním)
// // // //                 int display_y = start_y + current_img_y;
                
// // // //                 // Cílový index v poli (Řádek * Šířka_Displeje + Odsazení_X)
// // // //                 int display_idx = (display_y * screen_width) + start_x;

// // // //                 // Zdrojový index v chunku (Řádek * Šířka_Obrázku)
// // // //                 int chunk_idx = y * IMG_WIDTH;

// // // //                 // Kopírujeme jeden celý řádek (160 pixelů * 2 bajty)
// // // //                 memcpy(&lcd_pixels[display_idx], &chunk_pixels[chunk_idx], IMG_WIDTH * 2);
// // // //             }
// // // //         }

// // // //         chunk_index++;

// // // //         // 5. Máme celý snímek? Obnovíme displej!
// // // //         if (chunk_index >= TOTAL_CHUNKS) {
// // // //             chunk_index = 0;
            
// // // //             // Synchronizace Cache -> RAM (Aby to displej viděl)
// // // //             esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            
// // // //             // (Volitelné) Výpis FPS nebo jen tečka
// // // //             // printf("."); 
// // // //             // fflush(stdout);
// // // //         }
// // // //     }
// // // // }




// // // /*
// // //  * P4 Master SPI Receiver + MIPI DSI Display
// // //  * Fix: Byte Swap (Endianita) + Odstranění Bit-Shiftu
// // //  */
// // // #include <stdio.h>
// // // #include <stdlib.h>
// // // #include <string.h>
// // // #include "sdkconfig.h"
// // // #include "esp_attr.h"
// // // #include "esp_log.h"
// // // #include "freertos/FreeRTOS.h"
// // // #include "freertos/task.h"
// // // #include "freertos/semphr.h"
// // // #include "esp_lcd_mipi_dsi.h"
// // // #include "esp_lcd_panel_ops.h"
// // // #include "esp_ldo_regulator.h"
// // // #include "esp_cache.h"
// // // #include "example_dsi_init.h"
// // // #include "example_dsi_init_config.h"
// // // #include "example_config.h"
// // // #include "driver/spi_master.h"
// // // #include "driver/gpio.h"
// // // #include "esp_timer.h"

// // // static const char *TAG = "P4_CAM_DISPLAY";

// // // // ==========================================
// // // // 1. KONFIGURACE SPI
// // // // ==========================================
// // // #define GPIO_HANDSHAKE      20
// // // #define GPIO_SCLK           21
// // // #define GPIO_MOSI           22
// // // #define GPIO_MISO           24
// // // #define GPIO_CS             25
// // // #define SENDER_HOST         SPI2_HOST

// // // // Nastavení obrazu (QQVGA)
// // // #define IMG_WIDTH           160
// // // #define IMG_HEIGHT          120
// // // #define CHUNK_HEIGHT        20
// // // #define BYTES_PER_PIXEL     2 // RGB565
// // // // 160 * 20 * 2 = 6400 Bajtů
// // // #define CHUNK_SIZE          (IMG_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
// // // #define TOTAL_CHUNKS        (IMG_HEIGHT / CHUNK_HEIGHT) // 6 chunků

// // // static QueueHandle_t rdySem;

// // // // Přerušení od Handshake pinu
// // // static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
// // // {
// // //     static uint32_t lasthandshaketime_us;
// // //     uint32_t currtime_us = esp_timer_get_time();
// // //     if (currtime_us - lasthandshaketime_us < 1000) return; 
// // //     lasthandshaketime_us = currtime_us;
// // //     BaseType_t mustYield = false;
// // //     xSemaphoreGiveFromISR(rdySem, &mustYield);
// // //     if (mustYield) portYIELD_FROM_ISR();
// // // }

// // // void app_main(void)
// // // {
// // //     // --- A. INICIALIZACE DISPLEJE ---
// // //     esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
// // //     esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
// // //     esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
// // //     void *frame_buffer = NULL; // Ukazatel na paměť obrazovky
// // //     size_t frame_buffer_size = 0;

// // //     // MIPI LDO Power
// // //     esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
// // //     esp_ldo_channel_config_t ldo_mipi_phy_config = {
// // //         .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
// // //         .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
// // //     };
// // //     ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

// // //     // DSI Init (Získáme frame_buffer!)
// // //     example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer);

// // //     // Získáme rozlišení z konfigurace (Kconfig)
// // //     int screen_width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES;
// // //     int screen_height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES;
// // //     frame_buffer_size = screen_width * screen_height * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;

// // //     ESP_LOGI(TAG, "Display Resolution: %dx%d, buffer: %zu bytes", screen_width, screen_height, frame_buffer_size);

// // //     // Reset a Init panelu
// // //     example_dpi_panel_reset(mipi_dpi_panel);
// // //     example_dpi_panel_init(mipi_dpi_panel);

// // //     // Vyčistíme obrazovku na ČERNOU
// // //     memset(frame_buffer, 0, frame_buffer_size);
// // //     esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

// // //     // --- B. INICIALIZACE SPI ---
// // //     esp_err_t ret;
// // //     spi_device_handle_t spi_handle;

// // //     spi_bus_config_t buscfg = {
// // //         .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
// // //         .quadwp_io_num = -1, .quadhd_io_num = -1,
// // //         .max_transfer_sz = 8192, 
// // //     };

// // //     spi_device_interface_config_t devcfg = {
// // //         .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
// // //         .clock_speed_hz = 1000000, 
// // //         .mode = 0,                  
// // //         .spics_io_num = GPIO_CS,
// // //         .cs_ena_pretrans = 16, .cs_ena_posttrans = 4,
// // //         .queue_size = 3
// // //     };

// // //     gpio_config_t io_conf = {
// // //         .intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT,
// // //         .pull_up_en = GPIO_PULLUP_ENABLE, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
// // //     };

// // //     rdySem = xSemaphoreCreateBinary();
// // //     gpio_config(&io_conf);
// // //     gpio_install_isr_service(0);
// // //     gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

// // //     spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
// // //     gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);
// // //     spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
// // //     xSemaphoreGive(rdySem);

// // //     // Buffery pro SPI
// // //     uint8_t *chunk_buf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
// // //     uint8_t *sendbuf_dummy = heap_caps_malloc(32, MALLOC_CAP_DMA);
    
// // //     spi_transaction_t t;
// // //     memset(&t, 0, sizeof(t));
// // //     int chunk_index = 0;

// // //     // Výpočet pro vycentrování obrazu (160x120) na displeji
// // //     int start_x = (screen_width - IMG_WIDTH) / 2;
// // //     int start_y = (screen_height - IMG_HEIGHT) / 2;
    
// // //     // Ukazatel na paměť displeje jako uint16_t (pro pixely)
// // //     uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

// // //     ESP_LOGI(TAG, "Starting SPI Loop...");

// // //     while (1) {
// // //         memset(chunk_buf, 0, CHUNK_SIZE);
// // //         snprintf((char*)sendbuf_dummy, 32, "Get Chunk %d", chunk_index);
        
// // //         t.length = CHUNK_SIZE * 8;
// // //         t.tx_buffer = sendbuf_dummy;
// // //         t.rx_buffer = chunk_buf;

// // //         // 1. Čekáme na data od kamery
// // //         xSemaphoreTake(rdySem, portMAX_DELAY);
        
// // //         // 2. SPI Přenos
// // //         ret = spi_device_transmit(spi_handle, &t);
        
// // //         if (ret == ESP_OK) {
            
// // //             // --- ZDE JE TA HLAVNÍ ZMĚNA ---
// // //             // Odstranil jsem ten cyklus ">> 1" (Bit shift).
// // //             // Místo toho děláme prohození bajtů (Byte Swap) při kopírování.

// // //             // Přetypování bufferu na 16-bitové pixely
// // //             uint16_t *chunk_pixels = (uint16_t *)chunk_buf;

// // //             for (int y = 0; y < CHUNK_HEIGHT; y++) {
// // //                 int display_y = start_y + (chunk_index * CHUNK_HEIGHT) + y;
                
// // //                 // Ochrana proti zápisu mimo obrazovku
// // //                 if (display_y >= screen_height) break;

// // //                 int display_idx = (display_y * screen_width) + start_x;
// // //                 int chunk_row_start = y * IMG_WIDTH;

// // //                 for (int x = 0; x < IMG_WIDTH; x++) {
// // //                     uint16_t pixel = chunk_pixels[chunk_row_start + x];
                    
// // //                     // SWAP: Prohození horního a dolního bajtu (0xAABB -> 0xBBAA)
// // //                     // Tím opravíme "divné barvy"
// // //                     lcd_pixels[display_idx + x] = (pixel >> 8) | (pixel << 8);
// // //                 }
// // //             }
// // //         }

// // //         chunk_index++;

// // //         // 5. Máme celý snímek? Obnovíme displej!
// // //         if (chunk_index >= TOTAL_CHUNKS) {
// // //             chunk_index = 0;
            
// // //             // Synchronizace Cache -> RAM (Aby to displej viděl)
// // //             esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
// // //         }
// // //     }
// // // }



// // /*
// //  * P4 Master SPI Receiver + MIPI DSI Display
// //  * Verze: Bit-Shift FIX (>> 1) + Byte Swap (Endianita)
// //  */
// // #include <stdio.h>
// // #include <stdlib.h>
// // #include <string.h>
// // #include "sdkconfig.h"
// // #include "esp_attr.h"
// // #include "esp_log.h"
// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "freertos/semphr.h"
// // #include "esp_lcd_mipi_dsi.h"
// // #include "esp_lcd_panel_ops.h"
// // #include "esp_ldo_regulator.h"
// // #include "esp_cache.h"
// // #include "example_dsi_init.h"
// // #include "example_dsi_init_config.h"
// // #include "example_config.h"
// // #include "driver/spi_master.h"
// // #include "driver/gpio.h"
// // #include "esp_timer.h"

// // static const char *TAG = "P4_CAM_DISPLAY";

// // // ==========================================
// // // 1. KONFIGURACE SPI
// // // ==========================================
// // #define GPIO_HANDSHAKE      20
// // #define GPIO_SCLK           21
// // #define GPIO_MOSI           22
// // #define GPIO_MISO           24
// // #define GPIO_CS             25
// // #define SENDER_HOST         SPI2_HOST

// // // Nastavení obrazu (QQVGA)
// // #define IMG_WIDTH           160
// // #define IMG_HEIGHT          120
// // #define CHUNK_HEIGHT        20
// // #define BYTES_PER_PIXEL     2 // RGB565
// // #define CHUNK_SIZE          (IMG_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
// // #define TOTAL_CHUNKS        (IMG_HEIGHT / CHUNK_HEIGHT) 

// // static QueueHandle_t rdySem;

// // // Přerušení od Handshake pinu
// // static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
// // {
// //     static uint32_t lasthandshaketime_us;
// //     uint32_t currtime_us = esp_timer_get_time();
// //     if (currtime_us - lasthandshaketime_us < 1000) return; 
// //     lasthandshaketime_us = currtime_us;
// //     BaseType_t mustYield = false;
// //     xSemaphoreGiveFromISR(rdySem, &mustYield);
// //     if (mustYield) portYIELD_FROM_ISR();
// // }

// // void app_main(void)
// // {
// //     // --- A. INICIALIZACE DISPLEJE ---
// //     esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
// //     esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
// //     esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
// //     void *frame_buffer = NULL;
// //     size_t frame_buffer_size = 0;

// //     esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
// //     esp_ldo_channel_config_t ldo_mipi_phy_config = {
// //         .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
// //         .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
// //     };
// //     ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

// //     example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer);

// //     int screen_width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES;
// //     int screen_height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES;
// //     frame_buffer_size = screen_width * screen_height * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;

// //     ESP_LOGI(TAG, "Display: %dx%d", screen_width, screen_height);

// //     example_dpi_panel_reset(mipi_dpi_panel);
// //     example_dpi_panel_init(mipi_dpi_panel);

// //     // Vyčistíme obrazovku
// //     memset(frame_buffer, 0, frame_buffer_size);
// //     esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

// //     // --- B. INICIALIZACE SPI ---
// //     esp_err_t ret;
// //     spi_device_handle_t spi_handle;

// //     spi_bus_config_t buscfg = {
// //         .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
// //         .quadwp_io_num = -1, .quadhd_io_num = -1,
// //         .max_transfer_sz = 8192, 
// //     };

// //     spi_device_interface_config_t devcfg = {
// //         .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
// //         .clock_speed_hz = 1000000, 
// //         .mode = 0,                  
// //         .spics_io_num = GPIO_CS,
// //         .cs_ena_pretrans = 16, .cs_ena_posttrans = 4,
// //         .queue_size = 3
// //     };

// //     gpio_config_t io_conf = {
// //         .intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT,
// //         .pull_up_en = GPIO_PULLUP_ENABLE, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
// //     };

// //     rdySem = xSemaphoreCreateBinary();
// //     gpio_config(&io_conf);
// //     gpio_install_isr_service(0);
// //     gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

// //     spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
// //     gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);
// //     spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
// //     xSemaphoreGive(rdySem);

// //     uint8_t *chunk_buf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
// //     uint8_t *sendbuf_dummy = heap_caps_malloc(32, MALLOC_CAP_DMA);
    
// //     spi_transaction_t t;
// //     memset(&t, 0, sizeof(t));
// //     int chunk_index = 0;

// //     int start_x = (screen_width - IMG_WIDTH) / 2;
// //     int start_y = (screen_height - IMG_HEIGHT) / 2;
// //     uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

// //     ESP_LOGI(TAG, "Loop start. Shift >> 1 ACTIVE.");

// //     while (1) {
// //         memset(chunk_buf, 0, CHUNK_SIZE);
// //         snprintf((char*)sendbuf_dummy, 32, "Get Chunk %d", chunk_index);
        
// //         t.length = CHUNK_SIZE * 8;
// //         t.tx_buffer = sendbuf_dummy;
// //         t.rx_buffer = chunk_buf;

// //         xSemaphoreTake(rdySem, portMAX_DELAY);
// //         ret = spi_device_transmit(spi_handle, &t);
        
// //         if (ret == ESP_OK) {
            
// //             // ---------------------------------------------------------
// //             // 1. KROK: HARDWAROVÁ OPRAVA (BIT SHIFT)
// //             // Napravíme posun bitů způsobený dráty
// //             // ---------------------------------------------------------
// //             for(int i=0; i<CHUNK_SIZE; i++) {
// //                 chunk_buf[i] = (chunk_buf[i] >> 1);
// //             }

// //             // ---------------------------------------------------------
// //             // 2. KROK: PŘEVOD NA PIXELY A ENDIAN SWAP
// //             // ---------------------------------------------------------
// //             uint16_t *chunk_pixels = (uint16_t *)chunk_buf;

// //             for (int y = 0; y < CHUNK_HEIGHT; y++) {
// //                 int display_y = start_y + (chunk_index * CHUNK_HEIGHT) + y;
// //                 if (display_y >= screen_height) break;

// //                 int display_idx = (display_y * screen_width) + start_x;
// //                 int chunk_row_start = y * IMG_WIDTH;

// //                 for (int x = 0; x < IMG_WIDTH; x++) {
// //                     uint16_t pixel = chunk_pixels[chunk_row_start + x];
                    
// //                     // SWAP: Prohození bajtů pro správné barvy
// //                     lcd_pixels[display_idx + x] = (pixel >> 8) | (pixel << 8);
// //                 }
// //             }
// //         }

// //         chunk_index++;

// //         if (chunk_index >= TOTAL_CHUNKS) {
// //             chunk_index = 0;
// //             esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
// //         }
// //     }
// // }



// /*
//  * P4 Master SPI Receiver + MIPI DSI Display
//  * Verze: CLEAN DATA (Bez Bit-Shiftu) + SPI MODE 1 + 500kHz
//  */
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "sdkconfig.h"
// #include "esp_attr.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "esp_lcd_mipi_dsi.h"
// #include "esp_lcd_panel_ops.h"
// #include "esp_ldo_regulator.h"
// #include "esp_cache.h"
// #include "example_dsi_init.h"
// #include "example_dsi_init_config.h"
// #include "example_config.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"

// static const char *TAG = "P4_CAM_DISPLAY";

// // ==========================================
// // 1. KONFIGURACE SPI
// // ==========================================
// #define GPIO_HANDSHAKE      20
// #define GPIO_SCLK           21
// #define GPIO_MOSI           22
// #define GPIO_MISO           24
// #define GPIO_CS             25
// #define SENDER_HOST         SPI2_HOST

// // Nastavení obrazu (QQVGA 160x120)
// #define IMG_WIDTH           160
// #define IMG_HEIGHT          120
// #define CHUNK_HEIGHT        20
// #define BYTES_PER_PIXEL     2 // RGB565
// #define CHUNK_SIZE          (IMG_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
// #define TOTAL_CHUNKS        (IMG_HEIGHT / CHUNK_HEIGHT) 

// static QueueHandle_t rdySem;

// static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
// {
//     static uint32_t lasthandshaketime_us;
//     uint32_t currtime_us = esp_timer_get_time();
//     if (currtime_us - lasthandshaketime_us < 1000) return; 
//     lasthandshaketime_us = currtime_us;
//     BaseType_t mustYield = false;
//     xSemaphoreGiveFromISR(rdySem, &mustYield);
//     if (mustYield) portYIELD_FROM_ISR();
// }

// void app_main(void)
// {
//     // --- A. INICIALIZACE DISPLEJE ---
//     esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
//     esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
//     esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
//     void *frame_buffer = NULL;
//     size_t frame_buffer_size = 0;

//     esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
//     esp_ldo_channel_config_t ldo_mipi_phy_config = {
//         .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
//         .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
//     };
//     ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

//     example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer);

//     int screen_width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES;
//     int screen_height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES;
//     frame_buffer_size = screen_width * screen_height * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;

//     example_dpi_panel_reset(mipi_dpi_panel);
//     example_dpi_panel_init(mipi_dpi_panel);

//     // Vyčistíme obrazovku
//     memset(frame_buffer, 0, frame_buffer_size);
//     esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

//     // --- B. INICIALIZACE SPI ---
//     esp_err_t ret;
//     spi_device_handle_t spi_handle;

//     spi_bus_config_t buscfg = {
//         .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
//         .quadwp_io_num = -1, .quadhd_io_num = -1,
//         .max_transfer_sz = 8192, 
//     };

//     spi_device_interface_config_t devcfg = {
//         .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
        
//         // ZPOMALENÍ: 500 kHz pro stabilitu
//         .clock_speed_hz = 500000, 
        
//         // ZMĚNA: Mode 1 (čte data na sestupné hraně hodin)
//         // Toto by mělo vyřešit ten posun bitů bez softwarového hacku
//         .mode = 1,                  
        
//         .spics_io_num = GPIO_CS,
//         .cs_ena_pretrans = 16, .cs_ena_posttrans = 4,
//         .queue_size = 3
//     };

//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_POSEDGE, .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
//     };

//     rdySem = xSemaphoreCreateBinary();
//     gpio_config(&io_conf);
//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

//     spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
//     gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);
//     spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
//     xSemaphoreGive(rdySem);

//     uint8_t *chunk_buf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
//     uint8_t *sendbuf_dummy = heap_caps_malloc(32, MALLOC_CAP_DMA);
    
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     int chunk_index = 0;

//     int start_x = (screen_width - IMG_WIDTH) / 2;
//     int start_y = (screen_height - IMG_HEIGHT) / 2;
//     uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

//     ESP_LOGI(TAG, "Loop start. Mode 1. 500kHz. No Shift.");

//     while (1) {
//         memset(chunk_buf, 0, CHUNK_SIZE);
//         snprintf((char*)sendbuf_dummy, 32, "Get Chunk %d", chunk_index);
        
//         t.length = CHUNK_SIZE * 8;
//         t.tx_buffer = sendbuf_dummy;
//         t.rx_buffer = chunk_buf;

//         xSemaphoreTake(rdySem, portMAX_DELAY);
//         ret = spi_device_transmit(spi_handle, &t);
        
//         if (ret == ESP_OK) {
            
//             // ZDE NENÍ ŽÁDNÝ BIT-SHIFT (>> 1)
//             // Data by měla být správně díky .mode = 1

//             uint16_t *chunk_pixels = (uint16_t *)chunk_buf;

//             for (int y = 0; y < CHUNK_HEIGHT; y++) {
//                 int display_y = start_y + (chunk_index * CHUNK_HEIGHT) + y;
//                 if (display_y >= screen_height) break;

//                 int display_idx = (display_y * screen_width) + start_x;
//                 int chunk_row_start = y * IMG_WIDTH;

//                 for (int x = 0; x < IMG_WIDTH; x++) {
//                     uint16_t pixel = chunk_pixels[chunk_row_start + x];
                    
//                     // SWAP: Stále prohazujeme bajty (to je nutné pro displej)
//                     lcd_pixels[display_idx + x] = (pixel >> 8) | (pixel << 8);
//                 }
//             }
//         }

//         chunk_index++;

//         if (chunk_index >= TOTAL_CHUNKS) {
//             chunk_index = 0;
//             esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
//         }
//     }
// }


/*
 * P4 Master SPI Receiver + MIPI DSI Display
 * Verze: PRECISE TIMING (Mode 1 + Input Delay)
 */
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

static const char *TAG = "P4_CAM_DISPLAY";

// ==========================================
// 1. KONFIGURACE SPI
// ==========================================
#define GPIO_HANDSHAKE      20
#define GPIO_SCLK           21
#define GPIO_MOSI           22
#define GPIO_MISO           24
#define GPIO_CS             25
#define SENDER_HOST         SPI2_HOST

// QQVGA 160x120
#define IMG_WIDTH           160
#define IMG_HEIGHT          120
#define CHUNK_HEIGHT        20
#define BYTES_PER_PIXEL     2 // RGB565
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
    // --- A. INICIALIZACE DISPLEJE ---
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

    // --- B. INICIALIZACE SPI ---
    esp_err_t ret;
    spi_device_handle_t spi_handle;

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI, .miso_io_num = GPIO_MISO, .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 8192, 
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
        
        // ZRYCHLENÍ: 1 MHz (obraz bude plynulejší)
        .clock_speed_hz = 1000000, 
        
        // REŽIM: Mode 1 (fungoval ti lépe na barvy)
        .mode = 1,                  
        
        .spics_io_num = GPIO_CS,
        .cs_ena_pretrans = 16, 
        .cs_ena_posttrans = 4,
        .queue_size = 3,

        // === TADY JE TA MAGIE PRO "PŘESNOST" ===
        // Říkáme P4, ať s čtením bitu počká 60 nanosekund.
        // To kompenzuje délku drátů a čistí "tečky".
        .input_delay_ns = 60, 
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

    int start_x = (screen_width - IMG_WIDTH) / 2;
    int start_y = (screen_height - IMG_HEIGHT) / 2;
    uint16_t *lcd_pixels = (uint16_t *)frame_buffer;

    ESP_LOGI(TAG, "Start: Mode 1 + Input Delay 60ns + 1MHz");

    while (1) {
        memset(chunk_buf, 0, CHUNK_SIZE);
        snprintf((char*)sendbuf_dummy, 32, "Get Chunk %d", chunk_index);
        
        t.length = CHUNK_SIZE * 8;
        t.tx_buffer = sendbuf_dummy;
        t.rx_buffer = chunk_buf;

        xSemaphoreTake(rdySem, portMAX_DELAY);
        ret = spi_device_transmit(spi_handle, &t);
        
        if (ret == ESP_OK) {
            
            // Žádný softwarový posun (dělá to HW díky Mode 1 a Delay)
            uint16_t *chunk_pixels = (uint16_t *)chunk_buf;

            for (int y = 0; y < CHUNK_HEIGHT; y++) {
                int display_y = start_y + (chunk_index * CHUNK_HEIGHT) + y;
                if (display_y >= screen_height) break;

                int display_idx = (display_y * screen_width) + start_x;
                int chunk_row_start = y * IMG_WIDTH;

                for (int x = 0; x < IMG_WIDTH; x++) {
                    uint16_t pixel = chunk_pixels[chunk_row_start + x];
                    
                    // Byte Swap necháváme, ten opravil barvy stolu
                    lcd_pixels[display_idx + x] = (pixel >> 8) | (pixel << 8);
                }
            }
        }

        chunk_index++;

        if (chunk_index >= TOTAL_CHUNKS) {
            chunk_index = 0;
            esp_cache_msync((void *)frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
    }
}